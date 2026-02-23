/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_encoder_pipeline.h"

#include <errno.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "math_constants.h"

/* Include encoder-specific headers based on devicetree */
#if DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955)
#include <drivers/sensor/brcm_aeat9955.h>
#define encoder_decode_sample_f32 aeat9955_decode_sample_f32
#elif DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), magntek_mt6835)
#include <drivers/sensor/magntek_mt6835.h>
#define encoder_decode_sample_f32 mt6835_decode_sample_f32
#else
#error "Unsupported encoder type for encoder1 alias"
#endif

SENSOR_DT_READ_IODEV(motor_encoder_iodev, DT_ALIAS(encoder1), {SENSOR_CHAN_ROTATION, 0});
RTIO_DEFINE_WITH_MEMPOOL(motor_encoder_rtio_ctx, 8, 8, 16, 16, sizeof(void *));
static atomic_t motor_encoder_pipeline_enabled;
static atomic_t motor_encoder_read_in_flight_count;
static atomic_t motor_encoder_request_ok_count;
static atomic_t motor_encoder_request_busy_count;
static atomic_t motor_encoder_request_disabled_count;
static atomic_t motor_encoder_request_error_count;
static atomic_t motor_encoder_collect_ok_count;
static atomic_t motor_encoder_collect_pending_count;
static atomic_t motor_encoder_collect_empty_count;
static atomic_t motor_encoder_collect_error_count;
static atomic_t motor_encoder_collect_transport_error_count;
static atomic_t motor_encoder_collect_frame_error_count;
static atomic_t motor_encoder_collect_frame_parity_error_count;
static atomic_t motor_encoder_collect_frame_status_error_count;

#define MOTOR_ENCODER_PIPELINE_MAX_INFLIGHT 2

static inline void motor_encoder_inflight_decrement(void)
{
	if (atomic_get(&motor_encoder_read_in_flight_count) > 0) {
		atomic_dec(&motor_encoder_read_in_flight_count);
	}
}

void motor_encoder_pipeline_set_enabled(bool enabled)
{
	atomic_set(&motor_encoder_pipeline_enabled, enabled ? 1 : 0);
}

bool motor_encoder_pipeline_is_enabled(void)
{
	return atomic_get(&motor_encoder_pipeline_enabled) != 0;
}

bool motor_encoder_pipeline_is_busy(void)
{
	return atomic_get(&motor_encoder_read_in_flight_count) != 0;
}

void motor_encoder_pipeline_get_stats(struct motor_encoder_pipeline_stats *stats)
{
	if (stats == NULL) {
		return;
	}

	stats->request_ok = (uint32_t)atomic_get(&motor_encoder_request_ok_count);
	stats->request_busy = (uint32_t)atomic_get(&motor_encoder_request_busy_count);
	stats->request_disabled = (uint32_t)atomic_get(&motor_encoder_request_disabled_count);
	stats->request_error = (uint32_t)atomic_get(&motor_encoder_request_error_count);
	stats->collect_ok = (uint32_t)atomic_get(&motor_encoder_collect_ok_count);
	stats->collect_pending = (uint32_t)atomic_get(&motor_encoder_collect_pending_count);
	stats->collect_empty = (uint32_t)atomic_get(&motor_encoder_collect_empty_count);
	stats->collect_error = (uint32_t)atomic_get(&motor_encoder_collect_error_count);
	stats->collect_transport_error =
		(uint32_t)atomic_get(&motor_encoder_collect_transport_error_count);
	stats->collect_frame_error =
		(uint32_t)atomic_get(&motor_encoder_collect_frame_error_count);
	stats->collect_frame_parity_error =
		(uint32_t)atomic_get(&motor_encoder_collect_frame_parity_error_count);
	stats->collect_frame_status_error =
		(uint32_t)atomic_get(&motor_encoder_collect_frame_status_error_count);
}

void motor_encoder_pipeline_reset_stats(void)
{
	atomic_set(&motor_encoder_request_ok_count, 0);
	atomic_set(&motor_encoder_request_busy_count, 0);
	atomic_set(&motor_encoder_request_disabled_count, 0);
	atomic_set(&motor_encoder_request_error_count, 0);
	atomic_set(&motor_encoder_collect_ok_count, 0);
	atomic_set(&motor_encoder_collect_pending_count, 0);
	atomic_set(&motor_encoder_collect_empty_count, 0);
	atomic_set(&motor_encoder_collect_error_count, 0);
	atomic_set(&motor_encoder_collect_transport_error_count, 0);
	atomic_set(&motor_encoder_collect_frame_error_count, 0);
	atomic_set(&motor_encoder_collect_frame_parity_error_count, 0);
	atomic_set(&motor_encoder_collect_frame_status_error_count, 0);
}

int motor_encoder_pipeline_request_sample(void)
{
	if (!motor_encoder_pipeline_is_enabled()) {
		atomic_inc(&motor_encoder_request_disabled_count);
		return -ESHUTDOWN;
	}

	if (atomic_get(&motor_encoder_read_in_flight_count) >=
	    MOTOR_ENCODER_PIPELINE_MAX_INFLIGHT) {
		atomic_inc(&motor_encoder_request_busy_count);
		return -EALREADY;
	}

	int ret = sensor_read_async_mempool(&motor_encoder_iodev, &motor_encoder_rtio_ctx, NULL);
	if (ret != 0) {
		atomic_inc(&motor_encoder_request_error_count);
		return ret;
	}

	atomic_inc(&motor_encoder_read_in_flight_count);
	atomic_inc(&motor_encoder_request_ok_count);
	return 0;
}

int motor_encoder_pipeline_collect(struct motor_encoder_sample *sample)
{
	struct motor_encoder_sample scratch = {0};
	if (sample == NULL) {
		sample = &scratch;
	}

	memset(sample, 0, sizeof(*sample));

	struct rtio_cqe *cqe = rtio_cqe_consume(&motor_encoder_rtio_ctx);
	if (cqe == NULL) {
		/* Distinguish pending transfer from missing source trigger. */
		if (atomic_get(&motor_encoder_read_in_flight_count) != 0) {
			atomic_inc(&motor_encoder_collect_pending_count);
			return -EAGAIN;
		}
		atomic_inc(&motor_encoder_collect_empty_count);
		return -ENODATA;
	}

	if (cqe->result != 0) {
		/* Even failed CQEs may carry a mempool buffer; release it to avoid
		 * exhausting RTIO buffers under repeated transfer faults.
		 */
		uint8_t *buf = NULL;
		uint32_t buf_len = 0U;
		if (rtio_cqe_get_mempool_buffer(&motor_encoder_rtio_ctx, cqe, &buf, &buf_len) == 0 &&
		    buf != NULL) {
			rtio_release_buffer(&motor_encoder_rtio_ctx, buf, buf_len);
		}
		rtio_cqe_release(&motor_encoder_rtio_ctx, cqe);
		motor_encoder_inflight_decrement();
		atomic_inc(&motor_encoder_collect_error_count);
		atomic_inc(&motor_encoder_collect_transport_error_count);
		return -EIO;
	}

	uint8_t *buf = NULL;
	uint32_t buf_len = 0U;
	int rc = rtio_cqe_get_mempool_buffer(&motor_encoder_rtio_ctx, cqe, &buf, &buf_len);
	rtio_cqe_release(&motor_encoder_rtio_ctx, cqe);
	motor_encoder_inflight_decrement();
	if (rc != 0) {
		atomic_inc(&motor_encoder_collect_error_count);
		atomic_inc(&motor_encoder_collect_transport_error_count);
		return -EIO;
	}

	int decode_ret = encoder_decode_sample_f32(buf, &sample->angle_deg, &sample->status,
					    &sample->warning, &sample->error,
					    &sample->frame_status_error, &sample->frame_parity_error);
	if (decode_ret == 0) {
		sample->angle_rad = sample->angle_deg * (PI_F32 / 180.0f);
	}
	rtio_release_buffer(&motor_encoder_rtio_ctx, buf, buf_len);

	if (decode_ret != 0 || sample->error) {
		atomic_inc(&motor_encoder_collect_error_count);
		atomic_inc(&motor_encoder_collect_frame_error_count);
		if (sample->frame_parity_error) {
			atomic_inc(&motor_encoder_collect_frame_parity_error_count);
		}
		if (sample->frame_status_error) {
			atomic_inc(&motor_encoder_collect_frame_status_error_count);
		}
		return -EIO;
	}

	sample->fresh = true;
	atomic_inc(&motor_encoder_collect_ok_count);
	return 0;
}
