/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_encoder_pipeline.h"

#include <errno.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "motor/math/math_constants.h"

/* Include encoder-specific headers based on devicetree */
#if DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955_fast)
#include <drivers/encoder/aeat9955_fast.h>
#include <drivers/encoder_rt.h>
#define MOTOR_ENCODER_PIPELINE_FAST_AEAT 1

static const struct device *const motor_encoder_rt_dev = DEVICE_DT_GET(DT_ALIAS(encoder1));
#elif DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955)
#include <zephyr/drivers/sensor.h>
#include <drivers/sensor/brcm_aeat9955.h>
#define encoder_decode_sample_f32 aeat9955_decode_sample_f32

static inline void encoder_inject_status_fault(uint8_t *buffer)
{
	struct aeat9955_sample *sample = (struct aeat9955_sample *)buffer;

	/* AEAT has no warning-only status path in fast frame; force status error bit. */
	sample->raw[0] |= AEAT9955_POS_STATUS_ERROR_BIT;
}

static inline void encoder_inject_frame_fault(uint8_t *buffer)
{
	struct aeat9955_sample *sample = (struct aeat9955_sample *)buffer;

	/* Break parity check by toggling parity/status bit. */
	sample->raw[0] ^= AEAT9955_POS_STATUS_PARITY_BIT;
}
#elif DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), magntek_mt6835)
#include <zephyr/drivers/sensor.h>
#include <drivers/sensor/magntek_mt6835.h>
#define encoder_decode_sample_f32 mt6835_decode_sample_f32

static inline void encoder_inject_status_fault(uint8_t *buffer)
{
	struct mt6835_sample *sample = (struct mt6835_sample *)buffer;

	sample->raw[4] = (sample->raw[4] & (uint8_t)~MT6835_STATUS_MASK) |
			 MT6835_STATUS_BIT1_WEAK_MAGNETIC;
	/* Keep frame valid while injecting status warning bits. */
	sample->raw[5] = crc8_ccitt(0x00U, &sample->raw[2], 3U);
}

static inline void encoder_inject_frame_fault(uint8_t *buffer)
{
	struct mt6835_sample *sample = (struct mt6835_sample *)buffer;

	/* Flip CRC byte so decoder reports frame error. */
	sample->raw[5] ^= 0x01U;
}
#else
#error "Unsupported encoder type for encoder1 alias"
#endif

#ifndef MOTOR_ENCODER_PIPELINE_FAST_AEAT
SENSOR_DT_READ_IODEV(motor_encoder_iodev, DT_ALIAS(encoder1), {SENSOR_CHAN_ROTATION, 0});
RTIO_DEFINE_WITH_MEMPOOL(motor_encoder_rtio_ctx, 8, 8, 16, 16, sizeof(void *));
#endif
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
static atomic_t motor_encoder_test_inject_mode;

/*
 * Safe baseline for Zephyr STM32 SPI RTIO. Back-to-back overlapping encoder
 * reads can fault inside the upstream SPI completion path on the AEAT-9955
 * hardware. Keep only one request in flight until the SPI backend path is
 * proven safe for queued ISR-rate submissions.
 */
#define MOTOR_ENCODER_PIPELINE_MAX_INFLIGHT 1

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

void motor_encoder_pipeline_set_test_inject_mode(enum motor_encoder_test_inject_mode mode)
{
	if ((mode < MOTOR_ENCODER_TEST_INJECT_NONE) ||
	    (mode > MOTOR_ENCODER_TEST_INJECT_FRAME)) {
		mode = MOTOR_ENCODER_TEST_INJECT_NONE;
	}

	atomic_set(&motor_encoder_test_inject_mode, (atomic_val_t)mode);
}

enum motor_encoder_test_inject_mode motor_encoder_pipeline_get_test_inject_mode(void)
{
	return (enum motor_encoder_test_inject_mode)atomic_get(&motor_encoder_test_inject_mode);
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

#ifdef MOTOR_ENCODER_PIPELINE_FAST_AEAT
	int ret = encoder_rt_request_sample(motor_encoder_rt_dev);
	if (ret == -EALREADY || ret == -EBUSY) {
		atomic_inc(&motor_encoder_request_busy_count);
		return -EALREADY;
	}
	if (ret != 0) {
		atomic_inc(&motor_encoder_request_error_count);
		return ret;
	}
#else
	int ret = sensor_read_async_mempool(&motor_encoder_iodev, &motor_encoder_rtio_ctx, NULL);
	if (ret != 0) {
		atomic_inc(&motor_encoder_request_error_count);
		return ret;
	}
#endif

	atomic_inc(&motor_encoder_read_in_flight_count);
	atomic_inc(&motor_encoder_request_ok_count);
	return 0;
}

#ifdef MOTOR_ENCODER_PIPELINE_FAST_AEAT
static int motor_encoder_pipeline_collect_fast(struct motor_encoder_sample *sample)
{
	struct encoder_rt_sample enc_sample = {0};
	int ret = encoder_rt_collect_sample(motor_encoder_rt_dev, &enc_sample);

	if (ret == -EAGAIN) {
		atomic_inc(&motor_encoder_collect_pending_count);
		return -EAGAIN;
	}
	if (ret == -ENODATA) {
		atomic_inc(&motor_encoder_collect_empty_count);
		return -ENODATA;
	}

	motor_encoder_inflight_decrement();

	sample->angle_deg = enc_sample.mechanical_angle_deg;
	sample->angle_rad = enc_sample.mechanical_angle_rad;
	sample->status = (uint8_t)(enc_sample.status & 0xFFU);
	sample->warning = (enc_sample.flags & ENCODER_RT_SAMPLE_WARNING) != 0U;
	sample->error = (enc_sample.flags & ENCODER_RT_SAMPLE_ERROR) != 0U;
	sample->frame_status_error =
		(enc_sample.flags & ENCODER_RT_SAMPLE_FRAME_STATUS_ERROR) != 0U;
	sample->frame_parity_error =
		(enc_sample.flags & ENCODER_RT_SAMPLE_FRAME_PARITY_ERROR) != 0U;

	enum motor_encoder_test_inject_mode inject_mode =
		(enum motor_encoder_test_inject_mode)atomic_get(&motor_encoder_test_inject_mode);
	if (inject_mode == MOTOR_ENCODER_TEST_INJECT_STATUS) {
		sample->status |= AEAT9955_FAST_POS_STATUS_ERROR_BIT;
		sample->warning = true;
		sample->frame_status_error = true;
	} else if (inject_mode == MOTOR_ENCODER_TEST_INJECT_FRAME) {
		sample->status ^= AEAT9955_FAST_POS_STATUS_PARITY_BIT;
		sample->error = true;
		sample->frame_parity_error = true;
	}

	if (sample->frame_status_error) {
		atomic_inc(&motor_encoder_collect_frame_status_error_count);
	}

	if (ret != 0 || sample->error) {
		atomic_inc(&motor_encoder_collect_error_count);
		if ((enc_sample.flags & ENCODER_RT_SAMPLE_TRANSPORT_ERROR) != 0U) {
			atomic_inc(&motor_encoder_collect_transport_error_count);
		} else {
			atomic_inc(&motor_encoder_collect_frame_error_count);
		}
		if (sample->frame_parity_error) {
			atomic_inc(&motor_encoder_collect_frame_parity_error_count);
		}
		return -EIO;
	}

	sample->fresh = true;
	atomic_inc(&motor_encoder_collect_ok_count);
	return 0;
}
#else
static int motor_encoder_pipeline_decode_buffer(uint8_t *buf, struct motor_encoder_sample *sample)
{
	enum motor_encoder_test_inject_mode inject_mode =
		(enum motor_encoder_test_inject_mode)atomic_get(&motor_encoder_test_inject_mode);
	if (inject_mode != MOTOR_ENCODER_TEST_INJECT_NONE) {
		if (inject_mode == MOTOR_ENCODER_TEST_INJECT_STATUS) {
			encoder_inject_status_fault(buf);
		} else if (inject_mode == MOTOR_ENCODER_TEST_INJECT_FRAME) {
			encoder_inject_frame_fault(buf);
		}
	}

	int decode_ret = encoder_decode_sample_f32(buf, &sample->angle_deg, &sample->status,
						   &sample->warning, &sample->error,
						   &sample->frame_status_error,
						   &sample->frame_parity_error);
	if (decode_ret == 0) {
		sample->angle_rad = sample->angle_deg * (PI_F32 / 180.0f);
	}

	if (sample->frame_status_error) {
		/* Count all encoder status-flag assertions, including warning-only cases. */
		atomic_inc(&motor_encoder_collect_frame_status_error_count);
	}

	if (decode_ret != 0 || sample->error) {
		atomic_inc(&motor_encoder_collect_error_count);
		atomic_inc(&motor_encoder_collect_frame_error_count);
		if (sample->frame_parity_error) {
			atomic_inc(&motor_encoder_collect_frame_parity_error_count);
		}
		return -EIO;
	}

	sample->fresh = true;
	atomic_inc(&motor_encoder_collect_ok_count);
	return 0;
}

static int motor_encoder_pipeline_collect_rtio(struct motor_encoder_sample *sample)
{
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

	int decode_ret = motor_encoder_pipeline_decode_buffer(buf, sample);
	rtio_release_buffer(&motor_encoder_rtio_ctx, buf, buf_len);

	return decode_ret;
}
#endif

int motor_encoder_pipeline_collect(struct motor_encoder_sample *sample)
{
	struct motor_encoder_sample scratch = {0};
	if (sample == NULL) {
		sample = &scratch;
	}

	memset(sample, 0, sizeof(*sample));

#ifdef MOTOR_ENCODER_PIPELINE_FAST_AEAT
	return motor_encoder_pipeline_collect_fast(sample);
#else
	return motor_encoder_pipeline_collect_rtio(sample);
#endif
}
