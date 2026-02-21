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
#define encoder_decode_position_f32 aeat9955_decode_position_f32
#define ENCODER_FRAME_WARNING_BIT 0x80U
#define ENCODER_FRAME_ERROR_BIT 0x40U

static inline void encoder_parse_frame_flags(const uint8_t *buffer, uint8_t *status,
					     bool *warning, bool *error)
{
	const struct aeat9955_sample *sample = (const struct aeat9955_sample *)buffer;
	uint8_t frame_status = sample->raw[0] & (ENCODER_FRAME_WARNING_BIT | ENCODER_FRAME_ERROR_BIT);

	if (status != NULL) {
		*status = frame_status;
	}
	if (warning != NULL) {
		*warning = (frame_status & ENCODER_FRAME_WARNING_BIT) != 0U;
	}
	if (error != NULL) {
		*error = (frame_status & ENCODER_FRAME_ERROR_BIT) != 0U;
	}
}
#elif DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), magntek_mt6835)
#include <drivers/sensor/magntek_mt6835.h>
#define encoder_decode_position_f32 mt6835_decode_position_f32

static inline void encoder_parse_frame_flags(const uint8_t *buffer, uint8_t *status,
					     bool *warning, bool *error)
{
	ARG_UNUSED(buffer);
	if (status != NULL) {
		*status = 0U;
	}
	if (warning != NULL) {
		*warning = false;
	}
	if (error != NULL) {
		*error = false;
	}
}
#else
#error "Unsupported encoder type for encoder1 alias"
#endif

SENSOR_DT_READ_IODEV(motor_encoder_iodev, DT_ALIAS(encoder1), {SENSOR_CHAN_ROTATION, 0});
RTIO_DEFINE_WITH_MEMPOOL(motor_encoder_rtio_ctx, 8, 8, 16, 16, sizeof(void *));
static atomic_t motor_encoder_pipeline_enabled;
static atomic_t motor_encoder_read_in_flight;

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
	return atomic_get(&motor_encoder_read_in_flight) != 0;
}

int motor_encoder_pipeline_request_sample(void)
{
	if (!motor_encoder_pipeline_is_enabled()) {
		return -ESHUTDOWN;
	}

	if (atomic_cas(&motor_encoder_read_in_flight, 0, 1) == 0) {
		return -EALREADY;
	}

	int ret = sensor_read_async_mempool(&motor_encoder_iodev, &motor_encoder_rtio_ctx, NULL);
	if (ret != 0) {
		atomic_set(&motor_encoder_read_in_flight, 0);
		return ret;
	}

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
		return (atomic_get(&motor_encoder_read_in_flight) != 0) ? -EAGAIN : -ENODATA;
	}

	if (cqe->result != 0) {
		rtio_cqe_release(&motor_encoder_rtio_ctx, cqe);
		atomic_set(&motor_encoder_read_in_flight, 0);
		return -EIO;
	}

	uint8_t *buf = NULL;
	uint32_t buf_len = 0U;
	int rc = rtio_cqe_get_mempool_buffer(&motor_encoder_rtio_ctx, cqe, &buf, &buf_len);
	rtio_cqe_release(&motor_encoder_rtio_ctx, cqe);
	atomic_set(&motor_encoder_read_in_flight, 0);
	if (rc != 0) {
		return -EIO;
	}

	sample->angle_deg = encoder_decode_position_f32(buf);
	sample->angle_rad = sample->angle_deg * (PI_F32 / 180.0f);
	encoder_parse_frame_flags(buf, &sample->status, &sample->warning, &sample->error);
	rtio_release_buffer(&motor_encoder_rtio_ctx, buf, buf_len);

	if (sample->error) {
		return -EIO;
	}

	sample->fresh = true;
	return 0;
}
