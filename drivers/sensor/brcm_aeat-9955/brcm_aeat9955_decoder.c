/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/dsp/dsp.h>
#include <string.h>

#include "brcm_aeat9955.h"

LOG_MODULE_DECLARE(brcm_aeat_9955, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Convert AEAT9955 raw position to Q31 angle for arm_sin_cos_q31
 *
 * The Q31 format maps [-1, 0.999999] to [-180°, 179°] for trigonometric functions.
 * AEAT9955 provides 0 to 2097151 counts for 0° to 359.999°.
 *
 * Conversion: Q31 = (raw_position << (32 - 21)) - 2^31
 * This maps:
 * - 0 counts -> Q31 = -2^31 (-180°)
 * - AEAT9955_MAX_COUNT/2 -> Q31 = 0 (0°)
 * - AEAT9955_MAX_COUNT-1 -> Q31 = 2^31-1 (179.999°)
 */
static inline void aeat9955_position_convert_q31(q31_t *out, uint32_t raw_position)
{
	/* Ensure raw position is within valid range */
	raw_position &= (AEAT9955_MAX_COUNT - 1);

	/* Convert to Q31: shift left by (32-18=14) bits and subtract 2^31 to center around 0 */
	*out = (q31_t)((raw_position << (32 - AEAT9955_RESOLUTION_BITS)) - 0x80000000UL);
}

static int aeat9955_decoder_get_frame_count(const uint8_t *buffer,
					    struct sensor_chan_spec chan_spec,
					    uint16_t *frame_count)
{
	ARG_UNUSED(buffer);

	int ret = -ENOTSUP;

	if (chan_spec.chan_idx != 0) {
		return ret;
	}

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ROTATION:
		*frame_count = 1;
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int aeat9955_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					  size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ROTATION:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int aeat9955_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				   uint32_t *fit, uint16_t max_count, void *data_out)
{
	if (*fit != 0) {
		return 0;
	}

	if (max_count == 0 || chan_spec.chan_idx != 0) {
		return -EINVAL;
	}

	const struct aeat9955_sample *sample = (const struct aeat9955_sample *)buffer;
	const uint8_t *raw = sample->raw;
	struct sensor_q31_data *out = data_out;
	const uint8_t status = raw[2];

	if ((status & AEAT9955_STATUS_ERROR_BIT) != 0U) {
		return -EIO;
	}

	if ((status & AEAT9955_STATUS_PARITY_BIT) != 0U) {
		return -EBADMSG;
	}

	out->header.reading_count = 1;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ROTATION:
		uint32_t position = sys_get_be24(&raw[2]) >> 6;

		out->header.base_timestamp_ns = sample->header.timestamp_ns;
		out->shift = 0; /* Q31 format doesn't use shift */
		out->readings[0].timestamp_delta = 0;
		aeat9955_position_convert_q31(&out->readings[0].angle, position);
		break;
	default:
		return -ENOTSUP;
	}

	*fit = 1;
	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = aeat9955_decoder_get_frame_count,
	.get_size_info = aeat9955_decoder_get_size_info,
	.decode = aeat9955_decoder_decode,
};

int aeat9955_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
