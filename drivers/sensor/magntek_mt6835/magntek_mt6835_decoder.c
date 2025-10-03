/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/dsp/dsp.h>
#include <string.h>
#include <drivers/sensor/magntek_mt6835.h>

LOG_MODULE_DECLARE(magntek_mt6835, CONFIG_SENSOR_LOG_LEVEL);

/* MT6835 has 21-bit resolution (2^21 = 2097152 counts per revolution) */
#define MT6835_RESOLUTION_BITS 21
#define MT6835_MAX_COUNT       (1U << MT6835_RESOLUTION_BITS)

/**
 * @brief Convert MT6835 raw position to Q31 angle for arm_sin_cos_q31
 *
 * The Q31 format maps [-1, 0.999999] to [-180°, 179°] for trigonometric functions.
 * MT6835 provides 0 to 2097151 counts for 0° to 359.999°.
 *
 * Conversion: Q31 = (raw_position << (32 - 21)) - 2^31
 * This maps:
 * - 0 counts -> Q31 = -2^31 (-180°)
 * - MT6835_MAX_COUNT/2 -> Q31 = 0 (0°)
 * - MT6835_MAX_COUNT-1 -> Q31 = 2^31-1 (179.999°)
 */
static inline void mt6835_position_convert_q31(q31_t *out, uint32_t raw_position)
{
	/* Ensure raw position is within valid range */
	raw_position &= (MT6835_MAX_COUNT - 1);

	/* Convert to Q31: shift left by (32-21=11) bits and subtract 2^31 to center around 0 */
	*out = (q31_t)((raw_position << (32 - MT6835_RESOLUTION_BITS)) - 0x80000000UL);
}

static int mt6835_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					  uint16_t *frame_count)
{
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

static int mt6835_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
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

static int mt6835_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				 uint32_t *fit, uint16_t max_count, void *data_out)
{
	if (*fit != 0) {
		return 0;
	}

	if (max_count == 0 || chan_spec.chan_idx != 0) {
		return -EINVAL;
	}

	const struct mt6835_sample *sample = (const struct mt6835_sample *)buffer;
	const uint8_t *raw = sample->raw;
	struct sensor_q31_data *out = data_out;
	out->header.reading_count = 1;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ROTATION:
		uint32_t position = sys_get_be24(&raw[2]) >> 3;

		out->header.base_timestamp_ns = sample->header.timestamp_ns;
		out->shift = 0; /* Q31 format doesn't use shift */
		out->readings[0].timestamp_delta = 0;
		mt6835_position_convert_q31(&out->readings[0].angle, position);
		break;
	default:
		return -ENOTSUP;
	}

	*fit = 1;
	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = mt6835_decoder_get_frame_count,
	.get_size_info = mt6835_decoder_get_size_info,
	.decode = mt6835_decoder_decode,
};

int mt6835_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
