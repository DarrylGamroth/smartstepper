/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_aeat_9955_fast

#include <errno.h>
#include <string.h>

#include <drivers/encoder/aeat9955_fast.h>
#include <drivers/encoder_rt.h>
#include <drivers/rt_spi.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(brcm_aeat9955_fast, CONFIG_LOG_DEFAULT_LEVEL);

#define AEAT9955_FAST_SPI16_FRAME_LEN 3U
#define AEAT9955_FAST_SPI8_CRC16_FRAME_LEN 8U
#define AEAT9955_FAST_MAX_FRAME_LEN AEAT9955_FAST_SPI8_CRC16_FRAME_LEN
#define AEAT9955_FAST_SPI16_REG_FRAME_LEN 2U
#define AEAT9955_FAST_SPI8_REG_READ_FRAME_LEN 4U
#define AEAT9955_FAST_SPI8_REG_WRITE_FRAME_LEN 3U
#define AEAT9955_FAST_REG_TIMEOUT_US 1000U
#define AEAT9955_FAST_CFG_SETTLE_US 1000U
#define AEAT9955_FAST_DEG_TO_RAD 0.017453292519943295769f
#define AEAT9955_FAST_CRC16_INIT 0xFFFFU
#define AEAT9955_FAST_CRC16_POLY 0x1021U

/*
 * SPI4-8 is transported as 8-bit SPI words. At 18-bit single-turn resolution,
 * the safety frame is not byte-aligned: OC + 18b position + 8b status + 8b SC
 * + 16b CRC = 58 clocks. The transport clocks 64 bits and this decoder
 * ignores the unused pad clocks after the CRC.
 */
#define AEAT9955_FAST_SPI8_POS_START_BIT 8U
#define AEAT9955_FAST_SPI8_STATUS_START_BIT \
	(AEAT9955_FAST_SPI8_POS_START_BIT + AEAT9955_FAST_RESOLUTION_BITS)
#define AEAT9955_FAST_SPI8_SC_START_BIT (AEAT9955_FAST_SPI8_STATUS_START_BIT + 8U)
#define AEAT9955_FAST_SPI8_CRC_START_BIT (AEAT9955_FAST_SPI8_SC_START_BIT + 8U)
#define AEAT9955_FAST_SPI8_CRC_INPUT_BITS \
	(AEAT9955_FAST_RESOLUTION_BITS + 8U + 8U)

struct aeat9955_fast_config {
	const struct device *transport;
	uint8_t pipeline_delay_samples;
	enum aeat9955_fast_spi4_mode initial_spi4_mode;
};

struct aeat9955_fast_data {
	enum encoder_rt_mode mode;
	enum aeat9955_fast_spi4_mode spi4_mode;
	bool sample_in_flight;
	struct encoder_rt_stats stats;
};

static const struct rt_spi_config aeat9955_fast_spi16_transport = {
	.cpol = false,
	.cpha = true,
};

static const struct rt_spi_config aeat9955_fast_spi8_transport = {
	.cpol = true,
	.cpha = true,
};

static inline uint8_t aeat9955_fast_frame_len(enum aeat9955_fast_spi4_mode mode)
{
	return (mode == AEAT9955_FAST_SPI4_8_CRC16) ?
		       AEAT9955_FAST_SPI8_CRC16_FRAME_LEN :
		       AEAT9955_FAST_SPI16_FRAME_LEN;
}

static inline void aeat9955_fast_prepare_position_frame(uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN],
							enum aeat9955_fast_spi4_mode mode)
{
	memset(tx, 0, AEAT9955_FAST_MAX_FRAME_LEN);
	if (mode == AEAT9955_FAST_SPI4_8_CRC16) {
		tx[0] = AEAT9955_FAST_CMD_POS_SPI8;
		return;
	}

	tx[0] = AEAT9955_FAST_CMD_READ_SPI16 |
		((~POPCOUNT(AEAT9955_FAST_REG_POS) & 1U) << 7);
	tx[1] = AEAT9955_FAST_REG_POS;
}

static inline void aeat9955_fast_prepare_read_frame(uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN],
						    uint8_t reg)
{
	memset(tx, 0, AEAT9955_FAST_MAX_FRAME_LEN);
	tx[0] = AEAT9955_FAST_CMD_READ_SPI16 | ((~POPCOUNT(reg) & 1U) << 7);
	tx[1] = reg;
}

static inline void aeat9955_fast_prepare_write_addr_frame(
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN], uint8_t reg)
{
	memset(tx, 0, AEAT9955_FAST_MAX_FRAME_LEN);
	tx[0] = AEAT9955_FAST_CMD_WRITE_SPI16 | ((POPCOUNT(reg) & 1U) << 7);
	tx[1] = reg;
}

static inline void aeat9955_fast_prepare_write_value_frame(
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN], uint8_t value)
{
	memset(tx, 0, AEAT9955_FAST_MAX_FRAME_LEN);
	tx[0] = ((POPCOUNT(value) & 1U) << 7);
	tx[1] = value;
}

static inline uint8_t aeat9955_fast_get_bit_msb(const uint8_t *raw, uint16_t bit_index)
{
	return (raw[bit_index / 8U] >> (7U - (bit_index % 8U))) & 1U;
}

static uint32_t aeat9955_fast_get_bits_msb(const uint8_t *raw, uint16_t start_bit,
					   uint8_t bit_count)
{
	uint32_t value = 0U;

	for (uint8_t i = 0U; i < bit_count; i++) {
		value = (value << 1) | aeat9955_fast_get_bit_msb(raw, start_bit + i);
	}

	return value;
}

static uint16_t aeat9955_fast_crc16_update_bits(uint16_t crc, uint8_t bits,
						uint8_t bit_count)
{
	for (uint8_t i = 0U; i < bit_count; i++) {
		bool input_bit = ((bits << i) & BIT(7)) != 0U;
		bool crc_msb = (crc & 0x8000U) != 0U;

		crc <<= 1;
		if (crc_msb ^ input_bit) {
			crc ^= AEAT9955_FAST_CRC16_POLY;
		}
	}

	return crc;
}

static uint16_t aeat9955_fast_crc16_spi8_position(const uint8_t *raw)
{
	/*
	 * The AEAT SPI4-8 safety frame is bit-packed for 18-bit position:
	 * CRC input is position[17:0] + status[7:0] + sequence[7:0], exactly
	 * 34 bits starting after the echoed operation code. Keep this bitwise
	 * instead of mixing byte helpers with partial trailing bits.
	 */
	uint16_t crc = AEAT9955_FAST_CRC16_INIT;

	for (uint8_t i = 0U; i < AEAT9955_FAST_SPI8_CRC_INPUT_BITS; i++) {
		uint8_t bit = aeat9955_fast_get_bit_msb(
			raw, AEAT9955_FAST_SPI8_POS_START_BIT + i);

		crc = aeat9955_fast_crc16_update_bits(crc, (uint8_t)(bit << 7), 1U);
	}

	return crc;
}

static int aeat9955_fast_decode_spi16_position(
	const uint8_t raw[AEAT9955_FAST_SPI16_FRAME_LEN], uint32_t *position,
	uint8_t *status, bool *status_error, bool *parity_error)
{
	const uint8_t status0 = raw[0];
	const uint32_t raw24 = sys_get_be24(raw) & 0x00FFFFFFU;
	const bool status_bit_error =
		(status0 & AEAT9955_FAST_POS_STATUS_ERROR_BIT) != 0U;
	const bool frame_parity_error = (POPCOUNT(raw24) & 1U) != 0U;

	if (position != NULL) {
		*position = (raw24 >> 4) & (AEAT9955_FAST_MAX_COUNT - 1U);
	}
	if (status_error != NULL) {
		*status_error = status_bit_error;
	}
	if (status != NULL) {
		*status = status0 & AEAT9955_FAST_FRAME_STATUS_MASK;
	}
	if (parity_error != NULL) {
		*parity_error = frame_parity_error;
	}

	return frame_parity_error ? -EIO : 0;
}

static int aeat9955_fast_decode_spi8_crc16_position(
	const uint8_t raw[AEAT9955_FAST_SPI8_CRC16_FRAME_LEN], uint32_t *position,
	uint8_t *status, uint8_t *sequence_counter, bool *status_warning,
	bool *crc_error)
{
	uint32_t pos = aeat9955_fast_get_bits_msb(raw, AEAT9955_FAST_SPI8_POS_START_BIT,
						 AEAT9955_FAST_RESOLUTION_BITS);
	uint8_t status_byte = (uint8_t)aeat9955_fast_get_bits_msb(
		raw, AEAT9955_FAST_SPI8_STATUS_START_BIT, 8U);
	uint8_t sc = (uint8_t)aeat9955_fast_get_bits_msb(raw, AEAT9955_FAST_SPI8_SC_START_BIT,
							 8U);
	uint16_t received_crc = (uint16_t)aeat9955_fast_get_bits_msb(
		raw, AEAT9955_FAST_SPI8_CRC_START_BIT, 16U);
	uint16_t expected_crc = aeat9955_fast_crc16_spi8_position(raw);
	bool crc_bad = received_crc != expected_crc;

	if (position != NULL) {
		*position = pos & (AEAT9955_FAST_MAX_COUNT - 1U);
	}
	if (status != NULL) {
		*status = status_byte;
	}
	if (sequence_counter != NULL) {
		*sequence_counter = sc;
	}
	if (status_warning != NULL) {
		uint8_t ready = status_byte >> 6;
		bool ready_ok = (ready == 0x02U) || (ready == 0x03U);
		bool alarm = (status_byte & 0x3FU) != 0U;

		*status_warning = !ready_ok || alarm;
	}
	if (crc_error != NULL) {
		*crc_error = crc_bad;
	}

	return crc_bad ? -EIO : 0;
}

static int aeat9955_fast_set_mode(const struct device *dev, enum encoder_rt_mode mode)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;

	if (mode > ENCODER_RT_MODE_DIAGNOSTIC) {
		return -EINVAL;
	}

	if ((data->sample_in_flight || rt_spi_busy(cfg->transport)) && mode != data->mode) {
		return -EBUSY;
	}

	data->mode = mode;
	return 0;
}

static int aeat9955_fast_request_sample(const struct device *dev)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN];
	uint8_t frame_len = aeat9955_fast_frame_len(data->spi4_mode);
	int ret;

	aeat9955_fast_prepare_position_frame(tx, data->spi4_mode);
	const struct rt_spi_transfer frame = {
		.tx = tx,
		.len = frame_len,
	};

	ret = rt_spi_request(cfg->transport, &frame);
	if (ret != 0) {
		if (ret == -EBUSY) {
			data->stats.busy_count++;
		} else {
			data->stats.request_error_count++;
		}
		return (ret == -EBUSY) ? -EALREADY : ret;
	}

	data->sample_in_flight = true;
	data->stats.request_count++;
	return 0;
}

static int aeat9955_fast_collect_sample(const struct device *dev,
					struct encoder_rt_sample *sample)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint32_t position = 0U;
	uint8_t status = 0U;
	uint8_t sequence_counter = 0U;
	uint8_t expected_len;
	bool status_error = false;
	bool parity_error = false;
	bool crc_error = false;
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	memset(sample, 0, sizeof(*sample));
	ret = rt_spi_collect(cfg->transport, &result);
	if (ret == -EAGAIN) {
		data->stats.pending_count++;
		return -EAGAIN;
	}
	if (ret == -ENODATA) {
		data->stats.empty_count++;
		return -ENODATA;
	}

	data->sample_in_flight = false;

	sample->timestamp_cycles = result.timestamp_cycles;
	if (ret != 0 || (result.flags & RT_SPI_RESULT_ERROR) != 0U) {
		sample->flags = ENCODER_RT_SAMPLE_ERROR | ENCODER_RT_SAMPLE_TRANSPORT_ERROR;
		data->stats.collect_error_count++;
		data->stats.transport_error_count++;
		return -EIO;
	}
	expected_len = aeat9955_fast_frame_len(data->spi4_mode);
	if (result.len != expected_len) {
		sample->flags = ENCODER_RT_SAMPLE_ERROR | ENCODER_RT_SAMPLE_FRAME_ERROR;
		data->stats.collect_error_count++;
		data->stats.frame_error_count++;
		return -EIO;
	}

	if (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) {
		ret = aeat9955_fast_decode_spi8_crc16_position(result.raw, &position, &status,
							       &sequence_counter,
							       &status_error, &crc_error);
	} else {
		ret = aeat9955_fast_decode_spi16_position(result.raw, &position, &status,
							  &status_error, &parity_error);
	}
	sample->raw_count = position;
	sample->status = ((uint32_t)sequence_counter << 8) | status;
	sample->mechanical_angle_deg =
		(float)((int32_t)position -
			(1 << (AEAT9955_FAST_RESOLUTION_BITS - 1))) *
		AEAT9955_FAST_COUNTS_TO_DEGREES;
	sample->mechanical_angle_rad = sample->mechanical_angle_deg * AEAT9955_FAST_DEG_TO_RAD;
	sample->flags = ENCODER_RT_SAMPLE_VALID;

	if (status_error) {
		sample->flags |= ENCODER_RT_SAMPLE_WARNING |
				 ENCODER_RT_SAMPLE_FRAME_STATUS_ERROR;
	}
	if (parity_error) {
		sample->flags |= ENCODER_RT_SAMPLE_ERROR |
				 ENCODER_RT_SAMPLE_FRAME_ERROR |
				 ENCODER_RT_SAMPLE_FRAME_PARITY_ERROR;
	}
	if (crc_error) {
		sample->flags |= ENCODER_RT_SAMPLE_ERROR |
				 ENCODER_RT_SAMPLE_FRAME_ERROR |
				 ENCODER_RT_SAMPLE_FRAME_CRC_ERROR;
	}

	if (status_error) {
		data->stats.warning_count++;
		data->stats.frame_status_error_count++;
	}
	if (ret != 0) {
		data->stats.collect_error_count++;
		data->stats.frame_error_count++;
		if (parity_error) {
			data->stats.frame_parity_error_count++;
		}
		if (crc_error) {
			data->stats.frame_crc_error_count++;
		}
	} else {
		data->stats.collect_count++;
	}

	return ret;
}

static void aeat9955_fast_get_stats(const struct device *dev,
				    struct encoder_rt_stats *stats)
{
	struct aeat9955_fast_data *data = dev->data;
	unsigned int key;

	if (stats == NULL) {
		return;
	}

	key = irq_lock();
	*stats = data->stats;
	irq_unlock(key);
}

static void aeat9955_fast_reset_stats(const struct device *dev)
{
	struct aeat9955_fast_data *data = dev->data;
	unsigned int key = irq_lock();

	memset(&data->stats, 0, sizeof(data->stats));
	irq_unlock(key);
}

static uint8_t aeat9955_fast_get_pipeline_delay(const struct device *dev)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;

	if (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) {
		return 0U;
	}
	return cfg->pipeline_delay_samples;
}

static int aeat9955_fast_transfer_blocking(const struct device *dev, const uint8_t *tx,
					   uint8_t len, struct rt_spi_result *result)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	const struct rt_spi_transfer frame = {
		.tx = tx,
		.len = len,
	};

	return rt_spi_transceive(cfg->transport, &frame, result, AEAT9955_FAST_REG_TIMEOUT_US);
}

static int aeat9955_fast_read_register_spi16(const struct device *dev, uint8_t reg,
					     uint8_t *value)
{
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN];
	int ret;

	aeat9955_fast_prepare_read_frame(tx, reg);
	ret = aeat9955_fast_transfer_blocking(dev, tx, AEAT9955_FAST_SPI16_REG_FRAME_LEN,
					      &result);
	if (ret != 0) {
		return ret;
	}

	ret = aeat9955_fast_transfer_blocking(dev, tx, AEAT9955_FAST_SPI16_REG_FRAME_LEN,
					      &result);
	if (ret == 0) {
		*value = result.raw[1];
	}

	return ret;
}

static int aeat9955_fast_write_register_spi16(const struct device *dev, uint8_t reg,
					      uint8_t value)
{
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN];
	int ret;

	aeat9955_fast_prepare_write_addr_frame(tx, reg);
	ret = aeat9955_fast_transfer_blocking(dev, tx, AEAT9955_FAST_SPI16_REG_FRAME_LEN,
					      &result);
	if (ret != 0) {
		return ret;
	}

	aeat9955_fast_prepare_write_value_frame(tx, value);
	return aeat9955_fast_transfer_blocking(dev, tx, AEAT9955_FAST_SPI16_REG_FRAME_LEN,
					       &result);
}

static int aeat9955_fast_read_register_spi8(const struct device *dev, uint8_t reg,
					    uint8_t *value)
{
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN] = {
		AEAT9955_FAST_CMD_READ_SPI8,
		reg,
		0x00U,
		0x00U,
	};
	int ret = aeat9955_fast_transfer_blocking(dev, tx,
						  AEAT9955_FAST_SPI8_REG_READ_FRAME_LEN,
						  &result);
	if (ret == 0) {
		ret = aeat9955_fast_transfer_blocking(dev, tx,
						      AEAT9955_FAST_SPI8_REG_READ_FRAME_LEN,
						      &result);
	}
	if (ret == 0) {
		*value = result.raw[3];
	}

	return ret;
}

static int aeat9955_fast_write_register_spi8(const struct device *dev, uint8_t reg,
					     uint8_t value)
{
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN] = {
		AEAT9955_FAST_CMD_WRITE_SPI8,
		reg,
		value,
	};

	return aeat9955_fast_transfer_blocking(dev, tx,
					       AEAT9955_FAST_SPI8_REG_WRITE_FRAME_LEN,
					       &result);
}

static int aeat9955_fast_unlock_level1_current_mode(const struct device *dev)
{
	struct aeat9955_fast_data *data = dev->data;

	return (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) ?
		       aeat9955_fast_write_register_spi8(dev, AEAT9955_FAST_REG_UNLOCK,
							 AEAT9955_FAST_UNLOCK_LEVEL1) :
		       aeat9955_fast_write_register_spi16(dev, AEAT9955_FAST_REG_UNLOCK,
							  AEAT9955_FAST_UNLOCK_LEVEL1);
}

static int aeat9955_fast_lock_level1_current_mode(const struct device *dev)
{
	struct aeat9955_fast_data *data = dev->data;

	return (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) ?
		       aeat9955_fast_write_register_spi8(dev, AEAT9955_FAST_REG_UNLOCK, 0x00U) :
		       aeat9955_fast_write_register_spi16(dev, AEAT9955_FAST_REG_UNLOCK, 0x00U);
}

static int aeat9955_fast_unlock_level2_default_current_mode(const struct device *dev)
{
	int ret = aeat9955_fast_unlock_level1_current_mode(dev);

	if (ret != 0) {
		return ret;
	}
	k_busy_wait(AEAT9955_FAST_CFG_SETTLE_US);

	for (uint8_t reg = AEAT9955_FAST_REG_PASSCODE_0;
	     reg <= AEAT9955_FAST_REG_PASSCODE_6; reg++) {
		ret = aeat9955_fast_write_register(dev, reg, 0x00U);
		if (ret != 0) {
			return ret;
		}
	}
	k_busy_wait(AEAT9955_FAST_CFG_SETTLE_US);

	return 0;
}

static int aeat9955_fast_set_transport_mode(const struct device *dev,
					    enum aeat9955_fast_spi4_mode mode)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	const struct rt_spi_config *spi_cfg =
		(mode == AEAT9955_FAST_SPI4_8_CRC16) ?
			&aeat9955_fast_spi8_transport :
			&aeat9955_fast_spi16_transport;

	return rt_spi_configure(cfg->transport, spi_cfg);
}

static int aeat9955_fast_set_driver_transport_mode(
	const struct device *dev, enum aeat9955_fast_spi4_mode mode)
{
	struct aeat9955_fast_data *data = dev->data;
	int ret = aeat9955_fast_set_transport_mode(dev, mode);

	if (ret != 0) {
		return ret;
	}

	data->spi4_mode = mode;
	return 0;
}

static int aeat9955_fast_probe_mode_unlocked(const struct device *dev,
					     enum aeat9955_fast_spi4_mode mode)
{
	struct aeat9955_fast_data *data = dev->data;
	uint8_t chip_id = 0U;
	int ret = aeat9955_fast_set_transport_mode(dev, mode);

	if (ret != 0) {
		return ret;
	}

	data->spi4_mode = mode;
	ret = (mode == AEAT9955_FAST_SPI4_8_CRC16) ?
		      aeat9955_fast_read_register_spi8(dev, AEAT9955_FAST_REG_CHIP_ID,
						       &chip_id) :
		      aeat9955_fast_read_register_spi16(dev, AEAT9955_FAST_REG_CHIP_ID,
							&chip_id);
	if (ret != 0) {
		return ret;
	}

	return (chip_id == AEAT9955_FAST_CHIP_ID) ? 0 : -EIO;
}

static int aeat9955_fast_detect_mode_unlocked(const struct device *dev,
					      enum aeat9955_fast_spi4_mode *mode)
{
	struct aeat9955_fast_data *data = dev->data;
	enum aeat9955_fast_spi4_mode original_mode = data->spi4_mode;
	struct rt_spi_config original_spi = {0};
	const struct aeat9955_fast_config *cfg = dev->config;

	rt_spi_get_config(cfg->transport, &original_spi);

	if (aeat9955_fast_probe_mode_unlocked(dev,
					      AEAT9955_FAST_SPI4_16_PARITY) == 0) {
		if (mode != NULL) {
			*mode = AEAT9955_FAST_SPI4_16_PARITY;
		}
		return 0;
	}

	if (aeat9955_fast_probe_mode_unlocked(dev,
					      AEAT9955_FAST_SPI4_8_CRC16) == 0) {
		if (mode != NULL) {
			*mode = AEAT9955_FAST_SPI4_8_CRC16;
		}
		return 0;
	}

	(void)rt_spi_configure(cfg->transport, &original_spi);
	data->spi4_mode = original_mode;
	return -EIO;
}

static int aeat9955_fast_check_idle(const struct device *dev)
{
	struct aeat9955_fast_data *data = dev->data;
	unsigned int key;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	key = irq_lock();
	if (data->mode == ENCODER_RT_MODE_REALTIME || data->sample_in_flight) {
		irq_unlock(key);
		return -EBUSY;
	}
	irq_unlock(key);

	return 0;
}

int aeat9955_fast_read_register(const struct device *dev, uint8_t reg, uint8_t *value)
{
	struct aeat9955_fast_data *data = dev->data;
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}
	ret = aeat9955_fast_check_idle(dev);
	if (ret != 0) {
		return ret;
	}

	return (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) ?
		       aeat9955_fast_read_register_spi8(dev, reg, value) :
		       aeat9955_fast_read_register_spi16(dev, reg, value);
}

int aeat9955_fast_write_register(const struct device *dev, uint8_t reg, uint8_t value)
{
	struct aeat9955_fast_data *data = dev->data;
	int ret;

	ret = aeat9955_fast_check_idle(dev);
	if (ret != 0) {
		return ret;
	}

	return (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) ?
		       aeat9955_fast_write_register_spi8(dev, reg, value) :
		       aeat9955_fast_write_register_spi16(dev, reg, value);
}

int aeat9955_fast_get_spi4_mode(const struct device *dev,
				enum aeat9955_fast_spi4_mode *mode)
{
	struct aeat9955_fast_data *data = dev->data;
	unsigned int key;

	if (mode == NULL) {
		return -EINVAL;
	}

	key = irq_lock();
	*mode = data->spi4_mode;
	irq_unlock(key);

	return 0;
}

int aeat9955_fast_set_spi4_mode_runtime(const struct device *dev,
					enum aeat9955_fast_spi4_mode mode)
{
	struct aeat9955_fast_data *data = dev->data;
	int ret;

	if (mode != AEAT9955_FAST_SPI4_16_PARITY &&
	    mode != AEAT9955_FAST_SPI4_8_CRC16) {
		return -EINVAL;
	}

	ret = aeat9955_fast_check_idle(dev);
	if (ret != 0) {
		return ret;
	}
	data->spi4_mode = mode;

	return 0;
}

int aeat9955_fast_detect_spi4_mode(const struct device *dev,
				   enum aeat9955_fast_spi4_mode *mode)
{
	int ret;

	if (mode == NULL) {
		return -EINVAL;
	}

	ret = aeat9955_fast_check_idle(dev);
	if (ret != 0) {
		return ret;
	}

	return aeat9955_fast_detect_mode_unlocked(dev, mode);
}

int aeat9955_fast_configure_runtime_mode(const struct device *dev,
					 enum aeat9955_fast_spi4_mode target_mode)
{
	struct aeat9955_fast_data *data = dev->data;
	enum aeat9955_fast_spi4_mode current_mode = AEAT9955_FAST_SPI4_16_PARITY;
	uint8_t reg0 = 0U;
	uint8_t reg7 = 0U;
	uint8_t reg9 = 0U;
	int ret;

	if (target_mode != AEAT9955_FAST_SPI4_16_PARITY &&
	    target_mode != AEAT9955_FAST_SPI4_8_CRC16) {
		return -EINVAL;
	}

	ret = aeat9955_fast_check_idle(dev);
	if (ret != 0) {
		return ret;
	}

	ret = aeat9955_fast_detect_mode_unlocked(dev, &current_mode);
	if (ret != 0) {
		return ret;
	}
	if (current_mode == target_mode) {
		return aeat9955_fast_set_driver_transport_mode(dev, target_mode);
	}

	if (target_mode == AEAT9955_FAST_SPI4_8_CRC16) {
		ret = aeat9955_fast_read_register(dev, AEAT9955_FAST_REG_CONFIG0, &reg0);
		if (ret == 0) {
			ret = aeat9955_fast_read_register(dev,
							  AEAT9955_FAST_REG_CONFIG0_SPI4,
							  &reg7);
		}
		if (ret == 0) {
			ret = aeat9955_fast_read_register(dev,
							  AEAT9955_FAST_REG_CONFIG1_PSEL,
							  &reg9);
		}
		if (ret != 0) {
			return ret;
		}

		ret = aeat9955_fast_unlock_level2_default_current_mode(dev);
		if (ret != 0) {
			return ret;
		}

		reg0 |= AEAT9955_FAST_CONFIG0_SAFETY_BIT |
			AEAT9955_FAST_CONFIG0_CRC_SELECT |
			AEAT9955_FAST_CONFIG0_CRC_INIT_FFFF;
		reg9 &= (uint8_t)~AEAT9955_FAST_CONFIG1_PSEL_BIT;

		ret = aeat9955_fast_write_register(dev, AEAT9955_FAST_REG_CONFIG0, reg0);
		if (ret == 0) {
			ret = aeat9955_fast_write_register(dev,
							  AEAT9955_FAST_REG_CONFIG1_PSEL,
							  reg9);
		}
		if (ret != 0) {
			return ret;
		}
		k_busy_wait(AEAT9955_FAST_CFG_SETTLE_US);

		uint8_t verify_reg0 = 0U;
		uint8_t verify_reg9 = 0U;
		ret = aeat9955_fast_read_register(dev, AEAT9955_FAST_REG_CONFIG0,
						  &verify_reg0);
		if (ret == 0) {
			ret = aeat9955_fast_read_register(dev,
							  AEAT9955_FAST_REG_CONFIG1_PSEL,
							  &verify_reg9);
		}
		if (ret != 0 ||
		    ((verify_reg0 & (AEAT9955_FAST_CONFIG0_SAFETY_BIT |
				     AEAT9955_FAST_CONFIG0_CRC_SELECT |
				     AEAT9955_FAST_CONFIG0_CRC_INIT_MASK)) !=
		     (AEAT9955_FAST_CONFIG0_SAFETY_BIT |
		      AEAT9955_FAST_CONFIG0_CRC_SELECT |
		      AEAT9955_FAST_CONFIG0_CRC_INIT_FFFF)) ||
		    ((verify_reg9 & AEAT9955_FAST_CONFIG1_PSEL_BIT) != 0U)) {
			(void)aeat9955_fast_lock_level1_current_mode(dev);
			return ret != 0 ? ret : -EIO;
		}

		reg7 = (reg7 & (uint8_t)~AEAT9955_FAST_CONFIG0_SPI4_MODE_MASK) |
		       AEAT9955_FAST_CONFIG0_SPI4_MODE_8;
		ret = aeat9955_fast_write_register(dev, AEAT9955_FAST_REG_CONFIG0_SPI4,
						   reg7);
		if (ret != 0) {
			return ret;
		}
		k_busy_wait(AEAT9955_FAST_CFG_SETTLE_US);

		ret = aeat9955_fast_set_driver_transport_mode(dev,
							      AEAT9955_FAST_SPI4_8_CRC16);
		if (ret != 0) {
			data->spi4_mode = AEAT9955_FAST_SPI4_16_PARITY;
			return ret;
		}

		uint8_t chip_id = 0U;
		ret = aeat9955_fast_read_register_spi8(dev, AEAT9955_FAST_REG_CHIP_ID,
						       &chip_id);
		if (ret != 0 || chip_id != AEAT9955_FAST_CHIP_ID) {
			return ret != 0 ? ret : -EIO;
		}

		return 0;
	}

	ret = aeat9955_fast_unlock_level2_default_current_mode(dev);
	if (ret != 0) {
		return ret;
	}
	ret = aeat9955_fast_read_register(dev, AEAT9955_FAST_REG_CONFIG0_SPI4, &reg7);
	if (ret != 0) {
		return ret;
	}
	reg7 = (reg7 & (uint8_t)~AEAT9955_FAST_CONFIG0_SPI4_MODE_MASK) |
	       AEAT9955_FAST_CONFIG0_SPI4_MODE_16;

	ret = aeat9955_fast_write_register(dev, AEAT9955_FAST_REG_CONFIG0_SPI4, reg7);
	if (ret != 0) {
		return ret;
	}
	k_busy_wait(AEAT9955_FAST_CFG_SETTLE_US);

	ret = aeat9955_fast_set_driver_transport_mode(dev,
						      AEAT9955_FAST_SPI4_16_PARITY);
	if (ret != 0) {
		data->spi4_mode = AEAT9955_FAST_SPI4_8_CRC16;
		return ret;
	}

	uint8_t chip_id = 0U;
	ret = aeat9955_fast_read_register_spi16(dev, AEAT9955_FAST_REG_CHIP_ID,
						&chip_id);
	if (ret != 0 || chip_id != AEAT9955_FAST_CHIP_ID) {
		(void)aeat9955_fast_set_driver_transport_mode(dev,
							      AEAT9955_FAST_SPI4_8_CRC16);
		return ret != 0 ? ret : -EIO;
	}

	(void)aeat9955_fast_lock_level1_current_mode(dev);
	return 0;
}

int aeat9955_fast_configure_spi4_8_crc16_volatile(const struct device *dev)
{
	return aeat9955_fast_configure_runtime_mode(dev, AEAT9955_FAST_SPI4_8_CRC16);
}

int aeat9955_fast_configure_spi4_16_parity_volatile(const struct device *dev)
{
	return aeat9955_fast_configure_runtime_mode(dev, AEAT9955_FAST_SPI4_16_PARITY);
}

int aeat9955_fast_read_position_raw(const struct device *dev, uint8_t *raw,
				    uint8_t raw_len, uint8_t *frame_len)
{
	struct aeat9955_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN];
	uint8_t len;
	unsigned int key;
	int ret;

	if (raw == NULL || frame_len == NULL) {
		return -EINVAL;
	}
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	key = irq_lock();
	if (data->mode == ENCODER_RT_MODE_REALTIME || data->sample_in_flight) {
		irq_unlock(key);
		return -EBUSY;
	}
	len = aeat9955_fast_frame_len(data->spi4_mode);
	irq_unlock(key);

	if (raw_len < len) {
		return -ENOBUFS;
	}

	aeat9955_fast_prepare_position_frame(tx, data->spi4_mode);
	ret = aeat9955_fast_transfer_blocking(dev, tx, len, &result);
	if (ret != 0) {
		return ret;
	}
	if (result.len != len) {
		return -EIO;
	}

	memcpy(raw, result.raw, len);
	*frame_len = len;
	return 0;
}

int aeat9955_fast_read_register_raw(const struct device *dev, uint8_t reg,
				    uint8_t *raw, uint8_t raw_len,
				    uint8_t *frame_len)
{
	struct aeat9955_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_MAX_FRAME_LEN] = {0};
	uint8_t len;
	unsigned int key;
	int ret;

	if (raw == NULL || frame_len == NULL) {
		return -EINVAL;
	}
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	key = irq_lock();
	if (data->mode == ENCODER_RT_MODE_REALTIME || data->sample_in_flight) {
		irq_unlock(key);
		return -EBUSY;
	}
	if (data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) {
		tx[0] = AEAT9955_FAST_CMD_READ_SPI8;
		tx[1] = reg;
		len = AEAT9955_FAST_SPI8_REG_READ_FRAME_LEN;
	} else {
		aeat9955_fast_prepare_read_frame(tx, reg);
		len = AEAT9955_FAST_SPI16_REG_FRAME_LEN;
	}
	irq_unlock(key);

	if (raw_len < len) {
		return -ENOBUFS;
	}

	ret = aeat9955_fast_transfer_blocking(dev, tx, len, &result);
	if (ret != 0) {
		return ret;
	}
	if (data->spi4_mode == AEAT9955_FAST_SPI4_16_PARITY ||
	    data->spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) {
		ret = aeat9955_fast_transfer_blocking(dev, tx, len, &result);
		if (ret != 0) {
			return ret;
		}
	}
	if (result.len != len) {
		return -EIO;
	}

	memcpy(raw, result.raw, len);
	*frame_len = len;
	return 0;
}

static int aeat9955_fast_init(const struct device *dev)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->transport)) {
		return -ENODEV;
	}

	data->mode = ENCODER_RT_MODE_DISABLED;
	data->sample_in_flight = false;
	data->spi4_mode = AEAT9955_FAST_SPI4_16_PARITY;

	ret = aeat9955_fast_configure_runtime_mode(dev, cfg->initial_spi4_mode);
	if (ret != 0) {
		LOG_ERR("failed to configure AEAT-9955 runtime SPI4 mode %d: %d",
			cfg->initial_spi4_mode, ret);
		return ret;
	}

	LOG_INF("configured AEAT-9955 runtime SPI4 mode %d", cfg->initial_spi4_mode);

	return 0;
}

static DEVICE_API(encoder_rt, aeat9955_fast_api) = {
	.set_mode = aeat9955_fast_set_mode,
	.request_sample = aeat9955_fast_request_sample,
	.collect_sample = aeat9955_fast_collect_sample,
	.get_stats = aeat9955_fast_get_stats,
	.reset_stats = aeat9955_fast_reset_stats,
	.get_pipeline_delay = aeat9955_fast_get_pipeline_delay,
};

#define AEAT9955_FAST_INIT(inst)							\
	static const struct aeat9955_fast_config aeat9955_fast_config_##inst = {	\
		.transport = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),		\
		.pipeline_delay_samples =						\
			DT_INST_PROP(inst, pipeline_delay_samples),			\
		.initial_spi4_mode = DT_INST_ENUM_IDX_OR(inst, spi4_mode, 0),	\
	};										\
	static struct aeat9955_fast_data aeat9955_fast_data_##inst = {		\
		.mode = ENCODER_RT_MODE_DISABLED,				\
	};										\
	DEVICE_DT_INST_DEFINE(inst, aeat9955_fast_init, NULL,			\
			      &aeat9955_fast_data_##inst,			\
			      &aeat9955_fast_config_##inst, POST_KERNEL,		\
			      CONFIG_ENCODER_RT_INIT_PRIORITY, &aeat9955_fast_api);

DT_INST_FOREACH_STATUS_OKAY(AEAT9955_FAST_INIT)
