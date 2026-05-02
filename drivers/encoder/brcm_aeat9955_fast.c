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

#define AEAT9955_FAST_FRAME_LEN 3U
#define AEAT9955_FAST_REG_FRAME_LEN 2U
#define AEAT9955_FAST_REG_TIMEOUT_US 1000U
#define AEAT9955_FAST_DEG_TO_RAD 0.017453292519943295769f

struct aeat9955_fast_config {
	const struct device *transport;
	uint8_t pipeline_delay_samples;
};

struct aeat9955_fast_data {
	enum encoder_rt_mode mode;
	bool sample_in_flight;
	struct encoder_rt_stats stats;
};

static inline void aeat9955_fast_prepare_position_frame(uint8_t tx[AEAT9955_FAST_FRAME_LEN])
{
	tx[0] = AEAT9955_FAST_CMD_READ_SPI16 |
		((~POPCOUNT(AEAT9955_FAST_REG_POS) & 1U) << 7);
	tx[1] = AEAT9955_FAST_REG_POS;
	tx[2] = 0x00U;
}

static inline void aeat9955_fast_prepare_read_frame(uint8_t tx[AEAT9955_FAST_REG_FRAME_LEN],
						    uint8_t reg)
{
	tx[0] = AEAT9955_FAST_CMD_READ_SPI16 | ((~POPCOUNT(reg) & 1U) << 7);
	tx[1] = reg;
}

static inline void aeat9955_fast_prepare_write_addr_frame(
	uint8_t tx[AEAT9955_FAST_REG_FRAME_LEN], uint8_t reg)
{
	tx[0] = AEAT9955_FAST_CMD_WRITE_SPI16 | ((POPCOUNT(reg) & 1U) << 7);
	tx[1] = reg;
}

static inline void aeat9955_fast_prepare_write_value_frame(
	uint8_t tx[AEAT9955_FAST_REG_FRAME_LEN], uint8_t value)
{
	tx[0] = ((POPCOUNT(value) & 1U) << 7);
	tx[1] = value;
}

static int aeat9955_fast_decode_position(const uint8_t raw[AEAT9955_FAST_FRAME_LEN],
					 uint32_t *position, bool *status_error,
					 bool *parity_error)
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
	if (parity_error != NULL) {
		*parity_error = frame_parity_error;
	}

	return frame_parity_error ? -EIO : 0;
}

static int aeat9955_fast_set_mode(const struct device *dev, enum encoder_rt_mode mode)
{
	struct aeat9955_fast_data *data = dev->data;
	unsigned int key;

	if (mode > ENCODER_RT_MODE_DIAGNOSTIC) {
		return -EINVAL;
	}

	key = irq_lock();
	if (data->sample_in_flight && mode != data->mode) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->mode = mode;
	irq_unlock(key);
	return 0;
}

static int aeat9955_fast_request_sample(const struct device *dev)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;
	uint8_t tx[AEAT9955_FAST_FRAME_LEN];
	unsigned int key;
	int ret;

	key = irq_lock();
	if (data->mode != ENCODER_RT_MODE_REALTIME) {
		data->stats.disabled_count++;
		irq_unlock(key);
		return -ESHUTDOWN;
	}
	if (data->sample_in_flight) {
		data->stats.busy_count++;
		irq_unlock(key);
		return -EALREADY;
	}
	data->sample_in_flight = true;
	irq_unlock(key);

	aeat9955_fast_prepare_position_frame(tx);
	const struct rt_spi_transfer frame = {
		.tx = tx,
		.len = sizeof(tx),
	};

	ret = rt_spi_request(cfg->transport, &frame);
	if (ret != 0) {
		key = irq_lock();
		data->sample_in_flight = false;
		if (ret == -EBUSY) {
			data->stats.busy_count++;
		} else {
			data->stats.request_error_count++;
		}
		irq_unlock(key);
		return (ret == -EBUSY) ? -EALREADY : ret;
	}

	key = irq_lock();
	data->stats.request_count++;
	irq_unlock(key);
	return 0;
}

static int aeat9955_fast_collect_sample(const struct device *dev,
					struct encoder_rt_sample *sample)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	struct aeat9955_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint32_t position = 0U;
	bool status_error = false;
	bool parity_error = false;
	unsigned int key;
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	memset(sample, 0, sizeof(*sample));
	ret = rt_spi_collect(cfg->transport, &result);
	if (ret == -EAGAIN) {
		key = irq_lock();
		data->stats.pending_count++;
		irq_unlock(key);
		return -EAGAIN;
	}
	if (ret == -ENODATA) {
		key = irq_lock();
		data->stats.empty_count++;
		irq_unlock(key);
		return -ENODATA;
	}

	key = irq_lock();
	data->sample_in_flight = false;
	irq_unlock(key);

	sample->timestamp_cycles = result.timestamp_cycles;
	if (ret != 0 || (result.flags & RT_SPI_RESULT_ERROR) != 0U) {
		sample->flags = ENCODER_RT_SAMPLE_ERROR | ENCODER_RT_SAMPLE_TRANSPORT_ERROR;
		key = irq_lock();
		data->stats.collect_error_count++;
		data->stats.transport_error_count++;
		irq_unlock(key);
		return -EIO;
	}
	if (result.len != AEAT9955_FAST_FRAME_LEN) {
		sample->flags = ENCODER_RT_SAMPLE_ERROR | ENCODER_RT_SAMPLE_FRAME_ERROR;
		key = irq_lock();
		data->stats.collect_error_count++;
		data->stats.frame_error_count++;
		irq_unlock(key);
		return -EIO;
	}

	ret = aeat9955_fast_decode_position(result.raw, &position, &status_error, &parity_error);
	sample->raw_count = position;
	sample->status = result.raw[0] & AEAT9955_FAST_FRAME_STATUS_MASK;
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

	key = irq_lock();
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
	} else {
		data->stats.collect_count++;
	}
	irq_unlock(key);

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

	return cfg->pipeline_delay_samples;
}

static int aeat9955_fast_transfer_blocking(const struct device *dev, const uint8_t *tx,
					   uint8_t len, struct rt_spi_result *result)
{
	const struct aeat9955_fast_config *cfg = dev->config;
	const uint32_t start_cycles = k_cycle_get_32();
	const uint32_t timeout_cycles = k_us_to_cyc_ceil32(AEAT9955_FAST_REG_TIMEOUT_US);
	const struct rt_spi_transfer frame = {
		.tx = tx,
		.len = len,
	};
	int ret;

	ret = rt_spi_request(cfg->transport, &frame);
	if (ret != 0) {
		return ret;
	}

	do {
		ret = rt_spi_collect(cfg->transport, result);
		if (ret != -EAGAIN) {
			return ret;
		}
		k_busy_wait(2);
	} while ((k_cycle_get_32() - start_cycles) < timeout_cycles);

	return -ETIMEDOUT;
}

int aeat9955_fast_read_register(const struct device *dev, uint8_t reg, uint8_t *value)
{
	struct aeat9955_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_REG_FRAME_LEN];
	unsigned int key;
	int ret;

	if (value == NULL) {
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
	irq_unlock(key);

	aeat9955_fast_prepare_read_frame(tx, reg);
	ret = aeat9955_fast_transfer_blocking(dev, tx, sizeof(tx), &result);
	if (ret != 0) {
		return ret;
	}

	ret = aeat9955_fast_transfer_blocking(dev, tx, sizeof(tx), &result);
	if (ret == 0) {
		*value = result.raw[1];
	}

	return ret;
}

int aeat9955_fast_write_register(const struct device *dev, uint8_t reg, uint8_t value)
{
	struct aeat9955_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint8_t tx[AEAT9955_FAST_REG_FRAME_LEN];
	unsigned int key;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	key = irq_lock();
	if (data->mode == ENCODER_RT_MODE_REALTIME || data->sample_in_flight) {
		irq_unlock(key);
		return -EBUSY;
	}
	irq_unlock(key);

	aeat9955_fast_prepare_write_addr_frame(tx, reg);
	ret = aeat9955_fast_transfer_blocking(dev, tx, sizeof(tx), &result);
	if (ret != 0) {
		return ret;
	}

	aeat9955_fast_prepare_write_value_frame(tx, value);
	return aeat9955_fast_transfer_blocking(dev, tx, sizeof(tx), &result);
}

static int aeat9955_fast_init(const struct device *dev)
{
	const struct aeat9955_fast_config *cfg = dev->config;

	if (!device_is_ready(cfg->transport)) {
		return -ENODEV;
	}

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
	};										\
	static struct aeat9955_fast_data aeat9955_fast_data_##inst = {		\
		.mode = ENCODER_RT_MODE_DISABLED,				\
	};										\
	DEVICE_DT_INST_DEFINE(inst, aeat9955_fast_init, NULL,			\
			      &aeat9955_fast_data_##inst,			\
			      &aeat9955_fast_config_##inst, POST_KERNEL,		\
			      CONFIG_ENCODER_RT_INIT_PRIORITY, &aeat9955_fast_api);

DT_INST_FOREACH_STATUS_OKAY(AEAT9955_FAST_INIT)
