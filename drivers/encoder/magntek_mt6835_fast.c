/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT magntek_mt6835_fast

#include <errno.h>
#include <string.h>

#include <drivers/encoder_rt.h>
#include <drivers/rt_spi.h>
#include <drivers/sensor/magntek_mt6835.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(magntek_mt6835_fast, CONFIG_LOG_DEFAULT_LEVEL);

#define MT6835_FAST_CMD_ANGLE (0x0AU << 4)
#define MT6835_FAST_REG_ANGLE_H 0x0003U
#define MT6835_FAST_FRAME_LEN 6U
#define MT6835_FAST_DEG_TO_RAD 0.017453292519943295769f

struct mt6835_fast_config {
	const struct device *transport;
	struct gpio_dt_spec cal_en_gpio;
	uint8_t pipeline_delay_samples;
	bool has_cal_en_gpio;
};

struct mt6835_fast_data {
	enum encoder_rt_mode mode;
	bool sample_in_flight;
	struct encoder_rt_stats stats;
};

static const uint8_t mt6835_fast_position_tx[MT6835_FAST_FRAME_LEN] = {
	MT6835_FAST_CMD_ANGLE | ((MT6835_FAST_REG_ANGLE_H >> 8) & 0x0F),
	MT6835_FAST_REG_ANGLE_H & 0xFF,
	0x00,
	0x00,
	0x00,
	0x00,
};

static int mt6835_fast_set_mode(const struct device *dev, enum encoder_rt_mode mode)
{
	const struct mt6835_fast_config *cfg = dev->config;
	struct mt6835_fast_data *data = dev->data;

	if (mode > ENCODER_RT_MODE_DIAGNOSTIC) {
		return -EINVAL;
	}

	if ((data->sample_in_flight || rt_spi_busy(cfg->transport)) && mode != data->mode) {
		return -EBUSY;
	}

	data->mode = mode;
	return 0;
}

static int mt6835_fast_request_sample(const struct device *dev)
{
	const struct mt6835_fast_config *cfg = dev->config;
	struct mt6835_fast_data *data = dev->data;
	const struct rt_spi_transfer frame = {
		.tx = mt6835_fast_position_tx,
		.len = MT6835_FAST_FRAME_LEN,
	};
	int ret;

	if (data->mode == ENCODER_RT_MODE_DISABLED) {
		data->stats.disabled_count++;
		return -ESHUTDOWN;
	}

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

static int mt6835_fast_collect_sample(const struct device *dev,
				      struct encoder_rt_sample *sample)
{
	const struct mt6835_fast_config *cfg = dev->config;
	struct mt6835_fast_data *data = dev->data;
	struct rt_spi_result result = {0};
	uint32_t position = 0U;
	bool crc_error = false;
	uint8_t status_bits;
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
	if (result.len != MT6835_FAST_FRAME_LEN) {
		sample->flags = ENCODER_RT_SAMPLE_ERROR | ENCODER_RT_SAMPLE_FRAME_ERROR;
		data->stats.collect_error_count++;
		data->stats.frame_error_count++;
		return -EIO;
	}

	status_bits = result.raw[4] & MT6835_STATUS_MASK;
	ret = mt6835_decode_position(result.raw, &position, &crc_error);

	sample->raw_count = position;
	sample->status = status_bits;
	sample->mechanical_angle_deg =
		(float)((int32_t)position - (1 << (MT6835_RESOLUTION_BITS - 1))) *
		MT6835_COUNTS_TO_DEGREES;
	sample->mechanical_angle_rad = sample->mechanical_angle_deg * MT6835_FAST_DEG_TO_RAD;
	sample->flags = ENCODER_RT_SAMPLE_VALID;

	if (status_bits != 0U) {
		sample->flags |= ENCODER_RT_SAMPLE_WARNING |
				 ENCODER_RT_SAMPLE_FRAME_STATUS_ERROR;
		data->stats.warning_count++;
		data->stats.frame_status_error_count++;
	}
	if (crc_error) {
		sample->flags |= ENCODER_RT_SAMPLE_ERROR |
				 ENCODER_RT_SAMPLE_FRAME_ERROR |
				 ENCODER_RT_SAMPLE_FRAME_CRC_ERROR;
		data->stats.collect_error_count++;
		data->stats.frame_error_count++;
		data->stats.frame_crc_error_count++;
		return -EIO;
	}

	data->stats.collect_count++;
	return 0;
}

static void mt6835_fast_abort_sample(const struct device *dev)
{
	const struct mt6835_fast_config *cfg = dev->config;
	struct mt6835_fast_data *data = dev->data;

	rt_spi_abort(cfg->transport);
	data->sample_in_flight = false;
}

static void mt6835_fast_get_stats(const struct device *dev, struct encoder_rt_stats *stats)
{
	struct mt6835_fast_data *data = dev->data;
	unsigned int key;

	if (stats == NULL) {
		return;
	}

	key = irq_lock();
	*stats = data->stats;
	irq_unlock(key);
}

static void mt6835_fast_reset_stats(const struct device *dev)
{
	struct mt6835_fast_data *data = dev->data;
	unsigned int key = irq_lock();

	memset(&data->stats, 0, sizeof(data->stats));
	irq_unlock(key);
}

static uint8_t mt6835_fast_get_pipeline_delay(const struct device *dev)
{
	const struct mt6835_fast_config *cfg = dev->config;

	return cfg->pipeline_delay_samples;
}

static int mt6835_fast_init(const struct device *dev)
{
	const struct mt6835_fast_config *cfg = dev->config;
	struct mt6835_fast_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->transport)) {
		LOG_ERR("RT SPI transport %s is not ready", cfg->transport->name);
		return -ENODEV;
	}

	if (cfg->has_cal_en_gpio) {
		if (!gpio_is_ready_dt(&cfg->cal_en_gpio)) {
			LOG_ERR("CAL_EN GPIO is not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->cal_en_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("Failed to configure CAL_EN GPIO (err %d)", ret);
			return ret;
		}
	}

	data->mode = ENCODER_RT_MODE_DISABLED;
	data->sample_in_flight = false;
	memset(&data->stats, 0, sizeof(data->stats));

	LOG_INF("Device %s: initialized", dev->name);
	return 0;
}

static DEVICE_API(encoder_rt, mt6835_fast_api) = {
	.set_mode = mt6835_fast_set_mode,
	.request_sample = mt6835_fast_request_sample,
	.collect_sample = mt6835_fast_collect_sample,
	.abort_sample = mt6835_fast_abort_sample,
	.get_stats = mt6835_fast_get_stats,
	.reset_stats = mt6835_fast_reset_stats,
	.get_pipeline_delay = mt6835_fast_get_pipeline_delay,
};

#define MT6835_FAST_CAL_EN_SPEC(inst)						\
	GPIO_DT_SPEC_INST_GET_OR(inst, cal_en_gpios, {0})

#define MT6835_FAST_INIT(inst)							\
	static const struct mt6835_fast_config mt6835_fast_config_##inst = {	\
		.transport = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),	\
		.cal_en_gpio = MT6835_FAST_CAL_EN_SPEC(inst),			\
		.pipeline_delay_samples =					\
			DT_INST_PROP(inst, pipeline_delay_samples),		\
		.has_cal_en_gpio = DT_INST_NODE_HAS_PROP(inst, cal_en_gpios),	\
	};									\
	static struct mt6835_fast_data mt6835_fast_data_##inst = {		\
		.mode = ENCODER_RT_MODE_DISABLED,				\
	};									\
	DEVICE_DT_INST_DEFINE(inst, mt6835_fast_init, NULL,			\
			      &mt6835_fast_data_##inst,			\
			      &mt6835_fast_config_##inst, POST_KERNEL,		\
			      CONFIG_ENCODER_RT_INIT_PRIORITY, &mt6835_fast_api);

DT_INST_FOREACH_STATUS_OKAY(MT6835_FAST_INIT)
