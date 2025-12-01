/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT magntek_mt6835

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include "magntek_mt6835.h"

LOG_MODULE_REGISTER(magntek_mt6835, CONFIG_SENSOR_LOG_LEVEL);

#define MT6835_FULL_ANGLE     360
#define MT6835_PULSES_PER_REV 2097152
#define MT6835_MILLION_UNIT   1000000

struct mt6835_config {
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_cal_en;
};

/* Device run time data */
struct mt6835_data {
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
	uint32_t position;
};

static int mt6835_read_register(const struct device *dev, uint16_t reg, uint8_t *data)
{
	const struct mt6835_config *dev_cfg = dev->config;
	uint8_t tx_buf[3];
	uint8_t rx_buf[3];
	struct spi_buf spi_tx_buf = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	struct spi_buf spi_rx_buf = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &spi_tx_buf,
		.count = 1,
	};
	struct spi_buf_set rx = {
		.buffers = &spi_rx_buf,
		.count = 1,
	};

	tx_buf[0] = MT6835_CMD_READ | ((reg >> 8) & 0x0F);
	tx_buf[1] = reg & 0xFF;
	tx_buf[2] = 0;

	int ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret >= 0) {
		*data = rx_buf[2];
	}

	return ret;
}

static int mt6835_write_register(const struct device *dev, uint16_t reg, uint8_t value)
{
	const struct mt6835_config *dev_cfg = dev->config;
	uint8_t buf[3];
	struct spi_buf tx_buf = {
		.buf = buf,
		.len = sizeof(buf),
	};
	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	buf[0] = MT6835_CMD_WRITE | ((reg >> 8) & 0x0F);
	buf[1] = reg & 0xFF;
	buf[2] = value;

	return spi_write_dt(&dev_cfg->bus, &tx);
}

static int mt6835_read_angle(const struct device *dev, uint32_t *angle)
{
	const struct mt6835_config *dev_cfg = dev->config;
	uint8_t tx_buf[6];
	uint8_t rx_buf[6];
	struct spi_buf spi_tx_buf = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	struct spi_buf spi_rx_buf = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &spi_tx_buf,
		.count = 1,
	};
	struct spi_buf_set rx = {
		.buffers = &spi_rx_buf,
		.count = 1,
	};
	tx_buf[0] = MT6835_CMD_ANGLE | ((MT6835_REG_ANGLE_H >> 8) & 0x0F);
	tx_buf[1] = MT6835_REG_ANGLE_H & 0xFF;
	tx_buf[2] = 0;
	tx_buf[3] = 0;
	tx_buf[4] = 0;
	tx_buf[5] = 0;

	int ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret == 0) {
		*angle = sys_get_be24(&rx_buf[2]) >> 3;
	} else {
		*angle = 0;
	}

	return ret;
}

static int mt6835_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;
	uint8_t reg_val;

	switch ((int)attr) {
	case MT6835_ATTR_ABZ_RESOLUTION:
		/* Set ABZ resolution - val->val1 should be 0-3 for different PPR settings */
		if (val->val1 < 0 || val->val1 > 3) {
			LOG_ERR("Invalid ABZ resolution value: %d", val->val1);
			return -EINVAL;
		}

		/* Read current ABZ_RES1 register */
		ret = mt6835_read_register(dev, MT6835_REG_ABZ_RES1, &reg_val);
		if (ret < 0) {
			return ret;
		}

		/* Update resolution bits (bits 0-1) */
		reg_val = (reg_val & 0xFC) | (val->val1 & 0x03);
		ret = mt6835_write_register(dev, MT6835_REG_ABZ_RES1, reg_val);
		break;

	case MT6835_ATTR_ZERO_POSITION:
		/* Set zero position - val->val1 is high byte, val->val2 is low byte */
		ret = mt6835_write_register(dev, MT6835_REG_ZERO_POS_H, val->val1 & 0xFF);
		if (ret < 0) {
			return ret;
		}
		ret = mt6835_write_register(dev, MT6835_REG_ZERO_POS_L, val->val2 & 0xFF);
		break;

	case MT6835_ATTR_CALIBRATION:
		/* Trigger auto-calibration process */
		if (val->val1 == 1) {
			const struct mt6835_config *config = dev->config;
			uint8_t status_reg;
			int timeout_count = 0;
			const int max_timeout = 1000; /* 10 second timeout (10ms * 1000) */

			LOG_INF("Starting auto-calibration process");

			/* Assert CAL_EN pin to start calibration */
			ret = gpio_pin_set_dt(&config->gpio_cal_en, 1);
			if (ret < 0) {
				LOG_ERR("Failed to assert CAL_EN pin");
				return ret;
			}

			/* Monitor calibration status via register 0x113 */
			do {
				k_sleep(K_MSEC(10)); /* Small delay between status checks */

				ret = mt6835_read_register(dev, MT6835_REG_CAL_STATUS, &status_reg);
				if (ret < 0) {
					LOG_ERR("Failed to read calibration status register");
					gpio_pin_set_dt(&config->gpio_cal_en, 0);
					return ret;
				}

				/* Check calibration status bits [7:6] */
				uint8_t cal_status = status_reg & MT6835_CAL_STATUS_MASK;

				if (cal_status == MT6835_CAL_STATUS_NONE) {
					/* No calibration */
					LOG_DBG("Calibration not started yet");
				} else if (cal_status == MT6835_CAL_STATUS_RUNNING) {
					/* Running auto calibration */
					LOG_DBG("Auto-calibration in progress...");
				} else if (cal_status == MT6835_CAL_STATUS_FAILED) {
					/* Calibration failed */
					LOG_ERR("Auto-calibration failed");
					gpio_pin_set_dt(&config->gpio_cal_en, 0);
					return -EIO;
				} else if (cal_status == MT6835_CAL_STATUS_SUCCESS) {
					/* Calibration successful */
					LOG_INF("Auto-calibration completed successfully");
					break;
				}

				timeout_count++;
				if (timeout_count >= max_timeout) {
					LOG_ERR("Auto-calibration timeout after %d seconds",
						max_timeout / 100);
					gpio_pin_set_dt(&config->gpio_cal_en, 0);
					return -ETIMEDOUT;
				}

			} while (1);

			/* Release CAL_EN pin after successful calibration */
			ret = gpio_pin_set_dt(&config->gpio_cal_en, 0);
			if (ret < 0) {
				LOG_ERR("Failed to release CAL_EN pin");
				return ret;
			}

			/* Wait >6s before allowing other operations as per datasheet */
			LOG_INF("Waiting 6 seconds before allowing other operations...");
			k_sleep(K_SECONDS(6));

			LOG_INF("Auto-calibration process completed");
		}
		break;

	case MT6835_ATTR_OUTPUT_OPTIONS:
		/* Set output options - configure Z_PHASE_UVW and PWM_NLC registers */
		ret = mt6835_write_register(dev, MT6835_REG_Z_PHASE_UVW, val->val1 & 0xFF);
		if (ret < 0) {
			return ret;
		}
		ret = mt6835_write_register(dev, MT6835_REG_PWM_NLC, val->val2 & 0xFF);
		break;

	case MT6835_ATTR_AUTOCAL_FREQ:
		/* Set auto-calibration rotation speed range - val->val1 should be 0-7 */
		if (val->val1 < 0 || val->val1 > 7) {
			LOG_ERR("Invalid auto-calibration speed range value: %d", val->val1);
			return -EINVAL;
		}

		/* Read current GPIO_AUTOCAL register */
		ret = mt6835_read_register(dev, MT6835_REG_GPIO_AUTOCAL, &reg_val);
		if (ret < 0) {
			return ret;
		}

		/* Update auto-calibration speed range bits (bits 2:0) */
		reg_val = (reg_val & ~MT6835_AUTOCAL_FREQ_MASK) |
			  (val->val1 & MT6835_AUTOCAL_FREQ_MASK);
		ret = mt6835_write_register(dev, MT6835_REG_GPIO_AUTOCAL, reg_val);
		break;

	case MT6835_ATTR_ROTATION_DIRECTION:
		/* Set rotation direction - val->val1 should be 0 (clockwise) or 1
		 * (counter-clockwise) */
		if (val->val1 < 0 || val->val1 > 1) {
			LOG_ERR("Invalid rotation direction value: %d", val->val1);
			return -EINVAL;
		}

		/* Read current ROT_HYST register */
		ret = mt6835_read_register(dev, MT6835_REG_ROT_HYST, &reg_val);
		if (ret < 0) {
			return ret;
		}

		/* Update rotation direction bit (bit 7) */
		if (val->val1 == 1) {
			reg_val |= MT6835_ROT_DIR_MASK; /* Set bit 7 for counter-clockwise */
		} else {
			reg_val &= ~MT6835_ROT_DIR_MASK; /* Clear bit 7 for clockwise */
		}
		ret = mt6835_write_register(dev, MT6835_REG_ROT_HYST, reg_val);
		break;

	case MT6835_ATTR_HYSTERESIS:
		/* Set hysteresis - val->val1 should be 0-7 */
		if (val->val1 < 0 || val->val1 > 7) {
			LOG_ERR("Invalid hysteresis value: %d", val->val1);
			return -EINVAL;
		}

		/* Read current ROT_HYST register */
		ret = mt6835_read_register(dev, MT6835_REG_ROT_HYST, &reg_val);
		if (ret < 0) {
			return ret;
		}

		/* Update hysteresis bits (bits 2:0) */
		reg_val = (reg_val & ~MT6835_HYST_MASK) | (val->val1 & MT6835_HYST_MASK);
		ret = mt6835_write_register(dev, MT6835_REG_ROT_HYST, reg_val);
		break;

	default:
		LOG_ERR("Unsupported sensor attribute: %d", attr);
		return -ENOTSUP;
	}

	return ret;
}

static int mt6835_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val)
{
	int ret = 0;
	uint8_t reg_val;

	switch ((int)attr) {
	case MT6835_ATTR_ABZ_RESOLUTION:
		/* Get ABZ resolution setting */
		ret = mt6835_read_register(dev, MT6835_REG_ABZ_RES1, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & 0x03; /* Resolution bits 0-1 */
		val->val2 = 0;
		break;

	case MT6835_ATTR_ZERO_POSITION:
		/* Get zero position setting */
		ret = mt6835_read_register(dev, MT6835_REG_ZERO_POS_H, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val;

		ret = mt6835_read_register(dev, MT6835_REG_ZERO_POS_L, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val2 = reg_val;
		break;

	case MT6835_ATTR_OUTPUT_OPTIONS:
		/* Get output options */
		ret = mt6835_read_register(dev, MT6835_REG_Z_PHASE_UVW, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val;

		ret = mt6835_read_register(dev, MT6835_REG_PWM_NLC, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val2 = reg_val;
		break;

	case MT6835_ATTR_AUTOCAL_FREQ:
		/* Get auto-calibration rotation speed range setting */
		ret = mt6835_read_register(dev, MT6835_REG_GPIO_AUTOCAL, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & MT6835_AUTOCAL_FREQ_MASK; /* Auto-cal speed range bits 2:0 */
		val->val2 = 0;
		break;

	case MT6835_ATTR_CAL_STATUS:
		/* Get calibration status from register 0x113 */
		ret = mt6835_read_register(dev, MT6835_REG_CAL_STATUS, &reg_val);
		if (ret < 0) {
			return ret;
		}
		/* Return calibration status bits [7:6] shifted to logical values (0-3) */
		val->val1 = (reg_val & MT6835_CAL_STATUS_MASK) >> 6;
		val->val2 = 0;
		break;

	case MT6835_ATTR_ROTATION_DIRECTION:
		/* Get rotation direction setting */
		ret = mt6835_read_register(dev, MT6835_REG_ROT_HYST, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & MT6835_ROT_DIR_MASK) ? 1 : 0; /* Check bit 7 */
		val->val2 = 0;
		break;

	case MT6835_ATTR_HYSTERESIS:
		/* Get hysteresis setting */
		ret = mt6835_read_register(dev, MT6835_REG_ROT_HYST, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & MT6835_HYST_MASK; /* Hysteresis bits 2:0 */
		val->val2 = 0;
		break;

	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		/* MT6835 doesn't have configurable sampling frequency - it's continuous */
		val->val1 = 0;
		val->val2 = 0;
		break;

	default:
		LOG_ERR("Unsupported sensor attribute: %d", attr);
		return -ENOTSUP;
	}

	return ret;
}

static int mt6835_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mt6835_data *dev_data = dev->data;
	int retval;

	/* Read the angle register */
	retval = mt6835_read_angle(dev, &dev_data->position);
	if (retval < 0) {
		LOG_ERR("Failed to read angle register");
		return retval;
	}

	return 0;
}

static int mt6835_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct mt6835_data *dev_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		val->val1 =
			((int32_t)dev_data->position * MT6835_FULL_ANGLE) / MT6835_PULSES_PER_REV;

		val->val2 =
			(((int64_t)dev_data->position * MT6835_FULL_ANGLE * MT6835_MILLION_UNIT) /
			 MT6835_PULSES_PER_REV) %
			MT6835_MILLION_UNIT;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static void mt6835_complete_result(struct rtio *ctx, const struct rtio_sqe *sqe, int res,
				   void *arg0)
{
	ARG_UNUSED(res);

	struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)arg0;
	struct rtio_cqe *cqe;
	int err = 0;

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe != NULL) {
			err = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe != NULL);

	if (err) {
		rtio_iodev_sqe_err(iodev_sqe, err);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, 0);
	}
}

static void mt6835_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct mt6835_data *data = dev->data;
	uint32_t min_buf_len = sizeof(struct mt6835_sample);
	uint64_t cycles;
	int rc;
	uint8_t *buf;
	struct mt6835_sample *sample;

	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, NULL);
	if (rc) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	sample = (struct mt6835_sample *)buf;

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		LOG_ERR("Failed to get sensor clock cycles");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	sample->header.timestamp_ns = sensor_clock_cycles_to_ns(cycles);

	struct rtio *ctx = data->rtio_ctx;
	struct rtio_sqe *txrx_sqe = rtio_sqe_acquire(ctx);
	struct rtio_sqe *complete_sqe = rtio_sqe_acquire(ctx);

	if (!txrx_sqe || !complete_sqe) {
		LOG_ERR("Failed to acquire RTIO SQEs");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	/* Prepare 6-byte TX buffer: command + dummy bytes */
	static uint8_t __aligned(32) tx_buf[] = {
		MT6835_CMD_ANGLE | ((MT6835_REG_ANGLE_H >> 8) & 0x0F),
		MT6835_REG_ANGLE_H & 0xFF,
		0x00,
		0x00,
		0x00,
		0x00};

	rtio_sqe_prep_transceive(txrx_sqe, data->iodev, RTIO_PRIO_HIGH, tx_buf, sample->raw,
				 sizeof(sample->raw), (void *)dev);

	rtio_sqe_prep_callback_no_cqe(complete_sqe, mt6835_complete_result, iodev_sqe, NULL);

	rtio_submit(ctx, 0);
}

void mt6835_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

	if (!cfg->is_streaming) {
		mt6835_submit_one_shot(dev, iodev_sqe);
	} else {
		LOG_ERR("Streaming not supported for MT6835");
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}

static int mt6835_initialize(const struct device *dev)
{
	const struct mt6835_config *config = dev->config;
	struct mt6835_data *const dev_data = dev->data;
	int result;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI dev %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	result = gpio_pin_configure_dt(&config->gpio_cal_en, GPIO_OUTPUT_INACTIVE);
	if (result != 0) {
		LOG_ERR("%s: failed to initialize GPIO for CAL_EN", dev->name);
		return result;
	}

	dev_data->position = 0;

	LOG_INF("Device %s: initialized", dev->name);

	return 0;
}

static DEVICE_API(sensor, mt6835_driver_api) = {
	.sample_fetch = mt6835_sample_fetch,
	.channel_get = mt6835_channel_get,
	.attr_set = mt6835_attr_set,
	.attr_get = mt6835_attr_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = mt6835_submit,
	.get_decoder = mt6835_get_decoder,
#endif
};

#define MT6835_SPI_CFG (SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA)

#define MT6835_RTIO_DEFINE(inst)                                                                   \
	SPI_DT_IODEV_DEFINE(mt6835_spi_iodev_##inst, DT_DRV_INST(inst), MT6835_SPI_CFG);           \
	RTIO_DEFINE(mt6835_rtio_ctx_##inst, 8, 8);

#define MT6835_INIT(inst)                                                                          \
	MT6835_RTIO_DEFINE(inst);                                                                  \
                                                                                                   \
	static struct mt6835_data mt6835_data##inst = {                                            \
		.rtio_ctx = &mt6835_rtio_ctx_##inst,                                               \
		.iodev = &mt6835_spi_iodev_##inst,                                                 \
	};                                                                                         \
	static const struct mt6835_config mt6835_cfg##inst = {                                     \
		.bus = SPI_DT_SPEC_INST_GET(inst, MT6835_SPI_CFG),                                 \
		.gpio_cal_en = GPIO_DT_SPEC_INST_GET(inst, cal_en_gpios),                          \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mt6835_initialize, NULL, &mt6835_data##inst,            \
				     &mt6835_cfg##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
				     &mt6835_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MT6835_INIT)
