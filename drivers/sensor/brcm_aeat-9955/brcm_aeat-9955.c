/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_aeat_9955

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
LOG_MODULE_REGISTER(brcm_aeat_9955, CONFIG_SENSOR_LOG_LEVEL);

#define AEAT9955_CMD_READ_BIT 0x40U
#define AEAT9955_CMD_WRITE_BIT 0x00U
#define AEAT9955_CMD_PARITY_BIT 0x80U
#define AEAT9955_CMD_ERROR_BIT 0x40U

#define AEAT9955_CMD_READ (0x03U << 4)
#define AEAT9955_CMD_WRITE (0x06U << 4)
#define AEAT9955_CMD_PROG (0x0BU << 4)
#define AEAT9955_CMD_ZERO (0x05U << 4)
#define AEAT9955_CMD_ANGLE (0x0AU << 4)

#define AEAT9955_REG_POS 0x3FU
#define AEAT9955_REG_USER_ID 0x00U
#define AEAT9955_REG_ANGLE_H 0x03U
#define AEAT9955_REG_ANGLE_M 0x04U
#define AEAT9955_REG_ANGLE_L 0x05U
#define AEAT9955_REG_CRC 0x06U
#define AEAT9955_REG_ABZ_RES1 0x07U
#define AEAT9955_REG_ABZ_RES2 0x08U
#define AEAT9955_REG_ZERO1 0x09U
#define AEAT9955_REG_ZERO2 0x0AU
#define AEAT9955_REG_OPTS0 0x0AU
#define AEAT9955_REG_OPTS1 0x0BU
#define AEAT9955_REG_OPTS2 0x0CU
#define AEAT9955_REG_OPTS3 0x0DU
#define AEAT9955_REG_OPTS4 0x0EU
#define AEAT9955_REG_OPTS5 0x11U

#define AEAT9955_FULL_ANGLE 360
#define AEAT9955_PULSES_PER_REV 262144
#define AEAT9955_MILLION_UNIT 1000000

struct aeat9955_dev_config
{
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_zero;
	const struct gpio_dt_spec gpio_error;
};

/* Device run time data */
struct aeat9955_dev_data
{
	uint32_t position;
};

inline static int aeat9955_parity(uint32_t value)
{
	return (POPCOUNT(value) & 1) << 7;
}

static int aeat9955_read_register(const struct device *dev, uint8_t reg, uint8_t *data)
{
	const struct aeat9955_dev_config *dev_cfg = dev->config;
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];
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


	tx_buf[0] = AEAT9955_CMD_READ | aeat9955_parity(reg);
	tx_buf[1] = reg;
	tx_buf[2] = 0;
	tx_buf[3] = 0;

	int ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret >= 0)
	{
		*data = rx_buf[3];
	}

	return ret;
}

static int aeat9955_write_register(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct aeat9955_dev_config *dev_cfg = dev->config;
	uint8_t buf[4];
	struct spi_buf tx_buf = {
		.buf = buf,
		.len = sizeof(buf),
	};
	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	buf[0] = AEAT9955_CMD_WRITE | aeat9955_parity(reg);
	buf[1] = reg;
	buf[2] = aeat9955_parity(value);
	buf[3] = value;

	return spi_write_dt(&dev_cfg->bus, &tx);
}

static int aeat9955_read_angle(const struct device *dev, uint32_t *angle)
{
	const struct aeat9955_dev_config *dev_cfg = dev->config;
	uint8_t tx_buf[5];
	uint8_t rx_buf[5];
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
	
	tx_buf[0] = AEAT9955_CMD_READ_BIT;
	tx_buf[1] = AEAT9955_REG_POS;
	tx_buf[2] = 0;
	tx_buf[3] = 0;
	tx_buf[4] = 0;

	int ret = spi_transceive_dt(&dev_cfg->bus, &tx, &rx);
	if (ret == 0) {
		// Read the error and parity bits
		if (rx_buf[2] & AEAT9955_CMD_ERROR_BIT) {
			LOG_ERR("AEAT-9955 error bit set");
			return -EIO; // Error reading angle
		}
		if (rx_buf[2] & AEAT9955_CMD_PARITY_BIT) {
			LOG_ERR("AEAT-9955 parity error");
			return -EIO; // Parity error
		}
		// AEAT-9955 returns 18-bit position data in bytes 2-4
		// Use sys_get_be24 to extract and shift right by 6 to get 18-bit value
		*angle = sys_get_be24(&rx_buf[2]) >> 6;
	} else {
		*angle = 0;
	}

	return ret;
}

static int aeat9955_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct aeat9955_dev_data *dev_data = dev->data;
	int retval;

	/* Read the angle register */
	retval = aeat9955_read_angle(dev, &dev_data->position);
	if (retval < 0)
	{
		LOG_ERR("Failed to read angle register");
		return retval;
	}

	return 0;
}

static int aeat9955_channel_get(const struct device *dev, enum sensor_channel chan,
								struct sensor_value *val)
{
	struct aeat9955_dev_data *dev_data = dev->data;

	switch (chan)
	{
	case SENSOR_CHAN_ROTATION:
		val->val1 = ((int64_t)dev_data->position * AEAT9955_FULL_ANGLE) /
					AEAT9955_PULSES_PER_REV;

		val->val2 = (((int64_t)dev_data->position * AEAT9955_FULL_ANGLE * AEAT9955_MILLION_UNIT) /
					AEAT9955_PULSES_PER_REV) % AEAT9955_MILLION_UNIT;
		break;
	case SENSOR_CHAN_RPM:
		val->val1 = (int32_t)dev_data->position;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int aeat9955_initialize(const struct device *dev)
{
	const struct aeat9955_dev_config *config = dev->config;
	struct aeat9955_dev_data *const dev_data = dev->data;
	int result;

	if (!spi_is_ready_dt(&config->bus))
	{
		LOG_ERR("SPI dev %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	/* Configure ZERO GPIO if defined */
	if (config->gpio_zero.port != NULL) {
		result = gpio_pin_configure_dt(&config->gpio_zero, GPIO_OUTPUT_INACTIVE);
		if (result != 0) {
			LOG_ERR("%s: failed to initialize GPIO for ZERO", dev->name);
			return result;
		}
	}

	/* Configure ERROR GPIO if defined */
	if (config->gpio_error.port != NULL) {
		result = gpio_pin_configure_dt(&config->gpio_error, GPIO_INPUT);
		if (result != 0) {
			LOG_ERR("%s: failed to initialize GPIO for ERROR", dev->name);
			return result;
		}
	}

	dev_data->position = 0;

	LOG_INF("Device %s: initialized", dev->name);

	return 0;
}

static DEVICE_API(sensor, aeat9955_driver_api) = {
	.sample_fetch = aeat9955_sample_fetch,
	.channel_get = aeat9955_channel_get,
};

#define AEAT9955_INIT(inst)                                                     \
	static struct aeat9955_dev_data aeat9955_data##inst;                    \
	static const struct aeat9955_dev_config aeat9955_cfg##inst = {          \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                   \
									SPI_WORD_SET(8) | SPI_MODE_CPHA,       \
									0),                                     \
		.gpio_zero = GPIO_DT_SPEC_INST_GET_OR(inst, zero_gpios, {0}),       \
		.gpio_error = GPIO_DT_SPEC_INST_GET_OR(inst, error_gpios, {0}),     \
	};                                                                      \
                                                                            \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, aeat9955_initialize, NULL,           \
								 &aeat9955_data##inst, &aeat9955_cfg##inst, \
								 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  \
								 &aeat9955_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AEAT9955_INIT)
