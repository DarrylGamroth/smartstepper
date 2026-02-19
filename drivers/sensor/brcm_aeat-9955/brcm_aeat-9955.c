/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "brcm_aeat9955.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

LOG_MODULE_REGISTER(brcm_aeat_9955, CONFIG_SENSOR_LOG_LEVEL);

/* Sensing axis names corresponding to enum indices 0-3 */
static const char *sensing_axis_names[] = {"on-axis", "off-axis-radial", "off-axis-axial",
					   "off-axis-side-shaft"};

struct aeat9955_config {
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_zero;
	const struct gpio_dt_spec gpio_error;
	const struct gpio_dt_spec gpio_sleep;
	uint8_t sensing_axis;
};

struct aeat9955_data {
	struct rtio *rtio_ctx;
	struct rtio_iodev *iodev;
	struct gpio_callback error_cb;
	uint32_t position;
};

static void aeat9955_error_gpio_callback(const struct device *port, struct gpio_callback *cb,
					 gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	LOG_INF("error GPIO callback triggered");
}

static int aeat9955_write_register(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct aeat9955_config *cfg = dev->config;
	const struct spi_dt_spec *bus = &cfg->bus;
	int ret;
	uint8_t tx_buf[2];
	struct spi_buf spi_tx_buf = {.buf = tx_buf, .len = 2};
	struct spi_buf_set tx_set = {.buffers = &spi_tx_buf, .count = 1};

	/* SPI4-16 Write: Two separate 16-bit frames with parity */

	/* First frame: command + register address */
	tx_buf[0] = AEAT9955_CMD_WRITE_SPI16 | ((POPCOUNT(reg) & 1U) << 7);
	tx_buf[1] = reg;
	ret = spi_write_dt(bus, &tx_set);
	if (ret < 0) {
		return ret;
	}

	/* Second frame: value with parity */
	tx_buf[0] = ((POPCOUNT(value) & 1U) << 7);
	tx_buf[1] = value;
	return spi_write_dt(bus, &tx_set);
}

static int aeat9955_read_register(const struct device *dev, uint8_t reg, uint8_t *data)
{
	const struct aeat9955_config *cfg = dev->config;
	const struct spi_dt_spec *bus = &cfg->bus;
	uint8_t tx_buf[2];
	uint8_t rx_buf[2] = {0};
	struct spi_buf spi_tx_buf = {.buf = tx_buf, .len = 2};
	struct spi_buf spi_rx_buf = {.buf = rx_buf, .len = 2};
	struct spi_buf_set tx_set = {.buffers = &spi_tx_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &spi_rx_buf, .count = 1};

	/* SPI4-16 Read is pipelined: write command frame, then clock out response. */
	tx_buf[0] = AEAT9955_CMD_READ_SPI16 | ((~POPCOUNT(reg) & 1U) << 7);
	tx_buf[1] = reg;

	int ret = spi_transceive_dt(bus, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	ret = spi_transceive_dt(bus, &tx_set, &rx_set);
	if (ret == 0) {
		/* Data byte is returned in the second byte for this framing */
		*data = rx_buf[1];
	}

	return ret;
}

/**
 * Perform Level 1 memory unlock
 * This must be called before any write to configuration registers (0x00-0x16)
 * according to AEAT-9955 datasheet EEPROM Unlock requirements
 */
static int aeat9955_unlock_level1(const struct device *dev)
{
	return aeat9955_write_register(dev, AEAT9955_REG_UNLOCK, AEAT9955_UNLOCK_LEVEL1);
}

/**
 * Lock Level 1 memory access
 * This should be called after configuration is complete to prevent accidental changes
 */
static int aeat9955_lock_level1(const struct device *dev)
{
	return aeat9955_write_register(dev, AEAT9955_REG_UNLOCK, 0x00);
}

static int aeat9955_read_angle(const struct device *dev, uint32_t *angle)
{
	const struct aeat9955_config *cfg = dev->config;
	uint8_t tx_buf[3];
	uint8_t rx_buf[3] = {0};
	struct spi_buf spi_tx_buf = {.buf = tx_buf, .len = 3};
	struct spi_buf spi_rx_buf = {.buf = rx_buf, .len = 3};
	struct spi_buf_set tx_set = {.buffers = &spi_tx_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &spi_rx_buf, .count = 1};

	tx_buf[0] = AEAT9955_CMD_READ_SPI16 | ((~POPCOUNT(AEAT9955_REG_POS) & 1U) << 7);
	tx_buf[1] = AEAT9955_REG_POS;
	tx_buf[2] = 0x00;

	int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
	if (ret < 0) {
		return ret;
	}

	bool warning;
	bool parity;
	ret = aeat9955_decode_position(&rx_buf[0], angle, &warning, &parity);

	return ret;
}

static int aeat9955_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct aeat9955_data *data = dev->data;
	int retval;

	/* Read the angle register */
	retval = aeat9955_read_angle(dev, &data->position);
	if (retval < 0) {
		LOG_ERR("Failed to read angle register");
		return retval;
	}

	return 0;
}

static int aeat9955_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct aeat9955_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		val->val1 = ((int64_t)data->position * AEAT9955_FULL_ANGLE) / AEAT9955_MAX_COUNT;

		val->val2 =
			(((int64_t)data->position * AEAT9955_FULL_ANGLE * AEAT9955_MILLION_UNIT) /
			 AEAT9955_MAX_COUNT) %
			AEAT9955_MILLION_UNIT;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int aeat9955_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;
	uint8_t reg_val;

	/* Perform Level 1 unlock before any configuration writes */
	ret = aeat9955_unlock_level1(dev);
	if (ret < 0) {
		LOG_ERR("Failed to unlock Level 1 memory access");
		return ret;
	}

	switch ((int)attr) {
	case AEAT9955_ATTR_ZERO_POSITION:
		/* Set zero position - 18-bit value across 3 registers */
		/* val->val1 = MSB bits[17:10], val->val2 = bits[9:2] */
		ret = aeat9955_write_register(dev, AEAT9955_REG_ZERO_RESET_2, val->val1 & 0xFF);
		if (ret < 0) {
			return ret;
		}
		ret = aeat9955_write_register(dev, AEAT9955_REG_ZERO_RESET_1, val->val2 & 0xFF);
		break;

	case AEAT9955_ATTR_AUTO_CALIBRATION:
		/* Control accuracy angle calibration process
		 * val->val1 = 1: Start calibration (write 0x02 to register 0x12)
		 * val->val1 = 0: Exit calibration mode (write 0x00 to register 0x12)
		 * User must poll AEAT9955_ATTR_CAL_STATUS to monitor progress
		 */
		if (val->val1 == 1) {
			LOG_INF("Starting accuracy calibration (poll AEAT9955_ATTR_CAL_STATUS for "
				"progress)");
			ret = aeat9955_write_register(dev, AEAT9955_REG_CALIBRATION,
						      AEAT9955_CAL_CMD_ACCURACY);
			if (ret < 0) {
				LOG_ERR("Failed to start calibration");
				return ret;
			}
		} else {
			LOG_INF("Exiting accuracy calibration mode");
			ret = aeat9955_write_register(dev, AEAT9955_REG_CALIBRATION,
						      AEAT9955_CAL_CMD_EXIT);
			if (ret < 0) {
				LOG_ERR("Failed to exit calibration mode");
				return ret;
			}
		}
		break;

	case AEAT9955_ATTR_ZERO_RESET:
		/* Control zero reset calibration process
		 * val->val1 = 1: Start zero reset (write 0x08 to register 0x12)
		 * val->val1 = 0: Exit calibration mode (write 0x00 to register 0x12)
		 * User must poll AEAT9955_ATTR_CAL_STATUS to monitor progress
		 */
		if (val->val1 == 1) {
			LOG_INF("Starting zero reset (poll AEAT9955_ATTR_CAL_STATUS for progress)");
			ret = aeat9955_write_register(dev, AEAT9955_REG_CALIBRATION,
						      AEAT9955_CAL_CMD_ZERO_RESET);
			if (ret < 0) {
				LOG_ERR("Failed to start zero reset");
				return ret;
			}
		} else {
			LOG_INF("Exiting zero reset calibration mode");
			ret = aeat9955_write_register(dev, AEAT9955_REG_CALIBRATION,
						      AEAT9955_CAL_CMD_EXIT);
			if (ret < 0) {
				LOG_ERR("Failed to exit calibration mode");
				return ret;
			}
		}
		break;

	case AEAT9955_ATTR_MULTI_INDEX:
		/* Set multi-index pulses per revolution - val->val1 should be 0-7 */
		if (val->val1 < 0 || val->val1 > 7) {
			LOG_ERR("Invalid multi-index value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MULTI_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_MULTI_IDX_MASK) |
			  ((val->val1 << AEAT9955_CONFIG0_MULTI_IDX_SHIFT) &
			   AEAT9955_CONFIG0_MULTI_IDX_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_MULTI_IDX, reg_val);
		break;

	case AEAT9955_ATTR_SENSING_AXIS:
		/* Set sensing axis configuration - val->val1 should be 0-3 */
		if (val->val1 < 0 || val->val1 > 3) {
			LOG_ERR("Invalid sensing axis value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & 0x8F) | ((val->val1 & 0x03) << 4);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_SENSING, reg_val);
		break;

	case AEAT9955_ATTR_INCREMENTAL_RESOLUTION:
		/* Set incremental resolution - 15-bit value for CPR */
		if (val->val1 < 0 || val->val1 > 20000) {
			LOG_ERR("Invalid incremental resolution: %d (max 20000 CPR)", val->val1);
			return -EINVAL;
		}
		/* Write high byte to register 0x0A */
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG1_INCR_RES,
					      (val->val1 >> 8) & 0x7F);
		if (ret < 0) {
			return ret;
		}
		/* Note: Low byte would go to register 0x09 bits[7:0] - need to preserve PSEL bit */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG1_PSEL, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & 0x80) | (val->val1 & 0x7F);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG1_PSEL, reg_val);
		break;

	case AEAT9955_ATTR_DIRECTION:
		/* Set rotation direction - val->val1 should be 0 (clockwise) or 1
		 * (counter-clockwise)
		 */
		if (val->val1 < 0 || val->val1 > 1) {
			LOG_ERR("Invalid direction value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, &reg_val);
		if (ret < 0) {
			return ret;
		}
		if (val->val1 == 1) {
			reg_val |= AEAT9955_CONFIG2_DIR_BIT;
		} else {
			reg_val &= ~AEAT9955_CONFIG2_DIR_BIT;
		}
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, reg_val);
		break;

	case AEAT9955_ATTR_HYSTERESIS:
		/* Set hysteresis - val->val1 should be 0-7 */
		if (val->val1 < 0 || val->val1 > 7) {
			LOG_ERR("Invalid hysteresis value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG2_HYST_MASK) |
			  ((val->val1 << AEAT9955_CONFIG2_HYST_SHIFT) & AEAT9955_CONFIG2_HYST_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, reg_val);
		break;

	case AEAT9955_ATTR_UVW_RESOLUTION:
		/* Set UVW output resolution - val->val1 should be 0-32 */
		if (val->val1 < 0 || val->val1 > 32) {
			LOG_ERR("Invalid UVW resolution value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SPI4_UVW, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & 0xC0) | (val->val1 & 0x3F);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_SPI4_UVW, reg_val);
		break;

	case AEAT9955_ATTR_SINGLE_TURN_RESOLUTION:
		/* Set single turn resolution - val->val1 should be 0-8 */
		if (val->val1 < 0 || val->val1 > 8) {
			LOG_ERR("Invalid single turn resolution value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & 0xF0) | (val->val1 & 0x0F);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, reg_val);
		break;

	case AEAT9955_ATTR_ALARM_LATCH:
		/* Set alarm latch - val->val1 should be 0 or 1 */
		if (val->val1 < 0 || val->val1 > 1) {
			LOG_ERR("Invalid alarm latch value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG, &reg_val);
		if (ret < 0) {
			return ret;
		}
		if (val->val1 == 1) {
			reg_val |= AEAT9955_CONFIG0_ALARM_LATCH;
		} else {
			reg_val &= ~AEAT9955_CONFIG0_ALARM_LATCH;
		}
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_MAG, reg_val);
		break;

	case AEAT9955_ATTR_SPI_HIGHZ:
		/* Set SPI high-impedance mode - val->val1 should be 0 or 1 */
		if (val->val1 < 0 || val->val1 > 1) {
			LOG_ERR("Invalid SPI high-z value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG, &reg_val);
		if (ret < 0) {
			return ret;
		}
		if (val->val1 == 1) {
			reg_val |= AEAT9955_CONFIG0_SPI_HIGHZ;
		} else {
			reg_val &= ~AEAT9955_CONFIG0_SPI_HIGHZ;
		}
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_MAG, reg_val);
		break;

	case AEAT9955_ATTR_MAGNETIC_HIGH:
		/* Set magnetic field high limit - val->val1 should be 0-15 */
		if (val->val1 < 0 || val->val1 > 15) {
			LOG_ERR("Invalid magnetic high value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG_FIELD, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_MAG_HIGH_MASK) |
			  ((val->val1 << AEAT9955_CONFIG0_MAG_HIGH_SHIFT) &
			   AEAT9955_CONFIG0_MAG_HIGH_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_MAG_FIELD, reg_val);
		break;

	case AEAT9955_ATTR_MAGNETIC_LOW:
		/* Set magnetic field low limit - val->val1 should be 0-15 */
		if (val->val1 < 0 || val->val1 > 15) {
			LOG_ERR("Invalid magnetic low value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG_FIELD, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_MAG_LOW_MASK) |
			  (val->val1 & AEAT9955_CONFIG0_MAG_LOW_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_MAG_FIELD, reg_val);
		break;

	case AEAT9955_ATTR_VERTICAL_HALL_SEL:
		/* Set vertical hall selection - val->val1 should be 0-15 */
		if (val->val1 < 0 || val->val1 > 15) {
			LOG_ERR("Invalid vertical hall selection value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_VH_SEL_MASK) |
			  (val->val1 & AEAT9955_CONFIG0_VH_SEL_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_SENSING, reg_val);
		break;

	case AEAT9955_ATTR_PWM_RESOLUTION:
		/* Set PWM resolution - val->val1 should be 0-15 */
		if (val->val1 < 0 || val->val1 > 15) {
			LOG_ERR("Invalid PWM resolution value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_PWM_RES_MASK) |
			  ((val->val1 << AEAT9955_CONFIG0_PWM_RES_SHIFT) &
			   AEAT9955_CONFIG0_PWM_RES_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, reg_val);
		break;

	case AEAT9955_ATTR_INDEX_STATE:
		/* Set index state - val->val1 should be 0-3 */
		if (val->val1 < 0 || val->val1 > 3) {
			LOG_ERR("Invalid index state value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_IDX_STATE_MASK) |
			  ((val->val1 << AEAT9955_CONFIG0_IDX_STATE_SHIFT) &
			   AEAT9955_CONFIG0_IDX_STATE_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, reg_val);
		break;

	case AEAT9955_ATTR_INDEX_WIDTH:
		/* Set index width - val->val1 should be 0-3 */
		if (val->val1 < 0 || val->val1 > 3) {
			LOG_ERR("Invalid index width value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		reg_val = (reg_val & ~AEAT9955_CONFIG0_IDX_WIDTH_MASK) |
			  (val->val1 & AEAT9955_CONFIG0_IDX_WIDTH_MASK);
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, reg_val);
		break;

	case AEAT9955_ATTR_PROTOCOL_MODE:
		/* Set protocol mode PSEL - val->val1 should be 0 or 1 */
		if (val->val1 < 0 || val->val1 > 1) {
			LOG_ERR("Invalid protocol mode value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG1_PSEL, &reg_val);
		if (ret < 0) {
			return ret;
		}
		if (val->val1 == 1) {
			reg_val |= AEAT9955_CONFIG1_PSEL_BIT;
		} else {
			reg_val &= ~AEAT9955_CONFIG1_PSEL_BIT;
		}
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG1_PSEL, reg_val);
		break;

	case AEAT9955_ATTR_AUTO_CAL_HARDWARE:
		/* Set auto-calibration hardware enable - val->val1 should be 0 or 1 */
		if (val->val1 < 0 || val->val1 > 1) {
			LOG_ERR("Invalid auto-cal hardware value: %d", val->val1);
			return -EINVAL;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
		if (ret < 0) {
			return ret;
		}
		if (val->val1 == 1) {
			reg_val |= AEAT9955_CONFIG0_AUTO_CAL_HW;
		} else {
			reg_val &= ~AEAT9955_CONFIG0_AUTO_CAL_HW;
		}
		ret = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_SENSING, reg_val);
		break;

	case AEAT9955_ATTR_EEPROM_PROGRAM:
		/* Program current configuration to EEPROM */
		if (val->val1 == 1) {
			LOG_INF("Programming configuration to EEPROM");

			/* Note: Level 1 unlock already performed at function entry */
			/* Step 1: Write default passcode (all zeros) for Level 2 unlock */
			for (int i = 0; i < 7; i++) {
				ret = aeat9955_write_register(dev, AEAT9955_REG_PASSCODE_0 + i,
							      0x00);
				if (ret < 0) {
					LOG_ERR("Failed to write passcode byte %d", i);
					return ret;
				}
			}

			/* Step 3: Select configuration page */
			ret = aeat9955_write_register(dev, AEAT9955_REG_EEPROM_PAGE,
						      AEAT9955_EEPROM_PAGE_CONFIG);
			if (ret < 0) {
				LOG_ERR("Failed to select EEPROM page");
				return ret;
			}

			/* Step 4: Trigger EEPROM programming */
			ret = aeat9955_write_register(dev, AEAT9955_REG_EEPROM_PROGRAM,
						      AEAT9955_EEPROM_PROGRAM_CMD);
			if (ret < 0) {
				LOG_ERR("Failed to trigger EEPROM programming");
				return ret;
			}

			/* Step 5: Wait for programming to complete */
			int timeout_count = 0;
			const int max_timeout = 1000; /* 10 second timeout */

			while (timeout_count < max_timeout) {
				k_sleep(K_MSEC(10));

				ret = aeat9955_read_register(dev, AEAT9955_REG_STATUS, &reg_val);
				if (ret < 0) {
					LOG_ERR("Failed to read EEPROM busy status");
					return ret;
				}

				/* Bit 7 (Memory Busy) indicates EEPROM programming status */
				if ((reg_val & AEAT9955_STATUS_MEM_BUSY_BIT) == 0) {
					LOG_INF("EEPROM programming completed");
					break;
				}

				timeout_count++;
			}

			if (timeout_count >= max_timeout) {
				LOG_ERR("EEPROM programming timeout");
				return -ETIMEDOUT;
			}
		}
		break;

	default:
		LOG_ERR("Unsupported sensor attribute: %d", attr);
		return -ENOTSUP;
	}

	return ret;
}

static int aeat9955_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	int ret = 0;
	uint8_t reg_val, reg_val2;

	switch ((int)attr) {
	case AEAT9955_ATTR_ZERO_POSITION:
		/* Get zero position setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_ZERO_RESET_2, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val;

		ret = aeat9955_read_register(dev, AEAT9955_REG_ZERO_RESET_1, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val2 = reg_val;
		break;

	case AEAT9955_ATTR_CAL_STATUS:
		/* Get calibration status from register 0x2A */
		ret = aeat9955_read_register(dev, AEAT9955_REG_STATUS, &reg_val);
		if (ret < 0) {
			return ret;
		}
		/* val1 = auto calibration status [1:0] */
		val->val1 = reg_val & AEAT9955_STATUS_CAL_MASK;
		/* val2 = zero reset status [3:2] */
		val->val2 = (reg_val & AEAT9955_STATUS_ZERO_MASK) >> 2;
		break;

	case AEAT9955_ATTR_MULTI_INDEX:
		/* Get multi-index setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MULTI_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG0_MULTI_IDX_MASK) >>
			    AEAT9955_CONFIG0_MULTI_IDX_SHIFT;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_SENSING_AXIS:
		/* Get sensing axis configuration */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val >> 4) & 0x03;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_INCREMENTAL_RESOLUTION:
		/* Get incremental resolution */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG1_INCR_RES, &reg_val);
		if (ret < 0) {
			return ret;
		}
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG1_PSEL, &reg_val2);
		if (ret < 0) {
			return ret;
		}
		val->val1 = ((reg_val & 0x7F) << 8) | (reg_val2 & 0x7F);
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_DIRECTION:
		/* Get rotation direction */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG2_DIR_BIT) ? 1 : 0;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_HYSTERESIS:
		/* Get hysteresis setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG2_HYST_MASK) >> AEAT9955_CONFIG2_HYST_SHIFT;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_UVW_RESOLUTION:
		/* Get UVW output resolution */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SPI4_UVW, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & 0x3F;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_SINGLE_TURN_RESOLUTION:
		/* Get single turn resolution */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG2_HYST_DIR, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & 0x0F;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_ALARM_LATCH:
		/* Get alarm latch setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG0_ALARM_LATCH) ? 1 : 0;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_SPI_HIGHZ:
		/* Get SPI high-impedance mode setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG0_SPI_HIGHZ) ? 1 : 0;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_MAGNETIC_HIGH:
		/* Get magnetic field high limit */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG_FIELD, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG0_MAG_HIGH_MASK) >>
			    AEAT9955_CONFIG0_MAG_HIGH_SHIFT;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_MAGNETIC_LOW:
		/* Get magnetic field low limit */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_MAG_FIELD, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & AEAT9955_CONFIG0_MAG_LOW_MASK;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_VERTICAL_HALL_SEL:
		/* Get vertical hall selection */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & AEAT9955_CONFIG0_VH_SEL_MASK;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_PWM_RESOLUTION:
		/* Get PWM resolution setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 =
			(reg_val & AEAT9955_CONFIG0_PWM_RES_MASK) >> AEAT9955_CONFIG0_PWM_RES_SHIFT;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_INDEX_STATE:
		/* Get index state setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG0_IDX_STATE_MASK) >>
			    AEAT9955_CONFIG0_IDX_STATE_SHIFT;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_INDEX_WIDTH:
		/* Get index width setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_PWM_IDX, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = reg_val & AEAT9955_CONFIG0_IDX_WIDTH_MASK;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_PROTOCOL_MODE:
		/* Get protocol mode PSEL setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG1_PSEL, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG1_PSEL_BIT) ? 1 : 0;
		val->val2 = 0;
		break;

	case AEAT9955_ATTR_AUTO_CAL_HARDWARE:
		/* Get auto-calibration hardware enable setting */
		ret = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
		if (ret < 0) {
			return ret;
		}
		val->val1 = (reg_val & AEAT9955_CONFIG0_AUTO_CAL_HW) ? 1 : 0;
		val->val2 = 0;
		break;

	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		/* AEAT9955 doesn't have configurable sampling frequency - it's continuous */
		val->val1 = 0;
		val->val2 = 0;
		break;

	default:
		LOG_ERR("Unsupported sensor attribute: %d", attr);
		return -ENOTSUP;
	}

	return ret;
}

static void aeat9955_complete_result(struct rtio *ctx, const struct rtio_sqe *sqe, int res,
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

static void aeat9955_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct aeat9955_data *data = dev->data;
	uint32_t min_buf_len = sizeof(struct aeat9955_sample);
	uint64_t cycles;
	int rc;
	uint8_t *buf;
	uint32_t buf_len;
	struct aeat9955_sample *sample;

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		LOG_ERR("Failed to get sensor clock cycles");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	struct rtio_sqe *sqe = (struct rtio_sqe *)&iodev_sqe->sqe;

	if (sqe->rx.buf != NULL && sqe->rx.buf_len >= min_buf_len) {
		buf = sqe->rx.buf;
		buf_len = sqe->rx.buf_len;
		/* caller provided storage */
	} else {
		rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
		if (rc) {
			LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
			rtio_iodev_sqe_err(iodev_sqe, rc);
			return;
		}
	}

	sample = (struct aeat9955_sample *)buf;

	sample->header.timestamp_ns = sensor_clock_cycles_to_ns(cycles);

	struct rtio *ctx = data->rtio_ctx;
	struct rtio_sqe *txrx_sqe = rtio_sqe_acquire(ctx);
	struct rtio_sqe *complete_sqe = rtio_sqe_acquire(ctx);

	if (!txrx_sqe || !complete_sqe) {
		LOG_ERR("Failed to acquire RTIO SQEs");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	static uint8_t __aligned(32) tx_buf[] = {
		AEAT9955_CMD_READ_SPI16 | (1U << 7),
		AEAT9955_REG_POS,
		0x00,
	};

	rtio_sqe_prep_transceive(txrx_sqe, data->iodev, RTIO_PRIO_HIGH, tx_buf, sample->raw,
				 sizeof(sample->raw), NULL);

	rtio_sqe_prep_callback_no_cqe(complete_sqe, aeat9955_complete_result, iodev_sqe, NULL);

	rtio_submit(ctx, 0);
}

void aeat9955_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

	if (!cfg->is_streaming) {
		aeat9955_submit_one_shot(dev, iodev_sqe);
	} else {
		LOG_ERR("Streaming not supported for AEAT-9955");
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}

static int aeat9955_initialize(const struct device *dev)
{
	const struct aeat9955_config *config = dev->config;
	struct aeat9955_data *const data = dev->data;
	int result;
	uint8_t reg_val;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI dev %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	/* Step 1: Unlock Level 1 memory access */
	result = aeat9955_unlock_level1(dev);
	if (result < 0) {
		LOG_ERR("Failed to unlock Level 1 memory access");
		return result;
	}

	result = aeat9955_read_register(dev, AEAT9955_REG_CONFIG0_SENSING, &reg_val);
	if (result < 0) {
		LOG_ERR("Failed to read CONFIG0_SENSING register");
		return result;
	}

	reg_val = (reg_val & ~AEAT9955_CONFIG0_SENSING_MASK) |
		  ((config->sensing_axis << AEAT9955_CONFIG0_SENSING_SHIFT) &
		   AEAT9955_CONFIG0_SENSING_MASK);

	result = aeat9955_write_register(dev, AEAT9955_REG_CONFIG0_SENSING, reg_val);
	if (result < 0) {
		LOG_ERR("Failed to write CONFIG0_SENSING register");
		return result;
	}
	LOG_INF("Configured sensing axis: %s", sensing_axis_names[config->sensing_axis]);

	/* Configure ZERO GPIO if defined */
	if (config->gpio_zero.port != NULL) {
		result = gpio_pin_configure_dt(&config->gpio_zero, GPIO_OUTPUT_INACTIVE);
		if (result != 0) {
			LOG_ERR("%s: failed to initialize GPIO for ZERO", dev->name);
			return result;
		}
	}

	/* Configure SLEEP GPIO if defined */
	if (config->gpio_sleep.port != NULL) {
		result = gpio_pin_configure_dt(&config->gpio_sleep, GPIO_OUTPUT_INACTIVE);
		if (result != 0) {
			LOG_ERR("%s: failed to initialize GPIO for SLEEP", dev->name);
			return result;
		}
	}

	/* Configure ERROR GPIO if defined */
	if (config->gpio_error.port != NULL) {
		if (!gpio_is_ready_dt(&config->gpio_error)) {
			LOG_ERR("%s: ERROR GPIO port not ready", dev->name);
			return -ENODEV;
		}

		result = gpio_pin_configure_dt(&config->gpio_error, GPIO_INPUT);
		if (result != 0) {
			LOG_ERR("%s: failed to initialize GPIO for ERROR", dev->name);
			return result;
		}

		gpio_init_callback(&data->error_cb, aeat9955_error_gpio_callback,
				   BIT(config->gpio_error.pin));
		result = gpio_add_callback(config->gpio_error.port, &data->error_cb);
		if (result != 0) {
			LOG_ERR("%s: failed to add ERROR GPIO callback", dev->name);
			return result;
		}

		result = gpio_pin_interrupt_configure_dt(&config->gpio_error,
							 GPIO_INT_EDGE_TO_ACTIVE);
		if (result != 0) {
			LOG_ERR("%s: failed to configure ERROR GPIO interrupt", dev->name);
			return result;
		}
	}

	/* Lock Level 1 memory access after initialization */
	result = aeat9955_lock_level1(dev);
	if (result < 0) {
		LOG_WRN("Failed to lock Level 1 memory access");
	}

	data->position = 0;

	LOG_INF("Device %s: initialized", dev->name);

	return 0;
}

static DEVICE_API(sensor, aeat9955_driver_api) = {
	.sample_fetch = aeat9955_sample_fetch,
	.channel_get = aeat9955_channel_get,
	.attr_set = aeat9955_attr_set,
	.attr_get = aeat9955_attr_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = aeat9955_submit,
	.get_decoder = aeat9955_get_decoder,
#endif
};

#define AEAT9955_SPI_CFG (SPI_WORD_SET(8) | SPI_MODE_CPHA)

#define AEAT9955_RTIO_DEFINE(inst)                                                                 \
	SPI_DT_IODEV_DEFINE(aeat9955_spi_iodev_##inst, DT_DRV_INST(inst), AEAT9955_SPI_CFG);       \
	RTIO_DEFINE(aeat9955_rtio_ctx_##inst, 8, 8);

#define AEAT9955_INIT(inst)                                                                        \
	AEAT9955_RTIO_DEFINE(inst);                                                                \
                                                                                                   \
	static struct aeat9955_data aeat9955_data##inst = {                                        \
		.rtio_ctx = &aeat9955_rtio_ctx_##inst,                                             \
		.iodev = &aeat9955_spi_iodev_##inst,                                               \
	};                                                                                         \
	static const struct aeat9955_config aeat9955_cfg##inst = {                                 \
		.bus = SPI_DT_SPEC_INST_GET(inst, AEAT9955_SPI_CFG),                               \
		.gpio_zero = GPIO_DT_SPEC_INST_GET_OR(inst, zero_gpios, {0}),                      \
		.gpio_error = GPIO_DT_SPEC_INST_GET_OR(inst, error_gpios, {0}),                    \
		.gpio_sleep = GPIO_DT_SPEC_INST_GET_OR(inst, sleep_gpios, {0}),                      \
		.sensing_axis = DT_INST_ENUM_IDX_OR(inst, sensing_axis, 0),                        \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, aeat9955_initialize, NULL, &aeat9955_data##inst,        \
				     &aeat9955_cfg##inst, POST_KERNEL,                             \
				     CONFIG_SENSOR_INIT_PRIORITY, &aeat9955_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AEAT9955_INIT)
