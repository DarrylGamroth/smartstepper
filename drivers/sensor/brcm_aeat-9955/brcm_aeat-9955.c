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

struct aeat9955_config {
	struct spi_dt_spec bus;
	const struct gpio_dt_spec gpio_zero;
	const struct gpio_dt_spec gpio_error;
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

inline static int aeat9955_parity(uint32_t value)
{
	return (POPCOUNT(value) & 1) << 7;
}

static int __unused aeat9955_read_register(const struct device *dev, uint8_t reg, uint8_t *data)
{
	const struct aeat9955_config *cfg = dev->config;
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

	int ret = spi_transceive_dt(&cfg->bus, &tx, &rx);
	if (ret >= 0) {
		*data = rx_buf[3];
	}

	return ret;
}

static int __unused aeat9955_write_register(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct aeat9955_config *cfg = dev->config;
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

	return spi_write_dt(&cfg->bus, &tx);
}

static int aeat9955_read_angle(const struct device *dev, uint32_t *angle)
{
	const struct aeat9955_config *cfg = dev->config;
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

	int ret = spi_transceive_dt(&cfg->bus, &tx, &rx);
	if (ret == 0) {
		if (rx_buf[2] & AEAT9955_STATUS_ERROR_BIT) {
			LOG_ERR("AEAT-9955 error bit set");
			return -EIO;
		}
		if (rx_buf[2] & AEAT9955_STATUS_PARITY_BIT) {
			LOG_ERR("AEAT-9955 parity error");
			return -EIO;
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
		val->val1 =
			((int64_t)data->position * AEAT9955_FULL_ANGLE) / AEAT9955_PULSES_PER_REV;

		val->val2 =
			(((int64_t)data->position * AEAT9955_FULL_ANGLE * AEAT9955_MILLION_UNIT) /
			 AEAT9955_PULSES_PER_REV) %
			AEAT9955_MILLION_UNIT;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
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

	static uint8_t __aligned(32) tx_buf[] = {AEAT9955_CMD_READ_BIT, AEAT9955_REG_POS,
						  0x00, 0x00, 0x00};

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

	if (!spi_is_ready_dt(&config->bus)) {
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

	data->position = 0;

	LOG_INF("Device %s: initialized", dev->name);

	return 0;
}

static DEVICE_API(sensor, aeat9955_driver_api) = {
	.sample_fetch = aeat9955_sample_fetch,
	.channel_get = aeat9955_channel_get,
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
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, aeat9955_initialize, NULL, &aeat9955_data##inst,        \
				     &aeat9955_cfg##inst, POST_KERNEL,                             \
				     CONFIG_SENSOR_INIT_PRIORITY, &aeat9955_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AEAT9955_INIT)
