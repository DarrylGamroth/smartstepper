/*
 * Copyright (c) 2025 Darryl Gamroth
 * Based on MAX32 SPI driver pattern with STM32-specific features
 * Clean RTIO-first implementation for STM32 SPI
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_spi

#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>

#ifdef CONFIG_SPI_STM32_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/cache.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#include <stm32_cache.h>
#endif

#include <soc.h>
#include <stm32_ll_spi.h>

LOG_MODULE_REGISTER(spi_ll_stm32, CONFIG_SPI_LOG_LEVEL);

/* STM32-specific compatibility includes for device tree variants */
#include "spi_ll_stm32.h"

/* STM32 SPI error mask - depends on SoC family */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_UDR | LL_SPI_SR_CRCE | LL_SPI_SR_MODF | \
			   LL_SPI_SR_OVR | LL_SPI_SR_TIFRE)
#else
#if defined(LL_SPI_SR_UDR)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_UDR | LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | \
			   LL_SPI_SR_OVR | LL_SPI_SR_FRE)
#elif defined(SPI_SR_FRE)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | \
			   LL_SPI_SR_OVR | LL_SPI_SR_FRE)
#else
#define SPI_STM32_ERR_MSK (LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | LL_SPI_SR_OVR)
#endif
#endif

/* Forward declarations */
static void spi_stm32_iodev_complete(const struct device *dev, int status);
static int spi_stm32_configure(const struct device *dev, const struct spi_config *config);

static inline bool spi_stm32_transmit_buffer(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);
	
	if (data->tx_buf == NULL) {
		/* RX-only: send dummy data if needed */
		size_t tx_target = data->rx_len;
		if (data->tx_count < tx_target) {
			if (word_size == 16) {
				LL_SPI_TransmitData16(spi, 0xFFFF);
				data->tx_count += 2;
			} else {
				LL_SPI_TransmitData8(spi, 0xFF);
				data->tx_count++;
			}
		}
		return data->tx_count >= tx_target;
	} else {
		/* TX operation: send real data */
		if (data->tx_count < data->tx_len) {
			if (word_size == 16) {
				uint16_t tx_data = ((uint16_t *)data->tx_buf)[data->tx_count / 2];
				LL_SPI_TransmitData16(spi, tx_data);
				data->tx_count += 2;
			} else {
				LL_SPI_TransmitData8(spi, data->tx_buf[data->tx_count]);
				data->tx_count++;
			}
		}
		return data->tx_count >= data->tx_len;
	}
}

static inline bool spi_stm32_receive_buffer(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);
	
	if (data->rx_buf != NULL && data->rx_count < data->rx_len) {
		/* Store received data */
		if (word_size == 16) {
			uint16_t rx_data = LL_SPI_ReceiveData16(spi);
			((uint16_t *)data->rx_buf)[data->rx_count / 2] = rx_data;
			data->rx_count += 2;
		} else {
			uint8_t rx_data = LL_SPI_ReceiveData8(spi);
			data->rx_buf[data->rx_count] = rx_data;
			data->rx_count++;
		}
	} else {
		/* Discard received data (TX-only or buffer full) */
		if (word_size == 16) {
			(void)LL_SPI_ReceiveData16(spi);
		} else {
			(void)LL_SPI_ReceiveData8(spi);
		}
	}
	
	/* For TX-only operations, RX is always "complete" since we just discard data */
	if (data->rx_buf == NULL) {
		return true; /* TX-only: RX is always complete */
	}
	
	/* For RX operations, check if we've received all expected data */
	return data->rx_count >= data->rx_len;
}

static inline void spi_stm32_discard_rx_data(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);
	
	while (ll_func_rx_is_not_empty(spi)) {
		if (word_size == 16) {
			(void)LL_SPI_ReceiveData16(spi);
		} else {
			(void)LL_SPI_ReceiveData8(spi);
		}
	}
}

/* CS control function - matches upstream driver pattern */
static void spi_stm32_cs_control(const struct device *dev, bool on)
{
	struct spi_stm32_data *data = dev->data;

	spi_context_cs_control(&data->ctx, on);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
	const struct spi_stm32_config *cfg = dev->config;

	if (cfg->use_subghzspi_nss) {
		if (on) {
			LL_PWR_SelectSUBGHZSPI_NSS();
		} else {
			LL_PWR_UnselectSUBGHZSPI_NSS();
		}
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz) */
}

static int spi_stm32_wait_transfer_complete(SPI_TypeDef *spi)
{
	uint32_t timeout = 100000;
	
	/* Wait for transfer to complete */
	while (ll_func_transfer_ongoing(spi) && timeout--) {
		/* Small delay to avoid busy polling */
		k_busy_wait(1);
	}
	
	if (timeout == 0) {
		LOG_ERR("SPI transfer timeout - busy flag stuck");
		return -ETIMEDOUT;
	}
	
	return 0;
}

/* Power management helpers */
static void spi_stm32_pm_policy_state_lock_get(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_PM)) {
		struct spi_stm32_data *data = dev->data;

		if (!data->pm_policy_state_on) {
			data->pm_policy_state_on = true;
			pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
			if (IS_ENABLED(CONFIG_PM_S2RAM)) {
				pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
			}
			pm_device_runtime_get(dev);
		}
	}
}

static void spi_stm32_pm_policy_state_lock_put(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_PM)) {
		struct spi_stm32_data *data = dev->data;

		if (data->pm_policy_state_on) {
			data->pm_policy_state_on = false;
			pm_device_runtime_put(dev);
			pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
			if (IS_ENABLED(CONFIG_PM_S2RAM)) {
				pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
			}
		}
	}
}

static int spi_stm32_clock_enable(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int ret;

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enable primary SPI clock */
	ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret < 0) {
		LOG_ERR("Could not enable SPI clock");
		return ret;
	}

	/* Configure domain clock if available (STM32H7, MP1, U5, etc.) */
	if (IS_ENABLED(STM32_SPI_DOMAIN_CLOCK_SUPPORT) && (cfg->pclk_len > 1)) {
		ret = clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[1], NULL);
		if (ret < 0) {
			LOG_ERR("Could not select SPI domain clock");
			return ret;
		}
	}

	return 0;
}

static int spi_stm32_clock_disable(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int ret;

	/* Disable primary SPI clock */
	ret = clock_control_off(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret < 0) {
		LOG_ERR("Could not disable SPI clock");
		return ret;
	}

	return 0;
}

static int spi_stm32_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	const uint32_t scaler[] = {
		LL_SPI_BAUDRATEPRESCALER_DIV2,
		LL_SPI_BAUDRATEPRESCALER_DIV4,
		LL_SPI_BAUDRATEPRESCALER_DIV8,
		LL_SPI_BAUDRATEPRESCALER_DIV16,
		LL_SPI_BAUDRATEPRESCALER_DIV32,
		LL_SPI_BAUDRATEPRESCALER_DIV64,
		LL_SPI_BAUDRATEPRESCALER_DIV128,
		LL_SPI_BAUDRATEPRESCALER_DIV256
	};	
	SPI_TypeDef *spi = cfg->spi;
	LL_SPI_InitTypeDef spi_init;
	int ret;

	if (spi_context_configured(&data->ctx, config)) {
		/* This configuration is already in use */
		return 0;
	}

	LL_SPI_StructInit(&spi_init);
	
	if (SPI_OP_MODE_GET(config->operation) & SPI_OP_MODE_SLAVE) {
		spi_init.Mode = LL_SPI_MODE_SLAVE;
		
		/* In slave mode, clock frequency is determined by master */
		/* Use a default prescaler value */
		spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	} else {
		spi_init.Mode = LL_SPI_MODE_MASTER;
		
		/* Master mode: calculate prescaler for desired frequency */
		/* Get SPI clock source */
		uint32_t clock;
		int br;
		
		if (IS_ENABLED(STM32_SPI_DOMAIN_CLOCK_SUPPORT) && (cfg->pclk_len > 1)) {
			if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
						   (clock_control_subsys_t) &cfg->pclken[1], &clock) < 0) {
				LOG_ERR("Failed call clock_control_get_rate(pclk[1])");
				return -EIO;
			}
		} else {
			if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
						   (clock_control_subsys_t) &cfg->pclken[0], &clock) < 0) {
				LOG_ERR("Failed call clock_control_get_rate(pclk[0])");
				return -EIO;
			}
		}	

		/* Configure baudrate */
		for (br = 1 ; br <= ARRAY_SIZE(scaler) ; ++br) {
			uint32_t clk = clock >> br;

			if (clk <= config->frequency) {
				break;
			}
		}

		if (br > ARRAY_SIZE(scaler)) {
			LOG_ERR("Unsupported frequency %uHz, max %uHz, min %uHz",
				    config->frequency,
				    clock >> 1,
				    clock >> ARRAY_SIZE(scaler));
			return -EINVAL;
		}	
		spi_init.BaudRate = scaler[br - 1];
	}
	
	/* Configure clock polarity and phase */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
	} else {
		spi_init.ClockPolarity = LL_SPI_POLARITY_LOW;
	}
	
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
	} else {
		spi_init.ClockPhase = LL_SPI_PHASE_1EDGE;
	}

	/* Configure data size */
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);
	if (word_size == 8) {
		spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	} else if (word_size == 16) {
		spi_init.DataWidth = LL_SPI_DATAWIDTH_16BIT;
	} else {
		LOG_ERR("Unsupported word size: %d", word_size);
		return -ENOTSUP;
	}

	/* Configure bit order */
	if (config->operation & SPI_TRANSFER_LSB) {
		spi_init.BitOrder = LL_SPI_LSB_FIRST;
	} else {
		spi_init.BitOrder = LL_SPI_MSB_FIRST;
	}

	if (spi_cs_is_gpio(config) || !IS_ENABLED(CONFIG_SPI_STM32_USE_HW_SS)) {
		/* Use software NSS management (GPIO CS) */
		spi_init.NSS = LL_SPI_NSS_SOFT;
	} else {
		/* Use hardware NSS control */
		if (config->operation & SPI_CS_ACTIVE_HIGH) {
			LOG_ERR("Hardware SS does not support active high CS");
			return -ENOTSUP;
		}
		spi_init.NSS = LL_SPI_NSS_HARD_OUTPUT;
	}

	/* Disable SPI before configuration */
	ll_func_disable_spi(spi);

	/* Apply configuration */
	ret = LL_SPI_Init(spi, &spi_init);
	if (ret != SUCCESS) {
		LOG_ERR("SPI configuration failed");
		return -EINVAL;
	}

	ll_func_configure_fifo(spi, cfg);

	data->ctx.config = config;

	return 0;
}

#ifdef CONFIG_SPI_STM32_INTERRUPT
static int spi_stm32_transceive_interrupt(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);

	/* Determine transfer type and configure accordingly */
	if (data->tx_buf && data->rx_buf) {
		LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	} else if (data->tx_buf) {
		LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_TX);
	} else {
		LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_RX);
	}

	/* Reset counters for interrupt mode */
	data->tx_count = 0;
	data->rx_count = 0;
	data->error_status = 0;

	/* Start transmission with first byte/word */
	if (data->tx_buf && data->tx_len > 0) {
		/* Send real data for TX or TXRX operations */
		if (word_size == 16) {
			uint16_t tx_data = ((uint16_t *)data->tx_buf)[0];
			LL_SPI_TransmitData16(spi, tx_data);
			data->tx_count = 2;
		} else {
			LL_SPI_TransmitData8(spi, data->tx_buf[0]);
			data->tx_count = 1;
		}
	} else if (data->rx_buf) {
		/* Send dummy data for RX-only operations to generate clock */
		if (word_size == 16) {
			LL_SPI_TransmitData16(spi, 0xFFFF);
			data->tx_count = 2;
		} else {
			LL_SPI_TransmitData8(spi, 0xFF);
			data->tx_count = 1;
		}
	}

	/* Enable interrupts */
	ll_func_enable_int_tx_empty(spi);
	ll_func_enable_int_rx_not_empty(spi);
	ll_func_enable_int_errors(spi);

	return 0; /* Non-blocking - completion handled in ISR */
}

#endif /* CONFIG_SPI_STM32_INTERRUPT */

static int spi_stm32_transceive_polling(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	uint32_t timeout = 100000;
	const struct spi_config *config = data->ctx.config;

	/* Determine transfer type and configure accordingly */
	if (data->tx_buf && data->rx_buf) {
		LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	} else if (data->tx_buf) {
		LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_TX);
	} else {
		LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_RX);
	}

	/* Reset counters for polling mode */
	data->tx_count = 0;
	data->rx_count = 0;

	/* Main transfer loop */
	while ((data->tx_count < (data->tx_buf ? data->tx_len : data->rx_len)) || 
	       (data->rx_count < (data->rx_buf ? data->rx_len : 0))) {
		
		/* Check for SPI errors - Phase 2 enhanced error handling */
		uint32_t sr = LL_SPI_ReadReg(spi, SR);
		if (sr & SPI_STM32_ERR_MSK) {
			LOG_ERR("SPI error detected: SR=0x%08x", sr);
			
			/* Detailed error logging */
			if (sr & LL_SPI_SR_OVR) {
				LOG_ERR("SPI overrun error");
			}
			if (sr & LL_SPI_SR_MODF) {
				LOG_ERR("SPI mode fault error");
			}
			if (sr & LL_SPI_SR_CRCERR) {
				LOG_ERR("SPI CRC error");
			}
			
			/* Clear error flags and attempt recovery */
			LL_SPI_ClearFlag_OVR(spi);
			LL_SPI_ClearFlag_MODF(spi);
			LL_SPI_ClearFlag_CRCERR(spi);
			
			return -EIO;
		}

		/* Handle TX: Send data when TX buffer ready */
		size_t tx_target = data->tx_buf ? data->tx_len : data->rx_len;
		if ((data->tx_count < tx_target) && ll_func_tx_is_not_full(spi)) {
			spi_stm32_transmit_buffer(dev);
		}

		/* Handle RX: Receive data when available */
		if (ll_func_rx_is_not_empty(spi)) {
			spi_stm32_receive_buffer(dev);
		}

		if (--timeout == 0) {
			LOG_ERR("SPI polling timeout");
			return -ETIMEDOUT;
		}
	}

	int ret = spi_stm32_wait_transfer_complete(spi);
	if (ret != 0) {
		return ret;
	}
	
	/* Discard any remaining RX data */
	spi_stm32_discard_rx_data(dev);

	return 0;
}

static void spi_stm32_iodev_prepare_start(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct spi_dt_spec *spi_dt_spec = rtio_ctx->txn_curr->sqe.iodev->data;
	struct spi_config *spi_config = &spi_dt_spec->config;
	int ret;

	spi_stm32_pm_policy_state_lock_get(dev);

	ret = spi_stm32_configure(dev, spi_config);
	if (ret) {
		LOG_ERR("SPI configuration failed: %d", ret);
		spi_stm32_iodev_complete(dev, ret);
		return;
	}

	spi_stm32_cs_control(dev, true);

	LL_SPI_Enable(cfg->spi);
}

static void spi_stm32_iodev_start(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_curr->sqe;
	int ret = 0;

	/* Extract buffer information and execute appropriate operation */
	switch (sqe->op) {
	case RTIO_OP_RX:
		/* RX-only: receive data while sending dummy bytes */
		data->tx_buf = NULL;
		data->tx_len = 0;
		data->rx_buf = sqe->rx.buf;
		data->rx_len = sqe->rx.buf_len;
		
#ifdef CONFIG_SPI_STM32_INTERRUPT
		ret = spi_stm32_transceive_interrupt(dev);
#else
		ret = spi_stm32_transceive_polling(dev);
#endif
		break;
		
	case RTIO_OP_TX:
		/* TX-only: send data while discarding received bytes */
		data->tx_buf = (const uint8_t *)sqe->tx.buf;
		data->tx_len = sqe->tx.buf_len;
		data->rx_buf = NULL;
		data->rx_len = 0;
		
#ifdef CONFIG_SPI_STM32_INTERRUPT
		ret = spi_stm32_transceive_interrupt(dev);
#else
		ret = spi_stm32_transceive_polling(dev);
#endif
		break;
		
	case RTIO_OP_TINY_TX:
		/* Small TX-only: send data while discarding received bytes */
		data->tx_buf = (const uint8_t *)sqe->tiny_tx.buf;
		data->tx_len = sqe->tiny_tx.buf_len;
		data->rx_buf = NULL;
		data->rx_len = 0;
		
#ifdef CONFIG_SPI_STM32_INTERRUPT
		ret = spi_stm32_transceive_interrupt(dev);
#else
		ret = spi_stm32_transceive_polling(dev);
#endif
		break;
		
	case RTIO_OP_TXRX:
		/* Full-duplex: simultaneous send and receive */
		data->tx_buf = (const uint8_t *)sqe->txrx.tx_buf;
		data->tx_len = sqe->txrx.buf_len;
		data->rx_buf = sqe->txrx.rx_buf;
		data->rx_len = sqe->txrx.buf_len;
		
#ifdef CONFIG_SPI_STM32_INTERRUPT
		ret = spi_stm32_transceive_interrupt(dev);
#else
		ret = spi_stm32_transceive_polling(dev);
#endif
		break;
		
	default:
		LOG_ERR("Unsupported RTIO operation: %d", sqe->op);
		ret = -EINVAL;
		break;
	}

	/* For polling mode, complete immediately. For interrupt mode, completion happens in ISR */
#ifndef CONFIG_SPI_STM32_INTERRUPT
	spi_stm32_iodev_complete(dev, ret);
#else
	/* For interrupt mode, only complete if there was an immediate error */
	if (ret != 0) {
		spi_stm32_iodev_complete(dev, ret);
	}
#endif
}

static void spi_stm32_iodev_complete(const struct device *dev, int status)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;

	LOG_DBG("RTIO operation complete: status=%d", status);

	/* Handle transaction chaining if needed */
	if (!status && rtio_ctx->txn_curr->sqe.flags & RTIO_SQE_TRANSACTION) {
		rtio_ctx->txn_curr = rtio_txn_next(rtio_ctx->txn_curr);
		spi_stm32_iodev_start(dev);
	} else {
		ll_func_disable_spi(cfg->spi);
		spi_stm32_cs_control(dev, false);

		spi_stm32_pm_policy_state_lock_put(dev);

		if (spi_rtio_complete(rtio_ctx, status)) {
			/* Start next transaction if available */
			spi_stm32_iodev_prepare_start(dev);
			spi_stm32_iodev_start(dev);
		}
	}
}

static void spi_stm32_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;

	/* Submit to RTIO and start if this begins a new transaction */
	if (spi_rtio_submit(rtio_ctx, iodev_sqe)) {
		spi_stm32_iodev_prepare_start(dev);
		spi_stm32_iodev_start(dev);
	}
}

static int spi_stm32_transceive(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_stm32_data *data = dev->data;

	return spi_rtio_transceive(data->rtio_ctx, config, tx_bufs, rx_bufs);
}

static int spi_stm32_release(const struct device *dev, const struct spi_config *config)
{
	spi_stm32_pm_policy_state_lock_put(dev);
	return 0;
}

static inline bool spi_stm32_is_subghzspi(const struct device *dev)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
	const struct spi_stm32_config *cfg = dev->config;
	return cfg->use_subghzspi_nss;
#else
	ARG_UNUSED(dev);
	return false;
#endif
}

/* SPI driver API structure */
static const struct spi_driver_api spi_stm32_api = {
	.transceive = spi_stm32_transceive,
	.release = spi_stm32_release,
	.iodev_submit = spi_stm32_iodev_submit,
};

/* Device initialization */
static int spi_stm32_init(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	int ret;

	/* Enable clocks */
	ret = spi_stm32_clock_enable(dev);
	if (ret) {
		LOG_ERR("Failed to enable SPI clocks");
		return ret;
	}

	/* Configure pins (skip for SubGHz SPI) */
	if (!spi_stm32_is_subghzspi(dev)) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret) {
			LOG_ERR("Failed to configure SPI pins");
			return ret;
		}
	}

	/* Initialize SPI context for CS GPIO support */
	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		LOG_ERR("Failed to configure CS GPIOs");
		return ret;
	}

	/* Initialize RTIO context */
	spi_rtio_init(data->rtio_ctx, dev);

	/* Configure interrupts if enabled */
#ifdef CONFIG_SPI_STM32_INTERRUPT
	if (cfg->irq_config_func) {
		cfg->irq_config_func(dev);
	}
#endif

	/* Power management initialization */
	if (IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)) {
		ret = pm_device_runtime_enable(dev);
		if (ret) {
			LOG_ERR("Failed to enable runtime PM");
			return ret;
		}
	}

	/* Release spi context lock to allow operation */
	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int spi_stm32_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct spi_stm32_config *cfg = dev->config;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Disable clock first */
		ret = spi_stm32_clock_disable(dev);
		if (ret) {
			return ret;
		}
		
		/* Move pins to sleep state (skip for SubGHz SPI) */
		if (!spi_stm32_is_subghzspi(dev)) {
			ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
			if ((ret < 0) && (ret != -ENOENT)) {
				LOG_ERR("Failed to set sleep pin state: %d", ret);
				return ret;
			}
		}
		break;
		
	case PM_DEVICE_ACTION_RESUME:
		/* Set pins to active state first (skip for SubGHz SPI) */
		if (!spi_stm32_is_subghzspi(dev)) {
			ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
			if (ret < 0) {
				LOG_ERR("Failed to set default pin state: %d", ret);
				return ret;
			}
		}
		
		/* Enable clock */
		ret = spi_stm32_clock_enable(dev);
		break;
		
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif

static inline int spi_stm32_handle_errors(SPI_TypeDef *spi)
{
	if (!LL_SPI_IsEnabledIT_ERR(spi)) {
		return 0;
	}

	int err = 0;
	
	if (LL_SPI_IsActiveFlag_OVR(spi)) {
		LL_SPI_ClearFlag_OVR(spi);
		err = -EIO;
	}
	if (LL_SPI_IsActiveFlag_MODF(spi)) {
		LL_SPI_ClearFlag_MODF(spi);
		err = -EIO;
	}
	if (LL_SPI_IsActiveFlag_CRCERR(spi)) {
		LL_SPI_ClearFlag_CRCERR(spi);
		err = -EIO;
	}
#ifdef LL_SPI_SR_UDR
	if (LL_SPI_IsActiveFlag_UDR(spi)) {
		err = -EIO;
	}
#endif

	return err;
}

#ifdef CONFIG_SPI_STM32_INTERRUPT
static inline bool spi_stm32_handle_tx_interrupt(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	SPI_TypeDef *spi = cfg->spi;
	
	if (!ll_func_tx_is_not_full(spi)) {
		return false; /* TX FIFO full, can't send */
	}

	return spi_stm32_transmit_buffer(dev);
}

static inline bool spi_stm32_handle_rx_interrupt(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	SPI_TypeDef *spi = cfg->spi;
	
	if (!ll_func_rx_is_not_empty(spi)) {
		return false; /* No RX data available */
	}

	return spi_stm32_receive_buffer(dev);
}

static void spi_stm32_isr(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	bool tx_complete, rx_complete;
	int rc;

	if (ll_func_are_int_disabled(spi)) {
		LOG_DBG("SPI interrupt received while interrupts are disabled");
		return;
	}

	/* Check for spurious interrupts */
	if (!LL_SPI_IsEnabled(spi)) {
		LOG_DBG("SPI interrupt received while SPI is disabled");
		return;
	}

	rc = spi_stm32_handle_errors(spi);
	if (rc) {
		LOG_ERR("SPI error detected");	
		data->error_status = rc;
		/* Disable all interrupts before completing */
		ll_func_disable_int_tx_empty(spi);
		ll_func_disable_int_rx_not_empty(spi);
		ll_func_disable_int_errors(spi);
		spi_stm32_iodev_complete(dev, rc);
		return;
	}

	tx_complete = spi_stm32_handle_tx_interrupt(dev);
	if (tx_complete) {
		ll_func_disable_int_tx_empty(spi);
	}

	rx_complete = spi_stm32_handle_rx_interrupt(dev);
	if (rx_complete) {
		ll_func_disable_int_rx_not_empty(spi);
	}

	if (tx_complete && rx_complete) {
		ll_func_disable_int_errors(spi);
		spi_stm32_iodev_complete(dev, 0);
	} 
}
#endif

/* Device tree macros for driver instantiation */
#ifdef CONFIG_SPI_STM32_INTERRUPT
#define SPI_STM32_IRQ_CONFIG_FUNC(n)						\
	static void spi_stm32_irq_config_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    spi_stm32_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQN(n));					\
	}
#else
#define SPI_STM32_IRQ_CONFIG_FUNC(n)
#endif

#ifdef CONFIG_SPI_STM32_INTERRUPT
#define SPI_STM32_CONFIG_IRQ_FUNC(n) .irq_config_func = spi_stm32_irq_config_func_##n,
#else
#define SPI_STM32_CONFIG_IRQ_FUNC(n)
#endif

/* Device instantiation macro */
#define STM32_SPI_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);						\
	SPI_RTIO_DEFINE(stm32_spi_rtio_##n,					\
			CONFIG_SPI_STM32_RTIO_SQ_SIZE,				\
			CONFIG_SPI_STM32_RTIO_CQ_SIZE);				\
	SPI_STM32_IRQ_CONFIG_FUNC(n)						\
	static const struct stm32_pclken stm32_spi_pclken_##n[] =		\
		STM32_DT_INST_CLOCKS(n);					\
	static const struct spi_stm32_config spi_stm32_config_##n = {		\
		.spi = (SPI_TypeDef *)DT_INST_REG_ADDR(n),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.pclken = stm32_spi_pclken_##n,					\
		.pclk_len = ARRAY_SIZE(stm32_spi_pclken_##n),			\
		SPI_STM32_CONFIG_IRQ_FUNC(n)					\
		.fifo_enabled = DT_INST_PROP_OR(n, fifo_enable, false),	\
		IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi),		\
			(.midi_clocks = DT_INST_PROP_OR(n, midi_clock, 0),))	\
		IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi),		\
			(.mssi_clocks = DT_INST_PROP_OR(n, mssi_clock, 0),))	\
		IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz),	\
			(.use_subghzspi_nss = DT_INST_PROP_OR(n, use_subghzspi_nss, false),)) \
	};									\
	static struct spi_stm32_data spi_stm32_data_##n = {			\
		.rtio_ctx = &stm32_spi_rtio_##n,				\
		SPI_CONTEXT_INIT_LOCK(spi_stm32_data_##n, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_stm32_data_##n, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)		\
	};									\
	PM_DEVICE_DT_INST_DEFINE(n, spi_stm32_pm_action);			\
	DEVICE_DT_INST_DEFINE(n, spi_stm32_init, PM_DEVICE_DT_INST_GET(n),	\
			      &spi_stm32_data_##n, &spi_stm32_config_##n,	\
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
			      &spi_stm32_api);

/* Instantiate all DT_DRV_COMPAT devices */
DT_INST_FOREACH_STATUS_OKAY(STM32_SPI_INIT)
