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
#include "spi_context.h"

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

/* Domain clock support detection */
#if STM32_DT_INST_DEV_DOMAIN_CLOCK_SUPPORT
#define STM32_SPI_DOMAIN_CLOCK_SUPPORT 1
#else
#define STM32_SPI_DOMAIN_CLOCK_SUPPORT 0
#endif

/* STM32-specific DMA structures */
#ifdef CONFIG_SPI_STM32_DMA
struct spi_stm32_dma_stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
};

#define SPI_STM32_DMA_ERROR_FLAG	0x01
#define SPI_STM32_DMA_RX_DONE_FLAG	0x02
#define SPI_STM32_DMA_TX_DONE_FLAG	0x04
#define SPI_STM32_DMA_DONE_FLAG	\
	(SPI_STM32_DMA_RX_DONE_FLAG | SPI_STM32_DMA_TX_DONE_FLAG)
#endif

/* STM32 SPI configuration structure */
struct spi_stm32_config {
	SPI_TypeDef *spi;
	const struct pinctrl_dev_config *pcfg;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	void (*irq_config_func)(const struct device *dev);
	bool fifo_enabled;
	
	/* STM32-specific features */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	int midi_clocks;
	int mssi_clocks;
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
	bool use_subghzspi_nss;
#endif
#ifdef CONFIG_SPI_STM32_DMA
	struct stm32_dma_spec dma_tx;
	struct stm32_dma_spec dma_rx;
#endif
};

/* STM32 SPI data structure - pure RTIO */
struct spi_stm32_data {
	struct spi_rtio *rtio_ctx;		/* Single RTIO context */
	
	/* STM32-specific state */
	volatile int error_status;
	bool pm_policy_state_on;
	
	/* For sync API compatibility (handled by spi_rtio.c) */
	struct spi_context ctx;
	
	/* DMA state */
#ifdef CONFIG_SPI_STM32_DMA
	struct k_sem status_sem;
	volatile uint32_t status_flags;
	struct spi_stm32_dma_stream dma_rx;
	struct spi_stm32_dma_stream dma_tx;
#endif
	
	/* Current transaction state for polling mode */
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t tx_len;
	size_t rx_len;
	size_t tx_count;
	size_t rx_count;
};

/* Forward declarations */
static void spi_stm32_iodev_complete(const struct device *dev, int status);
static int spi_stm32_configure(const struct device *dev, const struct spi_config *config);

/* STM32-specific helper functions */
static inline uint32_t spi_stm32_tx_is_not_full(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	return LL_SPI_IsActiveFlag_TXP(spi);
#else
	return LL_SPI_IsActiveFlag_TXE(spi);
#endif
}

static inline uint32_t spi_stm32_rx_is_not_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	return LL_SPI_IsActiveFlag_RXP(spi);
#else
	return LL_SPI_IsActiveFlag_RXNE(spi);
#endif
}

static inline void spi_stm32_enable_int_tx_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_EnableIT_TXP(spi);
#else
	LL_SPI_EnableIT_TXE(spi);
#endif
}

static inline void spi_stm32_enable_int_rx_not_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_EnableIT_RXP(spi);
#else
	LL_SPI_EnableIT_RXNE(spi);
#endif
}

static inline void spi_stm32_disable_int_tx_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_DisableIT_TXP(spi);
#else
	LL_SPI_DisableIT_TXE(spi);
#endif
}

static inline void spi_stm32_disable_int_rx_not_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_DisableIT_RXP(spi);
#else
	LL_SPI_DisableIT_RXNE(spi);
#endif
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

/* Clock control helpers */
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

/* Basic SPI configuration function */
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
	uint32_t clock;
	int br;	
	int ret;

	if (SPI_OP_MODE_GET(config->operation) & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported yet");
		return -ENOTSUP;
	}

	/* Configure SPI for master mode */
	LL_SPI_StructInit(&spi_init);
	spi_init.Mode = LL_SPI_MODE_MASTER;
	
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

	/* Configure NSS (Chip Select) */
	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		/* For now, we'll handle this with GPIO CS control */
		spi_init.NSS = LL_SPI_NSS_SOFT;
	} else {
		spi_init.NSS = LL_SPI_NSS_SOFT;
	}

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

	/* Disable SPI before configuration */
	LL_SPI_Disable(spi);

	/* Apply configuration */
	ret = LL_SPI_Init(spi, &spi_init);
	if (ret != SUCCESS) {
		LOG_ERR("SPI configuration failed");
		return -EINVAL;
	}

	/* Store current configuration for spi_context compatibility */
	data->ctx.config = config;

	return 0;
}

/* Polling mode transceive function - Phase 1A implementation */
static int spi_stm32_transceive_polling(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	size_t tx_count = 0, rx_count = 0;
	uint32_t timeout = 100000; /* Simple timeout for polling */

	LOG_DBG("SPI transceive polling: tx_len=%zu, rx_len=%zu", data->tx_len, data->rx_len);

	/* Simplified polling transceive - will be enhanced in later phases */
	while ((tx_count < data->tx_len) || (rx_count < data->rx_len)) {
		/* Send data if TX buffer available and SPI ready */
		if ((tx_count < data->tx_len) && spi_stm32_tx_is_not_full(spi)) {
			if (data->tx_buf) {
				LL_SPI_TransmitData8(spi, data->tx_buf[tx_count]);
			} else {
				LL_SPI_TransmitData8(spi, 0xFF); /* Dummy data */
			}
			tx_count++;
		}

		/* Receive data if RX buffer available and data ready */
		if ((rx_count < data->rx_len) && spi_stm32_rx_is_not_empty(spi)) {
			uint8_t rx_data = LL_SPI_ReceiveData8(spi);
			if (data->rx_buf) {
				data->rx_buf[rx_count] = rx_data;
			}
			rx_count++;
		}

		/* Simple timeout check */
		if (--timeout == 0) {
			LOG_ERR("SPI transaction timeout");
			return -ETIMEDOUT;
		}
	}

	/* Wait for SPI to become idle */
	timeout = 10000;
	while (LL_SPI_IsActiveFlag_BSY(spi)) {
		if (--timeout == 0) {
			LOG_ERR("SPI busy timeout");
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}

	data->tx_count = tx_count;
	data->rx_count = rx_count;

	LOG_DBG("SPI transceive complete: tx_count=%zu, rx_count=%zu", tx_count, rx_count);
	return 0;
}

/* RTIO preparation function */
static void spi_stm32_iodev_prepare_start(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct spi_dt_spec *spi_dt_spec = rtio_ctx->txn_curr->sqe.iodev->data;
	struct spi_config *spi_config = &spi_dt_spec->config;
	int ret;

	LOG_DBG("RTIO prepare start");

	/* Configure SPI for this transaction */
	ret = spi_stm32_configure(dev, spi_config);
	if (ret) {
		LOG_ERR("SPI configuration failed: %d", ret);
		spi_stm32_iodev_complete(dev, ret);
		return;
	}

	/* Handle CS control */
	if (spi_cs_is_gpio(spi_config)) {
		spi_context_cs_control(&data->ctx, true);
	}

	/* Enable SPI for this transaction */
	LL_SPI_Enable(cfg->spi);

	/* Power management */
	spi_stm32_pm_policy_state_lock_get(dev);
}

/* RTIO operation start function */
static void spi_stm32_iodev_start(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_curr->sqe;
	int ret = 0;

	LOG_DBG("RTIO operation start: op=%d", sqe->op);

	/* Extract buffer information from RTIO operation */
	switch (sqe->op) {
	case RTIO_OP_RX:
		data->tx_buf = NULL;
		data->tx_len = sqe->rx.buf_len;  /* Send dummy data */
		data->rx_buf = sqe->rx.buf;
		data->rx_len = sqe->rx.buf_len;
		break;
	case RTIO_OP_TX:
		data->tx_buf = (const uint8_t *)sqe->tx.buf;
		data->tx_len = sqe->tx.buf_len;
		data->rx_buf = NULL;
		data->rx_len = 0;
		break;
	case RTIO_OP_TINY_TX:
		data->tx_buf = (const uint8_t *)sqe->tiny_tx.buf;
		data->tx_len = sqe->tiny_tx.buf_len;
		data->rx_buf = NULL;
		data->rx_len = 0;
		break;
	case RTIO_OP_TXRX:
		data->tx_buf = (const uint8_t *)sqe->txrx.tx_buf;
		data->tx_len = sqe->txrx.buf_len;
		data->rx_buf = sqe->txrx.rx_buf;
		data->rx_len = sqe->txrx.buf_len;
		break;
	default:
		LOG_ERR("Unsupported RTIO operation: %d", sqe->op);
		spi_stm32_iodev_complete(dev, -EINVAL);
		return;
	}

	/* Execute polling transceive for Phase 1A */
	ret = spi_stm32_transceive_polling(dev);
	
	/* Complete the operation */
	spi_stm32_iodev_complete(dev, ret);
}

/* RTIO operation completion function */
static void spi_stm32_iodev_complete(const struct device *dev, int status)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;

	LOG_DBG("RTIO operation complete: status=%d", status);

	/* Disable SPI after transaction */
	LL_SPI_Disable(cfg->spi);

	/* Handle CS release and power management */
	if (rtio_ctx->txn_curr) {
		struct spi_dt_spec *spi_dt_spec = rtio_ctx->txn_curr->sqe.iodev->data;
		struct spi_config *spi_config = &spi_dt_spec->config;

		/* Release CS if GPIO controlled */
		if (spi_cs_is_gpio(spi_config)) {
			spi_context_cs_control(&data->ctx, false);
		}
	}

	/* Handle transaction chaining if needed */
	if (!status && rtio_ctx->txn_curr->sqe.flags & RTIO_SQE_TRANSACTION) {
		rtio_ctx->txn_curr = rtio_txn_next(rtio_ctx->txn_curr);
		spi_stm32_iodev_start(dev);
	} else {
		/* Release power management lock */
		spi_stm32_pm_policy_state_lock_put(dev);

		/* Complete the RTIO transaction */
		if (spi_rtio_complete(rtio_ctx, status)) {
			/* Start next transaction if available */
			spi_stm32_iodev_prepare_start(dev);
			spi_stm32_iodev_start(dev);
		}
	}
}

/* RTIO submit function - main entry point */
static void spi_stm32_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;

	LOG_DBG("RTIO submit");

	/* Submit to RTIO and start if this begins a new transaction */
	if (spi_rtio_submit(rtio_ctx, iodev_sqe)) {
		spi_stm32_iodev_prepare_start(dev);
		spi_stm32_iodev_start(dev);
	}
}

/* Standard SPI API functions (for compatibility via spi_rtio.c) */
static int spi_stm32_transceive(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_stm32_data *data = dev->data;

	/* Use RTIO to handle the transceive operation */
	return spi_rtio_transceive(data->rtio_ctx, config, tx_bufs, rx_bufs);
}

static int spi_stm32_release(const struct device *dev, const struct spi_config *config)
{
	spi_stm32_pm_policy_state_lock_put(dev);
	return 0;
}

/* SubGHz SPI detection helper */
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

	LOG_DBG("Initializing STM32 SPI device");

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

	LOG_INF("STM32 SPI device initialized successfully");
	return 0;
}

/* Power management functions */
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

/* Placeholder ISR for interrupt support (Phase 3) */
#ifdef CONFIG_SPI_STM32_INTERRUPT
static void spi_stm32_isr(const struct device *dev)
{
	/* Will be implemented in Phase 3 */
	LOG_DBG("SPI ISR called (not implemented yet)");
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
