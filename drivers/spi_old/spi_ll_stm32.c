/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rubus_stm32_spi

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32_cache.h>
#include <stm32_ll_spi.h>
#include <errno.h>
#include <string.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>
#endif
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#include <zephyr/dt-bindings/memory-attr/memory-attr-arm.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/arch/cache.h>

LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_SPI_LOG_LEVEL);

#include "spi_ll_stm32.h"

/* Forward declarations */
static void spi_stm32_iodev_complete(const struct device *dev, int status);
static int spi_stm32_configure(const struct device *dev, const struct spi_config *config);
static inline int spi_stm32_handle_errors(SPI_TypeDef *spi);

#ifdef CONFIG_SPI_RUBUS_STM32_DMA
/* Nocache dummy buffer for DMA operations when no buffer provided */
static __aligned(32) uint32_t dummy_rx_tx_buffer __nocache;

static void spi_stm32_try_finalize(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	int status = data->error_status;

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	bool wait_eot = data->waiting_eot;
#endif

	if (!data->dma_active) {
		return;
	}

	if (data->status_flags & SPI_STM32_DMA_ERROR_FLAG) {
		if (status == 0) {
			status = -EIO;
		}
	} else {
		if ((data->status_flags & SPI_STM32_DMA_DONE_FLAG) != SPI_STM32_DMA_DONE_FLAG) {
			return;
		}
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
		if (IS_ENABLED(CONFIG_SPI_RUBUS_STM32_INTERRUPT) && wait_eot) {
			if ((data->status_flags & SPI_STM32_EOT_DONE_FLAG) == 0U) {
				return;
			}
		}
#endif
	}

	spi_stm32_iodev_complete(dev, status);
}

/* DMA callback function - executed in interrupt context */
static void dma_callback(const struct device *dma_dev, void *arg,
		 uint32_t channel, int status)
{
	ARG_UNUSED(dma_dev);

	/* arg contains the SPI device pointer */
	const struct device *dev = arg;
	struct spi_stm32_data *data = dev->data;

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d", channel);
		data->status_flags |= SPI_STM32_DMA_ERROR_FLAG;
		data->error_status = -EIO;
	} else {
		/* Identify which DMA channel completed */
		if (channel == data->dma_tx.channel) {
			data->status_flags |= SPI_STM32_DMA_TX_DONE_FLAG;
		} else if (channel == data->dma_rx.channel) {
			data->status_flags |= SPI_STM32_DMA_RX_DONE_FLAG;
		} else {
			LOG_ERR("DMA callback channel %d is not valid", channel);
			data->status_flags |= SPI_STM32_DMA_ERROR_FLAG;
			data->error_status = -EIO;
		}
	}

	/* Evaluate completion conditions */
	spi_stm32_try_finalize(dev);
}

/* Configure and start DMA TX transfer */
static int spi_stm32_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct stream *stream = &data->dma_tx;
	struct dma_block_config *blk_cfg = &stream->dma_blk_cfg;
	uint32_t src_addr;
	uint32_t dest_addr = ll_func_dma_get_reg_addr(cfg->spi, SPI_STM32_DMA_TX);
	uint16_t src_adj;
	uint16_t dest_adj;
	int ret;

	/* TX direction: memory -> peripheral */
	if (buf == NULL) {
		/* Send dummy data (NOP) when no TX buffer provided */
		dummy_rx_tx_buffer = 0;
#ifdef CONFIG_DCACHE
		sys_cache_data_flush_range((void *)&dummy_rx_tx_buffer, sizeof(uint32_t));
#endif
		src_addr = (uint32_t)&dummy_rx_tx_buffer;
		src_adj = DMA_ADDR_ADJ_NO_CHANGE;
		stream->last_checked_buf = NULL;
		stream->last_checked_len = 0;
	} else {
		src_addr = (uint32_t)buf;
		src_adj = stream->src_addr_increment ?
			DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;
#ifdef CONFIG_DCACHE
		if (IS_ENABLED(CONFIG_DCACHE) && len > 0U) {
			const void *prev_buf = stream->last_checked_buf;
			size_t prev_len = stream->last_checked_len;
			bool cached = !stm32_buf_in_nocache((uintptr_t)buf, len);
			if (cached) {
				sys_cache_data_flush_range((void *)buf, len);
				if (prev_buf != buf || prev_len != len) {
					LOG_DBG("Flushed TX buffer for DMA coherency");
				}
			}
		}
#endif
		stream->last_checked_buf = buf;
		stream->last_checked_len = len;
	}

	dest_adj = stream->dst_addr_increment ?
		DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;

	bool needs_config = !stream->configured ||
				stream->last_src_adj != src_adj ||
				stream->last_dst_adj != dest_adj;

	if (needs_config) {
		memset(blk_cfg, 0, sizeof(struct dma_block_config));
		blk_cfg->block_size = len;
		blk_cfg->source_address = src_addr;
		blk_cfg->dest_address = dest_addr;
		blk_cfg->source_addr_adj = src_adj;
		blk_cfg->dest_addr_adj = dest_adj;
		blk_cfg->fifo_mode_control = stream->fifo_threshold;

		stream->dma_cfg.head_block = blk_cfg;
		stream->dma_cfg.user_data = (void *)dev; /* Pass device pointer for callback */

		ret = dma_config(stream->dma_dev, stream->channel, &stream->dma_cfg);
		if (ret != 0) {
			LOG_ERR("Failed to configure TX DMA: %d", ret);
			return ret;
		}

		stream->configured = true;
		stream->last_src_adj = src_adj;
		stream->last_dst_adj = dest_adj;
	} else {
		blk_cfg->block_size = len;
	}

	ret = dma_reload(stream->dma_dev, stream->channel, src_addr, dest_addr, len);
	if (ret != 0) {
		LOG_ERR("Failed to reload TX DMA: %d", ret);
		return ret;
	}

	return dma_start(stream->dma_dev, stream->channel);
}

/* Configure and start DMA RX transfer */
static int spi_stm32_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct stream *stream = &data->dma_rx;
	struct dma_block_config *blk_cfg = &stream->dma_blk_cfg;
	uint32_t src_addr = ll_func_dma_get_reg_addr(cfg->spi, SPI_STM32_DMA_RX);
	uint32_t dest_addr;
	uint16_t src_adj;
	uint16_t dest_adj;
	int ret;

	/* RX direction: peripheral -> memory */
	if (buf == NULL) {
		/* Discard received data when no RX buffer provided */
		dest_addr = (uint32_t)&dummy_rx_tx_buffer;
		dest_adj = DMA_ADDR_ADJ_NO_CHANGE;
		stream->last_checked_buf = NULL;
		stream->last_checked_len = 0;
	} else {
		dest_addr = (uint32_t)buf;
		dest_adj = stream->dst_addr_increment ?
			DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;
#ifdef CONFIG_DCACHE
		if (IS_ENABLED(CONFIG_DCACHE) && len > 0U) {
			const void *prev_buf = stream->last_checked_buf;
			size_t prev_len = stream->last_checked_len;
			bool cached = !stm32_buf_in_nocache((uintptr_t)buf, len);
			if (cached) {
				sys_cache_data_flush_range((void *)buf, len);
				if (prev_buf != buf || prev_len != len) {
					LOG_DBG("Flushed RX buffer before DMA");
				}
			}
		}
#endif
		stream->last_checked_buf = buf;
		stream->last_checked_len = len;
	}

	src_adj = stream->src_addr_increment ?
		DMA_ADDR_ADJ_INCREMENT : DMA_ADDR_ADJ_NO_CHANGE;

	bool needs_config = !stream->configured ||
				stream->last_src_adj != src_adj ||
				stream->last_dst_adj != dest_adj;

	if (needs_config) {
		memset(blk_cfg, 0, sizeof(struct dma_block_config));
		blk_cfg->block_size = len;
		blk_cfg->source_address = src_addr;
		blk_cfg->dest_address = dest_addr;
		blk_cfg->source_addr_adj = src_adj;
		blk_cfg->dest_addr_adj = dest_adj;
		blk_cfg->fifo_mode_control = stream->fifo_threshold;

		stream->dma_cfg.head_block = blk_cfg;
		stream->dma_cfg.user_data = (void *)dev; /* Pass device pointer for callback */

		ret = dma_config(stream->dma_dev, stream->channel, &stream->dma_cfg);
		if (ret != 0) {
			LOG_ERR("Failed to configure RX DMA: %d", ret);
			return ret;
		}

		stream->configured = true;
		stream->last_src_adj = src_adj;
		stream->last_dst_adj = dest_adj;
	} else {
		blk_cfg->block_size = len;
	}

	ret = dma_reload(stream->dma_dev, stream->channel, src_addr, dest_addr, len);
	if (ret != 0) {
		LOG_ERR("Failed to reload RX DMA: %d", ret);
		return ret;
	}

	return dma_start(stream->dma_dev, stream->channel);
}
#endif /* CONFIG_SPI_RUBUS_STM32_DMA */

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

/* Transmit frame - non-FIFO mode (single byte/word) */
static void spi_stm32_shift_tx(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);

	if (!ll_func_tx_is_not_full(spi)) {
		return;
	}

	if (data->tx_buf != NULL) {
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
	} else {
		/* RX-only: send dummy data */
		if (data->tx_count < data->rx_len) {
			if (word_size == 16) {
				LL_SPI_TransmitData16(spi, 0xFFFF);
				data->tx_count += 2;
			} else {
				LL_SPI_TransmitData8(spi, 0xFF);
				data->tx_count++;
			}
		}
	}
}

/* Transmit frame - FIFO mode (packet-based) */
static void spi_stm32_shift_tx_fifo(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);

	if (data->tx_buf != NULL) {
		/* TX operation: send real data */
		while (data->tx_count < data->tx_len && ll_func_tx_is_not_full(spi)) {
			if (word_size == 16) {
				uint16_t tx_data = ((uint16_t *)data->tx_buf)[data->tx_count / 2];
				LL_SPI_TransmitData16(spi, tx_data);
				data->tx_count += 2;
			} else {
				LL_SPI_TransmitData8(spi, data->tx_buf[data->tx_count]);
				data->tx_count++;
			}
		}
	} else {
		/* RX-only: send dummy data to generate clock */
		while (data->tx_count < data->rx_len && ll_func_tx_is_not_full(spi)) {
			if (word_size == 16) {
				LL_SPI_TransmitData16(spi, 0xFFFF);
				data->tx_count += 2;
			} else {
				LL_SPI_TransmitData8(spi, 0xFF);
				data->tx_count++;
			}
		}
	}
}

/* Receive frame - non-FIFO mode (single byte/word) */
static void spi_stm32_shift_rx(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);

	if (!ll_func_rx_is_not_empty(spi)) {
		return;
	}

	if (data->rx_buf != NULL) {
		/* Store received data */
		if (data->rx_count < data->rx_len) {
			if (word_size == 16) {
				uint16_t rx_data = LL_SPI_ReceiveData16(spi);
				((uint16_t *)data->rx_buf)[data->rx_count / 2] = rx_data;
				data->rx_count += 2;
			} else {
				uint8_t rx_data = LL_SPI_ReceiveData8(spi);
				data->rx_buf[data->rx_count] = rx_data;
				data->rx_count++;
			}
		}
	} else {
		/* Discard received data (TX-only) */
		if (word_size == 16) {
			(void)LL_SPI_ReceiveData16(spi);
		} else {
			(void)LL_SPI_ReceiveData8(spi);
		}
	}
}

/* Receive frame - FIFO mode (packet-based) */
static void spi_stm32_shift_rx_fifo(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);

	if (data->rx_buf != NULL) {
		/* Store received data */
		while (data->rx_count < data->rx_len && ll_func_rx_is_not_empty(spi)) {
			if (word_size == 16) {
				uint16_t rx_data = LL_SPI_ReceiveData16(spi);
				((uint16_t *)data->rx_buf)[data->rx_count / 2] = rx_data;
				data->rx_count += 2;
			} else {
				uint8_t rx_data = LL_SPI_ReceiveData8(spi);
				data->rx_buf[data->rx_count] = rx_data;
				data->rx_count++;
			}
		}
	} else {
		/* Discard received data (TX-only) */
		while (ll_func_rx_is_not_empty(spi)) {
			if (word_size == 16) {
				(void)LL_SPI_ReceiveData16(spi);
			} else {
				(void)LL_SPI_ReceiveData8(spi);
			}
		}
	}
}

static inline bool spi_stm32_transmit_buffer(const struct device *dev)
{
	const struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;

	if (cfg->fifo_enabled) {
		spi_stm32_shift_tx_fifo(dev);
	} else {
		spi_stm32_shift_tx(dev);
	}

	/* Check if transmission is complete */
	if (data->tx_buf == NULL) {
		/* RX-only mode: check if we've sent enough dummy bytes */
		return data->tx_count >= data->rx_len;
	} else {
		/* TX mode: check if we've sent all data */
		return data->tx_count >= data->tx_len;
	}
}

static inline bool spi_stm32_receive_buffer(const struct device *dev)
{
	const struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *cfg = dev->config;

	if (cfg->fifo_enabled) {
		spi_stm32_shift_rx_fifo(dev);
	} else {
		spi_stm32_shift_rx(dev);
	}

	/* For TX-only operations, RX is always "complete" since we just discard data */
	if (data->rx_buf == NULL) {
		return true;
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
#ifndef DT_SPI_CTX_HAS_NO_CS_GPIOS
	struct spi_stm32_data *data = dev->data;

	spi_context_cs_control(&data->ctx, on);
#endif

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

#ifndef CONFIG_SPI_RUBUS_STM32_INTERRUPT
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
#endif /* !CONFIG_SPI_RUBUS_STM32_INTERRUPT */

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
	uint32_t clock;
	int br;

	if (spi_context_configured(&data->ctx, config)) {
		/* Already configured - transfer direction will be set in msg_start */
		return 0;
	}

	if ((SPI_WORD_SIZE_GET(config->operation) != 8) &&
	    (SPI_WORD_SIZE_GET(config->operation) != 16)) {
		return -ENOTSUP;
	}

	/* configure the frame format Motorola (default) or TI */
	if ((config->operation & SPI_FRAME_FORMAT_TI) == SPI_FRAME_FORMAT_TI) {
#ifdef LL_SPI_PROTOCOL_TI
		LL_SPI_SetStandard(spi, LL_SPI_PROTOCOL_TI);
#else
		LOG_ERR("Frame Format TI not supported");
		/* on stm32F1 or some stm32L1 (cat1,2) without SPI_CR2_FRF */
		return -ENOTSUP;
#endif
#if defined(LL_SPI_PROTOCOL_MOTOROLA) && defined(SPI_CR2_FRF)
	} else {
		LL_SPI_SetStandard(spi, LL_SPI_PROTOCOL_MOTOROLA);
#endif
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

	for (br = 1 ; br <= ARRAY_SIZE(scaler) ; ++br) {
		uint32_t clk = clock >> br;

		if (clk <= config->frequency) {
			break;
		}
	}

	if (br > ARRAY_SIZE(scaler)) {
		LOG_ERR("Unsupported frequency %uHz, max %uHz, min %uHz",
			config->frequency, clock >> 1, clock >> ARRAY_SIZE(scaler));
		return -EINVAL;
	}

	LL_SPI_Disable(spi);
	LL_SPI_SetBaudRatePrescaler(spi, scaler[br - 1]);

#if defined(SPI_CFG2_IOSWP)
	if (cfg->ioswp) {
		LL_SPI_EnableIOSwap(cfg->spi);
	}
#endif

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);
	} else {
		LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_LOW);
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
	} else {
		LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_1EDGE);
	}

	if ((config->operation & SPI_HALF_DUPLEX) != 0U) {
		if (write) {
			LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_TX);
		} else {
			LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_RX);
		}
	} else {
		LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	}

	if ((config->operation & SPI_TRANSFER_LSB) != 0) {
		LL_SPI_SetTransferBitOrder(spi, LL_SPI_LSB_FIRST);
	} else {
		LL_SPI_SetTransferBitOrder(spi, LL_SPI_MSB_FIRST);
	}

	LL_SPI_DisableCRC(spi);

	if (spi_cs_is_gpio(config) || !IS_ENABLED(CONFIG_SPI_STM32_USE_HW_SS)) {
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
		if ((SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) &&
		    (LL_SPI_GetNSSPolarity(spi) == LL_SPI_NSS_POLARITY_LOW)) {
			LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
		}
#endif
		LL_SPI_SetNSSMode(spi, LL_SPI_NSS_SOFT);
	} else {
		if ((config->operation & SPI_OP_MODE_SLAVE) != 0U) {
			LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_INPUT);
		} else {
			LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_OUTPUT);
		}
	}

	if ((config->operation & SPI_OP_MODE_SLAVE) != 0U) {
		LL_SPI_SetMode(spi, LL_SPI_MODE_SLAVE);
	} else {
		LL_SPI_SetMode(spi, LL_SPI_MODE_MASTER);
	}

	if (SPI_WORD_SIZE_GET(config->operation) == 8) {
		LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_8BIT);
	} else {
		LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_16BIT);
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_SetMasterSSIdleness(spi, cfg->mssi_clocks);
	LL_SPI_SetInterDataIdleness(spi, cfg->midi_clocks << SPI_CFG2_MIDI_Pos);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
	ll_func_set_fifo_threshold_8bit(spi);
#endif

	/* At this point, it's mandatory to set this on the context! */
	data->ctx.config = config;

	LOG_DBG("Installed config %p: freq %uHz (div = %u), mode %u/%u/%u, slave %u",
		config, clock >> br, 1 << br,
		(SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) ? 1 : 0,
		(SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 1 : 0,
		(SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) ? 1 : 0,
		config->slave);

	return 0;
}

/* Common SPI hardware setup - called by all transfer modes */
static void spi_stm32_msg_start(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	const struct spi_config *config = data->ctx.config;

	/* Determine transfer direction based on buffers */
	if (data->tx_buf && data->rx_buf) {
		LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	} else if (data->tx_buf) {
		LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_TX);
	} else {
		LL_SPI_SetTransferDirection(spi, LL_SPI_HALF_DUPLEX_RX);
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* Configure FIFO threshold for H7 based on transfer mode */
	uint8_t word_size = SPI_WORD_SIZE_GET(config->operation);
	
	if (cfg->fifo_enabled) {
		LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);
		
		/* Set transfer size for H7 when FIFO is enabled and in master mode */
		if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
			uint32_t total_frames = spi_stm32_compute_total_frames(data);
			if (total_frames > 0U) {
				LL_SPI_SetTransferSize(spi, total_frames);
			}
		}
	} else {
		/* FIFO disabled: set appropriate threshold for polling/interrupt without FIFO */
		if (word_size > 8) {
			LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_02DATA);
		} else {
			LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);
		}
	}
#endif

	/* Enable SPI peripheral */
	LL_SPI_Enable(spi);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* Start master transfer for H7/MP1/U5 */
	if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
		LL_SPI_StartMasterTransfer(spi);
		while (!LL_SPI_IsActiveMasterTransfer(spi)) {
			/* NOP */
		}
	}
#endif

#ifdef CONFIG_SOC_SERIES_STM32H7X
	/*
	 * Add a small delay after enabling to prevent transfer stalling at high
	 * system clock frequency (see errata sheet ES0392).
	 */
	k_busy_wait(1U);
#endif /* CONFIG_SOC_SERIES_STM32H7X */

	/* Assert CS - will be deasserted in spi_stm32_iodev_complete() */
	spi_stm32_cs_control(dev, true);
}

#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
static int spi_stm32_transceive_interrupt(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;

	/* Reset counters for interrupt mode */
	data->tx_count = 0;
	data->rx_count = 0;
	data->error_status = 0;

	/* Common hardware setup */
	spi_stm32_msg_start(dev);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* Enable EOT interrupt for H7 - only when FIFO is enabled */
	if (cfg->fifo_enabled) {
		data->waiting_eot = true;
		data->status_flags = 0;
		LL_SPI_ClearFlag_EOT(spi);
		ll_func_enable_int_eot(spi);
	} else {
		data->waiting_eot = false;
	}
#endif

	/* Pre-fill TX FIFO to start transfer - this will fill as much as possible */
	spi_stm32_transmit_buffer(dev);

	/* Enable interrupts */
	ll_func_enable_int_errors(spi);
	ll_func_enable_int_rx_not_empty(spi);
	ll_func_enable_int_tx_empty(spi);

	return 0; /* Non-blocking - completion handled in ISR */
}

#endif /* CONFIG_SPI_RUBUS_STM32_INTERRUPT */

#ifdef CONFIG_SPI_RUBUS_STM32_DMA
static int spi_stm32_transceive_dma(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	size_t dma_len_tx, dma_len_rx;
	int ret;

	/* Determine DMA lengths based on transfer type */
	if (data->tx_buf && data->rx_buf) {
		dma_len_tx = data->tx_len;
		dma_len_rx = data->rx_len;
	} else if (data->tx_buf) {
		dma_len_tx = data->tx_len;
		dma_len_rx = data->tx_len; /* RX DMA needs to run to discard data */
	} else {
		dma_len_tx = data->rx_len; /* TX DMA needs to run to generate clock */
		dma_len_rx = data->rx_len;
	}

	/* Reset DMA status flags */
	data->status_flags = 0;
	data->dma_active = true;
	data->error_status = 0;

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* Only enable EOT interrupt when FIFO is used and interrupts are enabled */
	if (IS_ENABLED(CONFIG_SPI_RUBUS_STM32_INTERRUPT) && cfg->fifo_enabled) {
		data->waiting_eot = true;
		LL_SPI_ClearFlag_EOT(spi);
		ll_func_enable_int_eot(spi);
	} else {
		data->waiting_eot = false;
	}
#endif

	/* Start RX DMA first (peripheral -> memory) */
	if (data->rx_buf || !data->tx_buf) {
		/* Use real RX buffer for RXONLY/TXRX, or dummy buffer for TXONLY */
		uint8_t *rx_buf = data->rx_buf;
		if (data->tx_buf && !data->rx_buf) {
			rx_buf = NULL; /* TXONLY - discard received data */
		}

		ret = spi_stm32_dma_rx_load(dev, rx_buf, dma_len_rx);
		if (ret != 0) {
			LOG_ERR("Failed to start RX DMA: %d", ret);
			return ret;
		}
	}

	/* Start TX DMA second (memory -> peripheral) */
	if (data->tx_buf || !data->rx_buf) {
		/* Use real TX buffer for TXONLY/TXRX, or dummy buffer for RXONLY */
		const uint8_t *tx_buf = data->tx_buf;
		if (!data->tx_buf && data->rx_buf) {
			tx_buf = NULL; /* RXONLY - send dummy data to generate clock */
		}

		ret = spi_stm32_dma_tx_load(dev, tx_buf, dma_len_tx);
		if (ret != 0) {
			LOG_ERR("Failed to start TX DMA: %d", ret);
			/* Stop RX DMA if it was started */
			if (data->rx_buf || !data->tx_buf) {
				dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
			}
			return ret;
		}
	}

	/* Enable SPI DMA requests */
	if (data->rx_buf || !data->tx_buf) {
		LL_SPI_EnableDMAReq_RX(spi);
	}
	if (data->tx_buf || !data->rx_buf) {
		LL_SPI_EnableDMAReq_TX(spi);
	}

	/* Common hardware setup (enables SPI and starts master transfer) */
	spi_stm32_msg_start(dev);

	return 0; /* Non-blocking - completion handled via DMA callbacks */
}
#endif /* CONFIG_SPI_RUBUS_STM32_DMA */

#ifndef CONFIG_SPI_RUBUS_STM32_INTERRUPT
static int spi_stm32_transceive_polling(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	uint32_t timeout = 100000;

	/* Reset counters for polling mode */
	data->tx_count = 0;
	data->rx_count = 0;

	/* Common hardware setup */
	spi_stm32_msg_start(dev);

	/* Main transfer loop */
	while ((data->tx_count < (data->tx_buf ? data->tx_len : data->rx_len)) ||
	       (data->rx_count < (data->rx_buf ? data->rx_len : 0))) {

		/* Check for errors using unified handler */
		int err = spi_stm32_handle_errors(spi);
		if (err) {
			return err;
		}

		spi_stm32_transmit_buffer(dev);
		spi_stm32_receive_buffer(dev);

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
#endif /* !CONFIG_SPI_RUBUS_STM32_INTERRUPT */

static void spi_stm32_iodev_msg_start(const struct device *dev,
				      const uint8_t *tx_buf, uint8_t *rx_buf,
				      size_t tx_len, size_t rx_len)
{
	struct spi_stm32_data *data = dev->data;
	int ret = 0;

	/* Setup buffer pointers and lengths */
	data->tx_buf = tx_buf;
	data->tx_len = tx_len;
	data->rx_buf = rx_buf;
	data->rx_len = rx_len;
	data->error_status = 0;
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
	data->status_flags = 0;
	data->dma_active = false;
#endif

	/* Determine transfer mode: DMA -> Interrupt -> Polling (in order of preference) */
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
	/* Check transfer size first - only proceed with DMA checks for larger transfers */
	size_t total_len = MAX(tx_len, rx_len);

	if (total_len >= CONFIG_SPI_RUBUS_STM32_DMA_MIN_LEN) {
		/* Large transfer - check if DMA is available and buffers are DMA-safe */
		bool use_dma = false;

		if (data->dma_tx.dma_dev && data->dma_rx.dma_dev) {
			use_dma = true;

#ifdef CONFIG_DCACHE
			if (IS_ENABLED(CONFIG_DCACHE)) {
				bool tx_cached = (tx_buf != NULL) && (tx_len > 0) &&
						!stm32_buf_in_nocache((uintptr_t)tx_buf, tx_len);
				bool rx_cached = (rx_buf != NULL) && (rx_len > 0) &&
						!stm32_buf_in_nocache((uintptr_t)rx_buf, rx_len);

				if (tx_cached || rx_cached) {
					LOG_DBG("DMA buffers cached - performing cache maintenance");
				}
			}
#endif
		}

		if (use_dma) {
			ret = spi_stm32_transceive_dma(dev);
			if (ret != 0) {
				spi_stm32_iodev_complete(dev, ret);
			}
			/* DMA mode is non-blocking - completion happens via callback */
			return;
		}
	}
#endif /* CONFIG_SPI_RUBUS_STM32_DMA */

#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
	/* Use interrupt mode for non-DMA transfers */
	ret = spi_stm32_transceive_interrupt(dev);
	if (ret != 0) {
		spi_stm32_iodev_complete(dev, ret);
	}
	/* Interrupt mode completion happens in ISR - return without completing */
	return;
#else
	/* Use polling mode */
	ret = spi_stm32_transceive_polling(dev);
	spi_stm32_iodev_complete(dev, ret);
#endif
}

static void spi_stm32_iodev_prepare_start(const struct device *dev)
{
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
}

static void spi_stm32_iodev_start(const struct device *dev)
{
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_curr->sqe;

	/* Extract buffer information and start transfer based on operation type */
	switch (sqe->op) {
	case RTIO_OP_RX:
		/* RX-only: receive data while sending dummy bytes */
		spi_stm32_iodev_msg_start(dev, NULL, sqe->rx.buf, 0, sqe->rx.buf_len);
		break;

	case RTIO_OP_TX:
		/* TX-only: send data while discarding received bytes */
		spi_stm32_iodev_msg_start(dev, sqe->tx.buf, NULL, sqe->tx.buf_len, 0);
		break;

	case RTIO_OP_TINY_TX:
		/* Small TX-only: send data while discarding received bytes */
		spi_stm32_iodev_msg_start(dev, sqe->tiny_tx.buf, NULL, sqe->tiny_tx.buf_len, 0);
		break;

	case RTIO_OP_TXRX:
		/* Full-duplex: simultaneous send and receive */
		spi_stm32_iodev_msg_start(dev, sqe->txrx.tx_buf, sqe->txrx.rx_buf,
					  sqe->txrx.buf_len, sqe->txrx.buf_len);
		break;

	default:
		LOG_ERR("Unsupported RTIO operation: %d", sqe->op);
		spi_stm32_iodev_complete(dev, -EINVAL);
		break;
	}
}

static void spi_stm32_iodev_complete(const struct device *dev, int status)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;

	/* Clean up DMA if it was used */
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
	if (data->dma_tx.dma_dev != NULL && data->dma_rx.dma_dev != NULL) {
		/* Disable SPI DMA requests */
		LL_SPI_DisableDMAReq_TX(cfg->spi);
		LL_SPI_DisableDMAReq_RX(cfg->spi);

		/* Stop DMA channels (safe to call even if not running) */
		dma_stop(data->dma_tx.dma_dev, data->dma_tx.channel);
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.channel);
	}

#if defined(CONFIG_DCACHE)
	if (IS_ENABLED(CONFIG_DCACHE) &&
	    (data->status_flags & SPI_STM32_DMA_RX_DONE_FLAG) != 0 &&
	    data->rx_buf != NULL && data->rx_len > 0 &&
	    !stm32_buf_in_nocache((uintptr_t)data->rx_buf, data->rx_len)) {
		sys_cache_data_invd_range(data->rx_buf, data->rx_len);
	}
#endif /* CONFIG_DCACHE */
#endif

#ifdef CONFIG_SPI_RUBUS_STM32_DMA
	data->dma_active = false;
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	data->waiting_eot = false;
#endif

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

/* SPI driver API structure */
static const struct spi_driver_api spi_stm32_api = {
	.transceive = spi_stm32_transceive,
	.release = spi_stm32_release,
	.iodev_submit = spi_stm32_iodev_submit,
};

static bool spi_stm32_is_subghzspi(const struct device *dev)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
	const struct spi_stm32_config *cfg = dev->config;

	return cfg->use_subghzspi_nss;
#else
	ARG_UNUSED(dev);
	return false;
#endif /* st_stm32_spi_subghz */
}

static int spi_stm32_pinctrl_apply(const struct device *dev, uint8_t id)
{
	const struct spi_stm32_config *config = dev->config;
	int err;

	if (spi_stm32_is_subghzspi(dev)) {
		return 0;
	}

	/* Move pins to requested state */
	err = pinctrl_apply_state(config->pcfg, id);
	if ((id == PINCTRL_STATE_SLEEP) && (err == -ENOENT)) {
		/* Sleep state is optional */
		err = 0;
	}
	return err;
}

static int spi_stm32_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	struct spi_stm32_data *data = dev->data;
	const struct spi_stm32_config *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Configure pins for active mode */
		err = spi_stm32_pinctrl_apply(dev, PINCTRL_STATE_DEFAULT);
		if (err < 0) {
			return err;
		}
		/* enable clock */
		err = clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]);
		if (err != 0) {
			LOG_ERR("Could not enable SPI clock");
			return err;
		}
		/* (re-)init SPI context and all CS configuration */
		err = spi_context_cs_configure_all(&data->ctx);
		if (err < 0) {
			return err;
		}
		spi_context_unlock_unconditionally(&data->ctx);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Stop device clock. */
		err = clock_control_off(clk, (clock_control_subsys_t)&config->pclken[0]);
		if (err != 0) {
			LOG_ERR("Could not disable SPI clock");
			return err;
		}
		/* Configure pins for sleep mode */
		return spi_stm32_pinctrl_apply(dev, PINCTRL_STATE_SLEEP);
	case PM_DEVICE_ACTION_TURN_ON:
		/* Configure pins for sleep mode */
		return spi_stm32_pinctrl_apply(dev, PINCTRL_STATE_SLEEP);
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/* Device initialization */
static int spi_stm32_init(const struct device *dev)
{
	const struct spi_stm32_config *cfg = dev->config;
	struct spi_stm32_data *data = dev->data;
	int ret;

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (IS_ENABLED(STM32_SPI_DOMAIN_CLOCK_SUPPORT) && (cfg->pclk_len > 1)) {
		ret = clock_control_configure(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					      (clock_control_subsys_t) &cfg->pclken[1],
					      NULL);
		if (ret < 0) {
			LOG_ERR("Could not select SPI domain clock");
			return ret;
		}
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

	/* Configure interrupts if enabled */
#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
	if (cfg->irq_config_func) {
		cfg->irq_config_func(dev);
	}
#endif

	/* Initialize DMA if enabled */
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
	if ((data->dma_rx.dma_dev != NULL) && !device_is_ready(data->dma_rx.dma_dev)) {
		LOG_ERR("%s device not ready", data->dma_rx.dma_dev->name);
		return -ENODEV;
	}

	if ((data->dma_tx.dma_dev != NULL) && !device_is_ready(data->dma_tx.dma_dev)) {
		LOG_ERR("%s device not ready", data->dma_tx.dma_dev->name);
		return -ENODEV;
	}

	LOG_DBG("SPI with DMA transfer");

#endif /* CONFIG_SPI_RUBUS_STM32_DMA */

	/* Initialize RTIO context */
	spi_rtio_init(data->rtio_ctx, dev);

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

	return pm_device_driver_init(dev, spi_stm32_pm_action);
}

static inline bool spi_stm32_error_irq_enabled(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	bool enabled = LL_SPI_IsEnabledIT_OVR(spi) || LL_SPI_IsEnabledIT_MODF(spi) ||
		       LL_SPI_IsEnabledIT_CRCERR(spi) || LL_SPI_IsEnabledIT_FRE(spi);
#ifdef LL_SPI_IsEnabledIT_UDR
	enabled = enabled || LL_SPI_IsEnabledIT_UDR(spi);
#endif
	return enabled;
#elif defined(LL_SPI_IsEnabledIT_ERR)
	return LL_SPI_IsEnabledIT_ERR(spi);
#else
	return true;
#endif
}

/* Unified error handler - checks and clears SPI error flags based on SPI_STM32_ERR_MSK */
static inline int spi_stm32_handle_errors(SPI_TypeDef *spi)
{
	uint32_t sr = LL_SPI_ReadReg(spi, SR);

	if (!(sr & SPI_STM32_ERR_MSK)) {
		return 0;
	}

	LOG_ERR("SPI error detected: SR=0x%08x", sr);

	/* Clear error flags based on what's in SPI_STM32_ERR_MSK */
	if (sr & LL_SPI_SR_OVR) {
		LL_SPI_ClearFlag_OVR(spi);
		LOG_ERR("SPI overrun error");
	}
	if (sr & LL_SPI_SR_MODF) {
		LL_SPI_ClearFlag_MODF(spi);
		LOG_ERR("SPI mode fault error");
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* H7 uses CRCE flag (not CRCERR) */
	if (sr & LL_SPI_SR_CRCE) {
		LL_SPI_ClearFlag_CRCERR(spi);
		LOG_ERR("SPI CRC error");
	}
	/* H7 has TIFRE (TI Frame Error) flag */
	if (sr & LL_SPI_SR_TIFRE) {
		LL_SPI_ClearFlag_FRE(spi);
		LOG_ERR("SPI TI frame error");
	}
	/* H7 has UDR (Underrun) flag */
	if (sr & LL_SPI_SR_UDR) {
		/* UDR cannot be cleared, only by disabling SPI */
		LOG_ERR("SPI underrun error");
	}
#else
	/* Non-H7 platforms use CRCERR flag */
	if (sr & LL_SPI_SR_CRCERR) {
		LL_SPI_ClearFlag_CRCERR(spi);
		LOG_ERR("SPI CRC error");
	}
#if defined(LL_SPI_SR_UDR)
	/* Some platforms have UDR flag */
	if (sr & LL_SPI_SR_UDR) {
		LOG_ERR("SPI underrun error");
	}
#endif
#if defined(LL_SPI_SR_FRE)
	/* Some platforms have FRE flag */
	if (sr & LL_SPI_SR_FRE) {
		LL_SPI_ClearFlag_FRE(spi);
		LOG_ERR("SPI frame error");
	}
#endif
#endif

	return -EIO;
}

#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
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
		LOG_DBG("SPI interrupt received while SPI is disabled: %p", spi);
		return;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* Handle EOT interrupt on H7 */
	if (LL_SPI_IsActiveFlag_EOT(spi)) {
		LL_SPI_ClearFlag_EOT(spi);
		ll_func_disable_int_eot(spi);
		data->status_flags |= SPI_STM32_EOT_DONE_FLAG;
		/* Note: Don't complete yet - let finalize check all conditions */
	}
#endif

	/* Check for errors using unified handler (only if error interrupts enabled) */
	if (spi_stm32_error_irq_enabled(spi)) {
		rc = spi_stm32_handle_errors(spi);
		if (rc) {
			data->error_status = rc;
			goto complete;
		}
	}

	tx_complete = spi_stm32_handle_tx_interrupt(dev);
	if (tx_complete) {
		ll_func_disable_int_tx_empty(spi);
	}

	rx_complete = spi_stm32_handle_rx_interrupt(dev);
	if (rx_complete) {
		ll_func_disable_int_rx_not_empty(spi);
	}

	if (!tx_complete || !rx_complete) {
		return;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	/* On H7, wait for EOT to ensure transfer is truly complete */
	if (data->waiting_eot && (data->status_flags & SPI_STM32_EOT_DONE_FLAG) == 0) {
		return; /* Wait for EOT interrupt */
	}
#endif

	rc = 0;
complete:
	ll_func_disable_int_tx_empty(spi);
	ll_func_disable_int_rx_not_empty(spi);
	ll_func_disable_int_errors(spi);
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	ll_func_disable_int_eot(spi);
	/* Clear any pending interrupt flags on H7 to prevent spurious interrupts */
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
#endif
	spi_stm32_iodev_complete(dev, rc);

	return;
}
#endif

/* Device tree macros for driver instantiation */
#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
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

#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
#define SPI_STM32_CONFIG_IRQ_FUNC(n) .irq_config_func = spi_stm32_irq_config_func_##n,
#else
#define SPI_STM32_CONFIG_IRQ_FUNC(n)
#endif

#define SPI_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)	\
	.dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(index, dir)),		\
	.channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),	\
	.dma_cfg = {							\
		.dma_slot = STM32_DMA_SLOT(index, dir, slot),		\
		.channel_direction = STM32_DMA_CONFIG_DIRECTION(	\
					STM32_DMA_CHANNEL_CONFIG(index, dir)), \
		.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE( \
					STM32_DMA_CHANNEL_CONFIG(index, dir)), \
		.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE( \
				STM32_DMA_CHANNEL_CONFIG(index, dir)),	\
		.source_burst_length = 1, /* SINGLE transfer */	\
		.dest_burst_length = 1, /* SINGLE transfer */		\
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(		\
					STM32_DMA_CHANNEL_CONFIG(index, dir)), \
		.dma_callback = dma_callback,				\
		.block_count = 2,					\
	},								\
	.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(	\
				STM32_DMA_CHANNEL_CONFIG(index, dir)),	\
	.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(	\
				STM32_DMA_CHANNEL_CONFIG(index, dir)),	\
	.fifo_threshold = STM32_DMA_FEATURES_FIFO_THRESHOLD(		\
				STM32_DMA_FEATURES(index, dir)),

/* DMA channel initialization - exactly copying upstream pattern */
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
#define SPI_DMA_CHANNEL(id, dir, DIR, src, dest)			\
	.dma_##dir = {							\
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(id, dir),		\
			(SPI_DMA_CHANNEL_INIT(id, dir, DIR, src, dest)),\
			(NULL))						\
		},
#else
#define SPI_DMA_CHANNEL(id, dir, DIR, src, dest)
#endif /* CONFIG_SPI_RUBUS_STM32_DMA */

/* Device instantiation macro */
#define STM32_SPI_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);						\
	SPI_RTIO_DEFINE(stm32_spi_rtio_##n,					\
			CONFIG_SPI_RUBUS_STM32_RTIO_SQ_SIZE,				\
			CONFIG_SPI_RUBUS_STM32_RTIO_CQ_SIZE);				\
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
		.ioswp = DT_INST_PROP(n, ioswp),				\
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
		SPI_DMA_CHANNEL(n, rx, RX, PERIPHERAL, MEMORY)			\
		SPI_DMA_CHANNEL(n, tx, TX, MEMORY, PERIPHERAL)			\
	};									\
	PM_DEVICE_DT_INST_DEFINE(n, spi_stm32_pm_action);			\
	DEVICE_DT_INST_DEFINE(n, spi_stm32_init, PM_DEVICE_DT_INST_GET(n),	\
			      &spi_stm32_data_##n, &spi_stm32_config_##n,	\
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
			      &spi_stm32_api);

/* Instantiate all DT_DRV_COMPAT devices */
DT_INST_FOREACH_STATUS_OKAY(STM32_SPI_INIT)
