/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_LL_STM32_H_
#define ZEPHYR_DRIVERS_SPI_SPI_LL_STM32_H_

#include "spi_context.h"
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

typedef void (*irq_config_func_t)(const struct device *port);

/* This symbol takes the value 1 if one of the device instances */
/* is configured in dts with a domain clock */
#if STM32_DT_INST_DEV_DOMAIN_CLOCK_SUPPORT
#define STM32_SPI_DOMAIN_CLOCK_SUPPORT 1
#else
#define STM32_SPI_DOMAIN_CLOCK_SUPPORT 0
#endif

/* DMA constants - needed even when DMA not enabled for helper functions */
#define SPI_STM32_DMA_TX	0x01
#define SPI_STM32_DMA_RX	0x02

#ifdef CONFIG_SPI_RUBUS_STM32_DMA
struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma or mux */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
	bool configured;
	uint16_t last_src_adj;
	uint16_t last_dst_adj;
	const void *last_checked_buf;
	size_t last_checked_len;
};

/* DMA status flags */
#define SPI_STM32_DMA_ERROR_FLAG	0x01
#define SPI_STM32_DMA_RX_DONE_FLAG	0x02
#define SPI_STM32_DMA_TX_DONE_FLAG	0x04
#define SPI_STM32_DMA_DONE_FLAG	\
	(SPI_STM32_DMA_RX_DONE_FLAG | SPI_STM32_DMA_TX_DONE_FLAG)

/* DMA register address helper - matches upstream */
static inline uint32_t ll_func_dma_get_reg_addr(SPI_TypeDef *spi, uint32_t location)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	if (location == SPI_STM32_DMA_TX) {
		/* use direct register location until the LL_SPI_DMA_GetTxRegAddr exists */
		return (uint32_t)&(spi->TXDR);
	}
	/* use direct register location until the LL_SPI_DMA_GetRxRegAddr exists */
	return (uint32_t)&(spi->RXDR);
#else
	ARG_UNUSED(location);
	return (uint32_t)LL_SPI_DMA_GetRegAddr(spi);
#endif /* st_stm32h7_spi */
}
#endif /* CONFIG_SPI_RUBUS_STM32_DMA */

/* H7 EOT flag - needed for both DMA and interrupt modes */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
#define SPI_STM32_EOT_DONE_FLAG	0x08
#endif

/* STM32 SPI configuration structure */
struct spi_stm32_config {
	SPI_TypeDef *spi;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_RUBUS_STM32_INTERRUPT
	irq_config_func_t irq_config_func;
#endif
#ifdef CONFIG_SOC_SERIES_STM32H7X
	uint32_t irq_line;
#endif /* CONFIG_SOC_SERIES_STM32H7X */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_subghz)
	bool use_subghzspi_nss;
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	int midi_clocks;
	int mssi_clocks;
#endif
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	bool fifo_enabled: 1;
	bool ioswp: 1;
	/* Note: DMA configuration comes from device tree macros, not stored in config */
};

/* STM32 SPI data structure - pure RTIO */
struct spi_stm32_data {
	struct spi_rtio *rtio_ctx;
	struct spi_context ctx;
	
	/* STM32-specific state */
	volatile int error_status;

	/* DMA state */
#ifdef CONFIG_SPI_RUBUS_STM32_DMA
	volatile uint32_t status_flags;
	struct stream dma_rx;
	struct stream dma_tx;
	bool dma_active;
#endif
	bool pm_policy_state_on;

	/* H7 EOT handling (needed for both DMA and interrupt modes) */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	volatile uint32_t status_flags;  /* Also used for interrupt mode EOT tracking */
	bool waiting_eot;
#endif
	
	/* Current transaction state for polling mode */
	const uint8_t *tx_buf;
	uint8_t *rx_buf;
	size_t tx_len;
	size_t rx_len;
	size_t tx_count;
	size_t rx_count;
};

/* STM32-specific helper functions following upstream ll_func pattern */
static inline uint32_t ll_func_tx_is_not_full(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	return LL_SPI_IsActiveFlag_TXP(spi);
#else
	return LL_SPI_IsActiveFlag_TXE(spi);
#endif
}

static inline uint32_t ll_func_rx_is_not_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	return LL_SPI_IsActiveFlag_RXP(spi);
#else
	return LL_SPI_IsActiveFlag_RXNE(spi);
#endif
}

static inline void ll_func_enable_int_tx_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_EnableIT_TXP(spi);
#else
	LL_SPI_EnableIT_TXE(spi);
#endif
}

static inline void ll_func_enable_int_rx_not_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_EnableIT_RXP(spi);
#else
	LL_SPI_EnableIT_RXNE(spi);
#endif
}

static inline void ll_func_enable_int_errors(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_EnableIT_UDR(spi);
	LL_SPI_EnableIT_OVR(spi);
	LL_SPI_EnableIT_CRCERR(spi);
	LL_SPI_EnableIT_FRE(spi);
	LL_SPI_EnableIT_MODF(spi);
#else
	LL_SPI_EnableIT_ERR(spi);
#endif
}

static inline void ll_func_disable_int_tx_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_DisableIT_TXP(spi);
#else
	LL_SPI_DisableIT_TXE(spi);
#endif
}

static inline void ll_func_disable_int_rx_not_empty(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_DisableIT_RXP(spi);
#else
	LL_SPI_DisableIT_RXNE(spi);
#endif
}

static inline void ll_func_disable_int_errors(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_DisableIT_UDR(spi);
	LL_SPI_DisableIT_OVR(spi);
	LL_SPI_DisableIT_CRCERR(spi);
	LL_SPI_DisableIT_FRE(spi);
	LL_SPI_DisableIT_MODF(spi);
#else
	LL_SPI_DisableIT_ERR(spi);
#endif
}

static inline bool ll_func_are_int_disabled(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	bool enabled = LL_SPI_IsEnabledIT_TXP(spi) || LL_SPI_IsEnabledIT_RXP(spi) ||
		       LL_SPI_IsEnabledIT_UDR(spi) || LL_SPI_IsEnabledIT_OVR(spi) ||
		       LL_SPI_IsEnabledIT_CRCERR(spi) || LL_SPI_IsEnabledIT_FRE(spi) ||
		       LL_SPI_IsEnabledIT_MODF(spi) || LL_SPI_IsEnabledIT_EOT(spi);
#else
	bool enabled = LL_SPI_IsEnabledIT_ERR(spi) || LL_SPI_IsEnabledIT_TXE(spi) || 
		       LL_SPI_IsEnabledIT_RXNE(spi);
#endif
	return !enabled;
}

static inline void ll_func_enable_int_eot(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_EnableIT_EOT(spi);
#else
	ARG_UNUSED(spi);
#endif
}

static inline void ll_func_disable_int_eot(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_DisableIT_EOT(spi);
#else
	ARG_UNUSED(spi);
#endif
}

static inline uint32_t ll_func_spi_is_busy(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	if (LL_SPI_GetTransferSize(spi) == 0) {
		return LL_SPI_IsActiveFlag_TXC(spi) == 0;
	} else {
		return LL_SPI_IsActiveFlag_EOT(spi) == 0;
	}
#else
	return LL_SPI_IsActiveFlag_BSY(spi);
#endif
}

/* Header is compiled first, this switch avoid the compiler to lookup for
 * non-existing LL FIFO functions for SoC without SPI FIFO
 */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
static inline void ll_func_set_fifo_threshold_8bit(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);
#else
	LL_SPI_SetRxFIFOThreshold(spi, LL_SPI_RX_FIFO_TH_QUARTER);
#endif /* st_stm32h7_spi */
}

static inline void ll_func_set_fifo_threshold_16bit(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_02DATA);
#else
	LL_SPI_SetRxFIFOThreshold(spi, LL_SPI_RX_FIFO_TH_HALF);
#endif /* st_stm32h7_spi */
}
#endif /* st_stm32_spi_fifo */

/* STM32 SPI busy flag errata workaround */
static inline bool ll_func_transfer_ongoing(SPI_TypeDef *spi)
{
#ifdef CONFIG_SPI_STM32_ERRATA_BUSY
	/* 
	 * Some STM32 chips have errata where busy flag is unreliable.
	 * Use TX/RX buffer status instead for these cases.
	 */
	return (ll_func_tx_is_not_full(spi) == false) || 
	       (ll_func_rx_is_not_empty(spi) == true);
#else
	return ll_func_spi_is_busy(spi);
#endif
}

static inline void ll_func_disable_spi(SPI_TypeDef *spi)
{
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo)
	/* Flush RX buffer */
	while (ll_func_rx_is_not_empty(spi)) {
		(void) LL_SPI_ReceiveData8(spi);
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32_spi_fifo) */

	LL_SPI_Disable(spi);

	while (LL_SPI_IsEnabled(spi)) {
		/* NOP */
	}
}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_spi)
static inline uint32_t spi_stm32_bytes_per_frame(const struct spi_config *config)
{
	return SPI_WORD_SIZE_GET(config->operation) / 8U;
}

static inline uint32_t spi_stm32_count_frames(uint32_t len_bytes, const struct spi_config *config)
{
	uint32_t bytes_per_frame = spi_stm32_bytes_per_frame(config);

	if (bytes_per_frame == 0U) {
		return 0U;
	}

	return len_bytes / bytes_per_frame;
}

static inline uint32_t spi_stm32_compute_total_frames(const struct spi_stm32_data *data)
{
	const struct spi_config *config = data->ctx.config;
	uint32_t tx_frames = spi_stm32_count_frames(data->tx_len, config);
	uint32_t rx_frames = spi_stm32_count_frames(data->rx_len, config);

	if (tx_frames == 0U && rx_frames == 0U) {
		return 0U;
	}

	return MAX(tx_frames, rx_frames);
}
#endif /* st_stm32h7_spi */

#endif	/* ZEPHYR_DRIVERS_SPI_SPI_LL_STM32_H_ */
