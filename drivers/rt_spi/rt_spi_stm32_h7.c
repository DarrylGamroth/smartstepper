/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rubus_stm32_rt_spi

#include <errno.h>
#include <string.h>

#include <soc.h>
#include <stm32_ll_spi.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <drivers/rt_spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rt_spi_stm32_h7, CONFIG_LOG_DEFAULT_LEVEL);

typedef void (*rt_spi_irq_config_func_t)(const struct device *dev);

enum rt_spi_stm32_cs_mode {
	RT_SPI_STM32_CS_HARDWARE = 0,
	RT_SPI_STM32_CS_GPIO,
	RT_SPI_STM32_CS_SOFTWARE,
};

struct rt_spi_stm32_config {
	SPI_TypeDef *spi;
	const struct pinctrl_dev_config *pcfg;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	struct gpio_dt_spec cs_gpio;
	rt_spi_irq_config_func_t irq_config;
	uint32_t frequency;
	uint8_t max_frame_len;
	bool cpol;
	bool cpha;
	bool fifo_enabled;
	bool has_cs_gpio;
	enum rt_spi_stm32_cs_mode cs_mode;
	uint32_t mssi_clocks;
	uint32_t midi_clocks;
};

struct rt_spi_stm32_data {
	volatile bool active;
	volatile bool sample_ready;
	uint8_t tx[RT_SPI_MAX_FRAME_BYTES];
	uint8_t rx[RT_SPI_MAX_FRAME_BYTES];
	uint8_t sample_raw[RT_SPI_MAX_FRAME_BYTES];
	uint8_t len;
	uint8_t tx_count;
	uint8_t rx_count;
	uint16_t sample_flags;
	uint32_t start_cycles;
	struct rt_spi_stats stats;
};

static void rt_spi_stm32_cs_control(const struct rt_spi_stm32_config *cfg, bool active)
{
	if (cfg->has_cs_gpio) {
		(void)gpio_pin_set_dt(&cfg->cs_gpio, active ? 1 : 0);
	}
}

static inline uint32_t rt_spi_stm32_cycle_get(void)
{
	return k_cycle_get_32();
}

static void rt_spi_stm32_disable_irqs(SPI_TypeDef *spi)
{
	LL_SPI_DisableIT_TXP(spi);
	LL_SPI_DisableIT_RXP(spi);
	LL_SPI_DisableIT_EOT(spi);
	LL_SPI_DisableIT_UDR(spi);
	LL_SPI_DisableIT_OVR(spi);
	LL_SPI_DisableIT_CRCERR(spi);
	LL_SPI_DisableIT_FRE(spi);
	LL_SPI_DisableIT_MODF(spi);
}

static void rt_spi_stm32_enable_irqs(SPI_TypeDef *spi)
{
	LL_SPI_EnableIT_UDR(spi);
	LL_SPI_EnableIT_OVR(spi);
	LL_SPI_EnableIT_CRCERR(spi);
	LL_SPI_EnableIT_FRE(spi);
	LL_SPI_EnableIT_MODF(spi);
	LL_SPI_EnableIT_RXP(spi);
	LL_SPI_EnableIT_TXP(spi);
	LL_SPI_EnableIT_EOT(spi);
}

static uint16_t rt_spi_stm32_collect_error_flags(SPI_TypeDef *spi)
{
	uint32_t sr = LL_SPI_ReadReg(spi, SR);
	uint16_t flags = 0U;

	if ((sr & LL_SPI_SR_OVR) != 0U) {
		flags |= RT_SPI_RESULT_OVR;
		LL_SPI_ClearFlag_OVR(spi);
	}
	if ((sr & LL_SPI_SR_MODF) != 0U) {
		flags |= RT_SPI_RESULT_MODF;
		LL_SPI_ClearFlag_MODF(spi);
	}
	if ((sr & LL_SPI_SR_CRCE) != 0U) {
		flags |= RT_SPI_RESULT_CRCE;
		LL_SPI_ClearFlag_CRCERR(spi);
	}
	if ((sr & LL_SPI_SR_TIFRE) != 0U) {
		flags |= RT_SPI_RESULT_TIFRE;
		LL_SPI_ClearFlag_FRE(spi);
	}
	if ((sr & LL_SPI_SR_UDR) != 0U) {
		flags |= RT_SPI_RESULT_UDR;
	}

	return flags;
}

static void rt_spi_stm32_drain_rx_fifo(const struct device *dev)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;

	while ((data->rx_count < data->len) && LL_SPI_IsActiveFlag_RXP(spi)) {
		data->rx[data->rx_count] = LL_SPI_ReceiveData8(spi);
		data->rx_count++;
	}
}

static void rt_spi_stm32_fill_tx_fifo(const struct device *dev)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;

	while ((data->tx_count < data->len) && LL_SPI_IsActiveFlag_TXP(spi)) {
		LL_SPI_TransmitData8(spi, data->tx[data->tx_count]);
		data->tx_count++;
	}
}

static void rt_spi_stm32_publish(const struct device *dev, uint16_t flags)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	uint32_t elapsed_cycles = rt_spi_stm32_cycle_get() - data->start_cycles;
	unsigned int key;

	rt_spi_stm32_disable_irqs(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_SetTransferSize(spi, 0U);
	LL_SPI_Disable(spi);
	rt_spi_stm32_cs_control(cfg, false);

	key = irq_lock();
	if (data->sample_ready) {
		flags |= RT_SPI_RESULT_OVERRUN;
		data->stats.overrun_count++;
	}

	memcpy(data->sample_raw, data->rx, data->len);
	data->sample_flags = flags;
	data->sample_ready = true;
	data->active = false;
	data->stats.complete_count++;
	if ((flags & RT_SPI_RESULT_ERROR) != 0U) {
		data->stats.error_count++;
		data->stats.last_error_flags = flags;
	}
	if (elapsed_cycles > data->stats.max_transaction_cycles) {
		data->stats.max_transaction_cycles = elapsed_cycles;
	}
	irq_unlock(key);
}

static int rt_spi_stm32_request(const struct device *dev,
				     const struct rt_spi_transfer *frame)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;

	if ((frame == NULL) || (frame->tx == NULL) || (frame->len == 0U) ||
	    (frame->len > cfg->max_frame_len) ||
	    (frame->len > RT_SPI_MAX_FRAME_BYTES)) {
		return -EINVAL;
	}

	if (data->active) {
		data->stats.busy_count++;
		return -EBUSY;
	}

	memcpy(data->tx, frame->tx, frame->len);
	memset(data->rx, 0, frame->len);
	data->len = frame->len;
	data->tx_count = 0U;
	data->rx_count = 0U;
	data->active = true;
	data->start_cycles = rt_spi_stm32_cycle_get();
	data->stats.request_count++;

	LL_SPI_Disable(spi);
	rt_spi_stm32_collect_error_flags(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_ClearFlag_OVR(spi);
	LL_SPI_SetTransferSize(spi, frame->len);
	LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);

	rt_spi_stm32_cs_control(cfg, true);
	LL_SPI_Enable(spi);
	rt_spi_stm32_fill_tx_fifo(dev);
	rt_spi_stm32_enable_irqs(spi);
	LL_SPI_StartMasterTransfer(spi);

	return 0;
}

static int rt_spi_stm32_collect(const struct device *dev,
				     struct rt_spi_result *sample)
{
	struct rt_spi_stm32_data *data = dev->data;

	if (sample == NULL) {
		return -EINVAL;
	}

	if (!data->sample_ready) {
		if (data->active) {
			return -EAGAIN;
		}
		data->stats.collect_empty_count++;
		return -ENODATA;
	}

	sample->timestamp_cycles = data->start_cycles;
	memcpy(sample->raw, data->sample_raw, data->len);
	sample->len = data->len;
	sample->flags = data->sample_flags;
	data->sample_ready = false;

	return ((sample->flags & RT_SPI_RESULT_ERROR) != 0U) ? -EIO : 0;
}

static int rt_spi_stm32_transceive(const struct device *dev,
				   const struct rt_spi_transfer *frame,
				   struct rt_spi_result *sample,
				   uint32_t timeout_us)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	uint8_t tx[RT_SPI_MAX_FRAME_BYTES];
	uint8_t rx[RT_SPI_MAX_FRAME_BYTES] = {0};
	uint8_t tx_count = 0U;
	uint8_t rx_count = 0U;
	uint16_t error_flags;
	uint32_t start_cycles;
	uint32_t timeout_cycles;
	unsigned int key;

	if ((frame == NULL) || (frame->tx == NULL) || (sample == NULL) ||
	    (frame->len == 0U) || (frame->len > cfg->max_frame_len) ||
	    (frame->len > RT_SPI_MAX_FRAME_BYTES) || (timeout_us == 0U)) {
		return -EINVAL;
	}

	key = irq_lock();
	if (data->active || data->sample_ready) {
		data->stats.busy_count++;
		irq_unlock(key);
		return -EBUSY;
	}
	data->active = true;
	data->stats.request_count++;
	irq_unlock(key);

	memcpy(tx, frame->tx, frame->len);
	memset(sample, 0, sizeof(*sample));

	rt_spi_stm32_disable_irqs(spi);
	LL_SPI_Disable(spi);
	rt_spi_stm32_collect_error_flags(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_ClearFlag_OVR(spi);
	LL_SPI_SetTransferSize(spi, frame->len);
	LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);

	start_cycles = rt_spi_stm32_cycle_get();
	timeout_cycles = k_us_to_cyc_ceil32(timeout_us);
	rt_spi_stm32_cs_control(cfg, true);
	LL_SPI_Enable(spi);
	LL_SPI_StartMasterTransfer(spi);

	while ((rt_spi_stm32_cycle_get() - start_cycles) < timeout_cycles) {
		error_flags = rt_spi_stm32_collect_error_flags(spi);
		if (error_flags != 0U) {
			sample->flags = RT_SPI_RESULT_ERROR | error_flags;
			goto done;
		}

		while ((tx_count < frame->len) && LL_SPI_IsActiveFlag_TXP(spi)) {
			LL_SPI_TransmitData8(spi, tx[tx_count++]);
		}
		while ((rx_count < frame->len) && LL_SPI_IsActiveFlag_RXP(spi)) {
			rx[rx_count++] = LL_SPI_ReceiveData8(spi);
		}

		if (LL_SPI_IsActiveFlag_EOT(spi)) {
			while ((rx_count < frame->len) && LL_SPI_IsActiveFlag_RXP(spi)) {
				rx[rx_count++] = LL_SPI_ReceiveData8(spi);
			}
			sample->flags = RT_SPI_RESULT_VALID;
			goto done;
		}
	}

	sample->flags = RT_SPI_RESULT_ERROR | RT_SPI_RESULT_TIMEOUT;

done:
	rt_spi_stm32_disable_irqs(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_SetTransferSize(spi, 0U);
	LL_SPI_Disable(spi);
	rt_spi_stm32_cs_control(cfg, false);

	key = irq_lock();
	memcpy(sample->raw, rx, frame->len);
	sample->len = frame->len;
	sample->timestamp_cycles = start_cycles;
	data->active = false;
	data->stats.complete_count++;
	if ((sample->flags & RT_SPI_RESULT_ERROR) != 0U) {
		data->stats.error_count++;
		data->stats.last_error_flags = sample->flags;
	}
	irq_unlock(key);

	return ((sample->flags & RT_SPI_RESULT_ERROR) != 0U) ? -EIO : 0;
}

static bool rt_spi_stm32_busy(const struct device *dev)
{
	struct rt_spi_stm32_data *data = dev->data;

	return data->active;
}

static void rt_spi_stm32_abort(const struct device *dev)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	unsigned int key;

	rt_spi_stm32_disable_irqs(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_ClearFlag_OVR(spi);
	LL_SPI_SetTransferSize(spi, 0U);
	LL_SPI_Disable(spi);
	rt_spi_stm32_cs_control(cfg, false);

	key = irq_lock();
	if (data->active) {
		data->stats.error_count++;
		data->stats.last_error_flags = RT_SPI_RESULT_ERROR | RT_SPI_RESULT_TIMEOUT;
	}
	data->active = false;
	data->sample_ready = false;
	data->len = 0U;
	data->tx_count = 0U;
	data->rx_count = 0U;
	irq_unlock(key);
}

static void rt_spi_stm32_get_stats(const struct device *dev,
					struct rt_spi_stats *stats)
{
	struct rt_spi_stm32_data *data = dev->data;
	unsigned int key;

	if (stats == NULL) {
		return;
	}

	key = irq_lock();
	*stats = data->stats;
	irq_unlock(key);
}

static void rt_spi_stm32_reset_stats(const struct device *dev)
{
	struct rt_spi_stm32_data *data = dev->data;
	unsigned int key = irq_lock();

	memset(&data->stats, 0, sizeof(data->stats));
	irq_unlock(key);
}

static void rt_spi_stm32_isr(const struct device *dev)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	struct rt_spi_stm32_data *data = dev->data;
	SPI_TypeDef *spi = cfg->spi;
	uint16_t error_flags;

	if (!data->active || !LL_SPI_IsEnabled(spi)) {
		return;
	}

	error_flags = rt_spi_stm32_collect_error_flags(spi);
	if (error_flags != 0U) {
		rt_spi_stm32_publish(dev, RT_SPI_RESULT_ERROR | error_flags);
		return;
	}

	rt_spi_stm32_drain_rx_fifo(dev);
	rt_spi_stm32_fill_tx_fifo(dev);

	if (LL_SPI_IsActiveFlag_EOT(spi)) {
		rt_spi_stm32_drain_rx_fifo(dev);
		rt_spi_stm32_publish(dev, RT_SPI_RESULT_VALID);
	}
}

static int rt_spi_stm32_configure(const struct device *dev)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	static const uint32_t scaler[] = {
#ifdef LL_SPI_BAUDRATEPRESCALER_BYPASS
		LL_SPI_BAUDRATEPRESCALER_BYPASS,
#endif
		LL_SPI_BAUDRATEPRESCALER_DIV2,
		LL_SPI_BAUDRATEPRESCALER_DIV4,
		LL_SPI_BAUDRATEPRESCALER_DIV8,
		LL_SPI_BAUDRATEPRESCALER_DIV16,
		LL_SPI_BAUDRATEPRESCALER_DIV32,
		LL_SPI_BAUDRATEPRESCALER_DIV64,
		LL_SPI_BAUDRATEPRESCALER_DIV128,
		LL_SPI_BAUDRATEPRESCALER_DIV256,
	};
	const int shift = (scaler[0] == LL_SPI_BAUDRATEPRESCALER_DIV2) ? 1 : 0;
	uint32_t clock = 0U;
	int br;
	int ret;
	SPI_TypeDef *spi = cfg->spi;

	if (cfg->max_frame_len > RT_SPI_MAX_FRAME_BYTES) {
		return -EINVAL;
	}
	if (!cfg->fifo_enabled) {
		return -ENOTSUP;
	}

	if (cfg->pclk_len > 1U) {
		ret = clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					     (clock_control_subsys_t)&cfg->pclken[1],
					     &clock);
	} else {
		ret = clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					     (clock_control_subsys_t)&cfg->pclken[0],
					     &clock);
	}
	if (ret != 0) {
		return ret;
	}

	for (br = 0; br < ARRAY_SIZE(scaler); br++) {
		if ((clock >> (br + shift)) <= cfg->frequency) {
			break;
		}
	}
	if (br >= ARRAY_SIZE(scaler)) {
		return -EINVAL;
	}

	LL_SPI_Disable(spi);
	LL_SPI_SetBaudRatePrescaler(spi, scaler[br]);
	LL_SPI_SetClockPolarity(spi, cfg->cpol ? LL_SPI_POLARITY_HIGH : LL_SPI_POLARITY_LOW);
	LL_SPI_SetClockPhase(spi, cfg->cpha ? LL_SPI_PHASE_2EDGE : LL_SPI_PHASE_1EDGE);
	LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetTransferBitOrder(spi, LL_SPI_MSB_FIRST);
	LL_SPI_DisableCRC(spi);

	if (cfg->cs_mode == RT_SPI_STM32_CS_GPIO ||
	    cfg->cs_mode == RT_SPI_STM32_CS_SOFTWARE) {
		if (LL_SPI_GetNSSPolarity(spi) == LL_SPI_NSS_POLARITY_LOW) {
			LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
		}
		LL_SPI_SetNSSMode(spi, LL_SPI_NSS_SOFT);
	} else {
		LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_OUTPUT);
	}

	LL_SPI_SetMode(spi, LL_SPI_MODE_MASTER);
	LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetMasterSSIdleness(spi, cfg->mssi_clocks);
	LL_SPI_SetInterDataIdleness(spi, cfg->midi_clocks << SPI_CFG2_MIDI_Pos);
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);
	rt_spi_stm32_disable_irqs(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_ClearFlag_OVR(spi);

	return 0;
}

static int rt_spi_stm32_init(const struct device *dev)
{
	const struct rt_spi_stm32_config *cfg = dev->config;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int ret;

	if (cfg->pclk_len > 1U) {
		ret = clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[1], NULL);
		if (ret < 0) {
			return ret;
		}
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret != 0) {
		return ret;
	}

	if ((cfg->cs_mode == RT_SPI_STM32_CS_GPIO) != cfg->has_cs_gpio) {
		return -EINVAL;
	}

	if (cfg->has_cs_gpio) {
		if (!gpio_is_ready_dt(&cfg->cs_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	ret = rt_spi_stm32_configure(dev);
	if (ret != 0) {
		return ret;
	}

	cfg->irq_config(dev);
	return 0;
}

static const struct rt_spi_driver_api rt_spi_stm32_api = {
	.request = rt_spi_stm32_request,
	.collect = rt_spi_stm32_collect,
	.transceive = rt_spi_stm32_transceive,
	.busy = rt_spi_stm32_busy,
	.abort = rt_spi_stm32_abort,
	.get_stats = rt_spi_stm32_get_stats,
	.reset_stats = rt_spi_stm32_reset_stats,
};

#define RT_SPI_STM32_SPI_NODE(inst) DT_DRV_INST(inst)
#define RT_SPI_STM32_HAS_CS(inst) DT_INST_NODE_HAS_PROP(inst, cs_gpios)
#define RT_SPI_STM32_CS_SPEC(inst)							\
	GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, cs_gpios, 0, {0})

#define RT_SPI_STM32_IRQ_CONFIG(inst)						\
	static void rt_spi_stm32_irq_config_##inst(const struct device *dev)	\
	{										\
		IRQ_CONNECT(DT_IRQN(RT_SPI_STM32_SPI_NODE(inst)),			\
			    DT_IRQ(RT_SPI_STM32_SPI_NODE(inst), priority),		\
			    rt_spi_stm32_isr, DEVICE_DT_INST_GET(inst), 0);	\
		irq_enable(DT_IRQN(RT_SPI_STM32_SPI_NODE(inst)));		\
	}

#define RT_SPI_STM32_INIT(inst)							\
	PINCTRL_DT_DEFINE(RT_SPI_STM32_SPI_NODE(inst));				\
	static const struct stm32_pclken rt_spi_stm32_pclken_##inst[] =		\
		STM32_DT_CLOCKS(RT_SPI_STM32_SPI_NODE(inst));			\
	RT_SPI_STM32_IRQ_CONFIG(inst)						\
	static const struct rt_spi_stm32_config rt_spi_stm32_config_##inst = {\
		.spi = (SPI_TypeDef *)DT_REG_ADDR(RT_SPI_STM32_SPI_NODE(inst)),	\
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(RT_SPI_STM32_SPI_NODE(inst)),	\
		.pclken = rt_spi_stm32_pclken_##inst,				\
		.pclk_len = DT_NUM_CLOCKS(RT_SPI_STM32_SPI_NODE(inst)),		\
		.cs_gpio = RT_SPI_STM32_CS_SPEC(inst),				\
		.irq_config = rt_spi_stm32_irq_config_##inst,			\
		.frequency = DT_INST_PROP(inst, spi_clock_frequency),		\
		.max_frame_len = DT_INST_PROP(inst, max_frame_len),			\
		.cpol = DT_INST_PROP(inst, spi_cpol),					\
		.cpha = DT_INST_PROP(inst, spi_cpha),					\
		.fifo_enabled = DT_INST_PROP(inst, fifo_enable),			\
		.has_cs_gpio = RT_SPI_STM32_HAS_CS(inst),				\
		.cs_mode = DT_INST_ENUM_IDX(inst, cs_mode),			\
		.mssi_clocks = DT_INST_PROP(inst, mssi_clock),				\
		.midi_clocks = DT_INST_PROP(inst, midi_clock),				\
	};										\
	static struct rt_spi_stm32_data rt_spi_stm32_data_##inst;		\
	DEVICE_DT_INST_DEFINE(inst, rt_spi_stm32_init, NULL,			\
			      &rt_spi_stm32_data_##inst,				\
			      &rt_spi_stm32_config_##inst, POST_KERNEL,		\
			      CONFIG_RT_SPI_INIT_PRIORITY, &rt_spi_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(RT_SPI_STM32_INIT)
