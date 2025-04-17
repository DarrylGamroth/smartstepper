/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2023 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_timer_ic

#include <errno.h>

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <drivers/timer_ic.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(timer_ic_stm32, CONFIG_TIMER_IC_LOG_LEVEL);

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

#define NUM_CC_CHANNELS 4

struct timer_ic_stm32_capture_data
{
	timer_ic_capture_callback_handler_t callback;
	void *user_data;
	bool continuous;
};

/** Capture data. */
struct timer_ic_stm32_data
{
	/** Timer clock (Hz). */
	uint32_t tim_clk;
	struct timer_ic_stm32_capture_data capture[NUM_CC_CHANNELS];
};

/** Timer configuration. */
struct timer_ic_stm32_config
{
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

/** Channel to LL mapping. */
static const uint32_t ch2ll[] = {
	LL_TIM_CHANNEL_CH1,
	LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3,
	LL_TIM_CHANNEL_CH4,
};

/** Channel to capture get function mapping. */
static uint32_t (*const get_channel_capture[])(const TIM_TypeDef *) = {
	LL_TIM_IC_GetCaptureCH1, LL_TIM_IC_GetCaptureCH2,
	LL_TIM_IC_GetCaptureCH3, LL_TIM_IC_GetCaptureCH4};

/** Channel to enable capture interrupt mapping. */
static void (*const enable_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_EnableIT_CC1, LL_TIM_EnableIT_CC2,
	LL_TIM_EnableIT_CC3, LL_TIM_EnableIT_CC4};

/** Channel to disable capture interrupt mapping. */
static void (*const disable_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_DisableIT_CC1, LL_TIM_DisableIT_CC2,
	LL_TIM_DisableIT_CC3, LL_TIM_DisableIT_CC4};

/** Channel to is capture active flag mapping. */
static uint32_t (*const is_capture_active[])(const TIM_TypeDef *) = {
	LL_TIM_IsActiveFlag_CC1, LL_TIM_IsActiveFlag_CC2,
	LL_TIM_IsActiveFlag_CC3, LL_TIM_IsActiveFlag_CC4};

/** Channel to clearing capture flag mapping. */
static void (*const clear_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_ClearFlag_CC1, LL_TIM_ClearFlag_CC2,
	LL_TIM_ClearFlag_CC3, LL_TIM_ClearFlag_CC4};

/** Channel to is capture active flag mapping. */
static uint32_t (*const is_capture_interrupt_enabled[])(const TIM_TypeDef *) = {
	LL_TIM_IsEnabledIT_CC1, LL_TIM_IsEnabledIT_CC2,
	LL_TIM_IsEnabledIT_CC3, LL_TIM_IsEnabledIT_CC4};

/**
 * Obtain timer clock speed.
 *
 * @param pclken  Timer clock control subsystem.
 * @param tim_clk Where computed timer clock will be stored.
 *
 * @return 0 on success, error code otherwise.
 */
static int get_tim_clk(const struct stm32_pclken *pclken, uint32_t *tim_clk)
{
	int r;
	const struct device *clk;
	uint32_t bus_clk, apb_psc;

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	r = clock_control_get_rate(clk, (clock_control_subsys_t)pclken,
							   &bus_clk);
	if (r < 0)
	{
		return r;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	if (pclken->bus == STM32_CLOCK_BUS_APB1)
	{
		apb_psc = STM32_D2PPRE1;
	}
	else
	{
		apb_psc = STM32_D2PPRE2;
	}
#else
	if (pclken->bus == STM32_CLOCK_BUS_APB1)
	{
		apb_psc = STM32_APB1_PRESCALER;
	}
#if !defined(CONFIG_SOC_SERIES_STM32C0X) && !defined(CONFIG_SOC_SERIES_STM32F0X) && \
	!defined(CONFIG_SOC_SERIES_STM32G0X)
	else
	{
		apb_psc = STM32_APB2_PRESCALER;
	}
#endif
#endif

#if defined(RCC_DCKCFGR_TIMPRE) || defined(RCC_DCKCFGR1_TIMPRE) || \
	defined(RCC_CFGR_TIMPRE)
	/*
	 * There are certain series (some F4, F7 and H7) that have the TIMPRE
	 * bit to control the clock frequency of all the timers connected to
	 * APB1 and APB2 domains.
	 *
	 * Up to a certain threshold value of APB{1,2} prescaler, timer clock
	 * equals to HCLK. This threshold value depends on TIMPRE setting
	 * (2 if TIMPRE=0, 4 if TIMPRE=1). Above threshold, timer clock is set
	 * to a multiple of the APB domain clock PCLK{1,2} (2 if TIMPRE=0, 4 if
	 * TIMPRE=1).
	 */

	if (LL_RCC_GetTIMPrescaler() == LL_RCC_TIM_PRESCALER_TWICE)
	{
		/* TIMPRE = 0 */
		if (apb_psc <= 2u)
		{
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		}
		else
		{
			*tim_clk = bus_clk * 2u;
		}
	}
	else
	{
		/* TIMPRE = 1 */
		if (apb_psc <= 4u)
		{
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		}
		else
		{
			*tim_clk = bus_clk * 4u;
		}
	}
#else
	/*
	 * If the APB prescaler equals 1, the timer clock frequencies
	 * are set to the same frequency as that of the APB domain.
	 * Otherwise, they are set to twice (×2) the frequency of the
	 * APB domain.
	 */
	if (apb_psc == 1u)
	{
		*tim_clk = bus_clk;
	}
	else
	{
		*tim_clk = bus_clk * 2u;
	}
#endif

	return 0;
}

static int init_capture_channel(const struct device *dev, uint32_t channel,
								timer_ic_flags_t flags)
{
	const struct timer_ic_stm32_config *cfg = dev->config;
	timer_ic_flags_t edge = (flags & TIMER_IC_CAPTURE_EDGE_MASK);
	LL_TIM_IC_InitTypeDef ic;

	if (channel > NUM_CC_CHANNELS - 1)
	{
		LOG_ERR("Input capture only supported on first four channels");
		return -EINVAL;
	}

	LL_TIM_IC_StructInit(&ic);
	ic.ICPrescaler = TIM_ICPSC_DIV1;
	ic.ICFilter = LL_TIM_IC_FILTER_FDIV1;
	ic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;

	if ((edge & TIMER_IC_CAPTURE_EDGE_BOTH) == TIMER_IC_CAPTURE_EDGE_BOTH)
	{
		ic.ICPolarity = LL_TIM_IC_POLARITY_BOTHEDGE;
	}
	else if (edge & TIMER_IC_CAPTURE_EDGE_RISING)
	{
		ic.ICPolarity = LL_TIM_IC_POLARITY_RISING;
	}
	else if (edge & TIMER_IC_CAPTURE_EDGE_FALLING)
	{
		ic.ICPolarity = LL_TIM_IC_POLARITY_FALLING;
	}
	else
	{
		LOG_ERR("Input capture edge not set");
		return -EINVAL;
	}

	if (LL_TIM_IC_Init(cfg->timer, ch2ll[channel], &ic) != SUCCESS)
	{
		LOG_ERR("Could not initialize channel for input capture");
		return -EIO;
	}

	return 0;
}

static int timer_ic_stm32_configure_capture(const struct device *dev,
											uint32_t channel, timer_ic_flags_t flags,
											timer_ic_capture_callback_handler_t cb,
											void *user_data)
{
	const struct timer_ic_stm32_config *cfg = dev->config;
	struct timer_ic_stm32_data *data = dev->data;
	struct timer_ic_stm32_capture_data *cpt;
	int ret;

	if (channel > NUM_CC_CHANNELS - 1)
	{
		LOG_ERR("Input capture only supported on first four channels");
		return -EINVAL;
	}

	if (is_capture_interrupt_enabled[channel](cfg->timer))
	{
		LOG_ERR("Input capture already in progress");
		return -EBUSY;
	}

	cpt = &data->capture[channel];
	cpt->callback = cb; /* even if the cb is reset, this is not an error */
	cpt->user_data = user_data;
	cpt->continuous = (flags & TIMER_IC_CAPTURE_MODE_CONTINUOUS) ? true : false;

	ret = init_capture_channel(dev, channel, flags);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int timer_ic_stm32_enable_capture(const struct device *dev, uint32_t channel)
{
	const struct timer_ic_stm32_config *cfg = dev->config;
	struct timer_ic_stm32_data *data = dev->data;

	if (channel > NUM_CC_CHANNELS - 1)
	{
		LOG_ERR("Input capture only supported on first four channels");
		return -EINVAL;
	}

	if (is_capture_interrupt_enabled[channel](cfg->timer))
	{
		LOG_ERR("Input capture already active");
		return -EBUSY;
	}

	if (!data->capture[channel].callback)
	{
		LOG_ERR("Input capture not configured");
		return -EINVAL;
	}

	clear_capture_interrupt[channel](cfg->timer);
	enable_capture_interrupt[channel](cfg->timer);
	LL_TIM_CC_EnableChannel(cfg->timer, ch2ll[channel]);

	return 0;
}

static int timer_ic_stm32_disable_capture(const struct device *dev, uint32_t channel)
{
	const struct timer_ic_stm32_config *cfg = dev->config;

	if (channel > NUM_CC_CHANNELS - 1)
	{
		LOG_ERR("Input capture only supported on first four channels");
		return -EINVAL;
	}

	disable_capture_interrupt[channel](cfg->timer);
	clear_capture_interrupt[channel](cfg->timer);
	LL_TIM_CC_DisableChannel(cfg->timer, ch2ll[channel]);

	return 0;
}

static void timer_ic_stm32_isr(const struct device *dev)
{
	const struct timer_ic_stm32_config *cfg = dev->config;
	struct timer_ic_stm32_data *data = dev->data;

	for (int channel = 0; channel < NUM_CC_CHANNELS; ++channel)
	{
		if (is_capture_active[channel](cfg->timer))
		{
			struct timer_ic_stm32_capture_data *cpt = &data->capture[channel];
			uint32_t counts;
			int status = 0;

			clear_capture_interrupt[channel](cfg->timer);
			counts = get_channel_capture[channel](cfg->timer);

			if (!cpt->continuous)
			{
				(void)timer_ic_stm32_disable_capture(dev, channel);
			}

			if (cpt->callback != NULL)
			{
				cpt->callback(dev, channel, counts,
							  status, cpt->user_data);
			}
		}
	}
}

static int timer_ic_stm32_get_cycles_per_sec(const struct device *dev,
											 uint32_t channel, uint64_t *cycles)
{
	struct timer_ic_stm32_data *data = dev->data;
	const struct timer_ic_stm32_config *cfg = dev->config;

	*cycles = (uint64_t)(data->tim_clk / (cfg->prescaler + 1));

	return 0;
}

static const struct timer_ic_driver_api timer_ic_stm32_driver_api = {
	.get_cycles_per_sec = timer_ic_stm32_get_cycles_per_sec,
	.configure_capture = timer_ic_stm32_configure_capture,
	.enable_capture = timer_ic_stm32_enable_capture,
	.disable_capture = timer_ic_stm32_disable_capture,
};

static int timer_ic_stm32_init(const struct device *dev)
{
	struct timer_ic_stm32_data *data = dev->data;
	const struct timer_ic_stm32_config *cfg = dev->config;

	int r;
	const struct device *clk;
	LL_TIM_InitTypeDef init;

	/* enable clock and store its speed */
	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk))
	{
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	r = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
	if (r < 0)
	{
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}

	r = get_tim_clk(&cfg->pclken, &data->tim_clk);
	if (r < 0)
	{
		LOG_ERR("Could not obtain timer clock (%d)", r);
		return r;
	}

	/* configure pinmux */
	r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (r < 0)
	{
		LOG_ERR("Input capture pinctrl setup failed (%d)", r);
		return r;
	}

	/* initialize timer */
	LL_TIM_StructInit(&init);

	init.Prescaler = cfg->prescaler;
	init.CounterMode = cfg->countermode;
	init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

	if (LL_TIM_Init(cfg->timer, &init) != SUCCESS)
	{
		LOG_ERR("Could not initialize timer");
		return -EIO;
	}

	LL_TIM_EnableCounter(cfg->timer);

	cfg->irq_config_func(dev);

	return 0;
}

#define IRQ_CONNECT_AND_ENABLE_BY_NAME(index, name)                        \
	{                                                                      \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(index), name, irq),      \
					DT_IRQ_BY_NAME(DT_INST_PARENT(index), name, priority), \
					timer_ic_stm32_isr, DEVICE_DT_INST_GET(index), 0);     \
		irq_enable(DT_IRQ_BY_NAME(DT_INST_PARENT(index), name, irq));      \
	}

#define IRQ_CONNECT_AND_ENABLE_DEFAULT(index)                          \
	{                                                                  \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(index)),                    \
					DT_IRQ(DT_INST_PARENT(index), priority),           \
					timer_ic_stm32_isr, DEVICE_DT_INST_GET(index), 0); \
		irq_enable(DT_IRQN(DT_INST_PARENT(index)));                    \
	}

#define IRQ_CONFIG_FUNC(index)                                                   \
	static void timer_ic_stm32_irq_config_func_##index(const struct device *dev) \
	{                                                                            \
		COND_CODE_1(DT_IRQ_HAS_NAME(DT_INST_PARENT(index), cc),                  \
					(IRQ_CONNECT_AND_ENABLE_BY_NAME(index, cc)),                 \
					(IRQ_CONNECT_AND_ENABLE_DEFAULT(index)));                    \
	}
#define CAPTURE_INIT(index) \
	.irq_config_func = timer_ic_stm32_irq_config_func_##index

#define TIMER_IC_DEVICE_INIT(index)                                             \
	static struct timer_ic_stm32_data timer_ic_stm32_data_##index;              \
	IRQ_CONFIG_FUNC(index)                                                      \
                                                                                \
	PINCTRL_DT_INST_DEFINE(index);                                              \
                                                                                \
	static const struct timer_ic_stm32_config timer_ic_stm32_config_##index = { \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(index)),             \
		.prescaler = DT_PROP(DT_INST_PARENT(index), st_prescaler),              \
		.countermode = DT_PROP(DT_INST_PARENT(index), st_countermode),          \
		.pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(index)),                   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                          \
		CAPTURE_INIT(index)};                                                   \
                                                                                \
	DEVICE_DT_INST_DEFINE(index, &timer_ic_stm32_init, NULL,                    \
						  &timer_ic_stm32_data_##index,                         \
						  &timer_ic_stm32_config_##index, POST_KERNEL,          \
						  CONFIG_TIMER_IC_INIT_PRIORITY,                        \
						  &timer_ic_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TIMER_IC_DEVICE_INIT)
