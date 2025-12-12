/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2023 Nobleo Technology
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_mcpwm

#include <errno.h>

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/dsp/types.h>

#include <zephyr/sys/util.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/dt-bindings/pwm/stm32_pwm.h>

#include <drivers/mcpwm.h>
#include <dt-bindings/pwm/stm32-mcpwm.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(DT_DRV_COMPAT, CONFIG_PWM_LOG_LEVEL);

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

#if defined(TIM_BREAK_INPUT_SUPPORT)
static const uint32_t break_polarity[] = {LL_TIM_BREAK_POLARITY_LOW, LL_TIM_BREAK_POLARITY_HIGH};
#else
static const uint32_t break_polarity[] = {0};
#endif

#if defined(TIM_BDTR_BKBID)
static const uint32_t break2_polarity[] = {LL_TIM_BREAK2_POLARITY_LOW, LL_TIM_BREAK2_POLARITY_HIGH};
#else
static const uint32_t break2_polarity[] = {0};
#endif

#if !defined(CONFIG_SOC_SERIES_STM32L0X) && !defined(CONFIG_SOC_SERIES_STM32L1X)
static const uint32_t lock_level[] = {LL_TIM_LOCKLEVEL_OFF, LL_TIM_LOCKLEVEL_1, LL_TIM_LOCKLEVEL_2,
				      LL_TIM_LOCKLEVEL_3};
#else
static const uint32_t lock_level[] = {0};
#endif

/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#if defined(LL_TIM_CHANNEL_CH6)
#define TIMER_HAS_6CH 1
#define TIMER_MAX_CH  6u
#else
#define TIMER_HAS_6CH 0
#define TIMER_MAX_CH  4u
#endif

/** PWM data. */
struct mcpwm_stm32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
	/** Calculated period cycles (adjusted for counter mode). */
	uint32_t period_cycles;
	uint32_t period_cycles_x2;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
	/* User break callback */
	mcpwm_break_cb_t break_cb;
	void *break_user_data;
	/* Output compare callbacks */
	mcpwm_compare_cb_t compare_cb[TIMER_MAX_CH];
	void *compare_user_data[TIMER_MAX_CH];
	/* Per-channel configuration flags */
	mcpwm_flags_t channel_flags[TIMER_MAX_CH];
};

/** PWM configuration. */
struct mcpwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	uint32_t trigger_selection;
	uint32_t master_mode_selection;
	uint32_t slave_mode_selection;
	uint32_t repetition_counter;
	uint32_t ossr;
	uint32_t ossi;
	uint32_t dead_time;
	uint32_t lock_level;
	uint32_t break_polarity;
	uint32_t break2_polarity;
	uint32_t period_ns;
	uint32_t cc_dma_channels_mask;
	bool master_mode;
	bool slave_mode;
	bool trigger_polarity;
	bool one_pulse_mode;
	bool break_enable;
	bool break2_enable;
	bool automatic_output;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if TIMER_HAS_6CH
	LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
};

/** STM32 MCUs have between 1 and 4 complementary channels */
static const uint32_t ch2ll_n[] = {
#if defined(LL_TIM_CHANNEL_CH1N)
	LL_TIM_CHANNEL_CH1N,
#if defined(LL_TIM_CHANNEL_CH2N)
	LL_TIM_CHANNEL_CH2N,
#if defined(LL_TIM_CHANNEL_CH3N)
	LL_TIM_CHANNEL_CH3N,
#if defined(LL_TIM_CHANNEL_CH4N)
	LL_TIM_CHANNEL_CH4N,
#endif /* LL_TIM_CHANNEL_CH4N */
#endif /* LL_TIM_CHANNEL_CH3N */
#endif /* LL_TIM_CHANNEL_CH2N */
#endif /* LL_TIM_CHANNEL_CH1N */
};
/** Maximum number of complemented timer channels is ARRAY_SIZE(ch2ll_n)*/

/** Channel to compare set function mapping. */
static void (*const set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *, uint32_t) = {
	LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2, LL_TIM_OC_SetCompareCH3,
	LL_TIM_OC_SetCompareCH4,
#if TIMER_HAS_6CH
	LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
};

/** Channel to enable capture interrupt mapping. */
static void __maybe_unused (*const enable_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_EnableIT_CC1,
	LL_TIM_EnableIT_CC2,
	LL_TIM_EnableIT_CC3,
	LL_TIM_EnableIT_CC4
};

/** Channel to disable capture interrupt mapping. */
static void __maybe_unused (*const disable_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_DisableIT_CC1,
	LL_TIM_DisableIT_CC2,
	LL_TIM_DisableIT_CC3,
	LL_TIM_DisableIT_CC4
};

/* Channel is interrupt enabled mapping*/
static uint32_t __maybe_unused (*const is_capture_interrupt_enabled[])(const TIM_TypeDef *) = {
	LL_TIM_IsEnabledIT_CC1,
	LL_TIM_IsEnabledIT_CC2,
	LL_TIM_IsEnabledIT_CC3,
	LL_TIM_IsEnabledIT_CC4
};

/** Channel to is capture active flag mapping. */
#if !defined(CONFIG_SOC_SERIES_STM32MP1X)
static uint32_t __maybe_unused (*const is_capture_active[])(const TIM_TypeDef *) = {
#else
static uint32_t __maybe_unused (*const is_capture_active[])(TIM_TypeDef *) = {
#endif
	LL_TIM_IsActiveFlag_CC1,
	LL_TIM_IsActiveFlag_CC2,
	LL_TIM_IsActiveFlag_CC3,
	LL_TIM_IsActiveFlag_CC4,
#if TIMER_HAS_6CH
	LL_TIM_IsActiveFlag_CC5,
	LL_TIM_IsActiveFlag_CC6
#endif
};

/** Channel to clearing capture flag mapping. */
static void __maybe_unused (*const clear_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_ClearFlag_CC1,
	LL_TIM_ClearFlag_CC2,
	LL_TIM_ClearFlag_CC3,
	LL_TIM_ClearFlag_CC4,
#if TIMER_HAS_6CH
	LL_TIM_ClearFlag_CC5,
	LL_TIM_ClearFlag_CC6
#endif
};

static void (*const enable_cc_dma[])(TIM_TypeDef *) = {
	LL_TIM_EnableDMAReq_CC1, LL_TIM_EnableDMAReq_CC2, LL_TIM_EnableDMAReq_CC3,
	LL_TIM_EnableDMAReq_CC4};

static void (*const disable_cc_dma[])(TIM_TypeDef *) = {
	LL_TIM_DisableDMAReq_CC1, LL_TIM_DisableDMAReq_CC2, LL_TIM_DisableDMAReq_CC3,
	LL_TIM_DisableDMAReq_CC4};

/**
 * Obtain LL polarity from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL polarity.
 */
static uint32_t get_polarity(mcpwm_flags_t flags)
{
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		return LL_TIM_OCPOLARITY_HIGH;
	}

	return LL_TIM_OCPOLARITY_LOW;
}

/**
 * Obtain LL output compare mode from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL output compare mode.
 */
static uint32_t get_output_compare_mode(mcpwm_flags_t flags)
{
	uint32_t mode_flags = flags & STM32_PWM_OC_MODE_MASK;

	if (mode_flags == STM32_PWM_OC_MODE_PWM2) {
		return LL_TIM_OCMODE_PWM2;
	}

	/* Default to PWM1 mode */
	return LL_TIM_OCMODE_PWM1;
}

/**
 * Obtain LL idle state from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL idle state.
 */
static uint32_t get_idle_state(mcpwm_flags_t flags)
{
	uint32_t idle_flags = flags & STM32_PWM_IDLE_STATE_MASK;

	if (idle_flags == STM32_PWM_IDLE_STATE_HIGH) {
		return LL_TIM_OCIDLESTATE_HIGH;
	}

	/* Default to low idle state */
	return LL_TIM_OCIDLESTATE_LOW;
}

/**
 * @brief  Check if LL counter mode is center-aligned.
 *
 * @param  ll_countermode LL counter mode.
 *
 * @return `true` when center-aligned, otherwise `false`.
 */
static inline bool is_center_aligned(const uint32_t ll_countermode)
{
	return ((ll_countermode == LL_TIM_COUNTERMODE_CENTER_DOWN) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP_DOWN));
}

static void mcpwm_stm32_brk_isr(const struct device *dev)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	struct mcpwm_stm32_data *data = dev->data;
	TIM_TypeDef *timer = cfg->timer;
	bool break_occurred = false;

	if (!IS_TIM_BREAK_INSTANCE(timer)) {
		return;
	}

	if (LL_TIM_IsActiveFlag_BRK(timer)) {
		LL_TIM_ClearFlag_BRK(timer);
		break_occurred = true;
		LOG_ERR("PWM break fault detected on timer %p", timer);
	}

#if defined(IS_TIM_BKIN2_INSTANCE)
	if (IS_TIM_BKIN2_INSTANCE(timer) && LL_TIM_IsActiveFlag_BRK2(timer)) {
		LL_TIM_ClearFlag_BRK2(timer);
		break_occurred = true;
		LOG_ERR("PWM break2 fault detected on timer %p", timer);
	}
#endif

	if (break_occurred && data->break_cb != NULL) {
		data->break_cb(dev, data->break_user_data);
	}
}

static int mcpwm_stm32_configure(const struct device *dev, uint32_t channel, mcpwm_flags_t flags)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	struct mcpwm_stm32_data *data = dev->data;
	TIM_TypeDef *timer = cfg->timer;

	uint32_t ll_channel;
	uint32_t current_ll_channel;
	uint32_t negative_ll_channel = 0;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* Store flags for this channel */
	data->channel_flags[channel - 1U] = flags;

	ll_channel = ch2ll[channel - 1U];

	if ((channel - 1U) < ARRAY_SIZE(ch2ll_n)) {
		negative_ll_channel = ch2ll_n[channel - 1U];
	} else {
		negative_ll_channel = 0;
	}

	/* in LL_TIM_CC_DisableChannel and LL_TIM_CC_IsEnabledChannel,
	 * the channel param could be the complementary one
	 */
	if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
		if (!negative_ll_channel) {
			LOG_ERR("Channel %d has no complementary output", channel);
			return -EINVAL;
		}
		current_ll_channel = negative_ll_channel;
	} else {
		current_ll_channel = ll_channel;
	}

	LL_TIM_OC_SetPolarity(timer, current_ll_channel, get_polarity(flags));

	if (!LL_TIM_CC_IsEnabledChannel(timer, current_ll_channel)) {
		LL_TIM_OC_SetMode(timer, ll_channel, get_output_compare_mode(flags));
#ifdef LL_TIM_OCIDLESTATE_LOW
		LL_TIM_OC_SetIdleState(timer, current_ll_channel, get_idle_state(flags));
#endif
	}

	return 0;
}

static int mcpwm_stm32_enable(const struct device *dev, uint32_t channel)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	struct mcpwm_stm32_data *data = dev->data;
	TIM_TypeDef *timer = cfg->timer;
	uint32_t ll_channel;
	uint32_t channels_to_enable;
	mcpwm_flags_t flags;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* Use stored configuration flags */
	flags = data->channel_flags[channel - 1U];

	ll_channel = ch2ll[channel - 1U];
	channels_to_enable = ll_channel;

	/* Add complementary channel only if explicitly requested */
	if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
		if ((channel - 1U) < ARRAY_SIZE(ch2ll_n)) {
			channels_to_enable |= ch2ll_n[channel - 1U];
		} else {
			LOG_ERR("Channel %d has no complementary output", channel);
			return -EINVAL;
		}
	}

	if ((cfg->cc_dma_channels_mask & BIT(channel - 1U)) != 0U) {
		enable_cc_dma[channel - 1U](timer);
	}

	LL_TIM_CC_EnableChannel(timer, channels_to_enable);

	return 0;
}

static int mcpwm_stm32_disable(const struct device *dev, uint32_t channel)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	struct mcpwm_stm32_data *data = dev->data;
	TIM_TypeDef *timer = cfg->timer;
	uint32_t ll_channel;
	uint32_t channels_to_disable;
	mcpwm_flags_t flags;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* Use stored configuration flags */
	flags = data->channel_flags[channel - 1U];

	ll_channel = ch2ll[channel - 1U];
	channels_to_disable = ll_channel;

	/* Add complementary channel only if explicitly requested */
	if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
		if ((channel - 1U) < ARRAY_SIZE(ch2ll_n)) {
			channels_to_disable |= ch2ll_n[channel - 1U];
		} else {
			LOG_ERR("Channel %d has no complementary output", channel);
			return -EINVAL;
		}
	}

	if ((cfg->cc_dma_channels_mask & BIT(channel - 1U)) != 0U) {
		disable_cc_dma[channel - 1U](timer);
	}

	LL_TIM_CC_DisableChannel(timer, channels_to_disable);

	return 0;
}

static int mcpwm_stm32_set_duty_cycle(const struct device *dev, uint32_t channel, q31_t duty_cycle)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	const struct mcpwm_stm32_data *data = dev->data;

	if (channel < 1U || channel > TIMER_MAX_CH) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	uint32_t pulse_cycles =
		(int32_t)(((int64_t)duty_cycle * data->period_cycles + 0x40000000LL) >> 31);

	set_timer_compare[channel - 1U](cfg->timer, pulse_cycles);
	return 0;
}

static int mcpwm_stm32_set_compare_callback(const struct device *dev, uint32_t channel,
					    mcpwm_compare_cb_t callback, void *user_data)
{
	struct mcpwm_stm32_data *data = dev->data;
	const struct mcpwm_stm32_config *cfg = dev->config;
	TIM_TypeDef *timer = cfg->timer;
	uint32_t index = channel - 1U;
	unsigned int key;

	if ((channel < 1U) || (channel > 4U)) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	key = irq_lock();

	data->compare_cb[index] = callback;
	data->compare_user_data[index] = user_data;

	if (callback != NULL) {
		clear_capture_interrupt[index](timer);
		enable_capture_interrupt[index](timer);
	} else {
		disable_capture_interrupt[index](timer);
	}

	irq_unlock(key);

	LOG_DBG("Compare callback %s for PWM device %s channel %d",
		callback != NULL ? "registered" : "cleared", dev->name, channel);

	return 0;
}

static int mcpwm_stm32_set_break_callback(const struct device *dev, mcpwm_break_cb_t callback,
					  void *user_data)
{
	struct mcpwm_stm32_data *data = dev->data;
	const struct mcpwm_stm32_config *cfg = dev->config;
	TIM_TypeDef *timer = cfg->timer;
	unsigned int key;

	if (!IS_TIM_BREAK_INSTANCE(timer)) {
		return callback != NULL ? -ENOTSUP : 0;
	}

	key = irq_lock();
	data->break_cb = callback;
	data->break_user_data = user_data;

	if (callback != NULL) {
		LL_TIM_ClearFlag_BRK(timer);
#if defined(IS_TIM_BKIN2_INSTANCE)
		if (IS_TIM_BKIN2_INSTANCE(timer)) {
			LL_TIM_ClearFlag_BRK2(timer);
		}
#endif
		LL_TIM_EnableIT_BRK(timer);
	} else {
		LL_TIM_DisableIT_BRK(timer);
	}
	irq_unlock(key);

	return 0;
}

static int mcpwm_stm32_start(const struct device *dev)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	TIM_TypeDef *timer = cfg->timer;

	/* Reset counter and generate update event for master timers only
	 * Slave timers are controlled by master's trigger signals
	 */
	if (!cfg->slave_mode) {
		LL_TIM_SetCounter(timer, 0);
		LL_TIM_GenerateEvent_UPDATE(timer);
	}

#if !defined(CONFIG_SOC_SERIES_STM32L0X) && !defined(CONFIG_SOC_SERIES_STM32L1X)
	/* Enable all outputs (MOE bit) - also recovers from break faults */
	if (IS_TIM_BREAK_INSTANCE(timer)) {
		LL_TIM_EnableAllOutputs(timer);
	}
#endif

	/* Enable counter for master timers only
	 * Slave timers start automatically via hardware trigger
	 */
	if (!cfg->slave_mode) {
		LL_TIM_EnableCounter(timer);
	}

	LOG_DBG("Timer %s for PWM device %s", 
		cfg->slave_mode ? "outputs enabled" : "started", dev->name);
	return 0;
}

static int mcpwm_stm32_stop(const struct device *dev)
{
	const struct mcpwm_stm32_config *cfg = dev->config;

	LL_TIM_DisableCounter(cfg->timer);

	LOG_DBG("Timer stopped for PWM device %s", dev->name);
	return 0;
}

static DEVICE_API(mcpwm, mcpwm_stm32_driver_api) = {
	.configure = mcpwm_stm32_configure,
	.enable = mcpwm_stm32_enable,
	.disable = mcpwm_stm32_disable,
	.start = mcpwm_stm32_start,
	.stop = mcpwm_stm32_stop,
	.set_duty_cycle = mcpwm_stm32_set_duty_cycle,
	.set_compare_callback = mcpwm_stm32_set_compare_callback,
	.set_break_callback = mcpwm_stm32_set_break_callback,
};

static int mcpwm_stm32_init(const struct device *dev)
{
	struct mcpwm_stm32_data *data = dev->data;
	const struct mcpwm_stm32_config *cfg = dev->config;
	TIM_TypeDef *timer = cfg->timer;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	uint32_t tim_clk;
	int r;

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enable clock and store its speed */
	r = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (r < 0) {
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}

	if (cfg->pclk_len > 1) {
		/* Enable Timer clock source */
		r = clock_control_configure(clk, (clock_control_subsys_t)&cfg->pclken[1], NULL);
		if (r != 0) {
			LOG_ERR("Could not configure clock (%d)", r);
			return r;
		}

		r = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[1], &tim_clk);
		if (r < 0) {
			LOG_ERR("Timer clock rate get error (%d)", r);
			return r;
		}
	} else {
		LOG_ERR("Timer clock source is not specified");
		return -EINVAL;
	}

	data->tim_clk = tim_clk;

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&data->reset);

	/* configure pinmux - only if pinctrl is provided */
	if (cfg->pcfg != NULL) {
		r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (r < 0) {
			LOG_ERR("PWM pinctrl setup failed (%d)", r);
			return r;
		}
		LOG_DBG("Pinctrl configured for timer %s", dev->name);
	} else {
		LOG_DBG("No pinctrl configured for timer %s (timer-only mode)", dev->name);
	}

	/* Protect against overflow */
	if (cfg->period_ns > (UINT64_MAX / data->tim_clk)) {
		LOG_ERR("Period too large, would cause overflow");
		return -EINVAL;
	}

	uint64_t period_ns_calc = (uint64_t)data->tim_clk * cfg->period_ns;
	uint32_t period_cycles = (uint32_t)DIV_ROUND_UP(period_ns_calc, NSEC_PER_SEC);

	/* Adjust period based on counter mode */
	if (is_center_aligned(cfg->countermode)) {
		/* For center-aligned mode, period is divided by 2 */
		period_cycles /= 2U;
	} else {
		/* For up/down-counting modes, ARR = period - 1 */
		period_cycles -= 1U;
	}

	/* Validate period fits in timer */
	if (!IS_TIM_32B_COUNTER_INSTANCE(timer) && period_cycles > UINT16_MAX) {
		LOG_ERR("Period too large for 16-bit timer: %u", period_cycles);
		return -EINVAL;
	}

	/* Store the calculated period for later use */
	data->period_cycles = period_cycles;
	data->period_cycles_x2 = data->period_cycles * 2;

	/* initialize timer */
	LL_TIM_SetPrescaler(timer, cfg->prescaler);
	LL_TIM_SetAutoReload(timer, period_cycles);

	if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE(timer)) {
		LL_TIM_SetCounterMode(timer, cfg->countermode);
	}

	if (IS_TIM_CLOCK_DIVISION_INSTANCE(timer)) {
		LL_TIM_SetClockDivision(timer, LL_TIM_CLOCKDIVISION_DIV1);
	}

#ifdef IS_TIM_REPETITION_COUNTER_INSTANCE
	if (IS_TIM_REPETITION_COUNTER_INSTANCE(timer)) {
		LL_TIM_SetRepetitionCounter(timer, cfg->repetition_counter);
	}
#endif

	/* Enable auto-reload preload (affects all channels) */
	LL_TIM_EnableARRPreload(timer);

	if (cfg->master_mode) {
		/* Configure master mode trigger output */
		LL_TIM_SetTriggerOutput(timer, cfg->master_mode_selection);
		/* Enable MSM to synchronize TRGO with trigger input (if slave) */
		LL_TIM_EnableMasterSlaveMode(timer);
	}

	if (cfg->slave_mode) {
		/* Configure slave mode trigger input */
		LL_TIM_SetSlaveMode(timer, cfg->slave_mode_selection);
		LL_TIM_SetTriggerInput(timer, cfg->trigger_selection);

		/* Configure ETR if using external trigger */
		if (cfg->trigger_selection == LL_TIM_TS_ETRF) {
			LL_TIM_ConfigETR(timer,
					 cfg->trigger_polarity ? LL_TIM_ETR_POLARITY_INVERTED
							       : LL_TIM_ETR_POLARITY_NONINVERTED,
					 LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
		}
	}

	/* Configure one-pulse mode if enabled */
	if (cfg->one_pulse_mode) {
		LL_TIM_SetOnePulseMode(timer, LL_TIM_ONEPULSEMODE_SINGLE);
	} else {
		LL_TIM_SetOnePulseMode(timer, LL_TIM_ONEPULSEMODE_REPETITIVE);
	}

	LL_TIM_GenerateEvent_UPDATE(timer);

	cfg->irq_config_func(dev);

#if !defined(CONFIG_SOC_SERIES_STM32L0X) && !defined(CONFIG_SOC_SERIES_STM32L1X)
	if (IS_TIM_BREAK_INSTANCE(timer)) {
		/* Configure off-states behavior */
		LL_TIM_SetOffStates(timer,
				    cfg->ossi ? LL_TIM_OSSI_ENABLE : LL_TIM_OSSI_DISABLE,
				    cfg->ossr ? LL_TIM_OSSR_ENABLE : LL_TIM_OSSR_DISABLE);

		/* Configure dead time */
		if (cfg->dead_time > 0) {
			LL_TIM_OC_SetDeadTime(timer, cfg->dead_time);
		}

		/* Configure lock level */
		if (cfg->lock_level > 0) {
			LL_TIM_CC_SetLockLevel(timer, cfg->lock_level);
		}

		/* Configure break input 1 */
		if (cfg->break_enable) {
#if defined(TIM_BDTR_BKBID)
			LL_TIM_ConfigBRK(timer, cfg->break_polarity, LL_TIM_BREAK_FILTER_FDIV1,
					 LL_TIM_BREAK_AFMODE_INPUT);
#elif defined(LL_TIM_BREAK_FILTER_FDIV1)
			LL_TIM_ConfigBRK(timer, cfg->break_polarity, LL_TIM_BREAK_FILTER_FDIV1);
#else
			LL_TIM_ConfigBRK(timer, cfg->break_polarity);
#endif
			LL_TIM_EnableBRK(timer);
		}

#if defined(IS_TIM_BKIN2_INSTANCE)
		/* Configure break input 2 if supported */
		if (IS_TIM_BKIN2_INSTANCE(timer) && cfg->break2_enable) {
#if defined(TIM_BDTR_BKBID)
			LL_TIM_ConfigBRK2(timer, cfg->break2_polarity,
					  LL_TIM_BREAK2_FILTER_FDIV1,
					  LL_TIM_BREAK2_AFMODE_INPUT);
#else
			LL_TIM_ConfigBRK2(timer, cfg->break2_polarity,
					  LL_TIM_BREAK2_FILTER_FDIV1);
#endif
			LL_TIM_EnableBRK2(timer);
		}
#endif

		/* Enable automatic output enable */
		if (cfg->automatic_output) {
			LL_TIM_EnableAutomaticOutput(timer);
		}
	}
#endif

	return 0;
}

#define PWM(index) DT_INST_PARENT(index)

#define IRQ_CONNECT_AND_ENABLE_BY_NAME_BRK(index)                                                  \
	do {                                                                                       \
		IRQ_CONNECT(DT_IRQ_BY_NAME(PWM(index), brk, irq),                                  \
			    DT_IRQ_BY_NAME(PWM(index), brk, priority), mcpwm_stm32_brk_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_IRQ_BY_NAME(PWM(index), brk, irq));                                  \
	} while (0)

#define MCPWM_ISR_HANDLER(index)                                                                       \
	ISR_DIRECT_DECLARE(mcpwm_stm32_isr_##index)                                                    \
	{                                                                                              \
		const struct device *dev = DEVICE_DT_INST_GET(index);                                  \
		const struct mcpwm_stm32_config *cfg = dev->config;                                    \
		struct mcpwm_stm32_data *data = dev->data;                                             \
		TIM_TypeDef *timer = cfg->timer;                                                       \
                                                                                                       \
		for (uint32_t i = 0U; i < TIMER_MAX_CH; ++i) {                                         \
			if (is_capture_active[i](timer) != 0U) {                                       \
				clear_capture_interrupt[i](timer);                                     \
				mcpwm_compare_cb_t cb = data->compare_cb[i];                           \
				if (cb != NULL) {                                                      \
					cb(dev, i + 1U, data->compare_user_data[i]);                   \
				}                                                                      \
			}                                                                              \
		}                                                                                      \
                                                                                                       \
		return 0;                                                                              \
	}                                                                                              \
                                                                                                       \
	static void mcpwm_stm32_isr_##index##_init(void)                                               \
	{                                                                                              \
		COND_CODE_1(DT_IRQ_HAS_NAME(PWM(index), cc),                                         \
			    (IRQ_DIRECT_CONNECT(DT_IRQ_BY_NAME(PWM(index), cc, irq),                 \
						DT_IRQ_BY_NAME(PWM(index), cc, priority),        \
						mcpwm_stm32_isr_##index, IRQ_ZERO_LATENCY);      \
			     irq_enable(DT_IRQ_BY_NAME(PWM(index), cc, irq));),                      \
			    (IRQ_DIRECT_CONNECT(DT_IRQN(PWM(index)),                                 \
						DT_IRQ(PWM(index), priority),                    \
						mcpwm_stm32_isr_##index, IRQ_ZERO_LATENCY);      \
			     irq_enable(DT_IRQN(PWM(index)));)) \
	}

#define IRQ_CONFIG_FUNC(index)                                                                     \
	MCPWM_ISR_HANDLER(index)                                                                   \
	static void mcpwm_stm32_irq_config_func_##index(const struct device *dev)                  \
	{                                                                                          \
		struct mcpwm_stm32_data *data = dev->data;                                         \
		data->break_cb = NULL;                                                             \
		data->break_user_data = NULL;                                                      \
		for (uint32_t i = 0U; i < TIMER_MAX_CH; ++i) {                                     \
			data->compare_cb[i] = NULL;                                                \
			data->compare_user_data[i] = NULL;                                         \
		}                                                                                  \
		COND_CODE_1(DT_IRQ_HAS_NAME(PWM(index), brk),                                   \
			    (IRQ_CONNECT_AND_ENABLE_BY_NAME_BRK(index);), ());      \
		mcpwm_stm32_isr_##index##_init();                                                  \
	}

#define STM32_CC_DMA_MASK_BIT(node_id, prop, idx) BIT(DT_PROP_BY_IDX(node_id, prop, idx) - 1) |

#define STM32_CC_DMA_MASK(node_id)                                                                 \
	(DT_FOREACH_PROP_ELEM(node_id, st_cc_dma_channels, STM32_CC_DMA_MASK_BIT) 0U)

#define MCPWM_PINCTRL_DT_INST_DEFINE(index)                                                        \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(index, default), (PINCTRL_DT_INST_DEFINE(index)), ())

#define MCPWM_PINCTRL_DT_INST_DEV_CONFIG_GET(index)                                                \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(index, default),                                          \
		    (PINCTRL_DT_INST_DEV_CONFIG_GET(index)), (NULL))

DT_INST_FOREACH_STATUS_OKAY(IRQ_CONFIG_FUNC)

#define PWM_DEVICE_INIT(index)                                                                     \
	static struct mcpwm_stm32_data mcpwm_stm32_data_##index = {                                \
		.reset = RESET_DT_SPEC_GET(PWM(index)),                                            \
	};                                                                                         \
	MCPWM_PINCTRL_DT_INST_DEFINE(index);                                                       \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_CLOCKS(PWM(index));           \
	static const struct mcpwm_stm32_config mcpwm_stm32_config_##index = {                      \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(PWM(index)),                                   \
		.prescaler = DT_PROP(PWM(index), st_prescaler),                                    \
		.countermode = DT_PROP(PWM(index), st_countermode),                                \
		.master_mode = DT_PROP_OR(PWM(index), st_master_mode, false),                      \
		.slave_mode = DT_PROP_OR(PWM(index), st_slave_mode, false),                        \
		.trigger_selection = DT_PROP_OR(PWM(index), st_trigger_selection, 0),              \
		.trigger_polarity = DT_PROP_OR(PWM(index), st_trigger_polarity, false),            \
		.master_mode_selection = DT_PROP_OR(PWM(index), st_master_mode_selection, 0),      \
		.slave_mode_selection = DT_PROP_OR(PWM(index), st_slave_mode_selection, 0),        \
		.one_pulse_mode = DT_PROP_OR(PWM(index), st_one_pulse_mode, false),                \
		.repetition_counter = DT_PROP_OR(PWM(index), st_repetition_counter, 0),            \
		.automatic_output = DT_INST_PROP_OR(index, st_automatic_output, true),             \
		.lock_level = lock_level[DT_INST_ENUM_IDX_OR(index, st_lock_level, 0)],            \
		.ossr = DT_INST_PROP_OR(index, st_ossr, false),                                    \
		.ossi = DT_INST_PROP_OR(index, st_ossi, false),                                    \
		.period_ns = DT_INST_PROP(index, period),                                          \
		.cc_dma_channels_mask = COND_CODE_1(DT_INST_NODE_HAS_PROP(index, st_cc_dma_channels),                  \
				    (STM32_CC_DMA_MASK(DT_DRV_INST(index))), (0U)),                             \
			 .break_enable = DT_INST_PROP_OR(index, st_break_enable, false),           \
			 .break_polarity =                                                         \
				 break_polarity[DT_INST_ENUM_IDX_OR(index, st_break_polarity, 0)], \
			 .break2_enable = DT_INST_PROP_OR(index, st_break2_enable, false),         \
			 .break2_polarity = break2_polarity[DT_INST_ENUM_IDX_OR(                   \
				 index, st_break2_polarity, 0)],                                   \
			 .dead_time = DT_INST_PROP_OR(index, st_dead_time, 0),                     \
			 .pclken = pclken_##index, .pclk_len = DT_NUM_CLOCKS(PWM(index)),          \
			 .pcfg = MCPWM_PINCTRL_DT_INST_DEV_CONFIG_GET(index),                      \
			 .irq_config_func = mcpwm_stm32_irq_config_func_##index,                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(index, &mcpwm_stm32_init, NULL, &mcpwm_stm32_data_##index,           \
			      &mcpwm_stm32_config_##index, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,  \
			      &mcpwm_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
