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
#include <drivers/mcpwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/dsp/types.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/dt-bindings/pwm/stm32_pwm.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(st_stm32_mcpwm, CONFIG_PWM_LOG_LEVEL);

/* User break callback function pointer type */
typedef void (*mcpwm_stm32_break_cb_t)(const struct device *dev, void *user_data);

static inline int32_t __SMMULR(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b + (1LL << 31)) >> 32);
}

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

#if defined(TIM_BREAK_INPUT_SUPPORT)
static const uint32_t break_polarity[] = {
	LL_TIM_BREAK_POLARITY_LOW,
	LL_TIM_BREAK_POLARITY_HIGH};
#else
static const uint32_t break_polarity[] = {0};
#endif

#if defined(TIM_BDTR_BKBID)
static const uint32_t break2_polarity[] = {
	LL_TIM_BREAK2_POLARITY_LOW,
	LL_TIM_BREAK2_POLARITY_HIGH};
#else
static const uint32_t break2_polarity[] = {0};
#endif

/** PWM data. */
struct mcpwm_stm32_data
{
	/** Timer clock (Hz). */
	uint32_t tim_clk;
	/** Calculated period cycles (adjusted for counter mode). */
    int32_t period_cycles;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
	/* User break callback */
	mcpwm_stm32_break_cb_t user_break_cb;
	void *user_data;
};

/** PWM configuration. */
struct mcpwm_stm32_config
{
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	uint32_t trigger_selection;
	uint32_t master_mode_selection;
	uint32_t slave_mode_selection;
	uint32_t ossr;
	uint32_t ossi;
	uint32_t dead_time;
	uint32_t lock_level;
	uint32_t break_polarity;
	uint32_t break2_polarity;
	uint32_t period_ns;
	bool slave_mode;
	bool trigger_polarity;
	bool one_pulse_mode;
	bool break_enable;
	bool break2_enable;
	bool automatic_output;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#if defined(LL_TIM_CHANNEL_CH6)
#define TIMER_HAS_6CH 1
#define TIMER_MAX_CH 6u
#else
#define TIMER_HAS_6CH 0
#define TIMER_MAX_CH 4u
#endif

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
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
void (*const mcpwm_stm32_set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *,
													 uint32_t) = {
	LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
	LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
#if TIMER_HAS_6CH
	LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
};

/**
 * Obtain LL polarity from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL polarity.
 */
static uint32_t get_polarity(mcpwm_flags_t flags)
{
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL)
	{
		return LL_TIM_OCPOLARITY_HIGH;
	}

	return LL_TIM_OCPOLARITY_LOW;
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

#if defined(CONFIG_SOC_SERIES_STM32WB0X)
	/* Timers are clocked by SYSCLK on STM32WB0 */
	apb_psc = 1;
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
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
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB1DIVR, RCC_APB1DIVR_APB1DIV));
#else
		apb_psc = STM32_APB1_PRESCALER;
#endif
	}
#if !defined(CONFIG_SOC_SERIES_STM32C0X) && !defined(CONFIG_SOC_SERIES_STM32F0X) && \
	!defined(CONFIG_SOC_SERIES_STM32G0X) && !defined(CONFIG_SOC_SERIES_STM32U0X)
	else
	{
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB2DIVR, RCC_APB2DIVR_APB2DIV));
#else
		apb_psc = STM32_APB2_PRESCALER;
#endif
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

static void mcpwm_stm32_brk_isr(const struct device *dev)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	struct mcpwm_stm32_data *data = dev->data;
	int status = 0;
	bool break_occurred = false;

	/* Handle break interrupt */
	if (IS_TIM_BREAK_INSTANCE(cfg->timer))
	{
		if (LL_TIM_IsActiveFlag_BRK(cfg->timer))
		{
			LL_TIM_ClearFlag_BRK(cfg->timer);
			status = -EFAULT;
			break_occurred = true;
			LOG_ERR("PWM break fault detected on timer %p", cfg->timer);
		}
	}

#if defined(IS_TIM_BKIN2_INSTANCE)
	/* Handle break2 interrupt */
	if (IS_TIM_BKIN2_INSTANCE(cfg->timer))
	{
		if (LL_TIM_IsActiveFlag_BRK2(cfg->timer))
		{
			LL_TIM_ClearFlag_BRK2(cfg->timer);
			status = -EFAULT;
			break_occurred = true;
			LOG_ERR("PWM break2 fault detected on timer %p", cfg->timer);
		}
	}
#endif

	/* Call user callback if break occurred and callback is registered */
	if (break_occurred && data->user_break_cb != NULL) {
		data->user_break_cb(dev, data->user_data);
	}
}

static int mcpwm_stm32_configure(const struct device *dev, uint32_t channel, mcpwm_flags_t flags)
{
    const struct mcpwm_stm32_config *cfg = dev->config;
    uint32_t ll_channel;
    uint32_t current_ll_channel;
    uint32_t negative_ll_channel = 0;

    if (channel < 1u || channel > TIMER_MAX_CH) {
        LOG_ERR("Invalid channel (%d)", channel);
        return -EINVAL;
    }

    ll_channel = ch2ll[channel - 1u];

    /* Get complementary channel if available */
    if (channel <= ARRAY_SIZE(ch2ll_n)) {
        negative_ll_channel = ch2ll_n[channel - 1u];
    }

    /* Determine which channel to configure based on flags */
    if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
        if (!negative_ll_channel) {
            LOG_ERR("Channel %d has no complementary output", channel);
            return -EINVAL;
        }
        current_ll_channel = negative_ll_channel;
    } else {
        current_ll_channel = ll_channel;
    }

    /* Only configure if channel is not already enabled */
    if (!LL_TIM_CC_IsEnabledChannel(cfg->timer, current_ll_channel)) {
        LL_TIM_OC_InitTypeDef oc_init;

        LL_TIM_OC_StructInit(&oc_init);
        oc_init.OCMode = LL_TIM_OCMODE_PWM1;
        oc_init.CompareValue = 0;

#if defined(LL_TIM_CHANNEL_CH1N)
        if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
            /* Configuring complementary output */
            oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
            oc_init.OCNPolarity = get_polarity(flags);

            /* Inherit polarity from positive output if it's already configured */
            if (LL_TIM_CC_IsEnabledChannel(cfg->timer, ll_channel)) {
                oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
                oc_init.OCPolarity = LL_TIM_OC_GetPolarity(cfg->timer, ll_channel);
            } else {
                oc_init.OCState = LL_TIM_OCSTATE_DISABLE;
                oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH; /* Default */
            }
        } else {
            /* Configuring positive output */
            oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
            oc_init.OCPolarity = get_polarity(flags);

            /* Configure complementary output if it exists and isn't already configured */
            if (negative_ll_channel) {
                if (LL_TIM_CC_IsEnabledChannel(cfg->timer, negative_ll_channel)) {
                    /* Inherit existing complementary settings */
                    oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
                    oc_init.OCNPolarity = LL_TIM_OC_GetPolarity(cfg->timer, negative_ll_channel);
                } else {
                    /* Disable complementary output by default */
                    oc_init.OCNState = LL_TIM_OCSTATE_DISABLE;
                    oc_init.OCNPolarity = LL_TIM_OCPOLARITY_HIGH; /* Default */
                }
            }
        }
#else
        /* No complementary channels available */
        oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
        oc_init.OCPolarity = get_polarity(flags);
#endif

        /* Initialize the output compare channel */
        if (LL_TIM_OC_Init(cfg->timer, ll_channel, &oc_init) != SUCCESS) {
            LOG_ERR("Could not initialize timer channel %u output", channel);
            return -EIO;
        }

        /* Enable output compare preload for this channel */
        LL_TIM_OC_EnablePreload(cfg->timer, ll_channel);
    }

    return 0;
}

static int mcpwm_stm32_enable(const struct device *dev, uint32_t channel)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	uint32_t ll_channel;
	uint32_t negative_ll_channel = 0;

	if (channel < 1u || channel > TIMER_MAX_CH)
	{
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}
	ll_channel = ch2ll[channel - 1u];
	
	/* Get complementary channel if available */
	if (channel <= ARRAY_SIZE(ch2ll_n)) {
		negative_ll_channel = ch2ll_n[channel - 1u];
	}

	/* Enable main channel and complementary channel (if it exists) */
	LL_TIM_CC_EnableChannel(cfg->timer, ll_channel | negative_ll_channel);

	return 0;
}

static int mcpwm_stm32_disable(const struct device *dev, uint32_t channel)
{
	const struct mcpwm_stm32_config *cfg = dev->config;
	uint32_t ll_channel;
	uint32_t negative_ll_channel = 0;

	if (channel < 1u || channel > TIMER_MAX_CH)
	{
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}
	ll_channel = ch2ll[channel - 1u];
	
	/* Get complementary channel if available */
	if (channel <= ARRAY_SIZE(ch2ll_n)) {
		negative_ll_channel = ch2ll_n[channel - 1u];
	}

	/* Disable main channel and complementary channel (if it exists) */
	LL_TIM_CC_DisableChannel(cfg->timer, ll_channel | negative_ll_channel);

	return 0;
}

static int mcpwm_stm32_set_duty_cycle(const struct device *dev, uint32_t channel,
                                      q31_t duty_cycle)
{
    const struct mcpwm_stm32_config *cfg = dev->config;
    const struct mcpwm_stm32_data *data = dev->data;
    
    if (channel < 1u || channel > TIMER_MAX_CH) {
        LOG_ERR("Invalid channel (%d)", channel);
        return -EINVAL;
    }
    
    uint32_t pulse_cycles = (uint32_t)__SMMULR(duty_cycle, data->period_cycles);
    mcpwm_stm32_set_timer_compare[channel - 1u](cfg->timer, pulse_cycles);
    return 0;
}

static int mcpwm_stm32_register_break_callback(const struct device *dev, 
                                            mcpwm_stm32_break_cb_t callback, 
                                            void *user_data)
{
    struct mcpwm_stm32_data *data = dev->data;
    
    data->user_break_cb = callback;
    data->user_data = user_data;
    
    LOG_DBG("User break callback registered for PWM device %s", dev->name);
    return 0;
}

static DEVICE_API(mcpwm, mcpwm_stm32_driver_api) = {
	.configure = mcpwm_stm32_configure,
	.enable = mcpwm_stm32_enable,
	.disable = mcpwm_stm32_disable,
	.set_duty_cycle = mcpwm_stm32_set_duty_cycle,
};

static int mcpwm_stm32_init(const struct device *dev)
{
	struct mcpwm_stm32_data *data = dev->data;
	const struct mcpwm_stm32_config *cfg = dev->config;

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

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&data->reset);

	/* configure pinmux */
	r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (r < 0)
	{
		LOG_ERR("PWM pinctrl setup failed (%d)", r);
		return r;
	}

	/* initialize timer */
	LL_TIM_StructInit(&init);

	init.Prescaler = cfg->prescaler;
	init.CounterMode = cfg->countermode;
	init.Autoreload = 0u;
	init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

	if (LL_TIM_Init(cfg->timer, &init) != SUCCESS)
	{
		LOG_ERR("Could not initialize timer");
		return -EIO;
	}

	/* Configure Master/Slave Mode controller */
	if (cfg->slave_mode)
	{
		/* Configure as slave timer */
		LL_TIM_SetSlaveMode(cfg->timer, cfg->slave_mode_selection);
		LL_TIM_SetTriggerInput(cfg->timer, cfg->trigger_selection);

		/* Configure ETR if using external trigger */
		if (cfg->trigger_selection == LL_TIM_TS_ETRF)
		{
			LL_TIM_ConfigETR(cfg->timer, cfg->trigger_polarity ? LL_TIM_ETR_POLARITY_INVERTED : LL_TIM_ETR_POLARITY_NONINVERTED,
							 LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
		}

		/* Only enable master-slave mode if this timer is also a master */
		if (cfg->master_mode_selection != 0)
		{
			LL_TIM_EnableMasterSlaveMode(cfg->timer);
		}
	}

	/* Configure master mode output (can be set regardless of slave mode) */
	if (cfg->master_mode_selection != 0)
	{
		LL_TIM_SetTriggerOutput(cfg->timer, cfg->master_mode_selection);
		if (!cfg->slave_mode)
		{
			LL_TIM_DisableMasterSlaveMode(cfg->timer);
		}
	}

#if !defined(CONFIG_SOC_SERIES_STM32L0X) && !defined(CONFIG_SOC_SERIES_STM32L1X)
	if (IS_TIM_BREAK_INSTANCE(cfg->timer))
	{
		if (cfg->break_enable || cfg->break2_enable || cfg->dead_time > 0 || cfg->lock_level > 0 || cfg->automatic_output)
		{
			LL_TIM_BDTR_InitTypeDef bdtr_init;

			/* Initialize BDTR structure with default values */
			LL_TIM_BDTR_StructInit(&bdtr_init);

			/* Configure break and dead time settings */
			bdtr_init.OSSRState = cfg->ossr ? LL_TIM_OSSR_ENABLE : LL_TIM_OSSR_DISABLE;
			bdtr_init.OSSIState = cfg->ossi ? LL_TIM_OSSI_ENABLE : LL_TIM_OSSI_DISABLE;
			bdtr_init.DeadTime = cfg->dead_time;
			bdtr_init.BreakState = cfg->break_enable ? LL_TIM_BREAK_ENABLE : LL_TIM_BREAK_DISABLE;
			bdtr_init.BreakPolarity = cfg->break_polarity;
			bdtr_init.AutomaticOutput = cfg->automatic_output ? LL_TIM_AUTOMATICOUTPUT_ENABLE : LL_TIM_AUTOMATICOUTPUT_DISABLE;

			/* Configure lock level */
			switch (cfg->lock_level)
			{
			case 1:
				bdtr_init.LockLevel = LL_TIM_LOCKLEVEL_1;
				break;
			case 2:
				bdtr_init.LockLevel = LL_TIM_LOCKLEVEL_2;
				break;
			case 3:
				bdtr_init.LockLevel = LL_TIM_LOCKLEVEL_3;
				break;
			default:
				bdtr_init.LockLevel = LL_TIM_LOCKLEVEL_OFF;
				break;
			}
#if defined(IS_TIM_BKIN2_INSTANCE)
			/* Configure BRK2 if supported */
			if (IS_TIM_BKIN2_INSTANCE(cfg->timer))
			{
				bdtr_init.Break2State = cfg->break2_enable ? LL_TIM_BREAK2_ENABLE : LL_TIM_BREAK2_DISABLE;
				bdtr_init.Break2Polarity = cfg->break2_polarity;
			}

			/* Apply BDTR configuration */
			if (LL_TIM_BDTR_Init(cfg->timer, &bdtr_init) != SUCCESS)
			{
				LOG_ERR("Could not initialize timer BDTR");
				return -EIO;
			}
#endif
		}

		cfg->irq_config_func(dev);

		/* enable outputs and counter */
		LL_TIM_EnableAllOutputs(cfg->timer);
	}
#endif

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
    if (!IS_TIM_32B_COUNTER_INSTANCE(cfg->timer) && period_cycles > UINT16_MAX) {
        LOG_ERR("Period too large for 16-bit timer: %u", period_cycles);
        return -EINVAL;
    }

    /* Store the calculated period for later use */
    data->period_cycles = period_cycles + 1u;

    /* Set the timer auto-reload register */
    LL_TIM_SetAutoReload(cfg->timer, period_cycles);

	/* Enable auto-reload preload (affects all channels) */
	LL_TIM_EnableARRPreload(cfg->timer);

	LL_TIM_EnableCounter(cfg->timer);

	/* Initialize user callback fields */
	data->user_break_cb = NULL;
	data->user_data = NULL;

	return 0;
}

#define PWM(index) DT_INST_PARENT(index)

#define IRQ_CONNECT_AND_ENABLE_BY_NAME(index, name)                   \
	{                                                                 \
		IRQ_CONNECT(DT_IRQ_BY_NAME(PWM(index), name, irq),            \
					DT_IRQ_BY_NAME(PWM(index), name, priority),       \
					mcpwm_stm32_brk_isr, DEVICE_DT_INST_GET(index), 0); \
		irq_enable(DT_IRQ_BY_NAME(PWM(index), name, irq));            \
	}

#define IRQ_CONNECT_AND_ENABLE_DEFAULT(index)                         \
	{                                                                 \
		IRQ_CONNECT(DT_IRQN(PWM(index)),                              \
					DT_IRQ(PWM(index), priority),                     \
					mcpwm_stm32_brk_isr, DEVICE_DT_INST_GET(index), 0); \
		irq_enable(DT_IRQN(PWM(index)));                              \
	}

#define IRQ_CONFIG_FUNC(index)                                                  \
	static void mcpwm_stm32_brk_irq_config_func_##index(const struct device *dev) \
	{                                                                           \
		COND_CODE_1(DT_IRQ_HAS_NAME(PWM(index), brk),                           \
					(IRQ_CONNECT_AND_ENABLE_BY_NAME(index, brk)),               \
					(IRQ_CONNECT_AND_ENABLE_DEFAULT(index)));                   \
	}

#define DT_INST_CLK(index, inst)                \
	{                                           \
		.bus = DT_CLOCKS_CELL(PWM(index), bus), \
		.enr = DT_CLOCKS_CELL(PWM(index), bits)}

#define PWM_DEVICE_INIT(index)                                                                 \
	static struct mcpwm_stm32_data mcpwm_stm32_data_##index = {                                    \
		.reset = RESET_DT_SPEC_GET(PWM(index)),                                                \
	};                                                                                         \
                                                                                               \
	IRQ_CONFIG_FUNC(index)                                                                     \
                                                                                               \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                               \
	static const struct mcpwm_stm32_config mcpwm_stm32_config_##index = {                          \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(PWM(index)),                                       \
		.prescaler = DT_PROP(PWM(index), st_prescaler),                                        \
		.countermode = DT_PROP(PWM(index), st_countermode),                                    \
		.slave_mode = DT_PROP_OR(PWM(index), st_slave_mode, false),                            \
		.trigger_selection = DT_PROP_OR(PWM(index), st_trigger_selection, 0),                  \
		.trigger_polarity = DT_PROP_OR(PWM(index), st_trigger_polarity, false),                \
		.master_mode_selection = DT_PROP_OR(PWM(index), st_master_mode_selection, 0),          \
		.slave_mode_selection = DT_PROP_OR(PWM(index), st_slave_mode_selection, 0),            \
		.automatic_output = DT_INST_PROP_OR(index, st_automatic_output, true),                 \
		.lock_level = DT_INST_PROP_OR(index, st_lock_level, 0),                                \
		.ossr = DT_INST_PROP_OR(index, st_ossr, false),                                        \
		.ossi = DT_INST_PROP_OR(index, st_ossi, false),                                        \
		.one_pulse_mode = DT_INST_PROP_OR(index, st_one_pulse_mode, false),                    \
		.period_ns = DT_INST_PROP(index, period),											   \
		.break_enable = DT_INST_PROP_OR(index, st_break_enable, false),                        \
		.break_polarity = break_polarity[DT_INST_ENUM_IDX_OR(index, st_break_polarity, 0)],    \
		.break2_enable = DT_INST_PROP_OR(index, st_break2_enable, false),                      \
		.break2_polarity = break2_polarity[DT_INST_ENUM_IDX_OR(index, st_break2_polarity, 0)], \
		.dead_time = DT_INST_PROP_OR(index, st_dead_time, 0),                                  \
		.pclken = DT_INST_CLK(index, timer),                                                   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                         \
		.irq_config_func = mcpwm_stm32_brk_irq_config_func_##index,                              \
	};                                                                                         \
                                                                                               \
	DEVICE_DT_INST_DEFINE(index, &mcpwm_stm32_init, NULL,                                        \
						  &mcpwm_stm32_data_##index,                                             \
						  &mcpwm_stm32_config_##index, POST_KERNEL,                              \
						  CONFIG_PWM_INIT_PRIORITY,                                            \
						  &mcpwm_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
