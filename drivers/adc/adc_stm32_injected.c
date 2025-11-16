/*
 * Copyright (c) 2018 Kokoon Technology Limited
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 * Copyright (c) 2019 Endre Karlson
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2021 Marius Scholtz, RIC Electronics
 * Copyright (c) 2023 Hein Wessels, Nobleo Technology
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_adc_injected

#include <errno.h>

#include <zephyr/drivers/adc.h>
#include <drivers/adc_injected.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/toolchain.h>
#include <soc.h>
#include <stm32_bitops.h>
#include <stm32_cache.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <stm32_ll_adc.h>
#include <stm32_ll_system.h>
#if defined(CONFIG_SOC_SERIES_STM32N6X) || \
	defined(CONFIG_SOC_SERIES_STM32U3X) || \
	defined(CONFIG_SOC_SERIES_STM32U5X)
#include <stm32_ll_pwr.h>
#endif /* CONFIG_SOC_SERIES_STM32U5X */

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_stm32_injected);

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/dt-bindings/adc/stm32_adc.h>
#include <zephyr/irq.h>

#include <zephyr/linker/linker-defs.h>

/* Here are some redefinitions of ADC versions for better readability */
#if defined(CONFIG_SOC_SERIES_STM32F3X)
#if defined(ADC1_V2_5)
#define STM32F37X_ADC /* F37x */
#elif defined(ADC5_V1_1)
#define STM32F3XX_ADC /* Other F3xx */
#endif /* ADC5_V1_1 */
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
#if defined(ADC_VER_V5_V90)
#define STM32H72X_ADC /* H72x and H73x */
#elif defined(ADC_VER_V5_X)
#define STM32H74X_ADC /* H74x and H75x */
#elif defined(ADC_VER_V5_3)
#define STM32H7AX_ADC /* H7Ax and H7Bx */
#endif /* ADC_VER_V5_3 */
#endif /* CONFIG_SOC_SERIES_STM32H7X */

/*
 * Other ADC versions:
 * compat st_stm32f1_adc -> STM32F1, F37x (ADC1_V2_5)
 * compat st_stm32f4_adc -> STM32F2, F4, F7, L1
 */

/* Clock source values */
#define SYNC			1
#define ASYNC			2

/* Sequencer type */
#define SEQUENCER_FIXED		0
#define SEQUENCER_PROGRAMMABLE	1

/* Internal regulator type */
#define INTERNAL_REGULATOR_NONE			0
#define INTERNAL_REGULATOR_STARTUP_SW_DELAY	1
#define INTERNAL_REGULATOR_STARTUP_HW_STATUS	2

#define ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(value) \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_STRING_PROP, \
					   st_adc_internal_regulator,\
					   value, INTERNAL_REGULATOR_) 0)

#define ANY_ADC_HAS_DEEP_POWERDOWN \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_PROP_OR, \
					   st_adc_has_deep_powerdown,\
					   0, 1) 0)

#define ANY_ADC_HAS_CHANNEL_PRESELECTION \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_PROP_OR, \
					   st_adc_has_channel_preselection,\
					   0, 1) 0)

#define ANY_ADC_HAS_DIFFERENTIAL_SUPPORT \
	(DT_INST_FOREACH_STATUS_OKAY_VARGS(IS_EQ_PROP_OR, \
					   st_adc_has_differential_support,\
					   0, 1) 0)

#define ANY_CHILD_NODE_IS_DIFFERENTIAL(inst) \
	(DT_INST_FOREACH_CHILD_VARGS(inst, IS_EQ_NODE_PROP_OR, \
				     zephyr_differential, \
				     0, 1) 0)

#define IS_EQ_PROP_OR(inst, prop, default_value, compare_value) \
	IS_EQ(DT_INST_PROP_OR(inst, prop, default_value), compare_value) ||

#define IS_EQ_NODE_PROP_OR(node, prop, default_value, compare_value) \
	IS_EQ(DT_PROP_OR(node, prop, default_value), compare_value) ||

#define IS_EQ_STRING_PROP(inst, prop, compare_value, prefix) \
	IS_EQ(CONCAT(prefix, DT_INST_STRING_UPPER_TOKEN(inst, prop)), compare_value) ||

/* Number of different sampling time values */
#define STM32_NB_SAMPLING_TIME	8

struct adc_stm32_data {
	const struct device *dev;
	uint8_t resolution;
	
	/* Injected-specific fields */
	adc_injected_callback_t callback;
	void *user_data;
	
	/* Error callback (optional) */
	adc_injected_error_callback_t error_callback;
	void *error_user_data;
	
	volatile q31_t inj_values[4];  /* Converted Q31 values */
};

struct adc_stm32_cfg {
	ADC_TypeDef *base;
	void (*irq_cfg_func)(void);
	const struct stm32_pclken pclken;
	const struct stm32_pclken pclken_ker;
	const struct stm32_pclken pclken_pre;
	uint32_t clk_prescaler;
	const struct pinctrl_dev_config *pcfg;
	const uint16_t sampling_time_table[STM32_NB_SAMPLING_TIME];
	int8_t num_sampling_time_common_channels;
	int8_t sequencer_type;
	int8_t internal_regulator;
	bool has_pclken_ker		:1;
	bool has_pclken_pre		:1;
	bool has_deep_powerdown		:1;
	bool has_channel_preselection	:1;
	bool has_differential_support	:1;
	bool differential_channels_used	:1;
	/* Injected-specific config fields (before flexible array) */
	uint8_t num_channels;         /* Number of injected channels (1-4) */
	uint32_t channel_ranks[4];    /* LL_ADC_INJ_RANK_1..4 for each channel */
	uint32_t channel_ids[4];      /* ADC channel numbers (0-19) */
	uint32_t channel_acq_times[4]; /* Acquisition time for each channel */
	bool channel_differential[4]; /* Differential mode flags */
	uint32_t trigger_source;      /* LL_ADC_INJ_TRIG_xxx constant */
	uint32_t trigger_edge;        /* LL_ADC_INJ_TRIG_EXT_xxx constant */
	int8_t res_table_size;
	const uint32_t res_table[];
};

/* Forward declarations for injected configuration functions */
static int adc_stm32_configure_injected_channels(const struct device *dev);
static int adc_stm32_configure_injected_trigger(const struct device *dev);

/*
 * Enable ADC peripheral, and wait until ready if required by SOC.
 */
static int adc_stm32_enable(ADC_TypeDef *adc)
{
	if (LL_ADC_IsEnabled(adc) == 1UL) {
		return 0;
	}

#if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) && \
	!DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_adc)
	LL_ADC_ClearFlag_ADRDY(adc);
	LL_ADC_Enable(adc);

	/*
	 * Enabling ADC modules in many series may fail if they are
	 * still not stabilized, this will wait for a short time (about 1ms)
	 * to ensure ADC modules are properly enabled.
	 */
	uint32_t count_timeout = 0;

	while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0) {
#ifdef CONFIG_SOC_SERIES_STM32F0X
		/* For F0, continue to write ADEN=1 until ADRDY=1 */
		if (LL_ADC_IsEnabled(adc) == 0UL) {
			LL_ADC_Enable(adc);
		}
#endif /* CONFIG_SOC_SERIES_STM32F0X */
		count_timeout++;
		k_busy_wait(100);
		if (count_timeout >= 10) {
			return -ETIMEDOUT;
		}
	}
#else
	/*
	 * On STM32F1, F2, F37x, F4, F7 and L1, do not re-enable the ADC.
	 * On F1 and F37x if ADON holds 1 (LL_ADC_IsEnabled is true) and 1 is
	 * written, then conversion starts. That's not what is expected.
	 */
	LL_ADC_Enable(adc);
#endif

	return 0;
}

/*
 * Disable ADC peripheral, and wait until it is disabled
 */
static void adc_stm32_disable(ADC_TypeDef *adc)
{
	if (LL_ADC_IsEnabled(adc) != 1UL) {
		return;
	}

	/* Stop ongoing conversion if any
	 * Software must poll ADSTART (or JADSTART) until the bit is reset before assuming
	 * the ADC is completely stopped.
	 */

#if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) && \
	!DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_adc)
	if (LL_ADC_REG_IsConversionOngoing(adc)) {
		LL_ADC_REG_StopConversion(adc);
		while (LL_ADC_REG_IsConversionOngoing(adc)) {
		}
	}
#endif

#if !defined(CONFIG_SOC_SERIES_STM32C0X) && \
	!defined(CONFIG_SOC_SERIES_STM32F0X) && \
	!DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) && \
	!DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_adc) && \
	!defined(CONFIG_SOC_SERIES_STM32G0X) && \
	!defined(CONFIG_SOC_SERIES_STM32L0X) && \
	!defined(CONFIG_SOC_SERIES_STM32U0X) && \
	!defined(CONFIG_SOC_SERIES_STM32WBAX) && \
	!defined(CONFIG_SOC_SERIES_STM32WLX)
	if (LL_ADC_INJ_IsConversionOngoing(adc)) {
		LL_ADC_INJ_StopConversion(adc);
		while (LL_ADC_INJ_IsConversionOngoing(adc)) {
		}
	}
#endif

	LL_ADC_Disable(adc);

	/* Wait ADC is fully disabled so that we don't leave the driver into intermediate state
	 * which could prevent enabling the peripheral
	 */
	while (LL_ADC_IsEnabled(adc) == 1UL) {
	}
}

#if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_adc)

#define HAS_CALIBRATION

/* Number of ADC clock cycles to wait before of after starting calibration */
#if defined(LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES)
#define ADC_DELAY_CALIB_ADC_CYCLES	LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES
#elif defined(LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES)
#define ADC_DELAY_CALIB_ADC_CYCLES	LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES
#elif defined(LL_ADC_DELAY_DISABLE_CALIB_ADC_CYCLES)
#define ADC_DELAY_CALIB_ADC_CYCLES	LL_ADC_DELAY_DISABLE_CALIB_ADC_CYCLES
#endif

static void adc_stm32_calibration_delay(const struct device *dev)
{
	/*
	 * Calibration of F1 and F3 (ADC1_V2_5) must start two cycles after ADON
	 * is set.
	 * Other ADC modules have to wait for some cycles after calibration to
	 * be enabled.
	 */
	const struct adc_stm32_cfg *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	uint32_t adc_rate, wait_cycles;

	if (clock_control_get_rate(clk, (clock_control_subsys_t)&config->pclken, &adc_rate) < 0) {
		LOG_ERR("ADC clock rate get error.");
	}

	if (adc_rate == 0) {
		LOG_ERR("ADC Clock rate null");
		return;
	}
	wait_cycles = SystemCoreClock / adc_rate *
		      ADC_DELAY_CALIB_ADC_CYCLES;

	for (int i = wait_cycles; i >= 0; i--) {
	}
}

#if defined(CONFIG_SOC_SERIES_STM32N6X)
/* Number of ADC measurement during calibration procedure */
#define ADC_CALIBRATION_STEPS (8U)

static void adc_stm32_calibration_measure(ADC_TypeDef *adc, uint32_t *calibration_factor)
{
	uint32_t calib_step;
	uint32_t calib_factor_avg = 0;
	uint8_t done = 0;

	do {
		for (calib_step = 0; calib_step < ADC_CALIBRATION_STEPS; calib_step++) {
			LL_ADC_REG_StartConversion(adc);
			while (LL_ADC_REG_IsConversionOngoing(adc) != 0UL) {
			}

			calib_factor_avg += LL_ADC_REG_ReadConversionData32(adc);
		}

		/* Compute the average data */
		calib_factor_avg /= ADC_CALIBRATION_STEPS;

		if ((calib_factor_avg == 0) && (LL_ADC_IsCalibrationOffsetEnabled(adc) == 0UL)) {
			/* If average is 0 and offset is disabled
			 * set offset and repeat measurements
			 */
			LL_ADC_EnableCalibrationOffset(adc);
		} else {
			*calibration_factor = (uint32_t)(calib_factor_avg);
			done = 1;
		}
	} while (done == 0);
}
#endif

static void adc_stm32_calibration_start(const struct device *dev, bool single_ended)
{
	const struct adc_stm32_cfg *config =
		(const struct adc_stm32_cfg *)dev->config;
	ADC_TypeDef *adc = config->base;
#if defined(LL_ADC_SINGLE_ENDED) && defined(LL_ADC_DIFFERENTIAL_ENDED)
	uint32_t calib_type = single_ended ? LL_ADC_SINGLE_ENDED : LL_ADC_DIFFERENTIAL_ENDED;
#else
	ARG_UNUSED(single_ended);
#endif

#if defined(STM32F3XX_ADC) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32L5X) || \
	defined(CONFIG_SOC_SERIES_STM32H5X) || \
	defined(CONFIG_SOC_SERIES_STM32H7RSX) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X)
	LL_ADC_StartCalibration(adc, calib_type);
#elif defined(CONFIG_SOC_SERIES_STM32C0X) || \
	defined(CONFIG_SOC_SERIES_STM32F0X) || \
	DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) || \
	defined(CONFIG_SOC_SERIES_STM32G0X) || \
	defined(CONFIG_SOC_SERIES_STM32L0X) || \
	defined(CONFIG_SOC_SERIES_STM32U0X) || \
	defined(CONFIG_SOC_SERIES_STM32WLX) || \
	defined(CONFIG_SOC_SERIES_STM32WBAX)

	LL_ADC_StartCalibration(adc);
#elif defined(CONFIG_SOC_SERIES_STM32U5X)
	ARG_UNUSED(calib_type);
	if (adc != ADC4) {
		uint32_t dev_id = LL_DBGMCU_GetDeviceID();
		uint32_t rev_id = LL_DBGMCU_GetRevisionID();

		/* Some U5 implement an extended calibration to enhance ADC performance.
		 * It is not available for ADC4.
		 * It is available on all U5 except U575/585 (dev ID 482) revision X (rev ID 2001).
		 * The code below applies the procedure described in RM0456 in the ADC chapter:
		 * "Extended calibration mode"
		 */
		if ((dev_id != 0x482UL) && (rev_id != 0x2001UL)) {
			adc_stm32_enable(adc);
			stm32_reg_modify_bits(&adc->CR, ADC_CR_CALINDEX,
					      0x9UL << ADC_CR_CALINDEX_Pos);
			__DMB();
			stm32_reg_modify_bits(&adc->CALFACT2, 0xFFFFFF00UL, 0x03021100UL);
			__DMB();
			stm32_reg_set_bits(&adc->CALFACT, ADC_CALFACT_LATCH_COEF);
			adc_stm32_disable(adc);
		}
	}
	LL_ADC_StartCalibration(adc, LL_ADC_CALIB_OFFSET);
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_ADC_StartCalibration(adc, LL_ADC_CALIB_OFFSET, calib_type);
#elif defined(CONFIG_SOC_SERIES_STM32N6X)
	uint32_t calibration_factor;

	ARG_UNUSED(calib_type);
	/* Start ADC calibration */
	LL_ADC_StartCalibration(adc, LL_ADC_SINGLE_ENDED);
	/* Disable additional offset before calibration start */
	LL_ADC_DisableCalibrationOffset(adc);

	adc_stm32_calibration_measure(adc, &calibration_factor);

	LL_ADC_SetCalibrationFactor(adc, LL_ADC_SINGLE_ENDED, calibration_factor);
	LL_ADC_StopCalibration(adc);
#endif
	/* Make sure ADCAL is cleared before returning for proper operations
	 * on the ADC control register, for enabling the peripheral for example
	 */
	while (LL_ADC_IsCalibrationOnGoing(adc)) {
	}
}

static int adc_stm32_calibrate(const struct device *dev)
{
	const struct adc_stm32_cfg *config =
		(const struct adc_stm32_cfg *)dev->config;
	ADC_TypeDef *adc = config->base;
	int err;

#if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) && \
	!defined(CONFIG_SOC_SERIES_STM32N6X)
	adc_stm32_disable(adc);
	adc_stm32_calibration_start(dev, true);
	if (config->differential_channels_used) {
		adc_stm32_calibration_start(dev, false);
	}
	adc_stm32_calibration_delay(dev);
#endif /* !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) */

	err = adc_stm32_enable(adc);
	if (err < 0) {
		return err;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) || \
	defined(CONFIG_SOC_SERIES_STM32N6X)
	adc_stm32_calibration_delay(dev);
	adc_stm32_calibration_start(dev, true);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_adc) */

#if defined(CONFIG_SOC_SERIES_STM32H7X) && \
	defined(CONFIG_CPU_CORTEX_M7)
	/*
	 * To ensure linearity the factory calibration values
	 * should be loaded on initialization.
	 */
	uint32_t channel_offset = 0U;
	uint32_t linear_calib_buffer = 0U;

	if (adc == ADC1) {
		channel_offset = 0UL;
	} else if (adc == ADC2) {
		channel_offset = 8UL;
	} else   /*Case ADC3*/ {
		channel_offset = 16UL;
	}
	/* Read factory calibration factors */
	for (uint32_t count = 0UL; count < ADC_LINEAR_CALIB_REG_COUNT; count++) {
		linear_calib_buffer = *(uint32_t *)(
			ADC_LINEAR_CALIB_REG_1_ADDR + channel_offset + count
		);

		LL_ADC_SetCalibrationLinearFactor(
			adc, LL_ADC_CALIB_LINEARITY_WORD1 << count,
			linear_calib_buffer
		);
	}
#endif /* CONFIG_SOC_SERIES_STM32H7X */

	return 0;
}
#endif /* !DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_adc) */

static uint8_t get_reg_value(const struct device *dev, uint32_t reg,
			     uint32_t shift, uint32_t mask)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;

	uintptr_t addr = (uintptr_t)adc + reg;

	return ((*(volatile uint32_t *)addr >> shift) & mask);
}

static void set_reg_value(const struct device *dev, uint32_t reg,
			  uint32_t shift, uint32_t mask, uint32_t value)
{
	const struct adc_stm32_cfg *config = dev->config;
	size_t reg32_offset = reg / sizeof(uint32_t);
	volatile uint32_t *addr = (volatile uint32_t *)config->base + reg32_offset;

	stm32_reg_modify_bits(addr, mask << shift, value << shift);
}

static int set_resolution(const struct device *dev, uint8_t resolution)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;
	uint8_t res_reg_addr = 0xFF;
	uint8_t res_shift = 0;
	uint8_t res_mask = 0;
	uint8_t res_reg_val = 0;
	int i;

	for (i = 0; i < config->res_table_size; i++) {
		if (resolution == STM32_ADC_GET_REAL_VAL(config->res_table[i])) {
			res_reg_addr = STM32_ADC_GET_REG(config->res_table[i]);
			res_shift = STM32_ADC_GET_SHIFT(config->res_table[i]);
			res_mask = STM32_ADC_GET_MASK(config->res_table[i]);
			res_reg_val = STM32_ADC_GET_REG_VAL(config->res_table[i]);
			break;
		}
	}

	if (i == config->res_table_size) {
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	/*
	 * Some MCUs (like STM32F1x) have no register to configure resolution.
	 * These MCUs have a register address value of 0xFF and should be
	 * ignored.
	 */
	if (res_reg_addr != 0xFF) {
		/*
		 * We don't use LL_ADC_SetResolution and LL_ADC_GetResolution
		 * because they don't strictly use hardware resolution values
		 * and makes internal conversions for some series.
		 * (see stm32h7xx_ll_adc.h)
		 * Instead we set the register ourselves if needed.
		 */
		if (get_reg_value(dev, res_reg_addr, res_shift, res_mask) != res_reg_val) {
			/*
			 * Writing ADC_CFGR1 register while ADEN bit is set
			 * resets RES[1:0] bitfield. We need to disable and enable adc.
			 */
			adc_stm32_disable(adc);
			set_reg_value(dev, res_reg_addr, res_shift, res_mask, res_reg_val);
		}
	}

	return 0;
}

#if defined(CONFIG_SOC_SERIES_STM32C0X) ||                                                     \
    defined(CONFIG_SOC_SERIES_STM32G0X) ||                                                     \
	defined(CONFIG_SOC_SERIES_STM32L0X) ||                                                     \
	defined(CONFIG_SOC_SERIES_STM32U0X) ||                                                     \
	(defined(CONFIG_SOC_SERIES_STM32WBX) && defined(ADC_SUPPORT_2_5_MSPS)) ||                  \
	defined(CONFIG_SOC_SERIES_STM32WLX)
#define ADC_STM32_HAS_INDIVIDUAL_CLOCKS
#endif

#if defined(CONFIG_SOC_SERIES_STM32H7X) || defined(ADC_STM32_HAS_INDIVIDUAL_CLOCKS)
static bool adc_stm32_is_clk_sync(const struct adc_stm32_cfg *config)
{
	if (config->clk_prescaler == LL_ADC_CLOCK_SYNC_PCLK_DIV1 ||
	    config->clk_prescaler == LL_ADC_CLOCK_SYNC_PCLK_DIV2 ||
	    config->clk_prescaler == LL_ADC_CLOCK_SYNC_PCLK_DIV4) {
		return true;
	}

	return false;
}
#endif

#if defined(CONFIG_SOC_SERIES_STM32H7X)
static inline int adc_stm32_get_input_freq_prescaler(void)
{
	int presc = 2;

#ifdef STM32H74X_ADC
	/* For revision Y we have no prescaler of 2 */
	if (LL_DBGMCU_GetRevisionID() <= 0x1003) {
		presc = 1;
	}
#endif

	return presc;
}

static int adc_stm32_get_clock_prescaler(const struct adc_stm32_cfg *config)
{
	switch (config->clk_prescaler) {
	case LL_ADC_CLOCK_SYNC_PCLK_DIV1:
	case LL_ADC_CLOCK_ASYNC_DIV1:
		return 1;
	case LL_ADC_CLOCK_SYNC_PCLK_DIV2:
	case LL_ADC_CLOCK_ASYNC_DIV2:
		return 2;
	case LL_ADC_CLOCK_SYNC_PCLK_DIV4:
	case LL_ADC_CLOCK_ASYNC_DIV4:
		return 4;
	case LL_ADC_CLOCK_ASYNC_DIV6:
		return 6;
	case LL_ADC_CLOCK_ASYNC_DIV8:
		return 8;
	case LL_ADC_CLOCK_ASYNC_DIV10:
		return 10;
	case LL_ADC_CLOCK_ASYNC_DIV12:
		return 12;
	case LL_ADC_CLOCK_ASYNC_DIV16:
		return 16;
	case LL_ADC_CLOCK_ASYNC_DIV32:
		return 32;
	case LL_ADC_CLOCK_ASYNC_DIV64:
		return 64;
	case LL_ADC_CLOCK_ASYNC_DIV128:
		return 128;
	case LL_ADC_CLOCK_ASYNC_DIV256:
		return 256;
	default:
		return -EINVAL;
	}
}

static int adc_stm32h7_setup_boost(const struct adc_stm32_cfg *config, ADC_TypeDef *adc,
				   const struct device *clk)
{
	clock_control_subsys_t clk_src;
	uint32_t input_freq;
	uint32_t boost;
	int presc;

	/* Get the input frequency */
	clk_src = (clock_control_subsys_t)(adc_stm32_is_clk_sync(config) ? &config->pclken
									 : &config->pclken_ker);

	if (clock_control_get_rate(clk, clk_src, &input_freq) != 0) {
		LOG_ERR("Failed to get ADC clock frequency");
		return -EIO;
	}

	/* Adjust the pre-scaler value so that we can divide down the clock */
	presc = adc_stm32_get_clock_prescaler(config);
	if (presc < 0) {
		LOG_ERR("Invalid clock prescaler value");
		return presc;
	}

	input_freq /= presc * adc_stm32_get_input_freq_prescaler();

	if (input_freq <= KHZ(6250)) {
		boost = LL_ADC_BOOST_MODE_6MHZ25;
	} else if (input_freq <= KHZ(12500)) {
		boost = LL_ADC_BOOST_MODE_12MHZ5;
	} else if (input_freq <= MHZ(20)) {
		boost = LL_ADC_BOOST_MODE_20MHZ;
	} else if (input_freq <= MHZ(25)) {
		boost = LL_ADC_BOOST_MODE_25MHZ;
	} else if (input_freq <= MHZ(50)) {
		boost = LL_ADC_BOOST_MODE_50MHZ;
	} else {
		LOG_WRN("ADC clock frequency too high %u", input_freq);
		return -ERANGE;
	}

	LL_ADC_SetBoostMode(adc, boost);

	return 0;
}
#endif

static int adc_stm32_set_clock(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	__maybe_unused ADC_TypeDef *adc = config->base;
	int ret = 0;

	if (clock_control_on(clk, (clock_control_subsys_t)&config->pclken) != 0) {
		return -EIO;
	}

	/* Enable ADC clock source if applicable */
	if (config->has_pclken_ker &&
	    clock_control_configure(clk, (clock_control_subsys_t)&config->pclken_ker, NULL) != 0) {
		return -EIO;
	}

	/* Configure ADC prescaler (at RCC level) if applicable */
	if (config->has_pclken_pre &&
	    clock_control_configure(clk, (clock_control_subsys_t)&config->pclken_pre, NULL) != 0) {
		return -EIO;
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(st_adc_clock_source)
#if defined(CONFIG_SOC_SERIES_STM32F0X)
	LL_ADC_SetClock(adc, config->clk_prescaler);
#elif defined(ADC_STM32_HAS_INDIVIDUAL_CLOCKS)
	if (adc_stm32_is_clk_sync(config)) {
		LL_ADC_SetClock(adc, config->clk_prescaler);
	} else {
		LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc),
				      config->clk_prescaler);
		LL_ADC_SetClock(adc, LL_ADC_CLOCK_ASYNC);
	}
#else
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(adc),
			      config->clk_prescaler);

#ifdef CONFIG_SOC_SERIES_STM32H7X
	/* Set boost according to input frequency */
	ret = adc_stm32h7_setup_boost(config, adc, clk);
#endif
#endif
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(st_adc_clock_source) */

	return ret;
}

static void adc_stm32_enable_analog_supply(void)
{
#if defined(CONFIG_SOC_SERIES_STM32N6X)
	LL_PWR_EnableVddADC();
#elif defined(CONFIG_SOC_SERIES_STM32U5X) || defined(CONFIG_SOC_SERIES_STM32U3X)
	LL_PWR_EnableVDDA();
#endif /* CONFIG_SOC_SERIES_STM32U5X */
}

#ifdef CONFIG_PM_DEVICE
static void adc_stm32_disable_analog_supply(void)
{
#if defined(CONFIG_SOC_SERIES_STM32N6X)
	LL_PWR_DisableVddADC();
#elif defined(CONFIG_SOC_SERIES_STM32U5X) || defined(CONFIG_SOC_SERIES_STM32U3X)
	LL_PWR_DisableVDDA();
#endif /* CONFIG_SOC_SERIES_STM32U5X */
}
#endif

static int adc_stm32_init(const struct device *dev)
{
	struct adc_stm32_data *data = dev->data;
	const struct adc_stm32_cfg *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	__maybe_unused ADC_TypeDef *adc = config->base;
	int err;

	LOG_DBG("Initializing %s", dev->name);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	data->dev = dev;

	/*
	 * For series that use common channels for sampling time, all
	 * conversion time for all channels on one ADC instance has to
	 * be the same.
	 * For series that use two common channels, there can be up to two
	 * conversion times selected for all channels in a sequence.
	 * This additional table is for checking that the conversion time
	 * selection of all channels respects these requirements.
	 */

	adc_stm32_set_clock(dev);

	/* Configure ADC inputs as specified in Device Tree, if any */
	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if ((err < 0) && (err != -ENOENT)) {
		/*
		 * If the ADC is used only with internal channels, then no pinctrl is
		 * provided in Device Tree, and pinctrl_apply_state returns -ENOENT,
		 * but this should not be treated as an error.
		 */
		LOG_ERR("ADC pinctrl setup failed (%d)", err);
		return err;
	}

	adc_stm32_enable_analog_supply();

#if ANY_ADC_HAS_DEEP_POWERDOWN
	if (config->has_deep_powerdown) {
		LL_ADC_DisableDeepPowerDown(adc);
	}
#endif

#if ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_SW_DELAY) || \
	ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_HW_STATUS)
	/*
	 * Many ADC modules need some time to be stabilized before performing
	 * any enable or calibration actions.
	 */
	if (config->internal_regulator != INTERNAL_REGULATOR_NONE) {
		LL_ADC_EnableInternalRegulator(adc);
	}

	/* Wait for Internal regulator stabilisation
	 * Some series have a dedicated status bit, others rely on a delay
	 */
#if ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_SW_DELAY)
	if (config->internal_regulator == INTERNAL_REGULATOR_STARTUP_SW_DELAY) {
		k_busy_wait(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);
	}
#endif /* ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_SW_DELAY) */
#if ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_HW_STATUS)
	if (config->internal_regulator == INTERNAL_REGULATOR_STARTUP_HW_STATUS) {
		while (LL_ADC_IsActiveFlag_LDORDY(adc) == 0) {
		}
	}
#endif /* ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_HW_STATUS) */

#endif /* INTERNAL_REGULATOR_STARTUP_SW_DELAY || INTERNAL_REGULATOR_STARTUP_HW_STATUS */

	if (config->irq_cfg_func) {
		config->irq_cfg_func();
	}

#if defined(HAS_CALIBRATION)
	adc_stm32_calibrate(dev);
	LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_SOFTWARE);
#endif /* HAS_CALIBRATION */

	/* Set resolution from config table (use first/default resolution) */
	if (config->res_table_size > 0) {
		uint32_t res_entry = config->res_table[0];
		uint8_t resolution = STM32_ADC_GET_REAL_VAL(res_entry);
		
		err = set_resolution(dev, resolution);
		if (err < 0) {
			LOG_ERR("Failed to set ADC resolution: %d", err);
			return err;
		}
		
		/* Store the resolution value for ISR calculations */
		data->resolution = resolution;
	} else {
		/* Fallback to 12-bit if not specified */
		data->resolution = 12;
	}

	/* Configure injected channels if available */
	if (config->num_channels > 0) {
		/* Configure injected channels */
		err = adc_stm32_configure_injected_channels(dev);
		if (err < 0) {
			LOG_ERR("Failed to configure injected channels: %d", err);
			return err;
		}
		
		/* Configure injected trigger */
		err = adc_stm32_configure_injected_trigger(dev);
		if (err < 0) {
			LOG_ERR("Failed to configure injected trigger: %d", err);
			return err;
		}
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int adc_stm32_suspend_setup(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	int err;

	/* Disable ADC */
	adc_stm32_disable(adc);

#if ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_SW_DELAY) || \
	ANY_ADC_INTERNAL_REGULATOR_TYPE_IS(INTERNAL_REGULATOR_STARTUP_HW_STATUS)
	if (config->internal_regulator != INTERNAL_REGULATOR_NONE) {
		LL_ADC_DisableInternalRegulator(adc);
	}
#endif /* INTERNAL_REGULATOR_STARTUP_SW_DELAY || INTERNAL_REGULATOR_STARTUP_HW_STATUS */

#if ANY_ADC_HAS_DEEP_POWERDOWN
	if (config->has_deep_powerdown) {
		LL_ADC_EnableDeepPowerDown(adc);
	}
#endif

	adc_stm32_disable_analog_supply();

	/* Stop device clock. Note: fixed clocks are not handled yet. */
	err = clock_control_off(clk, (clock_control_subsys_t)&config->pclken);
	if (err != 0) {
		LOG_ERR("Could not disable ADC clock");
		return err;
	}

	/* Move pins to sleep state */
	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
	if ((err < 0) && (err != -ENOENT)) {
		/*
		 * If returning -ENOENT, no pins where defined for sleep mode :
		 * Do not output on console (might sleep already) when going to sleep,
		 * "ADC pinctrl sleep state not available"
		 * and don't block PM suspend.
		 * Else return the error.
		 */
		return err;
	}

	return 0;
}

static int adc_stm32_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return adc_stm32_init(dev);
	case PM_DEVICE_ACTION_SUSPEND:
		return adc_stm32_suspend_setup(dev);
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

/*
 * Injected ADC API implementation
 */

static int adc_stm32_set_callback(const struct device *dev,
				  adc_injected_callback_t callback,
				  void *user_data)
{
	struct adc_stm32_data *data = dev->data;
	
	data->callback = callback;
	data->user_data = user_data;
	
	return 0;
}

static int adc_stm32_set_error_callback(const struct device *dev,
					adc_injected_error_callback_t callback,
					void *user_data)
{
	struct adc_stm32_data *data = dev->data;
	
	data->error_callback = callback;
	data->error_user_data = user_data;
	
	return 0;
}

static int adc_stm32_inj_enable(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;
	int err;
	
	/* Enable ADC if not already enabled */
	err = adc_stm32_enable(adc);
	if (err < 0) {
		return err;
	}
	
	/* Enable JEOS (end of injected sequence) interrupt */
	LL_ADC_EnableIT_JEOS(adc);
	
	/* Start injected conversions (external trigger starts them) */
	LL_ADC_INJ_StartConversion(adc);
	
	return 0;
}

static int adc_stm32_inj_disable(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;
	
	/* Disable JEOS interrupt */
	LL_ADC_DisableIT_JEOS(adc);
	
	/* Stop injected conversions */
	LL_ADC_INJ_StopConversion(adc);
	
	return 0;
}

/*
 * Configure injected channels from devicetree
 * This must be called during init after ADC is enabled
 */
static int adc_stm32_configure_injected_channels(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;
	
	/* Configure each injected channel */
	for (uint8_t i = 0; i < config->num_channels; i++) {
		uint32_t channel = __LL_ADC_DECIMAL_NB_TO_CHANNEL(config->channel_ids[i]);
		
		/* Set channel sampling time - simplified, uses first sampling time */
		if (config->num_sampling_time_common_channels == 0) {
			LL_ADC_SetChannelSamplingTime(adc, channel, 
						      config->channel_acq_times[i]);
		}
		
		/* Configure differential mode if needed */
		if (config->has_differential_support && config->channel_differential[i]) {
			LL_ADC_SetChannelSingleDiff(adc, channel, LL_ADC_DIFFERENTIAL_ENDED);
		} else {
			/* Single-ended mode */
			if (config->has_differential_support) {
				LL_ADC_SetChannelSingleDiff(adc, channel, LL_ADC_SINGLE_ENDED);
			}
		}
		
		/* Set the channel to the appropriate rank in injected sequence */
		LL_ADC_INJ_SetSequencerRanks(adc, config->channel_ranks[i], channel);
	}
	
	/* Configure injected sequence length */
	LL_ADC_INJ_SetSequencerLength(adc, config->num_channels - 1);
	
	return 0;
}

/*
 * Configure injected sequence trigger from devicetree
 */
static int adc_stm32_configure_injected_trigger(const struct device *dev)
{
	const struct adc_stm32_cfg *config = dev->config;
	ADC_TypeDef *adc = config->base;
	
	/* Configure external trigger source */
	LL_ADC_INJ_SetTriggerSource(adc, config->trigger_source);
	
	/* Configure trigger edge */
	LL_ADC_INJ_SetTriggerEdge(adc, config->trigger_edge);
	
	/* Disable auto mode (we want external trigger) */
	LL_ADC_INJ_SetTrigAuto(adc, LL_ADC_INJ_TRIG_INDEPENDENT);
	
	return 0;
}

static DEVICE_API(adc_injected, api_stm32_driver_api) = {
	.set_callback = adc_stm32_set_callback,
	.set_error_callback = adc_stm32_set_error_callback,
	.enable = adc_stm32_inj_enable,
	.disable = adc_stm32_inj_disable,
};

/* Macros for ADC clock source and prescaler */
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(st_adc_clock_source)

#if defined(CONFIG_SOC_SERIES_STM32F0X)
/* LL_ADC_CLOCK_ASYNC_DIV1 doesn't exist in F0 LL. Define it here. */
#define LL_ADC_CLOCK_ASYNC_DIV1 LL_ADC_CLOCK_ASYNC
#endif

/* st_prescaler property requires 2 elements : clock ASYNC/SYNC and DIV */
#define ADC_STM32_CLOCK(x)	DT_INST_STRING_UPPER_TOKEN(x, st_adc_clock_source)
#define ADC_STM32_DIV(x)	DT_INST_PROP(x, st_adc_prescaler)

/* Macro to set the prefix depending on the 1st element: check if it is SYNC or ASYNC */
#define ADC_STM32_CLOCK_PREFIX(x)			\
	COND_CODE_1(IS_EQ(ADC_STM32_CLOCK(x), SYNC),	\
		(LL_ADC_CLOCK_SYNC_PCLK_DIV),		\
		(LL_ADC_CLOCK_ASYNC_DIV))

/* Concat prefix (1st element) and DIV value (2nd element) of st,adc-prescaler */
#define ADC_STM32_DT_PRESC(x)	\
	_CONCAT(ADC_STM32_CLOCK_PREFIX(x), ADC_STM32_DIV(x))

/* Macro to check if the ADC instance clock setup is correct */
#define ADC_STM32_CHECK_DT_CLOCK(x)								\
	BUILD_ASSERT(IS_EQ(ADC_STM32_CLOCK(x), SYNC) || DT_INST_CLOCKS_HAS_NAME(x, adc_ker),	\
		     "ASYNC clock mode defined without ASYNC clock defined in device tree")

#else /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(st_adc_clock_source) */

#define ADC_STM32_DT_PRESC(x)	0
#define ADC_STM32_CHECK_DT_CLOCK(x)

#endif /* !DT_ANY_INST_HAS_PROP_STATUS_OKAY(st_adc_clock_source) */

/*
 * For series that share interrupt lines for multiple ADC instances
 * and have separate interrupt lines for other ADCs (example,
 * STM32G473 has 5 ADC instances, ADC1 and ADC2 share IRQn 18 while
 * ADC3, ADC4 and ADC5 use IRQns 47, 61 and 62 respectively), generate
 * a single common ISR function for each IRQn and call adc_stm32_isr
 * for each device using that interrupt line for all enabled ADCs.
 *
 * To achieve the above, a "first" ADC instance must be chosen for all
 * ADC instances sharing the same IRQn. This "first" ADC instance
 * generates the code for the common ISR and for installing and
 * enabling it while any other ADC sharing the same IRQn skips this
 * code generation and does nothing. The common ISR code is generated
 * to include calls to adc_stm32_isr for all instances using that same
 * IRQn. From the example above, four ISR functions would be generated
 * for IRQn 18, 47, 61 and 62, with possible "first" ADC instances
 * being ADC1, ADC3, ADC4 and ADC5 if all ADCs were enabled, with the
 * ISR function 18 calling adc_stm32_isr for both ADC1 and ADC2.
 *
 * For some of the macros below, pseudo-code is provided to describe
 * its function.
 */

/*
 * return (irqn == device_irqn(index)) ? index : NULL
 */
#define FIRST_WITH_IRQN_INTERNAL(index, irqn)                                                      \
	COND_CODE_1(IS_EQ(irqn, DT_INST_IRQN(index)), (index,), (EMPTY,))

/*
 * Returns the "first" instance's index:
 *
 * instances = []
 * for instance in all_active_adcs:
 *     instances.append(first_with_irqn_internal(device_irqn(index)))
 * for instance in instances:
 *     if instance == NULL:
 *         instances.remove(instance)
 * return instances[0]
 */
#define FIRST_WITH_IRQN(index)                                                                     \
	GET_ARG_N(1, LIST_DROP_EMPTY(DT_INST_FOREACH_STATUS_OKAY_VARGS(FIRST_WITH_IRQN_INTERNAL,   \
								       DT_INST_IRQN(index))))

/*
 * Provides code for calling adc_stm32_isr for an instance if its IRQn
 * matches:
 *
 * if (irqn == device_irqn(index)):
 *     return "adc_stm32_isr(DEVICE_DT_INST_GET(index));"
 */
#define HANDLE_IRQS(index, irqn)                                                                   \
	COND_CODE_1(IS_EQ(irqn, DT_INST_IRQN(index)), (adc_stm32_isr(DEVICE_DT_INST_GET(index));), \
		    (EMPTY))

/*
 * Name of the common ISR for a given IRQn (taken from a device with a
 * given index). Example, for an ADC instance with IRQn 18, returns
 * "adc_stm32_isr_18".
 */
#define ISR_FUNC(index) UTIL_CAT(adc_stm32_isr_, DT_INST_IRQN(index))

/*
 * Macro for generating code for the common ISRs (by looping of all
 * ADC instances that share the same IRQn as that of the given device
 * by index) and the function for setting up the ISR.
 *
 * For injected ADC, we use zero-latency ISR_DIRECT for motor control.
 */
#define GENERATE_ISR_CODE(index)                                                                   \
	ISR_DIRECT_DECLARE(ISR_FUNC(index))                                                        \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(index);                              \
		const struct adc_stm32_cfg *config = dev->config;                                  \
		struct adc_stm32_data *data = dev->data;                                           \
		ADC_TypeDef *adc = config->base;                                                   \
		                                                                                   \
		/* Check for overrun error first */                                                \
		if (LL_ADC_IsActiveFlag_JQOVF(adc)) {                                              \
			/* Clear overrun flag */                                                   \
			LL_ADC_ClearFlag_JQOVF(adc);                                               \
			                                                                           \
			/* Call error callback if registered */                                    \
			if (data->error_callback != NULL) {                                        \
				data->error_callback(dev, ADC_INJ_ERROR_OVERRUN,                  \
						     data->error_user_data);                       \
			}                                                                          \
			                                                                           \
			/* Return without processing conversions */                                \
			return 0;                                                                  \
		}                                                                                  \
		                                                                                   \
		/* Check JEOS flag */                                                              \
		if (LL_ADC_IsActiveFlag_JEOS(adc)) {                                               \
			uint32_t num_channels = LL_ADC_INJ_GetSequencerLength(adc) + 1;           \
			                                                                           \
			/* Read all injected conversions and convert to Q31 */                    \
			for (uint32_t i = 0; i < num_channels; i++) {                             \
				uint32_t raw_value = LL_ADC_INJ_ReadConversionData32(adc,         \
								config->channel_ranks[i]);         \
				                                                                   \
				/* Convert to Q31 (single-ended: 0 to +full_scale) */             \
				data->inj_values[i] = (int32_t)(raw_value << (31 - data->resolution));\
			}                                                                          \
			                                                                           \
			/* Clear JEOS flag */                                                      \
			LL_ADC_ClearFlag_JEOS(adc);                                                \
			                                                                           \
			/* Call user callback if registered */                                     \
			if (data->callback != NULL) {                                              \
				data->callback(dev, (const q31_t *)data->inj_values, num_channels, \
					       data->user_data);                                   \
			}                                                                          \
		}                                                                                  \
		                                                                                   \
		return 0;										                                    \
	}                                                                                          \
                                                                                                   \
	static void UTIL_CAT(ISR_FUNC(index), _init)(void)                                         \
	{                                                                                          \
		IRQ_DIRECT_CONNECT(DT_INST_IRQN(index), 0, ISR_FUNC(index), IRQ_ZERO_LATENCY);    \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

/*
 * Limit generating code to only the "first" instance:
 *
 * if (first_with_irqn(index) == index):
 *     generate_isr_code(index)
 */
#define GENERATE_ISR(index)                                                                        \
	COND_CODE_1(IS_EQ(index, FIRST_WITH_IRQN(index)), (GENERATE_ISR_CODE(index)), (EMPTY))

DT_INST_FOREACH_STATUS_OKAY(GENERATE_ISR)

/* Only "first" instances need to call the ISR setup function */
#define ADC_STM32_IRQ_FUNC(index)                                                                  \
	.irq_cfg_func = COND_CODE_1(IS_EQ(index, FIRST_WITH_IRQN(index)),                          \
				    (UTIL_CAT(ISR_FUNC(index), _init)), (NULL)),

/*
 * Injected ADC trigger source mapping
 * Maps string trigger names to LL constants
 */
#define ADC_INJ_TRIGGER_tim1_trgo   LL_ADC_INJ_TRIG_EXT_TIM1_TRGO
#define ADC_INJ_TRIGGER_tim1_cc4    LL_ADC_INJ_TRIG_EXT_TIM1_CH4
#define ADC_INJ_TRIGGER_tim2_trgo   LL_ADC_INJ_TRIG_EXT_TIM2_TRGO
#define ADC_INJ_TRIGGER_tim2_cc1    LL_ADC_INJ_TRIG_EXT_TIM2_CH1
#define ADC_INJ_TRIGGER_tim3_cc2    LL_ADC_INJ_TRIG_EXT_TIM3_CH2
#define ADC_INJ_TRIGGER_tim3_cc4    LL_ADC_INJ_TRIG_EXT_TIM3_CH4
#define ADC_INJ_TRIGGER_tim3_trgo   LL_ADC_INJ_TRIG_EXT_TIM3_TRGO
#define ADC_INJ_TRIGGER_tim4_trgo   LL_ADC_INJ_TRIG_EXT_TIM4_TRGO
#define ADC_INJ_TRIGGER_tim5_trgo   LL_ADC_INJ_TRIG_EXT_TIM5_TRGO
#define ADC_INJ_TRIGGER_tim8_cc2    LL_ADC_INJ_TRIG_EXT_TIM8_CH2
#define ADC_INJ_TRIGGER_tim8_cc4    LL_ADC_INJ_TRIG_EXT_TIM8_CH4
#define ADC_INJ_TRIGGER_tim8_trgo   LL_ADC_INJ_TRIG_EXT_TIM8_TRGO

#define ADC_INJ_EDGE_rising    LL_ADC_INJ_TRIG_EXT_RISING
#define ADC_INJ_EDGE_falling   LL_ADC_INJ_TRIG_EXT_FALLING
#define ADC_INJ_EDGE_both      LL_ADC_INJ_TRIG_EXT_RISINGFALLING

/* Get trigger source from DT */
#define ADC_INJ_DT_TRIGGER_SRC(idx) \
	_CONCAT(ADC_INJ_TRIGGER_, DT_INST_STRING_TOKEN(idx, st_adc_trigger_source))

/* Get trigger edge from DT (with default) */
#define ADC_INJ_DT_TRIGGER_EDGE(idx) \
	_CONCAT(ADC_INJ_EDGE_, DT_INST_STRING_TOKEN_OR(idx, st_adc_trigger_edge, rising))

/*
 * Injected channel configuration from devicetree
 * 
 * These macros parse the child channel nodes and build the injected config structure.
 * Each channel child node has: reg (channel number), zephyr,acquisition-time, etc.
 */

/* Helper to get injected rank for channel index (0-3 -> LL_ADC_INJ_RANK_1..4) */
#define ADC_INJ_RANK_FOR_INDEX(i) \
	((i) == 0 ? LL_ADC_INJ_RANK_1 : \
	 (i) == 1 ? LL_ADC_INJ_RANK_2 : \
	 (i) == 2 ? LL_ADC_INJ_RANK_3 : LL_ADC_INJ_RANK_4)

/* Count child channel nodes - for injected ADC, we expect 1-4 channels */
#define ADC_INJ_NUM_CHANNELS(inst) \
	DT_INST_CHILD_NUM_STATUS_OKAY(inst)

/* Helper to get channel ID from child node */
#define ADC_INJ_CHANNEL_ID(node_id) DT_REG_ADDR(node_id)

/* Helper to get acquisition time from child node (with default) */
#define ADC_INJ_CHANNEL_ACQ_TIME(node_id) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, zephyr_acquisition_time), \
		(DT_PROP(node_id, zephyr_acquisition_time)), \
		(0))

/* Helper to check if channel is differential */
#define ADC_INJ_CHANNEL_IS_DIFF(node_id) \
	DT_PROP_OR(node_id, differential, 0)

/* Macros to populate channel arrays element-by-element */
/* These will be called for each child node during DT_INST_FOREACH_CHILD */

/* Macro to initialize one channel entry - called for each child */
#define ADC_INJ_INIT_CHANNEL_ID(child) \
	ADC_INJ_CHANNEL_ID(child)

#define ADC_INJ_INIT_CHANNEL_ACQ(child) \
	ADC_INJ_CHANNEL_ACQ_TIME(child)

#define ADC_INJ_INIT_CHANNEL_DIFF(child) \
	ADC_INJ_CHANNEL_IS_DIFF(child)

/* Rank is just the index (1-based for STM32) */
#define ADC_INJ_INIT_CHANNEL_RANK(child) \
	ADC_INJ_RANK_FOR_INDEX(DT_NODE_CHILD_IDX(child))

/* Build the injected config structure if we have trigger source defined */
#define ADC_INJ_HAS_CONFIG(inst) \
	DT_INST_NODE_HAS_PROP(inst, st_adc_trigger_source)

#define ADC_STM32_INIT(index)						\
									\
ADC_STM32_CHECK_DT_CLOCK(index);					\
									\
PINCTRL_DT_INST_DEFINE(index);						\
									\
static const struct adc_stm32_cfg adc_stm32_cfg_##index = {		\
	.base = (ADC_TypeDef *)DT_INST_REG_ADDR(index),			\
	ADC_STM32_IRQ_FUNC(index)					\
	.pclken = {.bus = DT_INST_CLOCKS_CELL_BY_NAME(index, adcx, bus),			\
		   .enr = DT_INST_CLOCKS_CELL_BY_NAME(index, adcx, bits)},			\
	IF_ENABLED(DT_INST_CLOCKS_HAS_NAME(index, adc_ker),					\
		   (.pclken_ker = {.bus = DT_INST_CLOCKS_CELL_BY_NAME(index, adc_ker, bus), \
				   .enr = DT_INST_CLOCKS_CELL_BY_NAME(index, adc_ker, bits)}, \
		    .has_pclken_ker = true,))							\
	IF_ENABLED(DT_INST_CLOCKS_HAS_NAME(index, adc_pre),					\
		   (.pclken_pre = {.bus = DT_INST_CLOCKS_CELL_BY_NAME(index, adc_pre, bus), \
				   .enr = DT_INST_CLOCKS_CELL_BY_NAME(index, adc_pre, bits)}, \
		    .has_pclken_pre = true,))							\
	.clk_prescaler = ADC_STM32_DT_PRESC(index),			\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
	.differential_channels_used = (ANY_CHILD_NODE_IS_DIFFERENTIAL(index) > 0), \
	.sequencer_type = CONCAT(SEQUENCER_,							\
		DT_INST_STRING_UPPER_TOKEN(index, st_adc_sequencer)),				\
	.internal_regulator = CONCAT(INTERNAL_REGULATOR_,					\
		DT_INST_STRING_UPPER_TOKEN(index, st_adc_internal_regulator)),			\
	.has_deep_powerdown = DT_INST_PROP(index, st_adc_has_deep_powerdown),			\
	.has_channel_preselection = DT_INST_PROP(index, st_adc_has_channel_preselection),	\
	.has_differential_support = DT_INST_PROP(index, st_adc_has_differential_support),	\
	.sampling_time_table = DT_INST_PROP(index, sampling_times),	\
	.num_sampling_time_common_channels =				\
		DT_INST_PROP_OR(index, num_sampling_time_common_channels, 0),\
	IF_ENABLED(ADC_INJ_HAS_CONFIG(index),				\
		   (.num_channels = DT_INST_CHILD_NUM_STATUS_OKAY(index), \
		    .channel_ids = {					\
			DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(index,	\
				ADC_INJ_INIT_CHANNEL_ID, (,))		\
		    },							\
		    .channel_ranks = {					\
			DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(index,	\
				ADC_INJ_INIT_CHANNEL_RANK, (,))		\
		    },							\
		    .channel_acq_times = {				\
			DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(index,	\
				ADC_INJ_INIT_CHANNEL_ACQ, (,))		\
		    },							\
		    .channel_differential = {				\
			DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(index,	\
				ADC_INJ_INIT_CHANNEL_DIFF, (,))		\
		    },							\
		    .trigger_source = ADC_INJ_DT_TRIGGER_SRC(index),	\
		    .trigger_edge = ADC_INJ_DT_TRIGGER_EDGE(index),))	\
	.res_table_size = DT_INST_PROP_LEN(index, resolutions),		\
	.res_table = DT_INST_PROP(index, resolutions),			\
};									\
									\
static struct adc_stm32_data adc_stm32_data_##index = {			\
};									\
									\
PM_DEVICE_DT_INST_DEFINE(index, adc_stm32_pm_action);			\
									\
DEVICE_DT_INST_DEFINE(index,						\
		    adc_stm32_init, PM_DEVICE_DT_INST_GET(index),	\
		    &adc_stm32_data_##index, &adc_stm32_cfg_##index,	\
		    POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,		\
		    &api_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_STM32_INIT)
