/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief STM32 MCPWM driver extensions
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_MCPWM_STM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM_MCPWM_STM32_H_

#include <zephyr/device.h>
#include <arm_math.h>
#include <zephyr/dsp/types.h>
#include <drivers/mcpwm.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration for TIM_TypeDef */
typedef struct TIM_TypeDef TIM_TypeDef;

/* Forward declarations for LL functions */
extern void LL_TIM_OC_SetCompareCH1(TIM_TypeDef *TIMx, uint32_t CompareValue);
extern void LL_TIM_OC_SetCompareCH2(TIM_TypeDef *TIMx, uint32_t CompareValue);
extern void LL_TIM_OC_SetCompareCH3(TIM_TypeDef *TIMx, uint32_t CompareValue);

static inline int32_t __SMMULR(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b + (1LL << 31)) >> 32);
}

/** Minimal PWM data structure for inline access */
struct mcpwm_stm32_data {
	uint32_t tim_clk;
	int32_t period_cycles;
	/* Additional fields exist but not needed for inline functions */
};

/** Minimal PWM configuration structure for inline access */
struct mcpwm_stm32_config {
	TIM_TypeDef *timer;
	/* Additional fields exist but not needed for inline functions */
};

/** Maximum number of timer channels */
#if defined(LL_TIM_CHANNEL_CH6)
#define TIMER_MAX_CH 6u
#else
#define TIMER_MAX_CH 4u
#endif

/** Channel to compare set function mapping for inline access */
extern void (*const mcpwm_stm32_set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *, uint32_t);

/**
 * @brief Fast inline PWM duty cycle setting for ISR use
 * 
 * This function provides zero-overhead duty cycle updates for real-time
 * motor control applications. It bypasses all validation and error checking
 * for maximum performance.
 * 
 * @warning This function assumes:
 * - Channel is valid (1 <= channel <= TIMER_MAX_CH)
 * - Device structures are valid
 * - Called from ISR context where validation is unnecessary
 * 
 * @param dev PWM device
 * @param channel PWM channel (1-based)
 * @param duty_cycle Duty cycle in q31_t format (0x00000000 = 0%, 0x7FFFFFFF = ~100%)
 */
static inline void mcpwm_stm32_set_duty_cycle_fast(const struct device *dev, 
                                                  uint32_t channel,
                                                  q31_t duty_cycle)
{
    const struct mcpwm_stm32_config *cfg = (const struct mcpwm_stm32_config *)dev->config;
    const struct mcpwm_stm32_data *data = (const struct mcpwm_stm32_data *)dev->data;
    
    uint32_t pulse_cycles = (uint32_t)__SMMULR(duty_cycle, data->period_cycles);
    mcpwm_stm32_set_timer_compare[channel - 1u](cfg->timer, pulse_cycles);
}

/**
 * @brief Fast inline three-phase PWM duty cycle setting
 * 
 * Optimized for FOC motor control - sets all three phases in minimal time.
 * 
 * @param dev PWM device
 * @param duty_a Phase A duty cycle (q31_t)
 * @param duty_b Phase B duty cycle (q31_t) 
 * @param duty_c Phase C duty cycle (q31_t)
 */
static inline void mcpwm_stm32_set_duty_cycle_3phase(const struct device *dev,
                                                    q31_t duty_a,
                                                    q31_t duty_b, 
                                                    q31_t duty_c)
{
    const struct mcpwm_stm32_config *cfg = (const struct mcpwm_stm32_config *)dev->config;
    const struct mcpwm_stm32_data *data = (const struct mcpwm_stm32_data *)dev->data;
    TIM_TypeDef *timer = cfg->timer;
    int32_t period = data->period_cycles;
    
    /* Calculate all pulse cycles */
    uint32_t pulse_a = (uint32_t)__SMMULR(duty_a, period);
    uint32_t pulse_b = (uint32_t)__SMMULR(duty_b, period);
    uint32_t pulse_c = (uint32_t)__SMMULR(duty_c, period);

    /* Direct register writes for minimum latency */
    LL_TIM_OC_SetCompareCH1(timer, pulse_a);
    LL_TIM_OC_SetCompareCH2(timer, pulse_b);
    LL_TIM_OC_SetCompareCH3(timer, pulse_c);
}

/**
 * @brief User break callback function pointer type
 * 
 * Called when a break fault is detected on the MCPWM timer
 * 
 * @param dev MCPWM device instance
 * @param user_data User data passed during callback registration
 */
typedef void (*mcpwm_stm32_break_cb_t)(const struct device *dev, void *user_data);

/**
 * @brief Register user break callback
 * 
 * Registers a callback function that will be called when a break fault
 * is detected on the MCPWM timer break input (BRK or BRK2)
 * 
 * @param dev MCPWM device instance
 * @param callback Break callback function (NULL to unregister)
 * @param user_data User data to pass to callback
 * @return 0 on success, negative errno on failure
 */
int mcpwm_stm32_register_break_callback(const struct device *dev, 
                                       mcpwm_stm32_break_cb_t callback, 
                                       void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PWM_MCPWM_STM32_H_ */
