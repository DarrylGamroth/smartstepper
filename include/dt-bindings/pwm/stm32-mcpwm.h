/*
 * Copyright (c) 2022 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_STM32_MCPWM_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_STM32_MCPWM_H_

/**
 * @name Idle State
 */
#define STM32_PWM_IDLE_STATE_LOW         (0U << 9)
#define STM32_PWM_IDLE_STATE_HIGH        (1U << 9)
/** @cond INTERNAL_HIDDEN */
#define STM32_PWM_IDLE_STATE_MASK	     0x200
/** @endcond */

/**
 * @name Output Compare Mode
 * These flags can be used to specify the output compare mode for individual PWM channels.
 * Use these flags in the pwm-cells flags parameter to override the default PWM mode.
 * @{
 */
#define STM32_PWM_OC_MODE_PWM1           (0U << 10)
#define STM32_PWM_OC_MODE_PWM2           (1U << 10)
/** @cond INTERNAL_HIDDEN */
#define STM32_PWM_OC_MODE_MASK           0x400
/** @endcond */
/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_STM32_MCPWM_H_ */
