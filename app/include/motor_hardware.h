/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_HARDWARE_H
#define MOTOR_HARDWARE_H

#include <zephyr/device.h>
#include <stdbool.h>

/* Hardware device handles */
extern const struct device *const pwm1;
extern const struct device *const pwm8;
extern const struct device *const pwm3;
extern const struct device *const encoder1;
extern const struct device *const adc1;
extern const struct device *const gate_driver_a;
extern const struct device *const gate_driver_b;

/**
 * @brief Initialize GPIO pins for motor control
 * 
 * Configures:
 * - Photo interruptor enable (pi_enable)
 * - Trigger/debug output (trig)
 * 
 * @return 0 on success, negative error code on failure
 */
int motor_hardware_init_gpio(void);

/**
 * @brief Check all required devices are ready
 * 
 * Validates readiness of:
 * - PWM timers (pwm1, pwm3, pwm8)
 * - Encoder or fast-path encoder transport (encoder1)
 * - ADC (adc1)
 * 
 * @return 0 if all devices ready, negative error code otherwise
 */
int motor_hardware_check_devices(void);

/**
 * @brief Pulse DRV8328 nSLEEP pins to clear latched gate-driver faults.
 *
 * This uses k_usleep/k_busy_wait internally through the DRV8328 driver, so it
 * must only be called from thread/shell context, never from a zero-latency ISR.
 *
 * @return 0 if both gate drivers were reset, negative error code otherwise
 */
int motor_hardware_reset_gate_driver_faults(void);

/**
 * @brief Clear latched DRV8328 faults while masking MCPWM break callbacks.
 *
 * Use this when recovering from an expected DRV8328 nSLEEP pulse. The pulse can
 * assert the timer break input, so the MCPWM break callbacks are disabled during
 * the reset and restored afterward with the supplied user data.
 *
 * @param break_user_data user data to restore on the MCPWM break callbacks
 * @return 0 if both gate drivers were reset, negative error code otherwise
 */
int motor_hardware_reset_gate_driver_faults_masked(void *break_user_data);

/**
 * @brief Read cached DRV8328 fault state for both gate drivers.
 *
 * The current board does not route nFAULT into devicetree, so these values only
 * reflect faults reported through the driver callback path when available.
 *
 * @param fault_a set true when gate driver A has a cached fault
 * @param fault_b set true when gate driver B has a cached fault
 * @return 0 on success, negative error code otherwise
 */
int motor_hardware_get_gate_driver_faults(bool *fault_a, bool *fault_b);

/**
 * @brief Control photo interruptor emitter enable GPIO
 *
 * @param enable true to drive enable pin active, false to drive inactive
 * @return 0 on success, negative error code on failure
 */
int motor_hardware_set_photo_interruptor_enable(bool enable);

#endif /* MOTOR_HARDWARE_H */
