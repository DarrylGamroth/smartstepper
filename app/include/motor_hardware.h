/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_HARDWARE_H
#define MOTOR_HARDWARE_H

#include <zephyr/device.h>

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
 * - Encoder (encoder1)
 * - ADC (adc1)
 * 
 * @return 0 if all devices ready, negative error code otherwise
 */
int motor_hardware_check_devices(void);

#endif /* MOTOR_HARDWARE_H */
