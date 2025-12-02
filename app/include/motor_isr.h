/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ISR_H
#define MOTOR_ISR_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <drivers/adc_injected.h>
#include <drivers/mcpwm.h>
#include "config.h"

/* Global device handles - accessed by ISR */
extern const struct device *const pwm1;
extern const struct device *const pwm8;
extern const struct device *const pwm3;
extern const struct device *const encoder1;
extern const struct device *const adc1;

/* Global GPIO specs - accessed by ISR */
extern const struct gpio_dt_spec trig;

/* RTIO context for encoder */
extern struct rtio encoder_rtio_ctx;

/* ISR callback functions */

/**
 * @brief ADC injected conversion complete callback - main control loop ISR
 * 
 * This is the primary motor control interrupt handler that runs at the control
 * loop frequency (typically 10-20 kHz). It performs:
 * - Double buffer swap if pending
 * - Encoder reading and observer update
 * - ADC value conversion (currents, bus voltage)
 * - Fault detection (overcurrent, overvoltage, encoder)
 * - State-specific processing (offset measurement, RoverL, Rs estimation)
 * - FOC current control (Park/Clarke transforms, PI controllers)
 * - SVPWM generation and output
 * - ISR performance measurement
 *
 * @param dev ADC device
 * @param values Array of q31 ADC values [Ib, Ia, Vbus]
 * @param count Number of values (should be 3)
 * @param user_data Pointer to motor_parameters struct
 */
void adc_callback(const struct device *dev, const q31_t *values,
                  uint8_t count, void *user_data);

/**
 * @brief Hardware break interrupt handler - emergency fault protection
 * 
 * Called when the break input (BKIN) goes active due to:
 * - Hardware overcurrent detection
 * - External fault signal
 * - Other safety-critical conditions
 * 
 * PWM outputs are already disabled by hardware when this fires.
 * This handler transitions the state machine to ERROR state.
 *
 * @param dev PWM device that triggered the break
 * @param user_data Pointer to motor_parameters struct
 */
void break_interrupt_handler(const struct device *dev, void *user_data);

/**
 * @brief Encoder timer compare callback - triggers encoder read
 * 
 * Called at encoder sampling rate (derived from PWM timer).
 * Initiates an asynchronous RTIO read of the encoder position.
 *
 * @param dev Timer device
 * @param channel Timer channel that triggered
 * @param user_data Unused
 */
void encoder1_callback(const struct device *dev, uint32_t channel,
                       void *user_data);

#endif /* MOTOR_ISR_H */
