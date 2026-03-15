/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_LOOP_H_
#define MOTOR_CONTROL_LOOP_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>
#include <zephyr/dsp/utils.h>

#include "motor/runtime/io.h"

struct motor_parameters;

/**
 * @brief Execute one pure motor-control loop iteration from sampled ADC values.
 *
 * This function runs control math/state updates only and does not touch devices.
 * PWM outputs are returned in @p pwm_out when @p update_pwm is true.
 */
void motor_control_loop_step(struct motor_parameters *params,
			     const q31_t *values,
			     uint8_t count,
			     const struct motor_control_encoder_sample *encoder_sample,
			     struct motor_control_pwm_output *pwm_out,
			     struct motor_control_step_report *report);

#endif /* MOTOR_CONTROL_LOOP_H_ */
