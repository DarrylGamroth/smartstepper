/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CORE_STEP_H_
#define MOTOR_CORE_STEP_H_

#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/runtime/io.h"

struct motor_parameters;

void motor_core_step_fast(struct motor_parameters *params,
			  const q31_t *values,
			  uint8_t count,
			  const struct motor_control_encoder_sample *encoder_sample,
			  struct motor_control_pwm_output *pwm_out,
			  struct motor_control_step_report *report);

#endif /* MOTOR_CORE_STEP_H_ */
