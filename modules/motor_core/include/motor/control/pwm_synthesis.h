/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_PWM_SYNTHESIS_H_
#define MOTOR_PWM_SYNTHESIS_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

struct motor_pwm_synthesis_input {
	float32_t va_v;
	float32_t vb_v;
	float32_t vbus_v;
	bool braking_enabled;
	float32_t braking_iq_ref_a;
	float32_t braking_speed_rad_s;
	float32_t braking_vbus_limit_v;
	float32_t braking_vbus_margin_inv;
};

struct motor_pwm_synthesis_output {
	float32_t ua_pu;
	float32_t ub_pu;
	float32_t da_pu;
	float32_t db_pu;
	float32_t da_hb1_pu;
	float32_t da_hb2_pu;
	float32_t db_hb1_pu;
	float32_t db_hb2_pu;
};

int motor_pwm_synthesis_step(const struct motor_pwm_synthesis_input *in,
			     struct motor_pwm_synthesis_output *out);

int motor_pwm_synthesis_step_fast(const struct motor_pwm_synthesis_input *in,
				  struct motor_pwm_synthesis_output *out);

#endif /* MOTOR_PWM_SYNTHESIS_H_ */
