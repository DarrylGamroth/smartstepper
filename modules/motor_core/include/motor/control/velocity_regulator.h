/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_VELOCITY_REGULATOR_H_
#define MOTOR_VELOCITY_REGULATOR_H_

#include <zephyr/dsp/types.h>

struct motor_velocity_regulator_config {
	float32_t kp_a_per_rad_s;
	float32_t ki_a_per_rad;
	float32_t integrator_limit_a;
	float32_t output_limit_a;
};

struct motor_velocity_regulator_state {
	float32_t integrator_a;
};

int motor_velocity_regulator_validate(const struct motor_velocity_regulator_config *cfg);

void motor_velocity_regulator_reset(struct motor_velocity_regulator_state *state,
				    float32_t integrator_a);

int motor_velocity_regulator_step(const struct motor_velocity_regulator_config *cfg,
				  struct motor_velocity_regulator_state *state,
				  float32_t speed_error_rad_s,
				  float32_t dt_s,
				  float32_t *iq_cmd_a_out);

#endif /* MOTOR_VELOCITY_REGULATOR_H_ */
