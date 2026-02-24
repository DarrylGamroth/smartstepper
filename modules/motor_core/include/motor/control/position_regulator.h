/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_POSITION_REGULATOR_H_
#define MOTOR_POSITION_REGULATOR_H_

#include <zephyr/dsp/types.h>

struct motor_position_regulator_config {
	float32_t kp_rad_s_per_rad;
	float32_t ki_rad_s2_per_rad;
	float32_t integrator_limit_rad_s;
	float32_t output_limit_rad_s;
};

struct motor_position_regulator_state {
	float32_t integrator_rad_s;
};

int motor_position_regulator_validate(const struct motor_position_regulator_config *cfg);

void motor_position_regulator_reset(struct motor_position_regulator_state *state,
				    float32_t integrator_rad_s);

int motor_position_regulator_step(const struct motor_position_regulator_config *cfg,
				  struct motor_position_regulator_state *state,
				  float32_t position_error_rad,
				  float32_t velocity_ff_rad_s,
				  float32_t dt_s,
				  float32_t *velocity_cmd_rad_s_out);

#endif /* MOTOR_POSITION_REGULATOR_H_ */
