/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_POSITION_REGULATOR_H_
#define MOTOR_POSITION_REGULATOR_H_

#include <stdbool.h>
#include <zephyr/dsp/types.h>

#include "motor/math/math_constants.h"

struct motor_position_regulator_config {
	float32_t kp_rad_s_per_rad;
	float32_t ki_rad_s2_per_rad;
	float32_t integrator_limit_rad_s;
	float32_t output_limit_rad_s;
};

struct motor_position_regulator_state {
	bool initialized;
	float32_t integrator_rad_s;
};

int motor_position_regulator_validate(const struct motor_position_regulator_config *cfg);

int motor_position_regulator_init(const struct motor_position_regulator_config *cfg,
				  struct motor_position_regulator_state *state,
				  float32_t integrator_rad_s);

void motor_position_regulator_reset(struct motor_position_regulator_state *state,
				    float32_t integrator_rad_s);

int motor_position_regulator_step(const struct motor_position_regulator_config *cfg,
				  struct motor_position_regulator_state *state,
				  float32_t position_error_rad,
				  float32_t velocity_ff_rad_s,
				  float32_t dt_s,
				  float32_t *velocity_cmd_rad_s_out);

/*
 * Fast ISR path. The caller must provide non-NULL pointers, a prevalidated
 * configuration, an initialized state, finite runtime inputs, and dt_s > 0.
 * Use motor_position_regulator_step() at configuration/test boundaries.
 */
static inline int motor_position_regulator_step_fast(
	const struct motor_position_regulator_config *cfg,
	struct motor_position_regulator_state *state,
	float32_t position_error_rad,
	float32_t velocity_ff_rad_s,
	float32_t dt_s,
	float32_t *velocity_cmd_rad_s_out)
{
	float32_t p_term = cfg->kp_rad_s_per_rad * position_error_rad;
	float32_t i_candidate = state->integrator_rad_s +
				(cfg->ki_rad_s2_per_rad * position_error_rad * dt_s);
	i_candidate = clampf(i_candidate,
			     -cfg->integrator_limit_rad_s,
			     cfg->integrator_limit_rad_s);

	float32_t candidate_cmd = velocity_ff_rad_s + p_term + i_candidate;
	float32_t current_cmd = velocity_ff_rad_s + p_term + state->integrator_rad_s;
	bool already_high = current_cmd >= cfg->output_limit_rad_s;
	bool already_low = current_cmd <= -cfg->output_limit_rad_s;
	bool windup_high = already_high && candidate_cmd > cfg->output_limit_rad_s &&
			   position_error_rad > 0.0f;
	bool windup_low = already_low && candidate_cmd < -cfg->output_limit_rad_s &&
			  position_error_rad < 0.0f;
	if (!windup_high && !windup_low) {
		state->integrator_rad_s = i_candidate;
	} else {
		state->integrator_rad_s = clampf(state->integrator_rad_s,
						 -cfg->integrator_limit_rad_s,
						 cfg->integrator_limit_rad_s);
	}

	float32_t feedback_velocity = p_term + state->integrator_rad_s;
	float32_t velocity_cmd = velocity_ff_rad_s + feedback_velocity;
	*velocity_cmd_rad_s_out =
		clampf(velocity_cmd, -cfg->output_limit_rad_s, cfg->output_limit_rad_s);
	return 0;
}

#endif /* MOTOR_POSITION_REGULATOR_H_ */
