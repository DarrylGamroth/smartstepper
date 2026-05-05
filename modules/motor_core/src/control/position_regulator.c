/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/position_regulator.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/math_constants.h"

int motor_position_regulator_validate(const struct motor_position_regulator_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (!isfinite(cfg->kp_rad_s_per_rad) || !isfinite(cfg->ki_rad_s2_per_rad)) {
		return -EINVAL;
	}
	if (!isfinite(cfg->integrator_limit_rad_s) || !isfinite(cfg->output_limit_rad_s)) {
		return -EINVAL;
	}
	if (cfg->integrator_limit_rad_s < 0.0f || cfg->output_limit_rad_s <= 0.0f) {
		return -EINVAL;
	}
	return 0;
}

int motor_position_regulator_init(const struct motor_position_regulator_config *cfg,
				  struct motor_position_regulator_state *state,
				  float32_t integrator_rad_s)
{
	if (motor_position_regulator_validate(cfg) != 0 || state == NULL) {
		return -EINVAL;
	}
	state->initialized = true;
	state->integrator_rad_s = isfinite(integrator_rad_s) ? integrator_rad_s : 0.0f;
	return 0;
}

void motor_position_regulator_reset(struct motor_position_regulator_state *state,
				    float32_t integrator_rad_s)
{
	if (state == NULL) {
		return;
	}
	state->initialized = true;
	state->integrator_rad_s = isfinite(integrator_rad_s) ? integrator_rad_s : 0.0f;
}

int motor_position_regulator_step(const struct motor_position_regulator_config *cfg,
				  struct motor_position_regulator_state *state,
				  float32_t position_error_rad,
				  float32_t velocity_ff_rad_s,
				  float32_t dt_s,
				  float32_t *velocity_cmd_rad_s_out)
{
	if (cfg == NULL || state == NULL || velocity_cmd_rad_s_out == NULL) {
		return -EINVAL;
	}
	if (!state->initialized) {
		return -EINVAL;
	}
	if (dt_s <= 0.0f || cfg->output_limit_rad_s <= 0.0f ||
	    cfg->integrator_limit_rad_s < 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(position_error_rad) || !isfinite(velocity_ff_rad_s) || !isfinite(dt_s)) {
		return -EINVAL;
	}

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
