/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/velocity_regulator.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/math_constants.h"

int motor_velocity_regulator_validate(const struct motor_velocity_regulator_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (!isfinite(cfg->kp_a_per_rad_s) || !isfinite(cfg->ki_a_per_rad)) {
		return -EINVAL;
	}
	if (!isfinite(cfg->integrator_limit_a) || !isfinite(cfg->output_limit_a)) {
		return -EINVAL;
	}
	if (cfg->integrator_limit_a < 0.0f || cfg->output_limit_a <= 0.0f) {
		return -EINVAL;
	}
	return 0;
}

int motor_velocity_regulator_init(const struct motor_velocity_regulator_config *cfg,
				  struct motor_velocity_regulator_state *state,
				  float32_t integrator_a)
{
	if (motor_velocity_regulator_validate(cfg) != 0 || state == NULL) {
		return -EINVAL;
	}
	state->initialized = true;
	state->integrator_a = isfinite(integrator_a) ? integrator_a : 0.0f;
	return 0;
}

void motor_velocity_regulator_reset(struct motor_velocity_regulator_state *state,
				    float32_t integrator_a)
{
	if (state == NULL) {
		return;
	}
	state->initialized = true;
	state->integrator_a = isfinite(integrator_a) ? integrator_a : 0.0f;
}

int motor_velocity_regulator_step(const struct motor_velocity_regulator_config *cfg,
				  struct motor_velocity_regulator_state *state,
				  float32_t speed_error_rad_s,
				  float32_t dt_s,
				  float32_t *iq_cmd_a_out)
{
	if (cfg == NULL || state == NULL || iq_cmd_a_out == NULL) {
		return -EINVAL;
	}
	if (!state->initialized) {
		return -EINVAL;
	}
	if (dt_s <= 0.0f || cfg->output_limit_a <= 0.0f || cfg->integrator_limit_a < 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(speed_error_rad_s) || !isfinite(dt_s)) {
		return -EINVAL;
	}

	float32_t p_term = cfg->kp_a_per_rad_s * speed_error_rad_s;
	float32_t i_candidate =
		state->integrator_a + (cfg->ki_a_per_rad * speed_error_rad_s * dt_s);
	i_candidate = clampf(i_candidate, -cfg->integrator_limit_a, cfg->integrator_limit_a);

	float32_t candidate_cmd = p_term + i_candidate;
	float32_t current_cmd = p_term + state->integrator_a;
	bool already_high = current_cmd >= cfg->output_limit_a;
	bool already_low = current_cmd <= -cfg->output_limit_a;
	bool windup_high = already_high && candidate_cmd > cfg->output_limit_a &&
			   speed_error_rad_s > 0.0f;
	bool windup_low = already_low && candidate_cmd < -cfg->output_limit_a &&
			  speed_error_rad_s < 0.0f;
	if (!windup_high && !windup_low) {
		state->integrator_a = i_candidate;
	} else {
		state->integrator_a = clampf(state->integrator_a,
					     -cfg->integrator_limit_a,
					     cfg->integrator_limit_a);
	}

	float32_t iq_cmd = p_term + state->integrator_a;
	*iq_cmd_a_out = clampf(iq_cmd, -cfg->output_limit_a, cfg->output_limit_a);
	return 0;
}
