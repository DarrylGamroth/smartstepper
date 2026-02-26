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

void motor_position_regulator_reset(struct motor_position_regulator_state *state,
				    float32_t integrator_rad_s)
{
	if (state == NULL) {
		return;
	}
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
	if (dt_s <= 0.0f || cfg->output_limit_rad_s <= 0.0f ||
	    cfg->integrator_limit_rad_s < 0.0f) {
		return -EINVAL;
	}

	float32_t i_next = state->integrator_rad_s +
			   (cfg->ki_rad_s2_per_rad * position_error_rad * dt_s);
	i_next = clampf(i_next, -cfg->integrator_limit_rad_s, cfg->integrator_limit_rad_s);
	state->integrator_rad_s = i_next;

	float32_t feedback_velocity =
		(cfg->kp_rad_s_per_rad * position_error_rad) + state->integrator_rad_s;
	float32_t velocity_cmd = velocity_ff_rad_s + feedback_velocity;
	*velocity_cmd_rad_s_out =
		clampf(velocity_cmd, -cfg->output_limit_rad_s, cfg->output_limit_rad_s);
	return 0;
}
