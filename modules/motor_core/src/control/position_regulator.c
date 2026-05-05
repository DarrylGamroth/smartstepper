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

	return motor_position_regulator_step_fast(cfg, state, position_error_rad,
						  velocity_ff_rad_s, dt_s,
						  velocity_cmd_rad_s_out);
}
