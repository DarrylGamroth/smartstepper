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

	return motor_velocity_regulator_step_fast(cfg, state, speed_error_rad_s,
						  dt_s, iq_cmd_a_out);
}
