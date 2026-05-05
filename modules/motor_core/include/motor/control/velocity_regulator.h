/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_VELOCITY_REGULATOR_H_
#define MOTOR_VELOCITY_REGULATOR_H_

#include <stdbool.h>
#include <zephyr/dsp/types.h>

#include "motor/math/math_constants.h"

struct motor_velocity_regulator_config {
	float32_t kp_a_per_rad_s;
	float32_t ki_a_per_rad;
	float32_t integrator_limit_a;
	float32_t output_limit_a;
};

struct motor_velocity_regulator_state {
	bool initialized;
	float32_t integrator_a;
};

int motor_velocity_regulator_validate(const struct motor_velocity_regulator_config *cfg);

int motor_velocity_regulator_init(const struct motor_velocity_regulator_config *cfg,
				  struct motor_velocity_regulator_state *state,
				  float32_t integrator_a);

void motor_velocity_regulator_reset(struct motor_velocity_regulator_state *state,
				    float32_t integrator_a);

int motor_velocity_regulator_step(const struct motor_velocity_regulator_config *cfg,
				  struct motor_velocity_regulator_state *state,
				  float32_t speed_error_rad_s,
				  float32_t dt_s,
				  float32_t *iq_cmd_a_out);

/*
 * Fast ISR path. The caller must provide non-NULL pointers, a prevalidated
 * configuration, an initialized state, finite runtime inputs, and dt_s > 0.
 * Use motor_velocity_regulator_step() at configuration/test boundaries.
 */
static inline int motor_velocity_regulator_step_fast(
	const struct motor_velocity_regulator_config *cfg,
	struct motor_velocity_regulator_state *state,
	float32_t speed_error_rad_s,
	float32_t dt_s,
	float32_t *iq_cmd_a_out)
{
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

#endif /* MOTOR_VELOCITY_REGULATOR_H_ */
