/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_dob.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "math_constants.h"

#define MOTOR_DOB_EPSILON 1e-9f
#define MOTOR_DOB_FRICTION_DEADBAND_RAD_S 1e-3f

static bool motor_dob_is_finite_positive(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static bool motor_dob_is_finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

static float32_t motor_dob_sign_with_deadband(float32_t value, float32_t deadband)
{
	if (value > deadband) {
		return 1.0f;
	}
	if (value < -deadband) {
		return -1.0f;
	}
	return 0.0f;
}

static void motor_dob_discretize(const struct motor_dob_model *model,
				 float32_t dt_s,
				 float32_t *a_out,
				 float32_t *bu_out,
				 float32_t *bd_out)
{
	float32_t j = model->inertia_kgm2;
	float32_t b = model->viscous_friction_nm_per_rad_s;
	float32_t kt = model->torque_constant_nm_per_a;

	if (b > MOTOR_DOB_EPSILON) {
		float32_t a = expf(-(b * dt_s) / j);
		float32_t one_minus_a = 1.0f - a;

		*a_out = a;
		*bu_out = (kt / b) * one_minus_a;
		*bd_out = (1.0f / b) * one_minus_a;
		return;
	}

	*a_out = 1.0f;
	*bu_out = (kt * dt_s) / j;
	*bd_out = dt_s / j;
}

static bool motor_dob_cache_matches(const struct motor_dob_state *state,
				    const struct motor_dob_config *cfg,
				    const struct motor_dob_model *model)
{
	return state->discretization_valid &&
	       state->cached_dt_s == cfg->dt_s &&
	       state->cached_inertia_kgm2 == model->inertia_kgm2 &&
	       state->cached_viscous_friction_nm_per_rad_s ==
		       model->viscous_friction_nm_per_rad_s &&
	       state->cached_torque_constant_nm_per_a == model->torque_constant_nm_per_a;
}

static void motor_dob_refresh_discretization(struct motor_dob_state *state,
					     const struct motor_dob_config *cfg,
					     const struct motor_dob_model *model)
{
	if (motor_dob_cache_matches(state, cfg, model)) {
		return;
	}

	motor_dob_discretize(model, cfg->dt_s, &state->a, &state->b_u, &state->b_d);
	state->cached_dt_s = cfg->dt_s;
	state->cached_inertia_kgm2 = model->inertia_kgm2;
	state->cached_viscous_friction_nm_per_rad_s = model->viscous_friction_nm_per_rad_s;
	state->cached_torque_constant_nm_per_a = model->torque_constant_nm_per_a;
	state->discretization_valid = true;
}

int motor_dob_validate(const struct motor_dob_config *cfg,
		       const struct motor_dob_model *model)
{
	if (cfg == NULL || model == NULL) {
		return -EINVAL;
	}
	if (!motor_dob_is_finite_positive(cfg->dt_s) ||
	    !motor_dob_is_finite_nonnegative(cfg->observer_gain_nm_per_rad_s) ||
	    !motor_dob_is_finite_positive(cfg->torque_limit_nm) ||
	    !motor_dob_is_finite_nonnegative(cfg->iq_ff_limit_a)) {
		return -EINVAL;
	}
	if (!motor_dob_is_finite_positive(model->inertia_kgm2) ||
	    !motor_dob_is_finite_positive(model->torque_constant_nm_per_a) ||
	    !motor_dob_is_finite_nonnegative(model->viscous_friction_nm_per_rad_s) ||
	    !motor_dob_is_finite_nonnegative(model->coulomb_friction_nm)) {
		return -EINVAL;
	}

	return 0;
}

void motor_dob_reset(struct motor_dob_state *state,
	    float32_t omega_initial_rad_s)
{
	if (state == NULL) {
		return;
	}

	state->initialized = true;
	state->omega_model_rad_s = isfinite(omega_initial_rad_s) ? omega_initial_rad_s : 0.0f;
	state->disturbance_nm = 0.0f;
	state->iq_ff_a = 0.0f;
	state->residual_rad_s = 0.0f;
	state->discretization_valid = false;
	state->cached_dt_s = 0.0f;
	state->cached_inertia_kgm2 = 0.0f;
	state->cached_viscous_friction_nm_per_rad_s = 0.0f;
	state->cached_torque_constant_nm_per_a = 0.0f;
	state->a = 0.0f;
	state->b_u = 0.0f;
	state->b_d = 0.0f;
}

int motor_dob_step(const struct motor_dob_config *cfg,
	  const struct motor_dob_model *model,
	  struct motor_dob_state *state,
	  float32_t omega_meas_rad_s,
	  float32_t iq_cmd_a,
	  float32_t *iq_ff_a_out)
{
	int ret = motor_dob_validate(cfg, model);

	if (ret != 0 || state == NULL || iq_ff_a_out == NULL) {
		return -EINVAL;
	}
	if (!isfinite(omega_meas_rad_s) || !isfinite(iq_cmd_a)) {
		return -EINVAL;
	}

	if (!state->initialized) {
		motor_dob_reset(state, omega_meas_rad_s);
	}

	if (!cfg->enabled) {
		state->omega_model_rad_s = omega_meas_rad_s;
		state->disturbance_nm = 0.0f;
		state->iq_ff_a = 0.0f;
		state->residual_rad_s = 0.0f;
		*iq_ff_a_out = 0.0f;
		return 0;
	}

	motor_dob_refresh_discretization(state, cfg, model);
	float32_t a = state->a;
	float32_t b_u = state->b_u;
	float32_t b_d = state->b_d;

	float32_t sign_speed = motor_dob_sign_with_deadband(state->omega_model_rad_s,
						    MOTOR_DOB_FRICTION_DEADBAND_RAD_S);
	if (sign_speed == 0.0f) {
		sign_speed = motor_dob_sign_with_deadband(omega_meas_rad_s,
						 MOTOR_DOB_FRICTION_DEADBAND_RAD_S);
	}

	float32_t tau_coulomb_nm = model->coulomb_friction_nm * sign_speed;
	float32_t omega_pred = a * state->omega_model_rad_s +
			       b_u * iq_cmd_a +
			       b_d * (state->disturbance_nm - tau_coulomb_nm);
	float32_t residual = omega_meas_rad_s - omega_pred;

	state->residual_rad_s = residual;
	if (cfg->observer_gain_nm_per_rad_s > 0.0f) {
		state->disturbance_nm += cfg->observer_gain_nm_per_rad_s * residual;
		state->disturbance_nm =
			clampf(state->disturbance_nm,
			       -cfg->torque_limit_nm,
			       cfg->torque_limit_nm);
	}

	float32_t iq_limit = cfg->iq_ff_limit_a;
	if (iq_limit <= 0.0f) {
		iq_limit = cfg->torque_limit_nm / model->torque_constant_nm_per_a;
	}
	if (!isfinite(iq_limit) || iq_limit <= 0.0f) {
		return -ERANGE;
	}

	float32_t iq_ff = -state->disturbance_nm / model->torque_constant_nm_per_a;
	if (!isfinite(iq_ff)) {
		return -ERANGE;
	}

	iq_ff = clampf(iq_ff, -iq_limit, iq_limit);
	state->iq_ff_a = iq_ff;
	state->omega_model_rad_s = omega_meas_rad_s;
	*iq_ff_a_out = iq_ff;

	return 0;
}
