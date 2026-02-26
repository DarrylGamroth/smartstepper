/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/mpr.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/math_constants.h"

#define MOTOR_MPR_EPSILON 1e-9f
#define MOTOR_MPR_FRICTION_DEADBAND_RAD_S 1e-3f

static bool motor_mpr_is_finite_positive(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static bool motor_mpr_is_finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

static float32_t motor_mpr_sign_with_deadband(float32_t value, float32_t deadband)
{
	if (value > deadband) {
		return 1.0f;
	}
	if (value < -deadband) {
		return -1.0f;
	}
	return 0.0f;
}

static void motor_mpr_velocity_discretize(const struct motor_mpr_velocity_model *model,
					  float32_t dt_s,
					  float32_t *a_out,
					  float32_t *bu_out,
					  float32_t *bd_out)
{
	float32_t j = model->inertia_kgm2;
	float32_t b = model->viscous_friction_nm_per_rad_s;
	float32_t kt = model->torque_constant_nm_per_a;

	if (b > MOTOR_MPR_EPSILON) {
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

static bool motor_mpr_velocity_cache_matches(const struct motor_mpr_velocity_state *state,
					     const struct motor_mpr_velocity_config *cfg,
					     const struct motor_mpr_velocity_model *model)
{
	return state->discretization_valid &&
	       state->cached_dt_s == cfg->dt_s &&
	       state->cached_inertia_kgm2 == model->inertia_kgm2 &&
	       state->cached_viscous_friction_nm_per_rad_s ==
		       model->viscous_friction_nm_per_rad_s &&
	       state->cached_torque_constant_nm_per_a == model->torque_constant_nm_per_a;
}

static void motor_mpr_velocity_refresh_discretization(struct motor_mpr_velocity_state *state,
						      const struct motor_mpr_velocity_config *cfg,
						      const struct motor_mpr_velocity_model *model)
{
	if (motor_mpr_velocity_cache_matches(state, cfg, model)) {
		return;
	}

	motor_mpr_velocity_discretize(model, cfg->dt_s, &state->a, &state->b_u, &state->b_d);
	state->cached_dt_s = cfg->dt_s;
	state->cached_inertia_kgm2 = model->inertia_kgm2;
	state->cached_viscous_friction_nm_per_rad_s = model->viscous_friction_nm_per_rad_s;
	state->cached_torque_constant_nm_per_a = model->torque_constant_nm_per_a;
	state->discretization_valid = true;
}

static bool motor_mpr_position_cache_matches(const struct motor_mpr_position_state *state,
					     const struct motor_mpr_position_config *cfg)
{
	return state->initialized &&
	       state->cached_dt_s == cfg->dt_s &&
	       state->cached_horizon == cfg->horizon &&
	       state->cached_q_position == cfg->q_position &&
	       state->cached_q_velocity_ff == cfg->q_velocity_ff &&
	       state->cached_r_delta_velocity == cfg->r_delta_velocity &&
	       state->cached_velocity_limit_rad_s == cfg->velocity_limit_rad_s &&
	       state->cached_max_delta_velocity_rad_s == cfg->max_delta_velocity_rad_s;
}

int motor_mpr_velocity_validate(const struct motor_mpr_velocity_config *cfg,
				const struct motor_mpr_velocity_model *model)
{
	if (cfg == NULL || model == NULL) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(cfg->dt_s) || cfg->horizon == 0U) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(cfg->q_speed) ||
	    !motor_mpr_is_finite_positive(cfg->r_delta_iq)) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(cfg->iq_limit_a) ||
	    !motor_mpr_is_finite_nonnegative(cfg->max_delta_iq_a) ||
	    !motor_mpr_is_finite_nonnegative(cfg->disturbance_ki_nm_per_rad_s)) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(model->inertia_kgm2) ||
	    !motor_mpr_is_finite_positive(model->torque_constant_nm_per_a)) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_nonnegative(model->viscous_friction_nm_per_rad_s) ||
	    !motor_mpr_is_finite_nonnegative(model->coulomb_friction_nm)) {
		return -EINVAL;
	}

	return 0;
}

int motor_mpr_velocity_init(const struct motor_mpr_velocity_config *cfg,
			    const struct motor_mpr_velocity_model *model,
			    struct motor_mpr_velocity_state *state,
			    float32_t omega_initial_rad_s,
			    float32_t iq_initial_a)
{
	int ret = motor_mpr_velocity_validate(cfg, model);

	if (ret != 0 || state == NULL) {
		return -EINVAL;
	}

	motor_mpr_velocity_refresh_discretization(state, cfg, model);
	state->initialized = true;
	state->iq_cmd_a = isfinite(iq_initial_a) ? iq_initial_a : 0.0f;
	state->omega_model_rad_s = isfinite(omega_initial_rad_s) ? omega_initial_rad_s : 0.0f;
	state->disturbance_nm = 0.0f;
	state->last_omega_error_rad_s = 0.0f;
	return 0;
}

bool motor_mpr_velocity_is_configured(const struct motor_mpr_velocity_state *state,
				      const struct motor_mpr_velocity_config *cfg,
				      const struct motor_mpr_velocity_model *model)
{
	if (state == NULL || cfg == NULL || model == NULL) {
		return false;
	}
	return motor_mpr_velocity_cache_matches(state, cfg, model);
}

void motor_mpr_velocity_reset(struct motor_mpr_velocity_state *state,
			      float32_t omega_initial_rad_s,
			      float32_t iq_initial_a)
{
	if (state == NULL) {
		return;
	}

	state->initialized = true;
	state->iq_cmd_a = isfinite(iq_initial_a) ? iq_initial_a : 0.0f;
	state->omega_model_rad_s = isfinite(omega_initial_rad_s) ? omega_initial_rad_s : 0.0f;
	state->disturbance_nm = 0.0f;
	state->last_omega_error_rad_s = 0.0f;
}

int motor_mpr_velocity_step(const struct motor_mpr_velocity_config *cfg,
			    const struct motor_mpr_velocity_model *model,
			    struct motor_mpr_velocity_state *state,
			    float32_t omega_meas_rad_s,
			    float32_t omega_ref_rad_s,
			    float32_t *iq_cmd_a_out)
{
	if (cfg == NULL || model == NULL || state == NULL || iq_cmd_a_out == NULL) {
		return -EINVAL;
	}
	if (cfg->dt_s <= 0.0f || cfg->horizon == 0U || cfg->q_speed <= 0.0f ||
	    cfg->r_delta_iq <= 0.0f || cfg->iq_limit_a <= 0.0f ||
	    cfg->max_delta_iq_a < 0.0f || cfg->disturbance_ki_nm_per_rad_s < 0.0f ||
	    model->inertia_kgm2 <= 0.0f || model->torque_constant_nm_per_a <= 0.0f ||
	    model->viscous_friction_nm_per_rad_s < 0.0f || model->coulomb_friction_nm < 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(omega_meas_rad_s) || !isfinite(omega_ref_rad_s)) {
		return -EINVAL;
	}
	if (!state->initialized || !state->discretization_valid) {
		return -EINVAL;
	}
	float32_t a = state->a;
	float32_t b_u = state->b_u;
	float32_t b_d = state->b_d;

	float32_t sign_speed = motor_mpr_sign_with_deadband(omega_meas_rad_s,
							    MOTOR_MPR_FRICTION_DEADBAND_RAD_S);
	if (sign_speed == 0.0f) {
		sign_speed = motor_mpr_sign_with_deadband(state->omega_model_rad_s,
							   MOTOR_MPR_FRICTION_DEADBAND_RAD_S);
	}
	float32_t tau_coulomb_nm = model->coulomb_friction_nm * sign_speed;
	float32_t d_term = b_d * (state->disturbance_nm - tau_coulomb_nm);

	float32_t omega_pred_prev = a * state->omega_model_rad_s +
				    b_u * state->iq_cmd_a + d_term;
	float32_t omega_pred_residual = omega_meas_rad_s - omega_pred_prev;

	if (cfg->disturbance_ki_nm_per_rad_s > 0.0f) {
		float32_t disturbance_limit = model->torque_constant_nm_per_a * cfg->iq_limit_a;

		state->disturbance_nm += cfg->disturbance_ki_nm_per_rad_s * omega_pred_residual;
		if (disturbance_limit > 0.0f) {
			state->disturbance_nm = clampf(state->disturbance_nm,
						      -disturbance_limit,
						      disturbance_limit);
		}
		d_term = b_d * (state->disturbance_nm - tau_coulomb_nm);
	}

	float32_t q = cfg->q_speed;
	float32_t sum_num = 0.0f;
	float32_t sum_den = cfg->r_delta_iq;
	float32_t m = omega_meas_rad_s;
	float32_t n = 0.0f;
	uint16_t horizon = cfg->horizon;

	for (uint16_t i = 0U; i < horizon; i++) {
		m = a * m + d_term;
		n = a * n + b_u;
		sum_num += q * (omega_ref_rad_s - m) * n;
		sum_den += q * n * n;
	}

	if (!isfinite(sum_den) || sum_den <= MOTOR_MPR_EPSILON || !isfinite(sum_num)) {
		return -ERANGE;
	}

	float32_t u_prev = state->iq_cmd_a;
	float32_t u_opt = (sum_num + (cfg->r_delta_iq * u_prev)) / sum_den;
	if (!isfinite(u_opt)) {
		return -ERANGE;
	}

	float32_t max_delta = cfg->max_delta_iq_a;
	if (max_delta <= 0.0f) {
		max_delta = cfg->iq_limit_a;
	}
	u_opt = clampf(u_opt, u_prev - max_delta, u_prev + max_delta);
	u_opt = clampf(u_opt, -cfg->iq_limit_a, cfg->iq_limit_a);

	state->iq_cmd_a = u_opt;
	state->omega_model_rad_s = omega_meas_rad_s;
	state->last_omega_error_rad_s = omega_ref_rad_s - omega_meas_rad_s;
	*iq_cmd_a_out = u_opt;

	return 0;
}

int motor_mpr_position_validate(const struct motor_mpr_position_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(cfg->dt_s) || cfg->horizon == 0U) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_nonnegative(cfg->q_position) ||
	    !motor_mpr_is_finite_nonnegative(cfg->q_velocity_ff) ||
	    !motor_mpr_is_finite_positive(cfg->r_delta_velocity)) {
		return -EINVAL;
	}
	if ((cfg->q_position + cfg->q_velocity_ff) <= 0.0f) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(cfg->velocity_limit_rad_s) ||
	    !motor_mpr_is_finite_nonnegative(cfg->max_delta_velocity_rad_s)) {
		return -EINVAL;
	}

	return 0;
}

int motor_mpr_position_init(const struct motor_mpr_position_config *cfg,
			    struct motor_mpr_position_state *state,
			    float32_t velocity_initial_rad_s)
{
	int ret = motor_mpr_position_validate(cfg);

	if (ret != 0 || state == NULL) {
		return -EINVAL;
	}

	state->initialized = true;
	state->velocity_cmd_rad_s = isfinite(velocity_initial_rad_s) ? velocity_initial_rad_s : 0.0f;
	state->last_position_error_rad = 0.0f;
	state->cached_dt_s = cfg->dt_s;
	state->cached_horizon = cfg->horizon;
	state->cached_q_position = cfg->q_position;
	state->cached_q_velocity_ff = cfg->q_velocity_ff;
	state->cached_r_delta_velocity = cfg->r_delta_velocity;
	state->cached_velocity_limit_rad_s = cfg->velocity_limit_rad_s;
	state->cached_max_delta_velocity_rad_s = cfg->max_delta_velocity_rad_s;
	return 0;
}

bool motor_mpr_position_is_configured(const struct motor_mpr_position_state *state,
				      const struct motor_mpr_position_config *cfg)
{
	if (state == NULL || cfg == NULL) {
		return false;
	}
	return motor_mpr_position_cache_matches(state, cfg);
}

void motor_mpr_position_reset(struct motor_mpr_position_state *state,
			      float32_t velocity_initial_rad_s)
{
	if (state == NULL) {
		return;
	}

	state->initialized = true;
	state->velocity_cmd_rad_s = isfinite(velocity_initial_rad_s) ? velocity_initial_rad_s : 0.0f;
	state->last_position_error_rad = 0.0f;
}

int motor_mpr_position_step(const struct motor_mpr_position_config *cfg,
			    struct motor_mpr_position_state *state,
			    float32_t position_error_rad,
			    float32_t velocity_ff_rad_s,
			    float32_t *velocity_cmd_rad_s_out)
{
	if (cfg == NULL || state == NULL || velocity_cmd_rad_s_out == NULL) {
		return -EINVAL;
	}
	if (cfg->dt_s <= 0.0f || cfg->horizon == 0U || cfg->r_delta_velocity <= 0.0f ||
	    cfg->velocity_limit_rad_s <= 0.0f || cfg->max_delta_velocity_rad_s < 0.0f ||
	    (cfg->q_position + cfg->q_velocity_ff) <= 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(position_error_rad) || !isfinite(velocity_ff_rad_s)) {
		return -EINVAL;
	}
	if (!state->initialized) {
		return -EINVAL;
	}

	float32_t dt = cfg->dt_s;
	float32_t sum_c = 0.0f;
	float32_t sum_c2 = 0.0f;
	uint16_t horizon = cfg->horizon;

	for (uint16_t i = 1U; i <= horizon; i++) {
		float32_t c_i = (float32_t)i * dt;

		sum_c += c_i;
		sum_c2 += c_i * c_i;
	}

	float32_t n = (float32_t)horizon;
	float32_t q_pos = cfg->q_position;
	float32_t q_vel = cfg->q_velocity_ff;
	float32_t r = cfg->r_delta_velocity;
	float32_t v_prev = state->velocity_cmd_rad_s;

	float32_t numerator = (q_pos * position_error_rad * sum_c) +
			      (q_vel * n * velocity_ff_rad_s) +
			      (r * v_prev);
	float32_t denominator = (q_pos * sum_c2) + (q_vel * n) + r;
	if (!isfinite(numerator) || !isfinite(denominator) || denominator <= MOTOR_MPR_EPSILON) {
		return -ERANGE;
	}

	float32_t v_opt = numerator / denominator;
	if (!isfinite(v_opt)) {
		return -ERANGE;
	}

	float32_t max_delta = cfg->max_delta_velocity_rad_s;
	if (max_delta <= 0.0f) {
		max_delta = cfg->velocity_limit_rad_s;
	}
	v_opt = clampf(v_opt, v_prev - max_delta, v_prev + max_delta);
	v_opt = clampf(v_opt, -cfg->velocity_limit_rad_s,
		       cfg->velocity_limit_rad_s);

	state->velocity_cmd_rad_s = v_opt;
	state->last_position_error_rad = position_error_rad;
	*velocity_cmd_rad_s_out = v_opt;

	return 0;
}
