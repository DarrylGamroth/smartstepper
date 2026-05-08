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
#define MOTOR_MPR_ZERO_HOLD_EPS_RAD_S (0.02f * 2.0f * PI_F32)

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

static bool motor_mpr_bandwidth_model_valid(float32_t inertia_kgm2,
					    float32_t torque_constant_nm_per_a)
{
	return motor_mpr_is_finite_positive(inertia_kgm2) &&
	       motor_mpr_is_finite_positive(torque_constant_nm_per_a);
}

int motor_mpr_velocity_config_from_bandwidth(
	const struct motor_mpr_velocity_bandwidth_input *in,
	struct motor_mpr_velocity_config *cfg,
	struct motor_mpr_bandwidth_result *result)
{
	if (in == NULL || cfg == NULL) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(in->bandwidth_hz) ||
	    !motor_mpr_is_finite_positive(in->iq_limit_a) ||
	    !motor_mpr_is_finite_positive(in->dt_s)) {
		return -EINVAL;
	}

	bool model_used =
		motor_mpr_bandwidth_model_valid(in->inertia_kgm2,
						in->torque_constant_nm_per_a);
	const float32_t bandwidth_hz =
		fmaxf(in->bandwidth_hz, MOTOR_MPR_VELOCITY_BW_Q_REF_HZ);
	float32_t q_speed =
		MOTOR_MPR_VELOCITY_BW_Q_MIN *
		((bandwidth_hz * bandwidth_hz) /
		 (MOTOR_MPR_VELOCITY_BW_Q_REF_HZ * MOTOR_MPR_VELOCITY_BW_Q_REF_HZ));
	if (model_used) {
		const float32_t omega = 2.0f * PI_F32 * in->bandwidth_hz;
		const float32_t model_iq_per_rad_s =
			(omega * in->inertia_kgm2) /
			(in->torque_constant_nm_per_a * in->iq_limit_a);
		q_speed = model_iq_per_rad_s * MOTOR_MPR_VELOCITY_BW_MODEL_SCALE;
	}

	const float32_t q_unclamped = q_speed;
	q_speed = clampf(q_speed, MOTOR_MPR_VELOCITY_BW_Q_MIN,
			 MOTOR_MPR_VELOCITY_BW_Q_MAX);
	const float32_t r_delta_iq =
		clampf(1.0f / (16.0f * q_speed), MOTOR_MPR_VELOCITY_BW_R_MIN,
		       MOTOR_MPR_VELOCITY_BW_R_MAX);
	const float32_t di_frac =
		MOTOR_MPR_VELOCITY_BW_DI_BASE_FRAC +
		(MOTOR_MPR_VELOCITY_BW_DI_BW_FRAC *
		 fminf(bandwidth_hz / MOTOR_MPR_VELOCITY_BW_DI_REF_HZ, 1.0f));
	const float32_t max_delta_iq =
		clampf(in->iq_limit_a * di_frac, MOTOR_MPR_VELOCITY_BW_DI_MIN_A,
		       fminf(in->iq_limit_a, MOTOR_MPR_VELOCITY_BW_DI_MAX_A));
	float32_t disturbance_ki = 0.0f;
	if (model_used) {
		const float32_t torque_limit_nm =
			in->torque_constant_nm_per_a * in->iq_limit_a;
		disturbance_ki = clampf(torque_limit_nm *
					MOTOR_MPR_VELOCITY_BW_DIST_KI_FRAC,
					MOTOR_MPR_VELOCITY_BW_DIST_KI_MIN,
					MOTOR_MPR_VELOCITY_BW_DIST_KI_MAX);
	}

	cfg->dt_s = in->dt_s;
	cfg->horizon = MOTOR_MPR_VELOCITY_BW_HORIZON;
	cfg->q_speed = q_speed;
	cfg->r_delta_iq = r_delta_iq;
	cfg->iq_limit_a = in->iq_limit_a;
	cfg->max_delta_iq_a = max_delta_iq;
	cfg->disturbance_ki_nm_per_rad_s = disturbance_ki;

	if (result != NULL) {
		result->requested_bandwidth_hz = in->bandwidth_hz;
		result->applied_bandwidth_hz = in->bandwidth_hz;
		result->model_used = model_used;
		result->clamped = (q_unclamped != q_speed);
	}

	return 0;
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

int motor_mpr_velocity_validate(const struct motor_mpr_velocity_config *cfg,
				const struct motor_mpr_velocity_model *model)
{
	if (cfg == NULL || model == NULL) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(cfg->dt_s) || cfg->horizon == 0U ||
	    cfg->horizon > MOTOR_MPR_HORIZON_MAX) {
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

	motor_mpr_velocity_discretize(model, cfg->dt_s, &state->a, &state->b_u, &state->b_d);
	state->initialized = true;
	state->iq_cmd_a = isfinite(iq_initial_a) ? iq_initial_a : 0.0f;
	state->omega_model_rad_s = isfinite(omega_initial_rad_s) ? omega_initial_rad_s : 0.0f;
	state->disturbance_nm = 0.0f;
	state->last_omega_error_rad_s = 0.0f;
	return 0;
}

void motor_mpr_velocity_reset(struct motor_mpr_velocity_state *state,
			      float32_t omega_initial_rad_s,
			      float32_t iq_initial_a)
{
	if (state == NULL) {
		return;
	}

	state->iq_cmd_a = isfinite(iq_initial_a) ? iq_initial_a : 0.0f;
	state->omega_model_rad_s = isfinite(omega_initial_rad_s) ? omega_initial_rad_s : 0.0f;
	state->disturbance_nm = 0.0f;
	state->last_omega_error_rad_s = 0.0f;
}

void motor_mpr_velocity_invalidate(struct motor_mpr_velocity_state *state)
{
	if (state == NULL) {
		return;
	}

	state->initialized = false;
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
	if (motor_mpr_velocity_validate(cfg, model) != 0) {
		return -EINVAL;
	}
	return motor_mpr_velocity_step_fast(cfg, model, state, omega_meas_rad_s,
					    omega_ref_rad_s, iq_cmd_a_out);
}

int motor_mpr_velocity_step_fast(const struct motor_mpr_velocity_config *cfg,
				 const struct motor_mpr_velocity_model *model,
				 struct motor_mpr_velocity_state *state,
				 float32_t omega_meas_rad_s,
				 float32_t omega_ref_rad_s,
				 float32_t *iq_cmd_a_out)
{
	if (cfg == NULL || model == NULL || state == NULL || iq_cmd_a_out == NULL ||
	    !state->initialized) {
		return -EINVAL;
	}

	float32_t a = state->a;
	float32_t b_u = state->b_u;
	float32_t b_d = state->b_d;

	if (fabsf(omega_ref_rad_s) <= MOTOR_MPR_ZERO_HOLD_EPS_RAD_S &&
	    fabsf(omega_meas_rad_s) <= MOTOR_MPR_ZERO_HOLD_EPS_RAD_S) {
		motor_mpr_velocity_reset(state, omega_meas_rad_s, 0.0f);
		*iq_cmd_a_out = 0.0f;
		return 0;
	}

	float32_t sign_speed = motor_mpr_sign_with_deadband(omega_meas_rad_s,
							    MOTOR_MPR_FRICTION_DEADBAND_RAD_S);
	if (sign_speed == 0.0f) {
		sign_speed = motor_mpr_sign_with_deadband(state->omega_model_rad_s,
							   MOTOR_MPR_FRICTION_DEADBAND_RAD_S);
	}
	if (sign_speed == 0.0f) {
		sign_speed = motor_mpr_sign_with_deadband(omega_ref_rad_s,
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

	/* One-move horizon expansion:
	 * m tracks free response, n tracks input influence.
	 * This accumulates J = q*||e||^2 + r*||delta_u||^2 over the horizon.
	 */
	for (uint16_t i = 0U; i < horizon; i++) {
		m = a * m + d_term;
		n = a * n + b_u;
		sum_num += q * (omega_ref_rad_s - m) * n;
		sum_den += q * n * n;
	}

	if (!(sum_den > MOTOR_MPR_EPSILON)) {
		return -ERANGE;
	}

	float32_t u_prev = state->iq_cmd_a;
	float32_t u_opt = (sum_num + (cfg->r_delta_iq * u_prev)) / sum_den;

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
	if (!motor_mpr_is_finite_positive(cfg->dt_s) || cfg->horizon == 0U ||
	    cfg->horizon > MOTOR_MPR_HORIZON_MAX) {
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

int motor_mpr_position_config_from_bandwidth(
	const struct motor_mpr_position_bandwidth_input *in,
	struct motor_mpr_position_config *cfg,
	struct motor_mpr_bandwidth_result *result)
{
	if (in == NULL || cfg == NULL) {
		return -EINVAL;
	}
	if (!motor_mpr_is_finite_positive(in->bandwidth_hz) ||
	    !motor_mpr_is_finite_positive(in->velocity_limit_rad_s) ||
	    !motor_mpr_is_finite_positive(in->accel_limit_rad_s2) ||
	    !motor_mpr_is_finite_positive(in->dt_s)) {
		return -EINVAL;
	}

	const float32_t omega = 2.0f * PI_F32 * in->bandwidth_hz;
	const float32_t q_pos_unclamped = 2.0f * omega;
	const float32_t q_vel_unclamped = 0.5f * omega;
	const float32_t q_position =
		clampf(q_pos_unclamped, MOTOR_MPR_POSITION_BW_Q_POS_MIN,
		       MOTOR_MPR_POSITION_BW_Q_POS_MAX);
	const float32_t q_velocity_ff =
		clampf(q_vel_unclamped, MOTOR_MPR_POSITION_BW_Q_VEL_MIN,
		       MOTOR_MPR_POSITION_BW_Q_VEL_MAX);
	const float32_t r_delta_velocity =
		clampf(1.0f / (5.0f * q_position), MOTOR_MPR_POSITION_BW_R_MIN,
		       MOTOR_MPR_POSITION_BW_R_MAX);
	const float32_t max_delta_velocity =
		clampf(in->accel_limit_rad_s2 * in->dt_s, 0.001f,
		       in->velocity_limit_rad_s);

	cfg->dt_s = in->dt_s;
	cfg->horizon = MOTOR_MPR_POSITION_BW_HORIZON;
	cfg->q_position = q_position;
	cfg->q_velocity_ff = q_velocity_ff;
	cfg->r_delta_velocity = r_delta_velocity;
	cfg->velocity_limit_rad_s = in->velocity_limit_rad_s;
	cfg->max_delta_velocity_rad_s = max_delta_velocity;

	if (result != NULL) {
		result->requested_bandwidth_hz = in->bandwidth_hz;
		result->applied_bandwidth_hz = in->bandwidth_hz;
		result->model_used = false;
		result->clamped = (q_pos_unclamped != q_position) ||
				  (q_vel_unclamped != q_velocity_ff);
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
	state->horizon = cfg->horizon;
	state->dt_s = cfg->dt_s;
	state->horizon_sum_c = 0.0f;
	state->horizon_sum_c2 = 0.0f;
	for (uint16_t i = 1U; i <= cfg->horizon; i++) {
		float32_t c_i = (float32_t)i * cfg->dt_s;

		state->horizon_sum_c += c_i;
		state->horizon_sum_c2 += c_i * c_i;
	}
	return 0;
}

void motor_mpr_position_reset(struct motor_mpr_position_state *state,
			      float32_t velocity_initial_rad_s)
{
	if (state == NULL) {
		return;
	}

	state->velocity_cmd_rad_s = isfinite(velocity_initial_rad_s) ? velocity_initial_rad_s : 0.0f;
	state->last_position_error_rad = 0.0f;
}

void motor_mpr_position_invalidate(struct motor_mpr_position_state *state)
{
	if (state == NULL) {
		return;
	}

	state->initialized = false;
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
	if (motor_mpr_position_validate(cfg) != 0) {
		return -EINVAL;
	}
	return motor_mpr_position_step_fast(cfg, state, position_error_rad, velocity_ff_rad_s,
					    velocity_cmd_rad_s_out);
}

int motor_mpr_position_step_fast(const struct motor_mpr_position_config *cfg,
				 struct motor_mpr_position_state *state,
				 float32_t position_error_rad,
				 float32_t velocity_ff_rad_s,
				 float32_t *velocity_cmd_rad_s_out)
{
	if (cfg == NULL || state == NULL || velocity_cmd_rad_s_out == NULL ||
	    !state->initialized || state->horizon != cfg->horizon ||
	    state->dt_s != cfg->dt_s) {
		return -EINVAL;
	}

	float32_t n = (float32_t)cfg->horizon;
	float32_t q_pos = cfg->q_position;
	float32_t q_vel = cfg->q_velocity_ff;
	float32_t r = cfg->r_delta_velocity;
	float32_t v_prev = state->velocity_cmd_rad_s;

	float32_t numerator = (q_pos * position_error_rad * state->horizon_sum_c) +
			      (q_vel * n * velocity_ff_rad_s) +
			      (r * v_prev);
	float32_t denominator = (q_pos * state->horizon_sum_c2) + (q_vel * n) + r;
	if (!(denominator > MOTOR_MPR_EPSILON)) {
		return -ERANGE;
	}

	float32_t v_opt = numerator / denominator;

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
