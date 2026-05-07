/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/estimation/commission_estimators.h"

#include "motor/math/matrix_solve.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/sys/util.h>

#define MOTOR_MECH_ID_VAR_EPS 1.0e-8f
#define MOTOR_MECH_ID_PIVOT_EPS 1.0e-9f

static bool motor_mech_id_is_finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

int motor_mech_id_validate_config(const struct motor_mech_id_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (!isfinite(cfg->kt_nm_per_a) ||
	    fabsf(cfg->kt_nm_per_a) < MOTOR_MECH_ID_PIVOT_EPS ||
	    !motor_mech_id_is_finite_nonnegative(cfg->sign_deadband_rad_s) ||
	    !isfinite(cfg->min_r2) ||
	    cfg->min_samples == 0U) {
		return -EINVAL;
	}

	if (cfg->min_r2 < 0.0f || cfg->min_r2 > 1.0f) {
		return -EINVAL;
	}

	return 0;
}

void motor_mech_id_init(struct motor_mech_id_state *state,
			const struct motor_mech_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (motor_mech_id_validate_config(cfg) == 0) {
		state->cfg = *cfg;
		state->config_valid = true;
	}
}

bool motor_mech_id_accumulate(struct motor_mech_id_state *state, float32_t mech_speed_rad_s,
			      float32_t mech_accel_rad_s2, float32_t iq_a)
{
	if (state == NULL) {
		return false;
	}
	if (!state->config_valid) {
		return false;
	}

	if (!isfinite(mech_speed_rad_s) || !isfinite(mech_accel_rad_s2) || !isfinite(iq_a) ||
	    !isfinite(state->cfg.kt_nm_per_a) || fabsf(state->cfg.kt_nm_per_a) < MOTOR_MECH_ID_PIVOT_EPS) {
		return false;
	}

	float32_t sign_term = 0.0f;
	if (mech_speed_rad_s > state->cfg.sign_deadband_rad_s) {
		sign_term = 1.0f;
	} else if (mech_speed_rad_s < -state->cfg.sign_deadband_rad_s) {
		sign_term = -1.0f;
	}

	const float32_t phi[4] = {
		mech_accel_rad_s2,
		mech_speed_rad_s,
		sign_term,
		1.0f,
	};
	const float32_t z = state->cfg.kt_nm_per_a * iq_a;
	if (!isfinite(z)) {
		return false;
	}

	for (uint32_t r = 0U; r < 4U; r++) {
		state->b[r] += phi[r] * z;
		for (uint32_t c = 0U; c < 4U; c++) {
			state->A[r][c] += phi[r] * phi[c];
		}
	}

	state->sum_z += z;
	state->sum_z2 += z * z;
	state->sample_count++;

	return true;
}

int motor_mech_id_finalize(const struct motor_mech_id_state *state, struct motor_mech_id_result *result)
{
	if (state == NULL || result == NULL) {
		return -EINVAL;
	}
	if (!state->config_valid) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->sample_count = (uint16_t)MIN(state->sample_count, UINT16_MAX);

	/* Config validation guarantees min_samples > 0. */
	const uint32_t min_samples = state->cfg.min_samples;
	if (state->sample_count < min_samples) {
		return -ENODATA;
	}

	float32_t A[4][4];
	float32_t b[4];
	float32_t theta[4];
	memcpy(A, state->A, sizeof(A));
	memcpy(b, state->b, sizeof(b));

	if (!motor_math_solve_linear_4x4(A, b, theta)) {
		return -ERANGE;
	}

	const float32_t n = (float32_t)state->sample_count;
	/* Use accumulated regressor vector state->b; local solver copy is mutated in-place. */
	const float32_t theta_dot_b = theta[0] * state->b[0] + theta[1] * state->b[1] +
				      theta[2] * state->b[2] + theta[3] * state->b[3];
	float32_t sse = state->sum_z2 - theta_dot_b;
	if (sse < 0.0f) {
		sse = 0.0f;
	}

	const float32_t mean_z = state->sum_z / n;
	float32_t sst = state->sum_z2 - n * mean_z * mean_z;
	if (sst < 0.0f) {
		sst = 0.0f;
	}

	/* 4-parameter model -> residual dof = n - 4 when available. */
	const float32_t residual_dof = (state->sample_count > 4U) ? (n - 4.0f) : n;
	const float32_t residual_rms_nm = sqrtf(sse / residual_dof);
	const float32_t r2 = (sst > MOTOR_MECH_ID_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;

	result->inertia_kgm2 = theta[0];
	result->viscous_friction_nm_per_rad_s = theta[1];
	result->coulomb_friction_nm = fabsf(theta[2]);
	result->offset_friction_nm = theta[3];
	result->residual_rms_nm = residual_rms_nm;
	result->r2 = r2;
	result->valid = isfinite(theta[0]) && isfinite(theta[1]) && isfinite(theta[2]) &&
			isfinite(theta[3]) && isfinite(residual_rms_nm) && isfinite(r2) &&
			(!state->cfg.require_positive_inertia || (theta[0] > 0.0f)) &&
			(!state->cfg.require_nonnegative_viscous || (theta[1] >= 0.0f)) &&
			(r2 >= state->cfg.min_r2);

	return 0;
}

static inline float32_t motor_mech_sign(float32_t speed_rad_s, float32_t deadband_rad_s)
{
	if (speed_rad_s > deadband_rad_s) {
		return 1.0f;
	}
	if (speed_rad_s < -deadband_rad_s) {
		return -1.0f;
	}

	return 0.0f;
}

int motor_mech_friction_id_validate_config(const struct motor_mech_friction_id_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (!isfinite(cfg->kt_nm_per_a) ||
	    fabsf(cfg->kt_nm_per_a) < MOTOR_MECH_ID_PIVOT_EPS ||
	    !motor_mech_id_is_finite_nonnegative(cfg->sign_deadband_rad_s) ||
	    !motor_mech_id_is_finite_nonnegative(cfg->max_abs_accel_rad_s2) ||
	    !isfinite(cfg->min_r2) ||
	    cfg->min_samples == 0U ||
	    cfg->min_samples_per_direction == 0U ||
	    cfg->min_r2 < 0.0f ||
	    cfg->min_r2 > 1.0f) {
		return -EINVAL;
	}

	return 0;
}

void motor_mech_friction_id_init(struct motor_mech_friction_id_state *state,
				 const struct motor_mech_friction_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (motor_mech_friction_id_validate_config(cfg) == 0) {
		state->cfg = *cfg;
		state->config_valid = true;
	}
}

bool motor_mech_friction_id_accumulate(struct motor_mech_friction_id_state *state,
				       float32_t mech_speed_rad_s,
				       float32_t mech_accel_rad_s2,
				       float32_t iq_a,
				       float32_t detent_torque_nm)
{
	if (state == NULL || !state->config_valid) {
		return false;
	}
	if (!isfinite(mech_speed_rad_s) ||
	    !isfinite(mech_accel_rad_s2) ||
	    !isfinite(iq_a) ||
	    !isfinite(detent_torque_nm)) {
		state->rejected_count++;
		return false;
	}

	const float32_t sign_term = motor_mech_sign(mech_speed_rad_s,
						    state->cfg.sign_deadband_rad_s);
	if (sign_term == 0.0f || fabsf(mech_accel_rad_s2) > state->cfg.max_abs_accel_rad_s2) {
		state->rejected_count++;
		return false;
	}

	const float32_t phi[3] = {
		mech_speed_rad_s,
		sign_term,
		1.0f,
	};
	const float32_t z = (state->cfg.kt_nm_per_a * iq_a) - detent_torque_nm;
	if (!isfinite(z)) {
		state->rejected_count++;
		return false;
	}

	for (uint32_t r = 0U; r < 3U; r++) {
		state->b[r] += phi[r] * z;
		for (uint32_t c = 0U; c < 3U; c++) {
			state->A[r][c] += phi[r] * phi[c];
		}
	}

	state->sum_z += z;
	state->sum_z2 += z * z;
	state->sample_count++;
	if (sign_term > 0.0f) {
		state->positive_count++;
	} else {
		state->negative_count++;
	}

	return true;
}

int motor_mech_friction_id_finalize(const struct motor_mech_friction_id_state *state,
				    struct motor_mech_friction_id_result *result)
{
	if (state == NULL || result == NULL) {
		return -EINVAL;
	}
	if (!state->config_valid) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->sample_count = (uint16_t)MIN(state->sample_count, UINT16_MAX);
	result->positive_count = (uint16_t)MIN(state->positive_count, UINT16_MAX);
	result->negative_count = (uint16_t)MIN(state->negative_count, UINT16_MAX);
	result->rejected_count = (uint16_t)MIN(state->rejected_count, UINT16_MAX);

	if (state->sample_count < state->cfg.min_samples ||
	    state->positive_count < state->cfg.min_samples_per_direction ||
	    state->negative_count < state->cfg.min_samples_per_direction) {
		return -ENODATA;
	}

	float32_t A[3][3];
	float32_t b[3];
	float32_t theta[3];
	memcpy(A, state->A, sizeof(A));
	memcpy(b, state->b, sizeof(b));

	if (!motor_math_solve_linear_3x3(A, b, theta)) {
		return -ERANGE;
	}

	const float32_t n = (float32_t)state->sample_count;
	const float32_t theta_dot_b = theta[0] * state->b[0] + theta[1] * state->b[1] +
				      theta[2] * state->b[2];
	float32_t sse = state->sum_z2 - theta_dot_b;
	if (sse < 0.0f) {
		sse = 0.0f;
	}
	const float32_t mean_z = state->sum_z / n;
	float32_t sst = state->sum_z2 - n * mean_z * mean_z;
	if (sst < 0.0f) {
		sst = 0.0f;
	}

	const float32_t residual_dof = (state->sample_count > 3U) ? (n - 3.0f) : n;
	const float32_t residual_rms_nm = sqrtf(sse / residual_dof);
	const float32_t r2 = (sst > MOTOR_MECH_ID_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;

	result->viscous_friction_nm_per_rad_s = theta[0];
	result->coulomb_friction_nm = fabsf(theta[1]);
	result->signed_coulomb_friction_nm = theta[1];
	result->offset_friction_nm = theta[2];
	result->residual_rms_nm = residual_rms_nm;
	result->r2 = r2;
	const float32_t coulomb_mag = fabsf(theta[1]);
	result->valid = isfinite(theta[0]) &&
			isfinite(theta[1]) &&
			isfinite(theta[2]) &&
			isfinite(residual_rms_nm) &&
			isfinite(r2) &&
			(!state->cfg.require_nonnegative_viscous || theta[0] >= 0.0f) &&
			(!state->cfg.require_nonnegative_coulomb || coulomb_mag >= 0.0f) &&
			r2 >= state->cfg.min_r2;

	return 0;
}

int motor_mech_friction_id_finalize_zero_viscous(
	const struct motor_mech_friction_id_state *state,
	struct motor_mech_friction_id_result *result)
{
	if (state == NULL || result == NULL) {
		return -EINVAL;
	}
	if (!state->config_valid) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->sample_count = (uint16_t)MIN(state->sample_count, UINT16_MAX);
	result->positive_count = (uint16_t)MIN(state->positive_count, UINT16_MAX);
	result->negative_count = (uint16_t)MIN(state->negative_count, UINT16_MAX);
	result->rejected_count = (uint16_t)MIN(state->rejected_count, UINT16_MAX);

	if (state->sample_count < state->cfg.min_samples ||
	    state->positive_count < state->cfg.min_samples_per_direction ||
	    state->negative_count < state->cfg.min_samples_per_direction) {
		return -ENODATA;
	}

	const float32_t a00 = state->A[1][1];
	const float32_t a01 = state->A[1][2];
	const float32_t a11 = state->A[2][2];
	const float32_t b0 = state->b[1];
	const float32_t b1 = state->b[2];
	const float32_t det = a00 * a11 - a01 * a01;
	if (fabsf(det) < MOTOR_MECH_ID_PIVOT_EPS) {
		return -ERANGE;
	}

	const float32_t inv_det = 1.0f / det;
	const float32_t tc = (b0 * a11 - b1 * a01) * inv_det;
	const float32_t t0 = (a00 * b1 - a01 * b0) * inv_det;

	const float32_t n = (float32_t)state->sample_count;
	const float32_t theta_dot_b = tc * state->b[1] + t0 * state->b[2];
	float32_t sse = state->sum_z2 - theta_dot_b;
	if (sse < 0.0f) {
		sse = 0.0f;
	}
	const float32_t mean_z = state->sum_z / n;
	float32_t sst = state->sum_z2 - n * mean_z * mean_z;
	if (sst < 0.0f) {
		sst = 0.0f;
	}

	const float32_t residual_dof = (state->sample_count > 2U) ? (n - 2.0f) : n;
	const float32_t residual_rms_nm = sqrtf(sse / residual_dof);
	const float32_t r2 = (sst > MOTOR_MECH_ID_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;

	result->viscous_friction_nm_per_rad_s = 0.0f;
	result->coulomb_friction_nm = fabsf(tc);
	result->signed_coulomb_friction_nm = tc;
	result->offset_friction_nm = t0;
	result->residual_rms_nm = residual_rms_nm;
	result->r2 = r2;
	const float32_t coulomb_mag = fabsf(tc);
	result->valid = isfinite(tc) &&
			isfinite(t0) &&
			isfinite(residual_rms_nm) &&
			isfinite(r2) &&
			(!state->cfg.require_nonnegative_coulomb || coulomb_mag >= 0.0f) &&
			r2 >= state->cfg.min_r2;

	return 0;
}

int motor_mech_inertia_id_validate_config(const struct motor_mech_inertia_id_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (!isfinite(cfg->kt_nm_per_a) ||
	    fabsf(cfg->kt_nm_per_a) < MOTOR_MECH_ID_PIVOT_EPS ||
	    !motor_mech_id_is_finite_nonnegative(cfg->sign_deadband_rad_s) ||
	    !motor_mech_id_is_finite_nonnegative(cfg->min_abs_accel_rad_s2) ||
	    !motor_mech_id_is_finite_nonnegative(cfg->viscous_friction_nm_per_rad_s) ||
	    !isfinite(cfg->coulomb_friction_nm) ||
	    !isfinite(cfg->offset_friction_nm) ||
	    !motor_mech_id_is_finite_nonnegative(cfg->fallback_inertia_kgm2) ||
	    !isfinite(cfg->min_plausibility_ratio) ||
	    !isfinite(cfg->max_plausibility_ratio) ||
	    !motor_mech_id_is_finite_nonnegative(cfg->max_residual_rms_nm) ||
	    cfg->min_samples == 0U ||
	    cfg->min_samples_per_accel_direction == 0U ||
	    cfg->min_plausibility_ratio < 0.0f ||
	    cfg->max_plausibility_ratio < cfg->min_plausibility_ratio) {
		return -EINVAL;
	}

	return 0;
}

void motor_mech_inertia_id_init(struct motor_mech_inertia_id_state *state,
				const struct motor_mech_inertia_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (motor_mech_inertia_id_validate_config(cfg) == 0) {
		state->cfg = *cfg;
		state->config_valid = true;
	}
}

bool motor_mech_inertia_id_accumulate(struct motor_mech_inertia_id_state *state,
				      float32_t mech_speed_rad_s,
				      float32_t mech_accel_rad_s2,
				      float32_t iq_a,
				      float32_t detent_torque_nm)
{
	if (state == NULL || !state->config_valid) {
		return false;
	}
	if (!isfinite(mech_speed_rad_s) ||
	    !isfinite(mech_accel_rad_s2) ||
	    !isfinite(iq_a) ||
	    !isfinite(detent_torque_nm)) {
		state->rejected_count++;
		return false;
	}
	if (fabsf(mech_accel_rad_s2) < state->cfg.min_abs_accel_rad_s2 ||
	    fabsf(mech_speed_rad_s) < state->cfg.sign_deadband_rad_s) {
		state->rejected_count++;
		return false;
	}

	const float32_t sign_term = (mech_speed_rad_s > 0.0f) ? 1.0f : -1.0f;
	const float32_t friction_nm =
		(state->cfg.viscous_friction_nm_per_rad_s * mech_speed_rad_s) +
		(state->cfg.coulomb_friction_nm * sign_term) +
		state->cfg.offset_friction_nm;
	const float32_t tau_nm = (state->cfg.kt_nm_per_a * iq_a) -
				 detent_torque_nm -
				 friction_nm;
	if (!isfinite(tau_nm)) {
		state->rejected_count++;
		return false;
	}

	state->sum_alpha_tau += mech_accel_rad_s2 * tau_nm;
	state->sum_alpha2 += mech_accel_rad_s2 * mech_accel_rad_s2;
	state->sum_tau += tau_nm;
	state->sum_tau2 += tau_nm * tau_nm;
	state->sample_count++;
	if (mech_accel_rad_s2 > 0.0f) {
		state->positive_accel_count++;
	} else {
		state->negative_accel_count++;
	}

	return true;
}

int motor_mech_inertia_id_finalize(const struct motor_mech_inertia_id_state *state,
				   struct motor_mech_inertia_id_result *result)
{
	if (state == NULL || result == NULL) {
		return -EINVAL;
	}
	if (!state->config_valid) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->sample_count = (uint16_t)MIN(state->sample_count, UINT16_MAX);
	result->positive_accel_count = (uint16_t)MIN(state->positive_accel_count, UINT16_MAX);
	result->negative_accel_count = (uint16_t)MIN(state->negative_accel_count, UINT16_MAX);
	result->rejected_count = (uint16_t)MIN(state->rejected_count, UINT16_MAX);

	if (state->sample_count < state->cfg.min_samples ||
	    state->positive_accel_count < state->cfg.min_samples_per_accel_direction ||
	    state->negative_accel_count < state->cfg.min_samples_per_accel_direction) {
		return -ENODATA;
	}
	if (state->sum_alpha2 < MOTOR_MECH_ID_VAR_EPS) {
		return -ERANGE;
	}

	const float32_t inertia = state->sum_alpha_tau / state->sum_alpha2;
	const float32_t n = (float32_t)state->sample_count;
	float32_t sse = state->sum_tau2 - 2.0f * inertia * state->sum_alpha_tau +
			inertia * inertia * state->sum_alpha2;
	if (sse < 0.0f) {
		sse = 0.0f;
	}
	const float32_t mean_tau = state->sum_tau / n;
	float32_t sst = state->sum_tau2 - n * mean_tau * mean_tau;
	if (sst < 0.0f) {
		sst = 0.0f;
	}

	const float32_t residual_dof = (state->sample_count > 1U) ? (n - 1.0f) : n;
	const float32_t residual_rms_nm = sqrtf(sse / residual_dof);
	const float32_t r2 = (sst > MOTOR_MECH_ID_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;
	const float32_t plausibility_ratio =
		(state->cfg.fallback_inertia_kgm2 > MOTOR_MECH_ID_VAR_EPS) ?
			(inertia / state->cfg.fallback_inertia_kgm2) : 1.0f;

	result->inertia_kgm2 = inertia;
	result->residual_rms_nm = residual_rms_nm;
	result->r2 = r2;
	result->plausibility_ratio = plausibility_ratio;
	result->valid = isfinite(inertia) &&
			isfinite(residual_rms_nm) &&
			isfinite(r2) &&
			isfinite(plausibility_ratio) &&
			inertia > 0.0f &&
			(state->cfg.max_residual_rms_nm <= 0.0f ||
			 residual_rms_nm <= state->cfg.max_residual_rms_nm) &&
			(!state->cfg.require_plausible ||
			 (plausibility_ratio >= state->cfg.min_plausibility_ratio &&
			  plausibility_ratio <= state->cfg.max_plausibility_ratio));

	return 0;
}

bool motor_mech_accel_window_velocity_fit(const float32_t *speed_rad_s,
					  const uint32_t *loop_count,
					  uint16_t sample_count,
					  uint16_t sample_index,
					  uint16_t half_window,
					  float32_t control_loop_frequency_hz,
					  float32_t *accel_rad_s2)
{
	if (speed_rad_s == NULL ||
	    loop_count == NULL ||
	    accel_rad_s2 == NULL ||
	    half_window == 0U ||
	    sample_count == 0U ||
	    sample_index < half_window ||
	    (uint32_t)sample_index + (uint32_t)half_window >= sample_count ||
	    !isfinite(control_loop_frequency_hz) ||
	    control_loop_frequency_hz <= 0.0f) {
		return false;
	}

	const uint16_t first = sample_index - half_window;
	const uint16_t last = sample_index + half_window;
	const float32_t inv_fs = 1.0f / control_loop_frequency_hz;
	float32_t sum_t = 0.0f;
	float32_t sum_w = 0.0f;
	float32_t sum_tt = 0.0f;
	float32_t sum_tw = 0.0f;
	uint16_t n = 0U;
	const uint32_t t0_count = loop_count[sample_index];

	for (uint16_t i = first; i <= last; i++) {
		if (!isfinite(speed_rad_s[i])) {
			return false;
		}
		const int32_t dt_count = (int32_t)(loop_count[i] - t0_count);
		const float32_t t = (float32_t)dt_count * inv_fs;
		sum_t += t;
		sum_w += speed_rad_s[i];
		sum_tt += t * t;
		sum_tw += t * speed_rad_s[i];
		n++;
	}

	const float32_t nf = (float32_t)n;
	const float32_t denom = nf * sum_tt - sum_t * sum_t;
	if (fabsf(denom) < MOTOR_MECH_ID_VAR_EPS) {
		return false;
	}

	const float32_t slope = (nf * sum_tw - sum_t * sum_w) / denom;
	if (!isfinite(slope)) {
		return false;
	}

	*accel_rad_s2 = slope;
	return true;
}
