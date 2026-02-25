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

	const uint32_t min_samples = (state->cfg.min_samples > 0U) ? state->cfg.min_samples : 1U;
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
	const float32_t theta_dot_b = theta[0] * state->b[0] + theta[1] * state->b[1] +
				      theta[2] * state->b[2] + theta[3] * state->b[3];
	float32_t sse = state->sum_z2 - theta_dot_b;
	if (sse < 0.0f) {
		sse = 0.0f;
	}

	const float32_t mean_z = state->sum_z / n;
	const float32_t sst = state->sum_z2 - n * mean_z * mean_z;
	const float32_t residual_rms_nm = sqrtf(sse / n);
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
