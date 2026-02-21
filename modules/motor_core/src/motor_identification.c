/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_identification.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/sys/util.h>

#define MOTOR_IDENT_VAR_EPS 1.0e-8f
#define MOTOR_IDENT_PIVOT_EPS 1.0e-9f

static bool motor_identification_solve_4x4(float32_t A[4][4], float32_t b[4], float32_t x[4])
{
	float32_t aug[4][5];

	for (uint32_t i = 0U; i < 4U; i++) {
		for (uint32_t j = 0U; j < 4U; j++) {
			aug[i][j] = A[i][j];
		}
		aug[i][4] = b[i];
	}

	for (uint32_t col = 0U; col < 4U; col++) {
		uint32_t pivot = col;
		float32_t pivot_abs = fabsf(aug[pivot][col]);

		for (uint32_t row = col + 1U; row < 4U; row++) {
			const float32_t a = fabsf(aug[row][col]);
			if (a > pivot_abs) {
				pivot = row;
				pivot_abs = a;
			}
		}

		if (pivot_abs < MOTOR_IDENT_PIVOT_EPS) {
			return false;
		}

		if (pivot != col) {
			for (uint32_t j = col; j < 5U; j++) {
				const float32_t tmp = aug[col][j];
				aug[col][j] = aug[pivot][j];
				aug[pivot][j] = tmp;
			}
		}

		const float32_t inv_pivot = 1.0f / aug[col][col];
		for (uint32_t j = col; j < 5U; j++) {
			aug[col][j] *= inv_pivot;
		}

		for (uint32_t row = 0U; row < 4U; row++) {
			if (row == col) {
				continue;
			}

			const float32_t f = aug[row][col];
			if (f == 0.0f) {
				continue;
			}

			for (uint32_t j = col; j < 5U; j++) {
				aug[row][j] -= f * aug[col][j];
			}
		}
	}

	for (uint32_t i = 0U; i < 4U; i++) {
		x[i] = aug[i][4];
	}

	return true;
}

void motor_flux_id_init(struct motor_flux_id_state *state,
			const struct motor_flux_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (cfg != NULL) {
		state->cfg = *cfg;
	}

	state->min_speed_rad_s = INFINITY;
	state->max_speed_rad_s = -INFINITY;
}

bool motor_flux_id_accumulate(struct motor_flux_id_state *state, float32_t elec_speed_rad_s, float32_t id_a,
			      float32_t iq_a, float32_t diq_dt_a_s, float32_t vq_v)
{
	if (state == NULL) {
		return false;
	}

	if (!isfinite(elec_speed_rad_s) || !isfinite(id_a) || !isfinite(iq_a) ||
	    !isfinite(diq_dt_a_s) || !isfinite(vq_v) || !isfinite(state->cfg.rs_ohm) ||
	    !isfinite(state->cfg.ld_h) || !isfinite(state->cfg.lq_h)) {
		return false;
	}

	const float32_t min_abs_speed = fabsf(state->cfg.min_abs_speed_rad_s);
	if (fabsf(elec_speed_rad_s) < min_abs_speed) {
		return false;
	}

	const float32_t y = vq_v - state->cfg.rs_ohm * iq_a - state->cfg.lq_h * diq_dt_a_s -
			    elec_speed_rad_s * state->cfg.ld_h * id_a;
	if (!isfinite(y)) {
		return false;
	}

	state->sample_count++;
	if (state->sample_count == 1U) {
		state->min_speed_rad_s = elec_speed_rad_s;
		state->max_speed_rad_s = elec_speed_rad_s;
	} else {
		state->min_speed_rad_s = MIN(state->min_speed_rad_s, elec_speed_rad_s);
		state->max_speed_rad_s = MAX(state->max_speed_rad_s, elec_speed_rad_s);
	}

	state->sx += elec_speed_rad_s;
	state->sy += y;
	state->sxx += elec_speed_rad_s * elec_speed_rad_s;
	state->sxy += elec_speed_rad_s * y;
	state->syy += y * y;

	return true;
}

int motor_flux_id_finalize(const struct motor_flux_id_state *state, struct motor_flux_id_result *result)
{
	if (state == NULL || result == NULL) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->sample_count = (uint16_t)MIN(state->sample_count, UINT16_MAX);

	const uint32_t min_samples = (state->cfg.min_samples > 0U) ? state->cfg.min_samples : 1U;
	if (state->sample_count < min_samples) {
		return -ENODATA;
	}

	if (!isfinite(state->min_speed_rad_s) || !isfinite(state->max_speed_rad_s)) {
		return -ENODATA;
	}

	const float32_t speed_span = state->max_speed_rad_s - state->min_speed_rad_s;
	if (speed_span < fmaxf(0.0f, state->cfg.min_speed_span_rad_s)) {
		return -ENODATA;
	}

	const float32_t n = (float32_t)state->sample_count;
	const float32_t den = n * state->sxx - state->sx * state->sx;
	if (fabsf(den) < MOTOR_IDENT_VAR_EPS) {
		return -ERANGE;
	}

	const float32_t psi_f = (n * state->sxy - state->sx * state->sy) / den;
	const float32_t bias = (state->sy - psi_f * state->sx) / n;
	if (!isfinite(psi_f) || !isfinite(bias)) {
		return -ERANGE;
	}

	const float32_t sst = state->syy - (state->sy * state->sy) / n;
	float32_t sse = state->syy - (bias * state->sy + psi_f * state->sxy);
	if (sse < 0.0f) {
		sse = 0.0f;
	}

	const float32_t residual_rms_v = sqrtf(sse / n);
	const float32_t r2 = (sst > MOTOR_IDENT_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;

	result->psi_f_wb = psi_f;
	result->bias_v = bias;
	result->residual_rms_v = residual_rms_v;
	result->r2 = r2;
	result->valid = isfinite(psi_f) && isfinite(bias) && isfinite(residual_rms_v) &&
			isfinite(r2) &&
			(!state->cfg.require_positive_psi || (psi_f > 0.0f)) &&
			(r2 >= state->cfg.min_r2);

	return 0;
}

void motor_mech_id_init(struct motor_mech_id_state *state,
			const struct motor_mech_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (cfg != NULL) {
		state->cfg = *cfg;
	}
}

bool motor_mech_id_accumulate(struct motor_mech_id_state *state, float32_t mech_speed_rad_s,
			      float32_t mech_accel_rad_s2, float32_t iq_a)
{
	if (state == NULL) {
		return false;
	}

	if (!isfinite(mech_speed_rad_s) || !isfinite(mech_accel_rad_s2) || !isfinite(iq_a) ||
	    !isfinite(state->cfg.kt_nm_per_a) || fabsf(state->cfg.kt_nm_per_a) < MOTOR_IDENT_PIVOT_EPS) {
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

	if (!motor_identification_solve_4x4(A, b, theta)) {
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
	const float32_t r2 = (sst > MOTOR_IDENT_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;

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
