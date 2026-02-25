/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/estimation/commission_estimators.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/sys/util.h>

#define MOTOR_FLUX_ID_VAR_EPS 1.0e-8f

static bool motor_flux_id_is_finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

int motor_flux_id_validate_config(const struct motor_flux_id_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (!motor_flux_id_is_finite_nonnegative(cfg->rs_ohm) ||
	    !motor_flux_id_is_finite_nonnegative(cfg->ld_h) ||
	    !motor_flux_id_is_finite_nonnegative(cfg->lq_h) ||
	    !motor_flux_id_is_finite_nonnegative(cfg->min_abs_speed_rad_s) ||
	    !motor_flux_id_is_finite_nonnegative(cfg->min_speed_span_rad_s) ||
	    !isfinite(cfg->min_r2) ||
	    cfg->min_samples == 0U) {
		return -EINVAL;
	}

	if (cfg->min_r2 < 0.0f || cfg->min_r2 > 1.0f) {
		return -EINVAL;
	}

	return 0;
}

void motor_flux_id_init(struct motor_flux_id_state *state,
			const struct motor_flux_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (motor_flux_id_validate_config(cfg) == 0) {
		state->cfg = *cfg;
		state->config_valid = true;
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
	if (!state->config_valid) {
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
	if (!state->config_valid) {
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
	if (fabsf(den) < MOTOR_FLUX_ID_VAR_EPS) {
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
	const float32_t r2 = (sst > MOTOR_FLUX_ID_VAR_EPS) ? (1.0f - sse / sst) : 0.0f;

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
