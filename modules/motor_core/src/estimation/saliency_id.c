/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/estimation/saliency_id.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "motor/math/math_constants.h"

#define MOTOR_SALIENCY_ID_EPS 1.0e-12f

static bool positive_finite(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static float32_t confidence_from_ratio(float32_t ratio, float32_t max_ratio)
{
	if (!isfinite(ratio) || !positive_finite(max_ratio)) {
		return 0.0f;
	}

	return clampf(1.0f - ratio / max_ratio, 0.0f, 1.0f);
}

static bool inductance_within_limits(float32_t value,
				     const struct motor_saliency_id_config *cfg)
{
	return positive_finite(value) &&
	       value >= cfg->min_inductance_h &&
	       value <= cfg->max_inductance_h;
}

static int solve_linear(float32_t a[7][8], uint32_t n, float32_t x[7])
{
	if (n == 0U || n > 7U) {
		return -EINVAL;
	}

	for (uint32_t col = 0U; col < n; ++col) {
		uint32_t pivot = col;
		float32_t pivot_abs = fabsf(a[col][col]);

		for (uint32_t row = col + 1U; row < n; ++row) {
			float32_t candidate_abs = fabsf(a[row][col]);

			if (candidate_abs > pivot_abs) {
				pivot = row;
				pivot_abs = candidate_abs;
			}
		}
		if (pivot_abs <= MOTOR_SALIENCY_ID_EPS) {
			return -ERANGE;
		}
		if (pivot != col) {
			for (uint32_t k = col; k <= n; ++k) {
				float32_t tmp = a[col][k];

				a[col][k] = a[pivot][k];
				a[pivot][k] = tmp;
			}
		}

		float32_t inv_pivot = 1.0f / a[col][col];

		for (uint32_t k = col; k <= n; ++k) {
			a[col][k] *= inv_pivot;
		}
		for (uint32_t row = 0U; row < n; ++row) {
			if (row == col) {
				continue;
			}
			float32_t f = a[row][col];

			for (uint32_t k = col; k <= n; ++k) {
				a[row][k] -= f * a[col][k];
			}
		}
	}

	for (uint32_t i = 0U; i < n; ++i) {
		x[i] = a[i][n];
	}
	return 0;
}

int motor_saliency_id_validate_config(const struct motor_saliency_id_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (cfg->min_samples < 3U ||
	    !positive_finite(cfg->scale_factor) ||
	    !positive_finite(cfg->min_inductance_h) ||
	    !positive_finite(cfg->max_inductance_h) ||
	    !positive_finite(cfg->max_residual_ratio) ||
	    !positive_finite(cfg->max_saliency_ratio) ||
	    cfg->min_inductance_h > cfg->max_inductance_h ||
	    !isfinite(cfg->min_confidence) ||
	    cfg->min_confidence < 0.0f ||
	    cfg->min_confidence > 1.0f) {
		return -EINVAL;
	}

	return 0;
}

void motor_saliency_id_init(struct motor_saliency_id_state *state,
			    const struct motor_saliency_id_config *cfg)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	if (motor_saliency_id_validate_config(cfg) == 0) {
		state->cfg = *cfg;
		state->config_valid = true;
	}
}

bool motor_saliency_id_add(struct motor_saliency_id_state *state,
			   float32_t theta_elec_rad,
			   float32_t inv_l_h_inv)
{
	if (state == NULL || !state->config_valid) {
		return false;
	}
	if (!isfinite(theta_elec_rad) || !positive_finite(inv_l_h_inv)) {
		state->rejected_samples++;
		return false;
	}

	float32_t c = cosf(2.0f * theta_elec_rad);
	float32_t s = sinf(2.0f * theta_elec_rad);
	float32_t c1 = cosf(theta_elec_rad);
	float32_t s1 = sinf(theta_elec_rad);
	float32_t c4 = cosf(4.0f * theta_elec_rad);
	float32_t s4 = sinf(4.0f * theta_elec_rad);
	float32_t basis[7] = { 1.0f };
	uint32_t n = 1U;

	if (state->cfg.fit_first_harmonic) {
		basis[n++] = c1;
		basis[n++] = s1;
	}
	basis[n++] = c;
	basis[n++] = s;
	if (state->cfg.fit_fourth_harmonic) {
		basis[n++] = c4;
		basis[n++] = s4;
	}

	for (uint32_t row = 0U; row < n; ++row) {
		state->rhs[row] += inv_l_h_inv * basis[row];
		for (uint32_t col = 0U; col < n; ++col) {
			state->gram[row][col] += basis[row] * basis[col];
		}
	}
	state->sum_yy += inv_l_h_inv * inv_l_h_inv;
	state->sample_count++;
	return true;
}

int motor_saliency_id_finalize(const struct motor_saliency_id_state *state,
			       struct motor_saliency_id_result *result)
{
	if (state == NULL || result == NULL) {
		return -EINVAL;
	}
	if (!state->config_valid) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->sample_count = state->sample_count;
	result->rejected_samples = state->rejected_samples;
	if (state->sample_count < state->cfg.min_samples) {
		return -EAGAIN;
	}

	uint32_t fit_dim = 1U + 2U +
			   (state->cfg.fit_first_harmonic ? 2U : 0U) +
			   (state->cfg.fit_fourth_harmonic ? 2U : 0U);
	uint32_t idx = 1U;
	uint32_t idx1 = 0U;
	uint32_t idx2;
	uint32_t idx4 = 0U;
	float32_t a[7][8] = {0};
	float32_t theta[7] = {0};

	for (uint32_t row = 0U; row < fit_dim; ++row) {
		for (uint32_t col = 0U; col < fit_dim; ++col) {
			a[row][col] = state->gram[row][col];
		}
		a[row][fit_dim] = state->rhs[row];
	}

	int ret = solve_linear(a, fit_dim, theta);

	if (ret != 0) {
		return ret;
	}

	if (state->cfg.fit_first_harmonic) {
		idx1 = idx;
		idx += 2U;
	}
	idx2 = idx;
	idx += 2U;
	if (state->cfg.fit_fourth_harmonic) {
		idx4 = idx;
	}

	const float32_t offset = theta[0];
	const float32_t cos1 = state->cfg.fit_first_harmonic ? theta[idx1] : 0.0f;
	const float32_t sin1 = state->cfg.fit_first_harmonic ? theta[idx1 + 1U] : 0.0f;
	const float32_t cos2 = theta[idx2];
	const float32_t sin2 = theta[idx2 + 1U];
	const float32_t cos4 = state->cfg.fit_fourth_harmonic ? theta[idx4] : 0.0f;
	const float32_t sin4 = state->cfg.fit_fourth_harmonic ? theta[idx4 + 1U] : 0.0f;
	const float32_t amplitude1 = sqrtf(cos1 * cos1 + sin1 * sin1);
	const float32_t amplitude = sqrtf(cos2 * cos2 + sin2 * sin2);
	const float32_t amplitude4 = sqrtf(cos4 * cos4 + sin4 * sin4);
	const float32_t inv_ld = offset + amplitude;
	const float32_t inv_lq = offset - amplitude;

	result->inv_l_offset = offset;
	result->inv_l_cos1 = cos1;
	result->inv_l_sin1 = sin1;
	result->inv_l_amplitude1 = amplitude1;
	result->inv_l_cos2 = cos2;
	result->inv_l_sin2 = sin2;
	result->inv_l_amplitude = amplitude;
	result->inv_l_cos4 = cos4;
	result->inv_l_sin4 = sin4;
	result->inv_l_amplitude4 = amplitude4;
	result->phase_rad = 0.5f * atan2f(sin2, cos2);
	result->flags = MOTOR_SALIENCY_ID_FLAG_PHASE_VALID;

	if (!positive_finite(offset) || !positive_finite(inv_ld) ||
	    !positive_finite(inv_lq)) {
		return -ERANGE;
	}
	result->flags |= MOTOR_SALIENCY_ID_FLAG_OFFSET_VALID |
			 MOTOR_SALIENCY_ID_FLAG_AMPLITUDE_VALID;

	const float32_t scale = state->cfg.scale_factor;
	result->ld_h = scale / inv_ld;
	result->lq_h = scale / inv_lq;
	result->l_avg_h = 0.5f * (result->ld_h + result->lq_h);
	result->lq_minus_ld_h = result->lq_h - result->ld_h;
	result->saliency_ratio = positive_finite(result->l_avg_h) ?
		fabsf(result->lq_minus_ld_h) / result->l_avg_h : INFINITY;

	if (inductance_within_limits(result->ld_h, &state->cfg)) {
		result->flags |= MOTOR_SALIENCY_ID_FLAG_LD_VALID;
	}
	if (inductance_within_limits(result->lq_h, &state->cfg)) {
		result->flags |= MOTOR_SALIENCY_ID_FLAG_LQ_VALID;
	}
	if (inductance_within_limits(result->l_avg_h, &state->cfg)) {
		result->flags |= MOTOR_SALIENCY_ID_FLAG_LAVG_VALID;
	}

	float32_t theta_dot_rhs = 0.0f;

	for (uint32_t i = 0U; i < fit_dim; ++i) {
		theta_dot_rhs += theta[i] * state->rhs[i];
	}
	float32_t sse = state->sum_yy - theta_dot_rhs;
	sse = fmaxf(sse, 0.0f);
	result->residual_rms = sqrtf(sse / (float32_t)state->sample_count);
	float32_t signal_rms = sqrtf(fmaxf(state->sum_yy /
					   (float32_t)state->sample_count,
					   MOTOR_SALIENCY_ID_EPS));
	result->residual_ratio = result->residual_rms / signal_rms;
	result->confidence = confidence_from_ratio(result->residual_ratio,
						  state->cfg.max_residual_ratio);
	bool confidence_ok = state->cfg.min_confidence <= 0.0f ||
			     result->confidence >= state->cfg.min_confidence;

	result->valid = (result->flags & MOTOR_SALIENCY_ID_FLAG_LD_VALID) != 0U &&
			(result->flags & MOTOR_SALIENCY_ID_FLAG_LQ_VALID) != 0U &&
			(result->flags & MOTOR_SALIENCY_ID_FLAG_LAVG_VALID) != 0U &&
			result->residual_ratio <= state->cfg.max_residual_ratio &&
			result->saliency_ratio <= state->cfg.max_saliency_ratio &&
			confidence_ok;

	return result->valid ? 0 : -ERANGE;
}

int motor_saliency_id_finalize_bins(const struct motor_saliency_id_bin *bins,
				    uint32_t bin_count,
				    uint32_t rejected_samples,
				    const struct motor_saliency_id_config *cfg,
				    struct motor_saliency_id_result *result)
{
	if (bins == NULL || cfg == NULL || result == NULL) {
		return -EINVAL;
	}
	if (motor_saliency_id_validate_config(cfg) != 0) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->rejected_samples = rejected_samples;

	uint32_t used_bins = 0U;
	uint32_t total_samples = 0U;
	float32_t sum_y = 0.0f;
	float32_t sum_y2 = 0.0f;
	float32_t mean_sse = 0.0f;
	float32_t cos1 = 0.0f;
	float32_t sin1 = 0.0f;
	float32_t cos2 = 0.0f;
	float32_t sin2 = 0.0f;
	float32_t cos4 = 0.0f;
	float32_t sin4 = 0.0f;

	for (uint32_t i = 0U; i < bin_count; ++i) {
		if (bins[i].samples == 0U) {
			continue;
		}
		if (!isfinite(bins[i].theta_elec_rad) ||
		    !isfinite(bins[i].sum_inv_l) ||
		    !isfinite(bins[i].sum_inv_l2)) {
			return -ERANGE;
		}

		float32_t inv_n = 1.0f / (float32_t)bins[i].samples;
		float32_t mean = bins[i].sum_inv_l * inv_n;
		float32_t bin_sse = bins[i].sum_inv_l2 -
				     bins[i].sum_inv_l * bins[i].sum_inv_l * inv_n;

		if (!positive_finite(mean)) {
			return -ERANGE;
		}

		/* The saliency sweep uses repeated pulses at each vector to estimate
		 * the vector mean. Qualify the uncertainty of that mean, not the raw
		 * pulse-to-pulse spread.
		 */
		mean_sse += fmaxf(bin_sse, 0.0f) * inv_n;
		sum_y += mean;
		sum_y2 += mean * mean;
		total_samples += bins[i].samples;
		used_bins++;

		float32_t theta = bins[i].theta_elec_rad;

		if (cfg->fit_first_harmonic) {
			cos1 += mean * cosf(theta);
			sin1 += mean * sinf(theta);
		}
		cos2 += mean * cosf(2.0f * theta);
		sin2 += mean * sinf(2.0f * theta);
		if (cfg->fit_fourth_harmonic) {
			cos4 += mean * cosf(4.0f * theta);
			sin4 += mean * sinf(4.0f * theta);
		}
	}

	result->sample_count = total_samples;
	if (total_samples < cfg->min_samples || used_bins < 6U) {
		return -EAGAIN;
	}

	float32_t inv_bins = 1.0f / (float32_t)used_bins;
	float32_t offset = sum_y * inv_bins;
	float32_t harmonic_scale = 2.0f * inv_bins;

	cos1 *= harmonic_scale;
	sin1 *= harmonic_scale;
	cos2 *= harmonic_scale;
	sin2 *= harmonic_scale;
	cos4 *= harmonic_scale;
	sin4 *= harmonic_scale;

	float32_t amplitude1 = sqrtf(cos1 * cos1 + sin1 * sin1);
	float32_t amplitude = sqrtf(cos2 * cos2 + sin2 * sin2);
	float32_t amplitude4 = sqrtf(cos4 * cos4 + sin4 * sin4);
	float32_t inv_ld = offset + amplitude;
	float32_t inv_lq = offset - amplitude;

	result->inv_l_offset = offset;
	result->inv_l_cos1 = cos1;
	result->inv_l_sin1 = sin1;
	result->inv_l_amplitude1 = amplitude1;
	result->inv_l_cos2 = cos2;
	result->inv_l_sin2 = sin2;
	result->inv_l_amplitude = amplitude;
	result->inv_l_cos4 = cos4;
	result->inv_l_sin4 = sin4;
	result->inv_l_amplitude4 = amplitude4;
	result->phase_rad = 0.5f * atan2f(sin2, cos2);
	result->flags = MOTOR_SALIENCY_ID_FLAG_PHASE_VALID;

	if (!positive_finite(offset) || !positive_finite(inv_ld) ||
	    !positive_finite(inv_lq)) {
		return -ERANGE;
	}
	result->flags |= MOTOR_SALIENCY_ID_FLAG_OFFSET_VALID |
			 MOTOR_SALIENCY_ID_FLAG_AMPLITUDE_VALID;

	float32_t scale = cfg->scale_factor;

	result->ld_h = scale / inv_ld;
	result->lq_h = scale / inv_lq;
	result->l_avg_h = 0.5f * (result->ld_h + result->lq_h);
	result->lq_minus_ld_h = result->lq_h - result->ld_h;
	result->saliency_ratio = positive_finite(result->l_avg_h) ?
		fabsf(result->lq_minus_ld_h) / result->l_avg_h : INFINITY;

	if (inductance_within_limits(result->ld_h, cfg)) {
		result->flags |= MOTOR_SALIENCY_ID_FLAG_LD_VALID;
	}
	if (inductance_within_limits(result->lq_h, cfg)) {
		result->flags |= MOTOR_SALIENCY_ID_FLAG_LQ_VALID;
	}
	if (inductance_within_limits(result->l_avg_h, cfg)) {
		result->flags |= MOTOR_SALIENCY_ID_FLAG_LAVG_VALID;
	}

	float32_t signal_rms = sqrtf(fmaxf(sum_y2 * inv_bins, MOTOR_SALIENCY_ID_EPS));

	result->residual_rms = sqrtf(mean_sse * inv_bins);
	result->residual_ratio = result->residual_rms / signal_rms;
	result->confidence = confidence_from_ratio(result->residual_ratio,
						  cfg->max_residual_ratio);
	bool confidence_ok = cfg->min_confidence <= 0.0f ||
			     result->confidence >= cfg->min_confidence;

	result->valid = (result->flags & MOTOR_SALIENCY_ID_FLAG_LD_VALID) != 0U &&
			(result->flags & MOTOR_SALIENCY_ID_FLAG_LQ_VALID) != 0U &&
			(result->flags & MOTOR_SALIENCY_ID_FLAG_LAVG_VALID) != 0U &&
			result->residual_ratio <= cfg->max_residual_ratio &&
			result->saliency_ratio <= cfg->max_saliency_ratio &&
			confidence_ok;

	return result->valid ? 0 : -ERANGE;
}
