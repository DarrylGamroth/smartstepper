/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/estimation/electrical_id.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "motor/math/math_constants.h"

#define ELECTRICAL_ID_EPS 1.0e-12f
#define ELECTRICAL_ID_DEFAULT_CONFIDENCE 1.0f

static bool positive_finite(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static float32_t residual_confidence(float32_t residual_ratio,
					     float32_t max_residual_ratio)
{
	if (!isfinite(residual_ratio) || !positive_finite(max_residual_ratio)) {
		return 0.0f;
	}

	return clampf(1.0f - (residual_ratio / max_residual_ratio), 0.0f, 1.0f);
}

static bool rs_within_limits(float32_t rs_ohm,
			     const struct motor_electrical_id_limits *limits)
{
	if (!positive_finite(rs_ohm)) {
		return false;
	}
	if (limits == NULL || !positive_finite(limits->rs_min_ohm) ||
	    !positive_finite(limits->rs_max_ohm)) {
		return true;
	}

	return rs_ohm >= limits->rs_min_ohm && rs_ohm <= limits->rs_max_ohm;
}

static bool l_within_limits(float32_t l_h,
			    const struct motor_electrical_id_limits *limits)
{
	if (!positive_finite(l_h)) {
		return false;
	}
	if (limits == NULL || !positive_finite(limits->l_min_h) ||
	    !positive_finite(limits->l_max_h)) {
		return true;
	}

	return l_h >= limits->l_min_h && l_h <= limits->l_max_h;
}

void motor_electrical_id_rs_reset(struct motor_electrical_id_rs_accum *accum)
{
	if (accum == NULL) {
		return;
	}

	memset(accum, 0, sizeof(*accum));
}

int motor_electrical_id_rs_add(struct motor_electrical_id_rs_accum *accum,
				       const struct motor_electrical_id_rs_config *cfg,
				       float32_t voltage_v,
				       float32_t current_a)
{
	if (accum == NULL || cfg == NULL || !positive_finite(cfg->min_abs_current_a)) {
		return -EINVAL;
	}
	if (!isfinite(voltage_v) || !isfinite(current_a)) {
		return -EINVAL;
	}
	if (fabsf(current_a) < cfg->min_abs_current_a) {
		accum->rejected_low_current++;
		return 1;
	}

	accum->sum_i_a += current_a;
	accum->sum_v_v += voltage_v;
	accum->sum_i2_a2 += current_a * current_a;
	accum->sum_v2_v2 += voltage_v * voltage_v;
	accum->sum_iv_va += current_a * voltage_v;
	accum->samples++;
	return 0;
}

int motor_electrical_id_rs_finalize(const struct motor_electrical_id_rs_accum *accum,
					    const struct motor_electrical_id_rs_config *cfg,
					    const struct motor_electrical_id_limits *limits,
					    struct motor_electrical_id_rs_result *result)
{
	if (accum == NULL || cfg == NULL || result == NULL || cfg->min_samples == 0U ||
	    !positive_finite(cfg->min_abs_current_a) || !positive_finite(cfg->max_residual_ratio)) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->samples = accum->samples;
	result->rejected_low_current = accum->rejected_low_current;
	if (accum->samples < cfg->min_samples || accum->sum_i2_a2 <= ELECTRICAL_ID_EPS) {
		return -EAGAIN;
	}

	float32_t rs_ohm = accum->sum_iv_va / accum->sum_i2_a2;
	float32_t sse = accum->sum_v2_v2 - (2.0f * rs_ohm * accum->sum_iv_va) +
			  (rs_ohm * rs_ohm * accum->sum_i2_a2);
	sse = fmaxf(sse, 0.0f);
	float32_t residual_rms = sqrtf(sse / (float32_t)accum->samples);
	float32_t signal_rms = sqrtf(fmaxf(accum->sum_v2_v2 / (float32_t)accum->samples,
					 ELECTRICAL_ID_EPS));
	float32_t residual_ratio = residual_rms / signal_rms;
	float32_t confidence = residual_confidence(residual_ratio, cfg->max_residual_ratio);

	result->rs_ohm = rs_ohm;
	result->avg_current_a = accum->sum_i_a / (float32_t)accum->samples;
	result->avg_voltage_v = accum->sum_v_v / (float32_t)accum->samples;
	result->residual_rms_v = residual_rms;
	result->residual_ratio = residual_ratio;
	result->confidence = confidence;
	result->valid = rs_within_limits(rs_ohm, limits) &&
			residual_ratio <= cfg->max_residual_ratio &&
			(limits == NULL || limits->min_confidence <= 0.0f ||
			 confidence >= limits->min_confidence);

	return result->valid ? 0 : -ERANGE;
}

void motor_electrical_id_l_reset(struct motor_electrical_id_l_accum *accum)
{
	if (accum == NULL) {
		return;
	}

	memset(accum, 0, sizeof(*accum));
}

int motor_electrical_id_l_add(struct motor_electrical_id_l_accum *accum,
				      const struct motor_electrical_id_l_config *cfg,
				      float32_t voltage_v,
				      float32_t current_a,
				      float32_t previous_current_a,
				      float32_t rs_ohm)
{
	if (accum == NULL || cfg == NULL || !positive_finite(cfg->dt_s) ||
	    !positive_finite(cfg->min_abs_di_dt_a_per_s) || !positive_finite(rs_ohm)) {
		return -EINVAL;
	}
	if (!isfinite(voltage_v) || !isfinite(current_a) || !isfinite(previous_current_a)) {
		return -EINVAL;
	}

	float32_t di_dt = (current_a - previous_current_a) / cfg->dt_s;
	if (fabsf(di_dt) < cfg->min_abs_di_dt_a_per_s) {
		accum->rejected_low_slew++;
		return 1;
	}

	float32_t v_eff = voltage_v - (rs_ohm * current_a);
	accum->sum_x2 += di_dt * di_dt;
	accum->sum_xy += di_dt * v_eff;
	accum->sum_y2 += v_eff * v_eff;
	accum->samples++;
	return 0;
}

int motor_electrical_id_l_add_integral(struct motor_electrical_id_l_accum *accum,
				       const struct motor_electrical_id_l_config *cfg,
				       float32_t flux_linkage_vs,
				       float32_t delta_current_a)
{
	if (accum == NULL || cfg == NULL || !positive_finite(cfg->min_abs_delta_current_a)) {
		return -EINVAL;
	}
	if (!isfinite(flux_linkage_vs) || !isfinite(delta_current_a)) {
		return -EINVAL;
	}
	if (fabsf(delta_current_a) < cfg->min_abs_delta_current_a) {
		accum->rejected_low_slew++;
		return 1;
	}

	accum->sum_x2 += delta_current_a * delta_current_a;
	accum->sum_xy += delta_current_a * flux_linkage_vs;
	accum->sum_y2 += flux_linkage_vs * flux_linkage_vs;
	accum->samples++;
	return 0;
}

int motor_electrical_id_l_finalize(const struct motor_electrical_id_l_accum *accum,
					   const struct motor_electrical_id_l_config *cfg,
					   const struct motor_electrical_id_limits *limits,
					   struct motor_electrical_id_l_result *result)
{
	if (accum == NULL || cfg == NULL || result == NULL || cfg->min_samples == 0U ||
	    !positive_finite(cfg->dt_s) || !positive_finite(cfg->min_abs_di_dt_a_per_s) ||
	    !positive_finite(cfg->max_residual_ratio)) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->samples = accum->samples;
	result->rejected_low_slew = accum->rejected_low_slew;
	if (accum->samples < cfg->min_samples || accum->sum_x2 <= ELECTRICAL_ID_EPS) {
		return -EAGAIN;
	}

	float32_t l_h = accum->sum_xy / accum->sum_x2;
	float32_t sse = accum->sum_y2 - (2.0f * l_h * accum->sum_xy) +
			  (l_h * l_h * accum->sum_x2);
	sse = fmaxf(sse, 0.0f);
	float32_t residual_rms = sqrtf(sse / (float32_t)accum->samples);
	float32_t signal_rms = sqrtf(fmaxf(accum->sum_y2 / (float32_t)accum->samples,
					 ELECTRICAL_ID_EPS));
	float32_t residual_ratio = residual_rms / signal_rms;
	float32_t confidence = residual_confidence(residual_ratio, cfg->max_residual_ratio);

	result->inductance_h = l_h;
	result->residual_rms_v = residual_rms;
	result->residual_ratio = residual_ratio;
	result->confidence = confidence;
	result->valid = l_within_limits(l_h, limits) &&
			residual_ratio <= cfg->max_residual_ratio &&
			(limits == NULL || limits->min_confidence <= 0.0f ||
			 confidence >= limits->min_confidence);

	return result->valid ? 0 : -ERANGE;
}

void motor_electrical_id_demod_reset(struct motor_electrical_id_demod_accum *accum)
{
	if (accum == NULL) {
		return;
	}

	memset(accum, 0, sizeof(*accum));
}

int motor_electrical_id_demod_add(struct motor_electrical_id_demod_accum *accum,
				  const struct motor_electrical_id_demod_config *cfg,
				  float32_t flux_linkage_vs,
				  float32_t delta_current_a)
{
	if (accum == NULL || cfg == NULL || !positive_finite(cfg->min_abs_flux_vs)) {
		return -EINVAL;
	}
	if (!isfinite(flux_linkage_vs) || !isfinite(delta_current_a)) {
		return -EINVAL;
	}
	if (fabsf(flux_linkage_vs) < cfg->min_abs_flux_vs) {
		accum->rejected_low_signal++;
		return 1;
	}

	const float32_t inv_l = delta_current_a / flux_linkage_vs;
	if (!positive_finite(inv_l)) {
		accum->rejected_non_positive++;
		return 1;
	}

	accum->sum_inv_l += inv_l;
	accum->sum_inv_l2 += inv_l * inv_l;
	accum->samples++;
	return 0;
}

int motor_electrical_id_demod_add_pair(struct motor_electrical_id_demod_accum *accum,
				       const struct motor_electrical_id_demod_config *cfg,
				       float32_t positive_flux_vs,
				       float32_t positive_delta_current_a,
				       float32_t negative_flux_vs,
				       float32_t negative_delta_current_a)
{
	if (accum == NULL || cfg == NULL) {
		return -EINVAL;
	}

	/*
	 * Pairing adjacent positive/negative half-cycles cancels current-sense
	 * offset and slow drift before forming one inverse-L sample.
	 */
	return motor_electrical_id_demod_add(accum, cfg,
					     positive_flux_vs - negative_flux_vs,
					     positive_delta_current_a -
					     negative_delta_current_a);
}

int motor_electrical_id_demod_finalize(const struct motor_electrical_id_demod_accum *accum,
				       const struct motor_electrical_id_demod_config *cfg,
				       const struct motor_electrical_id_limits *limits,
				       struct motor_electrical_id_demod_result *result)
{
	if (accum == NULL || cfg == NULL || result == NULL || cfg->min_samples == 0U ||
	    !positive_finite(cfg->min_abs_flux_vs) ||
	    !positive_finite(cfg->max_spread_ratio)) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->samples = accum->samples;
	result->rejected_low_signal = accum->rejected_low_signal;
	result->rejected_non_positive = accum->rejected_non_positive;
	if (accum->samples < cfg->min_samples || accum->sum_inv_l <= ELECTRICAL_ID_EPS) {
		return -EAGAIN;
	}

	const float32_t inv_l_mean = accum->sum_inv_l / (float32_t)accum->samples;
	float32_t inv_l_var = (accum->sum_inv_l2 / (float32_t)accum->samples) -
			      (inv_l_mean * inv_l_mean);
	inv_l_var = fmaxf(inv_l_var, 0.0f);
	const float32_t inv_l_stddev = sqrtf(inv_l_var);
	const float32_t spread_ratio = inv_l_stddev / fmaxf(inv_l_mean, ELECTRICAL_ID_EPS);
	const float32_t scale_factor = positive_finite(cfg->scale_factor) ?
		cfg->scale_factor : 1.0f;
	const float32_t l_h = scale_factor / inv_l_mean;
	const float32_t confidence = residual_confidence(spread_ratio,
							 cfg->max_spread_ratio);

	result->inductance_h = l_h;
	result->inv_l_mean = inv_l_mean;
	result->inv_l_stddev = inv_l_stddev;
	result->spread_ratio = spread_ratio;
	result->confidence = confidence;
	result->valid = l_within_limits(l_h, limits) &&
			spread_ratio <= cfg->max_spread_ratio &&
			(limits == NULL || limits->min_confidence <= 0.0f ||
			 confidence >= limits->min_confidence);

	return result->valid ? 0 : -ERANGE;
}

int motor_electrical_id_combine(const struct motor_electrical_id_rs_result *rs,
					const struct motor_electrical_id_l_result *ld,
					const struct motor_electrical_id_l_result *lq,
					const struct motor_electrical_id_limits *limits,
					struct motor_electrical_id_result *result)
{
	if (rs == NULL || ld == NULL || lq == NULL || result == NULL) {
		return -EINVAL;
	}

	memset(result, 0, sizeof(*result));
	result->rs_ohm = rs->rs_ohm;
	result->ld_h = ld->inductance_h;
	result->lq_h = lq->inductance_h;
	result->l_avg_h = 0.5f * (result->ld_h + result->lq_h);
	result->lq_minus_ld_h = result->lq_h - result->ld_h;
	float32_t max_l = fmaxf(fabsf(result->ld_h), fabsf(result->lq_h));
	result->axis_mismatch_ratio = (max_l > ELECTRICAL_ID_EPS) ?
		fabsf(result->lq_minus_ld_h) / max_l : 0.0f;
	result->confidence = fminf(rs->confidence, fminf(ld->confidence, lq->confidence));
	result->flags = (rs->valid ? MOTOR_ELECTRICAL_ID_FLAG_RS_VALID : 0U) |
			(ld->valid ? MOTOR_ELECTRICAL_ID_FLAG_LD_VALID : 0U) |
			(lq->valid ? MOTOR_ELECTRICAL_ID_FLAG_LQ_VALID : 0U);
	if (l_within_limits(result->l_avg_h, limits)) {
		result->flags |= MOTOR_ELECTRICAL_ID_FLAG_LAVG_VALID;
	}
	if (isfinite(result->lq_minus_ld_h)) {
		result->flags |= MOTOR_ELECTRICAL_ID_FLAG_SALIENCY_VALID;
	}

	bool mismatch_ok = true;
	if (limits != NULL && positive_finite(limits->max_axis_mismatch_ratio)) {
		mismatch_ok = result->axis_mismatch_ratio <= limits->max_axis_mismatch_ratio;
	}
	bool confidence_ok = limits == NULL || limits->min_confidence <= 0.0f ||
			     result->confidence >= limits->min_confidence;
	result->valid = rs->valid && ld->valid && lq->valid && mismatch_ok && confidence_ok;

	return result->valid ? 0 : -ERANGE;
}

int motor_electrical_id_recommend_current_pi(float32_t rs_ohm,
						     float32_t ld_h,
						     float32_t lq_h,
						     float32_t bandwidth_hz,
						     float32_t sample_time_s,
						     struct motor_electrical_id_pi_recommendation *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	if (!positive_finite(rs_ohm) || !positive_finite(ld_h) || !positive_finite(lq_h) ||
	    !positive_finite(bandwidth_hz) || !positive_finite(sample_time_s)) {
		return -EINVAL;
	}

	float32_t bw_rps = 2.0f * PI_F32 * bandwidth_hz;
	out->bandwidth_hz = bandwidth_hz;
	out->sample_time_s = sample_time_s;
	out->kp_d = ld_h * bw_rps;
	out->ki_d = (rs_ohm / ld_h) * sample_time_s;
	out->kp_q = lq_h * bw_rps;
	out->ki_q = (rs_ohm / lq_h) * sample_time_s;
	out->valid = isfinite(out->kp_d) && isfinite(out->ki_d) &&
		     isfinite(out->kp_q) && isfinite(out->ki_q) &&
		     out->kp_d > 0.0f && out->ki_d >= 0.0f &&
		     out->kp_q > 0.0f && out->ki_q >= 0.0f;

	return out->valid ? 0 : -ERANGE;
}
