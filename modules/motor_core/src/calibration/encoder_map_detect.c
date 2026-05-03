/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/calibration/encoder_map_detect.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"

#define MOTOR_ENCODER_MAP_MIN_SAMPLES 4U

static bool motor_encoder_map_config_valid(const struct motor_encoder_map_detect_config *cfg)
{
	return cfg != NULL &&
	       isfinite(cfg->pole_pairs) &&
	       isfinite(cfg->min_mech_motion_rad) &&
	       isfinite(cfg->max_offset_residual_rad) &&
	       isfinite(cfg->max_direction_residual_rad) &&
	       isfinite(cfg->min_direction_correlation) &&
	       cfg->pole_pairs > 0.0f &&
	       cfg->min_mech_motion_rad >= 0.0f &&
	       cfg->max_offset_residual_rad > 0.0f &&
	       cfg->max_direction_residual_rad > 0.0f &&
	       cfg->min_direction_correlation >= 0.0f &&
	       cfg->min_direction_correlation <= 1.0f;
}

int motor_encoder_map_detect_compute(const struct motor_encoder_map_detect_config *cfg,
				     const struct motor_encoder_map_detect_sample *samples,
				     uint32_t sample_count,
				     struct motor_encoder_map_detect_result *out)
{
	if (!motor_encoder_map_config_valid(cfg) || samples == NULL || out == NULL) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	if (sample_count < MOTOR_ENCODER_MAP_MIN_SAMPLES) {
		out->rejected_samples = sample_count;
		return -ENODATA;
	}

	bool have_prev = false;
	float32_t prev_gen_mech_wrapped = 0.0f;
	float32_t prev_enc_wrapped = 0.0f;
	float32_t gen_mech_unwrapped = 0.0f;
	float32_t enc_unwrapped = 0.0f;
	float32_t first_enc_unwrapped = 0.0f;
	float32_t last_enc_unwrapped = 0.0f;
	float32_t first_gen_mech_unwrapped = 0.0f;
	float32_t last_gen_mech_unwrapped = 0.0f;
	float32_t corr_num = 0.0f;
	float32_t corr_gen_sq = 0.0f;
	float32_t corr_enc_sq = 0.0f;
	uint32_t accepted = 0U;
	uint32_t deltas = 0U;

	for (uint32_t i = 0U; i < sample_count; i++) {
		const struct motor_encoder_map_detect_sample *sample = &samples[i];
		if ((sample->flags & MOTOR_ENCODER_MAP_SAMPLE_WARNING) != 0U) {
			out->encoder_warning_count++;
		}
		if ((sample->flags & MOTOR_ENCODER_MAP_SAMPLE_ERROR) != 0U ||
		    !isfinite(sample->generated_mech_rad) ||
		    !isfinite(sample->generated_elec_rad) ||
		    !isfinite(sample->encoder_mech_rad)) {
			out->rejected_samples++;
			if ((sample->flags & MOTOR_ENCODER_MAP_SAMPLE_ERROR) != 0U) {
				out->encoder_error_count++;
			}
			continue;
		}

		float32_t gen_mech_wrapped = wrap_rad_2pi(sample->generated_mech_rad);
		float32_t enc_wrapped = wrap_rad_2pi(sample->encoder_mech_rad);
		if (!have_prev) {
			have_prev = true;
			prev_gen_mech_wrapped = gen_mech_wrapped;
			prev_enc_wrapped = enc_wrapped;
			gen_mech_unwrapped = gen_mech_wrapped;
			enc_unwrapped = enc_wrapped;
			first_gen_mech_unwrapped = gen_mech_unwrapped;
			first_enc_unwrapped = enc_unwrapped;
			last_gen_mech_unwrapped = gen_mech_unwrapped;
			last_enc_unwrapped = enc_unwrapped;
			accepted++;
			continue;
		}

		float32_t dgen = wrap_rad_pi(gen_mech_wrapped - prev_gen_mech_wrapped);
		float32_t denc = wrap_rad_pi(enc_wrapped - prev_enc_wrapped);
		gen_mech_unwrapped += dgen;
		enc_unwrapped += denc;
		corr_num += dgen * denc;
		corr_gen_sq += dgen * dgen;
		corr_enc_sq += denc * denc;
		deltas++;

		prev_gen_mech_wrapped = gen_mech_wrapped;
		prev_enc_wrapped = enc_wrapped;
		last_gen_mech_unwrapped = gen_mech_unwrapped;
		last_enc_unwrapped = enc_unwrapped;
		accepted++;
	}

	out->sample_count = accepted;
	if (accepted < MOTOR_ENCODER_MAP_MIN_SAMPLES || deltas == 0U) {
		return -ENODATA;
	}

	out->mech_motion_rad = fabsf(last_enc_unwrapped - first_enc_unwrapped);
	float32_t gen_motion_rad = fabsf(last_gen_mech_unwrapped - first_gen_mech_unwrapped);
	if (out->mech_motion_rad < cfg->min_mech_motion_rad ||
	    gen_motion_rad < cfg->min_mech_motion_rad) {
		return -ERANGE;
	}

	float32_t corr_den = sqrtf(corr_gen_sq * corr_enc_sq);
	if (corr_den <= 1.0e-9f) {
		return -ERANGE;
	}

	out->direction_corr = corr_num / corr_den;
	out->direction_sign = (out->direction_corr >= 0.0f) ? 1 : -1;
	out->direction_valid = fabsf(out->direction_corr) >= cfg->min_direction_correlation;

	float32_t total_enc_delta = last_enc_unwrapped - first_enc_unwrapped;
	if (fabsf(total_enc_delta) > 1.0e-6f) {
		float32_t gen_elec_delta =
			(last_gen_mech_unwrapped - first_gen_mech_unwrapped) * cfg->pole_pairs;
		out->ratio = fabsf(gen_elec_delta / total_enc_delta);
	} else {
		out->ratio = 0.0f;
	}

	float32_t direction_sse = 0.0f;
	float32_t offset_sum_sin = 0.0f;
	float32_t offset_sum_cos = 0.0f;
	have_prev = false;
	prev_gen_mech_wrapped = 0.0f;
	prev_enc_wrapped = 0.0f;

	for (uint32_t i = 0U; i < sample_count; i++) {
		const struct motor_encoder_map_detect_sample *sample = &samples[i];
		if ((sample->flags & MOTOR_ENCODER_MAP_SAMPLE_ERROR) != 0U ||
		    !isfinite(sample->generated_mech_rad) ||
		    !isfinite(sample->generated_elec_rad) ||
		    !isfinite(sample->encoder_mech_rad)) {
			continue;
		}

		float32_t gen_mech_wrapped = wrap_rad_2pi(sample->generated_mech_rad);
		float32_t gen_elec_wrapped = wrap_rad_2pi(sample->generated_elec_rad);
		float32_t enc_wrapped = wrap_rad_2pi(sample->encoder_mech_rad);
		float32_t offset_sample =
			wrap_rad_pi(gen_elec_wrapped -
				    ((float32_t)out->direction_sign * cfg->pole_pairs * enc_wrapped));
		offset_sum_sin += sinf(offset_sample);
		offset_sum_cos += cosf(offset_sample);

		if (have_prev) {
			float32_t dgen = wrap_rad_pi(gen_mech_wrapped - prev_gen_mech_wrapped);
			float32_t denc = wrap_rad_pi(enc_wrapped - prev_enc_wrapped);
			float32_t direction_err = dgen - ((float32_t)out->direction_sign * denc);
			direction_sse += direction_err * direction_err;
		}
		have_prev = true;
		prev_gen_mech_wrapped = gen_mech_wrapped;
		prev_enc_wrapped = enc_wrapped;
	}

	if ((fabsf(offset_sum_sin) < 1.0e-6f) && (fabsf(offset_sum_cos) < 1.0e-6f)) {
		return -ERANGE;
	}

	out->offset_elec_rad = wrap_rad_pi(atan2f(offset_sum_sin, offset_sum_cos));
	out->offset_mech_rad = wrap_rad_pi(out->offset_elec_rad / cfg->pole_pairs);
	out->direction_residual_rad = sqrtf(direction_sse / (float32_t)deltas);
	out->ratio_valid = !cfg->estimate_ratio || isfinite(out->ratio);

	float32_t offset_sse = 0.0f;
	for (uint32_t i = 0U; i < sample_count; i++) {
		const struct motor_encoder_map_detect_sample *sample = &samples[i];
		if ((sample->flags & MOTOR_ENCODER_MAP_SAMPLE_ERROR) != 0U ||
		    !isfinite(sample->generated_mech_rad) ||
		    !isfinite(sample->generated_elec_rad) ||
		    !isfinite(sample->encoder_mech_rad)) {
			continue;
		}

		float32_t predicted =
			wrap_rad_pi(((float32_t)out->direction_sign * cfg->pole_pairs *
				     wrap_rad_2pi(sample->encoder_mech_rad)) +
				    out->offset_elec_rad);
		float32_t err = wrap_rad_pi(wrap_rad_pi(sample->generated_elec_rad) - predicted);
		offset_sse += err * err;
	}

	out->offset_residual_rad = sqrtf(offset_sse / (float32_t)accepted);
	out->offset_valid = out->offset_residual_rad <= cfg->max_offset_residual_rad;
	out->direction_valid = out->direction_valid &&
			       (out->direction_residual_rad <= cfg->max_direction_residual_rad);
	out->valid = out->direction_valid && out->offset_valid && out->ratio_valid &&
		     out->encoder_error_count == 0U;

	return out->valid ? 0 : -ERANGE;
}
