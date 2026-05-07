/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor/estimation/saliency_id.h"
#include "motor/math/math_constants.h"

#define EPS 1.0e-5f

static const struct motor_saliency_id_config default_cfg = {
	.min_samples = 8U,
	.scale_factor = 1.0f,
	.min_inductance_h = 0.0001f,
	.max_inductance_h = 0.0200f,
	.max_residual_ratio = 0.05f,
	.max_saliency_ratio = 0.80f,
	.min_confidence = 0.10f,
};

static float inv_l_sample(float theta, float ld, float lq, float phase)
{
	float inv_offset = 0.5f * ((1.0f / ld) + (1.0f / lq));
	float inv_amp = 0.5f * ((1.0f / ld) - (1.0f / lq));

	return inv_offset + inv_amp * cosf(2.0f * (theta - phase));
}

static float inv_l_sample_with_fourth(float theta, float ld, float lq, float phase,
				      float amp4, float phase4)
{
	return inv_l_sample(theta, ld, lq, phase) +
	       amp4 * cosf(4.0f * (theta - phase4));
}

static void add_uniform_samples(struct motor_saliency_id_state *state,
				float ld, float lq, float phase, uint32_t samples)
{
	for (uint32_t i = 0U; i < samples; ++i) {
		float theta = 2.0f * PI_F32 * (float)i / (float)samples;

		zassert_true(motor_saliency_id_add(state, theta,
						   inv_l_sample(theta, ld, lq, phase)),
			     NULL);
	}
}

static void add_uniform_bins(struct motor_saliency_id_bin *bins, uint32_t bin_count,
			     float ld, float lq, float phase, float extra_amp3,
			     uint32_t repeats)
{
	for (uint32_t i = 0U; i < bin_count; ++i) {
		float theta = 2.0f * PI_F32 * (float)i / (float)bin_count;

		bins[i].theta_elec_rad = theta;
		for (uint32_t repeat = 0U; repeat < repeats; ++repeat) {
			float deterministic_noise =
				0.7f * (float)((int32_t)(repeat % 3U) - 1);
			float y = inv_l_sample(theta, ld, lq, phase) +
				  extra_amp3 * cosf(3.0f * theta + 0.19f) +
				  deterministic_noise;

			bins[i].sum_inv_l += y;
			bins[i].sum_inv_l2 += y * y;
			bins[i].samples++;
		}
	}
}

ZTEST(motor_saliency_id, test_isotropic_motor_has_equal_axes)
{
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;
	const float l = 0.0034f;

	motor_saliency_id_init(&state, &default_cfg);
	add_uniform_samples(&state, l, l, 0.0f, 32U);

	zassert_ok(motor_saliency_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.ld_h - l) < EPS, "Ld %.8f", (double)result.ld_h);
	zassert_true(fabsf(result.lq_h - l) < EPS, "Lq %.8f", (double)result.lq_h);
	zassert_true(result.saliency_ratio < 1.0e-4f, NULL);
}

ZTEST(motor_saliency_id, test_salient_motor_recovers_ld_lq)
{
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;
	const float ld = 0.0028f;
	const float lq = 0.0042f;

	motor_saliency_id_init(&state, &default_cfg);
	add_uniform_samples(&state, ld, lq, 0.0f, 32U);

	zassert_ok(motor_saliency_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.ld_h - ld) < EPS, "Ld %.8f", (double)result.ld_h);
	zassert_true(fabsf(result.lq_h - lq) < EPS, "Lq %.8f", (double)result.lq_h);
	zassert_true(result.lq_minus_ld_h > 0.0f, NULL);
}

ZTEST(motor_saliency_id, test_phase_offset_recovered)
{
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;
	const float phase = 0.37f;

	motor_saliency_id_init(&state, &default_cfg);
	add_uniform_samples(&state, 0.0028f, 0.0042f, phase, 64U);

	zassert_ok(motor_saliency_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.phase_rad - phase) < EPS,
		     "phase %.8f", (double)result.phase_rad);
}

ZTEST(motor_saliency_id, test_fourth_harmonic_fit_preserves_second_harmonic_axes)
{
	struct motor_saliency_id_config cfg = default_cfg;
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;
	const float ld = 0.0030f;
	const float lq = 0.0040f;
	const float phase = 0.21f;
	const float amp4 = 80.0f;
	const float phase4 = -0.13f;
	const uint32_t samples = 64U;

	cfg.fit_fourth_harmonic = true;
	motor_saliency_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < samples; ++i) {
		float theta = 2.0f * PI_F32 * (float)i / (float)samples;

		zassert_true(motor_saliency_id_add(
				     &state, theta,
				     inv_l_sample_with_fourth(theta, ld, lq, phase,
							      amp4, phase4)),
			     NULL);
	}

	zassert_ok(motor_saliency_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.ld_h - ld) < EPS, "Ld %.8f", (double)result.ld_h);
	zassert_true(fabsf(result.lq_h - lq) < EPS, "Lq %.8f", (double)result.lq_h);
	zassert_true(fabsf(result.inv_l_amplitude4 - fabsf(amp4)) < 1.0e-2f,
		     "amp4 %.8f", (double)result.inv_l_amplitude4);
	zassert_true(result.residual_ratio < 1.0e-4f, NULL);
}

ZTEST(motor_saliency_id, test_nuisance_harmonics_do_not_bias_second_harmonic_axes)
{
	struct motor_saliency_id_config cfg = default_cfg;
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;
	const float ld = 0.0030f;
	const float lq = 0.0040f;
	const float phase = -0.14f;
	const float amp1 = 65.0f;
	const float amp4 = 80.0f;
	const uint32_t samples = 96U;

	cfg.fit_first_harmonic = true;
	cfg.fit_fourth_harmonic = true;
	motor_saliency_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < samples; ++i) {
		float theta = 2.0f * PI_F32 * (float)i / (float)samples;
		float y = inv_l_sample_with_fourth(theta, ld, lq, phase, amp4, 0.23f) +
			  amp1 * sinf(theta + 0.31f);

		zassert_true(motor_saliency_id_add(&state, theta, y), NULL);
	}

	zassert_ok(motor_saliency_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.ld_h - ld) < EPS, "Ld %.8f", (double)result.ld_h);
	zassert_true(fabsf(result.lq_h - lq) < EPS, "Lq %.8f", (double)result.lq_h);
	zassert_true(fabsf(result.inv_l_amplitude1 - fabsf(amp1)) < 1.0e-2f,
		     "amp1 %.8f", (double)result.inv_l_amplitude1);
	zassert_true(fabsf(result.inv_l_amplitude4 - fabsf(amp4)) < 1.0e-2f,
		     "amp4 %.8f", (double)result.inv_l_amplitude4);
}

ZTEST(motor_saliency_id, test_bin_finalize_uses_repeatability_not_shape_residual)
{
	struct motor_saliency_id_config cfg = default_cfg;
	struct motor_saliency_id_bin bins[32] = {0};
	struct motor_saliency_id_result result;
	const float ld = 0.0027f;
	const float lq = 0.0035f;
	const float phase = -0.42f;

	cfg.fit_first_harmonic = true;
	cfg.fit_fourth_harmonic = true;
	add_uniform_bins(bins, ARRAY_SIZE(bins), ld, lq, phase, 90.0f, 6U);

	zassert_ok(motor_saliency_id_finalize_bins(bins, ARRAY_SIZE(bins), 0U,
						   &cfg, &result),
		   NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.ld_h - ld) < EPS, "Ld %.8f", (double)result.ld_h);
	zassert_true(fabsf(result.lq_h - lq) < EPS, "Lq %.8f", (double)result.lq_h);
	zassert_true(result.residual_ratio < cfg.max_residual_ratio,
		     "residual %.8f", (double)result.residual_ratio);
}

ZTEST(motor_saliency_id, test_rejects_insufficient_samples)
{
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;

	motor_saliency_id_init(&state, &default_cfg);
	add_uniform_samples(&state, 0.003f, 0.004f, 0.0f, 4U);

	zassert_equal(motor_saliency_id_finalize(&state, &result), -EAGAIN, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_saliency_id, test_rejects_invalid_samples)
{
	struct motor_saliency_id_state state;

	motor_saliency_id_init(&state, &default_cfg);
	zassert_false(motor_saliency_id_add(&state, 0.0f, -1.0f), NULL);
	zassert_false(motor_saliency_id_add(&state, NAN, 1.0f), NULL);
	zassert_equal(state.rejected_samples, 2U, NULL);
}

ZTEST(motor_saliency_id, test_rejects_excessive_residual)
{
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;

	motor_saliency_id_init(&state, &default_cfg);
	add_uniform_samples(&state, 0.003f, 0.004f, 0.0f, 32U);
	zassert_true(motor_saliency_id_add(&state, 0.25f, 5000.0f), NULL);

	zassert_equal(motor_saliency_id_finalize(&state, &result), -ERANGE, NULL);
	zassert_false(result.valid, NULL);
	zassert_true(result.residual_ratio > default_cfg.max_residual_ratio, NULL);
}

ZTEST(motor_saliency_id, test_rejects_impossible_saliency)
{
	struct motor_saliency_id_config cfg = default_cfg;
	struct motor_saliency_id_state state;
	struct motor_saliency_id_result result;

	cfg.max_saliency_ratio = 0.10f;
	motor_saliency_id_init(&state, &cfg);
	add_uniform_samples(&state, 0.0020f, 0.0060f, 0.0f, 32U);

	zassert_equal(motor_saliency_id_finalize(&state, &result), -ERANGE, NULL);
	zassert_false(result.valid, NULL);
	zassert_true(result.saliency_ratio > cfg.max_saliency_ratio, NULL);
}

ZTEST_SUITE(motor_saliency_id, NULL, NULL, NULL, NULL, NULL);
