/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "motor/estimation/electrical_id.h"
#include "motor/math/math_constants.h"

#define EPS 1e-5f

static const struct motor_electrical_id_limits limits = {
	.rs_min_ohm = 0.1f,
	.rs_max_ohm = 20.0f,
	.l_min_h = 1.0e-5f,
	.l_max_h = 0.1f,
	.max_axis_mismatch_ratio = 0.75f,
	.min_confidence = 0.2f,
};

ZTEST(motor_electrical_id, test_rs_fit_uses_all_samples)
{
	const struct motor_electrical_id_rs_config cfg = {
		.min_abs_current_a = 0.01f,
		.min_samples = 4U,
		.max_residual_ratio = 0.05f,
	};
	struct motor_electrical_id_rs_accum acc;
	struct motor_electrical_id_rs_result result;

	motor_electrical_id_rs_reset(&acc);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, 0.44f, 0.20f), NULL);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, -0.66f, -0.30f), NULL);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, 0.88f, 0.40f), NULL);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, -1.10f, -0.50f), NULL);

	zassert_ok(motor_electrical_id_rs_finalize(&acc, &cfg, &limits, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.rs_ohm - 2.2f) < EPS, "Rs %.8f", (double)result.rs_ohm);
	zassert_equal(result.samples, 4U, NULL);
	zassert_true(result.confidence > 0.9f, NULL);
}

ZTEST(motor_electrical_id, test_rs_rejects_low_current_and_insufficient_samples)
{
	const struct motor_electrical_id_rs_config cfg = {
		.min_abs_current_a = 0.1f,
		.min_samples = 2U,
		.max_residual_ratio = 0.05f,
	};
	struct motor_electrical_id_rs_accum acc;
	struct motor_electrical_id_rs_result result;

	motor_electrical_id_rs_reset(&acc);
	zassert_equal(motor_electrical_id_rs_add(&acc, &cfg, 0.01f, 0.001f), 1, NULL);
	zassert_equal(acc.rejected_low_current, 1U, NULL);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, 0.22f, 0.1f), NULL);
	zassert_equal(motor_electrical_id_rs_finalize(&acc, &cfg, &limits, &result), -EAGAIN,
		      NULL);
}

ZTEST(motor_electrical_id, test_rs_rejects_out_of_bounds)
{
	const struct motor_electrical_id_rs_config cfg = {
		.min_abs_current_a = 0.01f,
		.min_samples = 2U,
		.max_residual_ratio = 0.05f,
	};
	struct motor_electrical_id_rs_accum acc;
	struct motor_electrical_id_rs_result result;

	motor_electrical_id_rs_reset(&acc);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, 40.0f, 1.0f), NULL);
	zassert_ok(motor_electrical_id_rs_add(&acc, &cfg, -40.0f, -1.0f), NULL);
	zassert_equal(motor_electrical_id_rs_finalize(&acc, &cfg, &limits, &result), -ERANGE,
		      NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_electrical_id, test_inductance_fit_from_discrete_slew)
{
	const struct motor_electrical_id_l_config cfg = {
		.dt_s = 0.001f,
		.min_abs_di_dt_a_per_s = 1.0f,
		.min_samples = 4U,
		.max_residual_ratio = 0.05f,
	};
	const float rs = 2.0f;
	const float l = 0.003f;
	float prev = 0.0f;
	struct motor_electrical_id_l_accum acc;
	struct motor_electrical_id_l_result result;

	motor_electrical_id_l_reset(&acc);
	for (int i = 0; i < 8; ++i) {
		float cur = (float)(i + 1) * 0.05f;
		float di_dt = (cur - prev) / cfg.dt_s;
		float v = (rs * cur) + (l * di_dt);
		zassert_ok(motor_electrical_id_l_add(&acc, &cfg, v, cur, prev, rs), NULL);
		prev = cur;
	}

	zassert_ok(motor_electrical_id_l_finalize(&acc, &cfg, &limits, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.inductance_h - l) < EPS,
		     "L %.8f", (double)result.inductance_h);
}

ZTEST(motor_electrical_id, test_inductance_rejects_low_slew)
{
	const struct motor_electrical_id_l_config cfg = {
		.dt_s = 0.001f,
		.min_abs_di_dt_a_per_s = 100.0f,
		.min_samples = 1U,
		.max_residual_ratio = 0.05f,
	};
	struct motor_electrical_id_l_accum acc;

	motor_electrical_id_l_reset(&acc);
	zassert_equal(motor_electrical_id_l_add(&acc, &cfg, 0.2f, 0.01f, 0.0f, 2.0f), 1,
		      NULL);
	zassert_equal(acc.rejected_low_slew, 1U, NULL);
}

ZTEST(motor_electrical_id, test_inductance_fit_from_integrated_pulses)
{
	const struct motor_electrical_id_l_config cfg = {
		.dt_s = 0.001f,
		.min_abs_di_dt_a_per_s = 1.0f,
		.min_abs_delta_current_a = 0.005f,
		.min_samples = 4U,
		.max_residual_ratio = 0.05f,
	};
	const float l = 0.0029f;
	struct motor_electrical_id_l_accum acc;
	struct motor_electrical_id_l_result result;

	motor_electrical_id_l_reset(&acc);
	zassert_ok(motor_electrical_id_l_add_integral(&acc, &cfg, l * 0.030f, 0.030f), NULL);
	zassert_ok(motor_electrical_id_l_add_integral(&acc, &cfg, l * -0.040f, -0.040f), NULL);
	zassert_ok(motor_electrical_id_l_add_integral(&acc, &cfg, l * 0.050f, 0.050f), NULL);
	zassert_ok(motor_electrical_id_l_add_integral(&acc, &cfg, l * -0.060f, -0.060f), NULL);

	zassert_ok(motor_electrical_id_l_finalize(&acc, &cfg, &limits, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.inductance_h - l) < EPS,
		     "L %.8f", (double)result.inductance_h);
}

ZTEST(motor_electrical_id, test_demodulated_inductance_averages_inverse_l)
{
	const struct motor_electrical_id_demod_config cfg = {
		.min_abs_flux_vs = 1.0e-6f,
		.max_spread_ratio = 0.10f,
		.scale_factor = 0.90f,
		.min_samples = 4U,
	};
	const float l = 0.0030f;
	const float scaled_l = l * cfg.scale_factor;
	const float flux[] = { 0.00009f, -0.00012f, 0.00015f, -0.00018f };
	struct motor_electrical_id_demod_accum acc;
	struct motor_electrical_id_demod_result result;

	motor_electrical_id_demod_reset(&acc);
	for (size_t i = 0U; i < ARRAY_SIZE(flux); ++i) {
		zassert_ok(motor_electrical_id_demod_add(&acc, &cfg, flux[i], flux[i] / l),
			   NULL);
	}

	zassert_ok(motor_electrical_id_demod_finalize(&acc, &cfg, &limits, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.inductance_h - scaled_l) < EPS,
		     "L %.8f", (double)result.inductance_h);
	zassert_true(result.confidence > 0.95f, NULL);
	zassert_equal(result.samples, 4U, NULL);
}

ZTEST(motor_electrical_id, test_demodulated_inductance_pairs_cancel_offset)
{
	const struct motor_electrical_id_demod_config cfg = {
		.min_abs_flux_vs = 1.0e-6f,
		.max_spread_ratio = 0.10f,
		.scale_factor = 1.0f,
		.min_samples = 3U,
	};
	const float l = 0.0035f;
	const float flux[] = { 0.00020f, 0.00022f, 0.00024f };
	const float current_offset_a = 0.004f;
	struct motor_electrical_id_demod_accum acc;
	struct motor_electrical_id_demod_result result;

	motor_electrical_id_demod_reset(&acc);
	for (size_t i = 0U; i < ARRAY_SIZE(flux); ++i) {
		float pos_di = (flux[i] / l) + current_offset_a;
		float neg_di = (-flux[i] / l) + current_offset_a;

		zassert_ok(motor_electrical_id_demod_add_pair(&acc, &cfg,
							      flux[i], pos_di,
							      -flux[i], neg_di),
			   NULL);
	}

	zassert_ok(motor_electrical_id_demod_finalize(&acc, &cfg, &limits, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_true(fabsf(result.inductance_h - l) < EPS,
		     "L %.8f", (double)result.inductance_h);
	zassert_true(result.confidence > 0.95f, NULL);
}

ZTEST(motor_electrical_id, test_demodulated_inductance_rejects_bad_samples)
{
	const struct motor_electrical_id_demod_config cfg = {
		.min_abs_flux_vs = 1.0e-4f,
		.max_spread_ratio = 0.10f,
		.scale_factor = 1.0f,
		.min_samples = 2U,
	};
	struct motor_electrical_id_demod_accum acc;
	struct motor_electrical_id_demod_result result;

	motor_electrical_id_demod_reset(&acc);
	zassert_equal(motor_electrical_id_demod_add(&acc, &cfg, 1.0e-5f, 0.01f), 1,
		      NULL);
	zassert_equal(motor_electrical_id_demod_add(&acc, &cfg, 1.0e-3f, -0.01f), 1,
		      NULL);
	zassert_equal(acc.rejected_low_signal, 1U, NULL);
	zassert_equal(acc.rejected_non_positive, 1U, NULL);
	zassert_equal(motor_electrical_id_demod_finalize(&acc, &cfg, &limits, &result),
		      -EAGAIN, NULL);
}

ZTEST(motor_electrical_id, test_demodulated_inductance_rejects_excessive_spread)
{
	const struct motor_electrical_id_demod_config cfg = {
		.min_abs_flux_vs = 1.0e-6f,
		.max_spread_ratio = 0.05f,
		.scale_factor = 1.0f,
		.min_samples = 4U,
	};
	struct motor_electrical_id_demod_accum acc;
	struct motor_electrical_id_demod_result result;

	motor_electrical_id_demod_reset(&acc);
	zassert_ok(motor_electrical_id_demod_add(&acc, &cfg, 0.0010f, 0.333333f), NULL);
	zassert_ok(motor_electrical_id_demod_add(&acc, &cfg, 0.0010f, 0.333333f), NULL);
	zassert_ok(motor_electrical_id_demod_add(&acc, &cfg, 0.0010f, 0.250000f), NULL);
	zassert_ok(motor_electrical_id_demod_add(&acc, &cfg, 0.0010f, 0.250000f), NULL);

	zassert_equal(motor_electrical_id_demod_finalize(&acc, &cfg, &limits, &result),
		      -ERANGE, NULL);
	zassert_false(result.valid, NULL);
	zassert_true(result.spread_ratio > cfg.max_spread_ratio, NULL);
}

ZTEST(motor_electrical_id, test_combine_and_pi_recommendation)
{
	const struct motor_electrical_id_rs_result rs = {
		.rs_ohm = 2.2f,
		.confidence = 0.9f,
		.valid = true,
	};
	const struct motor_electrical_id_l_result ld = {
		.inductance_h = 0.003f,
		.confidence = 0.8f,
		.valid = true,
	};
	const struct motor_electrical_id_l_result lq = {
		.inductance_h = 0.004f,
		.confidence = 0.7f,
		.valid = true,
	};
	struct motor_electrical_id_result id;
	struct motor_electrical_id_pi_recommendation pi;

	zassert_ok(motor_electrical_id_combine(&rs, &ld, &lq, &limits, &id), NULL);
	zassert_true(id.valid, NULL);
	zassert_true(fabsf(id.l_avg_h - 0.0035f) < EPS, NULL);
	zassert_true(fabsf(id.lq_minus_ld_h - 0.001f) < EPS, NULL);
	zassert_true((id.flags & MOTOR_ELECTRICAL_ID_FLAG_LAVG_VALID) != 0U, NULL);

	zassert_ok(motor_electrical_id_recommend_current_pi(id.rs_ohm, id.ld_h, id.lq_h,
						      1000.0f, 0.00005f, &pi), NULL);
	zassert_true(pi.valid, NULL);
	zassert_true(fabsf(pi.kp_d - (0.003f * 2.0f * PI_F32 * 1000.0f)) < 1e-4f, NULL);
	zassert_true(fabsf(pi.ki_d - ((2.2f / 0.003f) * 0.00005f)) < EPS, NULL);
	zassert_true(fabsf(pi.kp_q - (0.004f * 2.0f * PI_F32 * 1000.0f)) < 1e-4f, NULL);
	zassert_true(fabsf(pi.ki_q - ((2.2f / 0.004f) * 0.00005f)) < EPS, NULL);
}

ZTEST(motor_electrical_id, test_combine_rejects_excessive_axis_mismatch)
{
	const struct motor_electrical_id_rs_result rs = {
		.rs_ohm = 2.2f,
		.confidence = 0.9f,
		.valid = true,
	};
	const struct motor_electrical_id_l_result ld = {
		.inductance_h = 0.001f,
		.confidence = 0.9f,
		.valid = true,
	};
	const struct motor_electrical_id_l_result lq = {
		.inductance_h = 0.010f,
		.confidence = 0.9f,
		.valid = true,
	};
	struct motor_electrical_id_result id;

	zassert_equal(motor_electrical_id_combine(&rs, &ld, &lq, &limits, &id), -ERANGE,
		      NULL);
	zassert_false(id.valid, NULL);
}

ZTEST(motor_electrical_id, test_pi_recommendation_rejects_invalid)
{
	struct motor_electrical_id_pi_recommendation pi;

	zassert_equal(motor_electrical_id_recommend_current_pi(0.0f, 0.003f, 0.003f,
							 1000.0f, 0.00005f, &pi), -EINVAL,
		      NULL);
	zassert_equal(motor_electrical_id_recommend_current_pi(2.0f, 0.003f, 0.003f,
							 0.0f, 0.00005f, &pi), -EINVAL,
		      NULL);
}

ZTEST_SUITE(motor_electrical_id, NULL, NULL, NULL, NULL, NULL);
