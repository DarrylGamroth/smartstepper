/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/calibration/align.h"
#include "motor/math/math_constants.h"
#include "motor/math/angle_wrap.h"

static float32_t deg_to_rad(float32_t deg)
{
	return deg * (PI_F32 / 180.0f);
}

static float32_t electrical_alignment_error(float32_t mech_rad,
					    float32_t offset_rad,
					    float32_t pole_pairs,
					    float32_t target_elec_rad)
{
	return fabsf(wrap_rad_pi(((mech_rad + offset_rad) * pole_pairs) -
				 target_elec_rad));
}

ZTEST(motor_align, test_accum_reset_clears_fields)
{
	struct motor_align_sample_accum acc = {
		.sum_sin = 1.0f,
		.sum_cos = -2.0f,
		.count = 123U,
	};

	motor_align_accum_reset(&acc);

	zassert_within(acc.sum_sin, 0.0f, 1e-6f, NULL);
	zassert_within(acc.sum_cos, 0.0f, 1e-6f, NULL);
	zassert_equal(acc.count, 0U, NULL);
}

ZTEST(motor_align, test_accum_push_and_circular_mean_single_sample)
{
	struct motor_align_sample_accum acc = {0};
	float32_t mean_rad = 0.0f;

	motor_align_accum_push(&acc, PI_F32 / 2.0f);

	zassert_equal(acc.count, 1U, NULL);
	zassert_true(motor_align_circular_mean(&acc, &mean_rad), NULL);
	zassert_within(mean_rad, PI_F32 / 2.0f, 1e-3f, NULL);
}

ZTEST(motor_align, test_circular_mean_fails_for_empty_accumulator)
{
	struct motor_align_sample_accum acc = {0};
	float32_t mean_rad = 0.0f;

	zassert_false(motor_align_circular_mean(&acc, &mean_rad), NULL);
}

ZTEST(motor_align, test_dual_polarity_succeeds_for_expected_spacing)
{
	const float32_t pole_pairs = 50.0f;
	const float32_t pos = 0.31f;
	const float32_t expected_delta = PI_F32 / pole_pairs;
	const float32_t neg = pos + expected_delta;
	struct motor_align_config cfg = {
		.pole_pairs = pole_pairs,
		.opposed_elec_tol_rad = (20.0f * PI_F32 / 180.0f),
	};
	struct motor_align_sample_accum acc_pos = {0};
	struct motor_align_sample_accum acc_neg = {0};
	struct motor_align_dual_result out = {0};

	motor_align_accum_push(&acc_pos, pos);
	motor_align_accum_push(&acc_neg, neg);

	zassert_true(motor_align_compute_dual_polarity(&cfg, &acc_pos, &acc_neg, &out), NULL);
	zassert_true(out.valid, NULL);
	zassert_within(out.expected_delta_mech_rad, expected_delta, 1e-4f, NULL);
	zassert_within(out.measured_delta_mech_rad, expected_delta, 1e-3f, NULL);
}

ZTEST(motor_align, test_dual_polarity_averages_offsets_in_electrical_domain)
{
	const float32_t pole_pairs = 50.0f;
	const float32_t pos = deg_to_rad(358.24f);
	const float32_t neg = deg_to_rad(355.03f);
	struct motor_align_config cfg = {
		.pole_pairs = pole_pairs,
		.opposed_elec_tol_rad = (25.0f * PI_F32 / 180.0f),
	};
	struct motor_align_sample_accum acc_pos = {0};
	struct motor_align_sample_accum acc_neg = {0};
	struct motor_align_dual_result out = {0};

	motor_align_accum_push(&acc_pos, pos);
	motor_align_accum_push(&acc_neg, neg);

	zassert_true(motor_align_compute_dual_polarity(&cfg, &acc_pos, &acc_neg, &out), NULL);
	zassert_true(out.valid, NULL);
	zassert_within(electrical_alignment_error(pos, out.final_offset_rad, pole_pairs, 0.0f),
		       0.0f, 0.25f, NULL);
	zassert_within(electrical_alignment_error(neg, out.final_offset_rad, pole_pairs, PI_F32),
		       0.0f, 0.25f, NULL);
}

ZTEST(motor_align, test_plan_id_traj_updates_target_and_rate)
{
	struct traj_f32 traj = {0};
	struct motor_align_traj_plan out = {0};

	traj.int_value = 0.02f;
	zassert_ok(motor_align_plan_id_traj(&traj, 0.08f, 0.08f, 20000.0f, &out), NULL);
	zassert_within(traj.target_value, 0.08f, 1e-7f, NULL);
	zassert_true(traj.max_delta > 0.0f, NULL);
	zassert_within(out.id_start_a, 0.02f, 1e-7f, NULL);
	zassert_within(out.id_target_a, 0.08f, 1e-7f, NULL);
	zassert_true(out.max_delta_a_per_tick > 0.0f, NULL);
	zassert_true(out.steps >= 1.0f, NULL);
}

ZTEST(motor_align, test_fallback_offset_from_mech_wraps_to_negative_mech)
{
	float32_t mech = 5.2f;
	float32_t out = motor_align_fallback_offset_from_mech(mech);
	float32_t expected = wrap_rad_pi(-wrap_rad_2pi(mech));

	zassert_within(out, expected, 1e-6f, NULL);
}

ZTEST(motor_align, test_resolve_offset_uses_dual_solution_when_valid)
{
	const float32_t pole_pairs = 50.0f;
	const float32_t pos = 1.2f;
	const float32_t neg = pos + (PI_F32 / pole_pairs);
	struct motor_align_config cfg = {
		.pole_pairs = pole_pairs,
		.opposed_elec_tol_rad = (20.0f * PI_F32 / 180.0f),
	};
	struct motor_align_sample_accum acc_pos = {0};
	struct motor_align_sample_accum acc_neg = {0};
	struct motor_align_offset_result out = {0};

	motor_align_accum_push(&acc_pos, pos);
	motor_align_accum_push(&acc_neg, neg);

	zassert_true(motor_align_resolve_offset(&cfg, &acc_pos, &acc_neg, &out), NULL);
	zassert_true(out.dual_solution_available, NULL);
	zassert_true(out.dual_solution_valid, NULL);
	zassert_within(out.pos_mech_rad, wrap_rad_2pi(pos), 1e-3f, NULL);
	zassert_within(out.neg_mech_rad, wrap_rad_2pi(neg), 1e-3f, NULL);
}

ZTEST(motor_align, test_resolve_offset_falls_back_when_dual_unavailable)
{
	struct motor_align_config cfg = {
		.pole_pairs = 50.0f,
		.opposed_elec_tol_rad = (20.0f * PI_F32 / 180.0f),
	};
	struct motor_align_sample_accum acc_pos = {0};
	struct motor_align_sample_accum acc_neg = {0};
	struct motor_align_offset_result out = {0};
	float32_t expected = 0.0f;

	motor_align_accum_push(&acc_pos, 0.7f);
	zassert_true(motor_align_circular_mean(&acc_pos, &expected), NULL);
	expected = motor_align_fallback_offset_from_mech(expected);

	zassert_true(motor_align_resolve_offset(&cfg, &acc_pos, &acc_neg, &out), NULL);
	zassert_false(out.dual_solution_available, NULL);
	zassert_false(out.dual_solution_valid, NULL);
	zassert_within(out.final_offset_rad, expected, 1e-4f, NULL);
}

ZTEST_SUITE(motor_align, NULL, NULL, NULL, NULL, NULL);
