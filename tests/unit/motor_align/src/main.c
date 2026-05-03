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

ZTEST(motor_align, test_circular_mean_handles_wrap_boundary)
{
	struct motor_align_sample_accum acc = {0};
	float32_t mean_rad = 0.0f;

	motor_align_accum_push(&acc, deg_to_rad(359.0f));
	motor_align_accum_push(&acc, deg_to_rad(1.0f));

	zassert_true(motor_align_circular_mean(&acc, &mean_rad), NULL);
	zassert_within(wrap_rad_pi(mean_rad), 0.0f, 2e-3f, NULL);
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

ZTEST(motor_align, test_offset_from_mech_sample_wraps_to_negative_mech)
{
	float32_t mech = 5.2f;
	float32_t out = motor_align_offset_from_mech_sample(mech);
	float32_t expected = wrap_rad_pi(-wrap_rad_2pi(mech));

	zassert_within(out, expected, 1e-6f, NULL);
}

ZTEST(motor_align, test_offset_from_invalid_mech_sample_returns_zero)
{
	zassert_within(motor_align_offset_from_mech_sample(NAN), 0.0f, 1e-6f, NULL);
	zassert_within(motor_align_offset_from_mech_sample(10.0f * PI_F32), 0.0f, 1e-6f, NULL);
}

ZTEST_SUITE(motor_align, NULL, NULL, NULL, NULL, NULL);
