/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <zephyr/ztest.h>

#include "motor/math/math_constants.h"
#include "motor/motion/motion_profile.h"

static void motion_profile_run_to_completion(struct motion_profile_quintic *profile,
					     uint32_t max_steps)
{
	for (uint32_t i = 0U; i < max_steps && motion_profile_quintic_is_active(profile); i++) {
		motion_profile_quintic_step(profile);
	}
}

ZTEST(motion_profile, test_plan_rejects_invalid_inputs)
{
	struct motion_profile_quintic profile;
	motion_profile_quintic_init(&profile, 0.001f);

	zassert_equal(motion_profile_quintic_plan(NULL, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f),
		      -EINVAL, NULL);
	zassert_equal(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f),
		      -EINVAL, NULL);

	motion_profile_quintic_init(&profile, 0.0f);
	zassert_equal(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f),
		      -EINVAL, NULL);

	motion_profile_quintic_init(&profile, NAN);
	zassert_equal(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f),
		      -EINVAL, NULL);

	motion_profile_quintic_init(&profile, 0.001f);
	zassert_equal(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, NAN),
		      -EINVAL, NULL);
	zassert_equal(
		motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, INFINITY),
		-EINVAL, NULL);
}

ZTEST(motion_profile, test_stop_to_stop_endpoints)
{
	struct motion_profile_quintic profile;
	const float32_t end_pos = 0.5f * PI_F32;
	motion_profile_quintic_init(&profile, 0.001f);

	zassert_ok(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f,
					       end_pos, 0.0f, 0.0f, 0.100f), NULL);

	motion_profile_run_to_completion(&profile, 200U);

	zassert_false(motion_profile_quintic_is_active(&profile), NULL);
	zassert_true(profile.valid, NULL);
	zassert_within(profile.position_rad, end_pos, 1e-4f, NULL);
	zassert_within(profile.velocity_rad_s, 0.0f, 1e-3f, NULL);
	zassert_within(profile.acceleration_rad_s2, 0.0f, 5e-2f, NULL);
}

ZTEST(motion_profile, test_general_endpoint_constraints)
{
	struct motion_profile_quintic profile;
	const float32_t pos_start = 0.25f;
	const float32_t vel_start = -1.5f;
	const float32_t acc_start = 4.0f;
	const float32_t pos_end = -0.75f;
	const float32_t vel_end = 0.8f;
	const float32_t acc_end = -3.0f;
	const float32_t duration_s = 0.05f;

	motion_profile_quintic_init(&profile, 0.0002f);
	zassert_ok(motion_profile_quintic_plan(&profile, pos_start, vel_start, acc_start,
					       pos_end, vel_end, acc_end, duration_s), NULL);

	zassert_within(profile.position_rad, pos_start, 1e-6f, NULL);
	zassert_within(profile.velocity_rad_s, vel_start, 1e-6f, NULL);
	zassert_within(profile.acceleration_rad_s2, acc_start, 1e-6f, NULL);

	motion_profile_run_to_completion(&profile, 800U);
	zassert_false(profile.active, NULL);
	zassert_within(profile.t_s, duration_s, 1e-7f, NULL);
	zassert_within(profile.position_rad, pos_end, 2e-4f, NULL);
	zassert_within(profile.velocity_rad_s, vel_end, 2e-3f, NULL);
	zassert_within(profile.acceleration_rad_s2, acc_end, 5e-2f, NULL);
}

ZTEST(motion_profile, test_step_clamps_time_and_holds_endpoint)
{
	struct motion_profile_quintic profile;

	motion_profile_quintic_init(&profile, 0.020f);
	zassert_ok(motion_profile_quintic_plan(&profile, 1.0f, 0.5f, 0.0f,
					       1.4f, -0.5f, 0.0f, 0.010f), NULL);

	motion_profile_quintic_step(&profile);
	zassert_false(profile.active, NULL);
	zassert_within(profile.t_s, profile.duration_s, 1e-8f, NULL);

	const float32_t end_pos = profile.position_rad;
	const float32_t end_vel = profile.velocity_rad_s;
	const float32_t end_acc = profile.acceleration_rad_s2;
	motion_profile_quintic_step(&profile);
	zassert_within(profile.position_rad, end_pos, 1e-8f, NULL);
	zassert_within(profile.velocity_rad_s, end_vel, 1e-8f, NULL);
	zassert_within(profile.acceleration_rad_s2, end_acc, 1e-8f, NULL);
}

ZTEST(motion_profile, test_constant_velocity_degenerates_to_linear)
{
	struct motion_profile_quintic profile;
	const float32_t duration_s = 0.2f;
	const float32_t pos_start = 0.5f;
	const float32_t omega = 2.0f;
	const float32_t pos_end = pos_start + omega * duration_s;

	motion_profile_quintic_init(&profile, 0.01f);
	zassert_ok(motion_profile_quintic_plan(&profile, pos_start, omega, 0.0f,
					       pos_end, omega, 0.0f, duration_s), NULL);

	zassert_within(profile.c[2], 0.0f, 1e-5f, NULL);
	zassert_within(profile.c[3], 0.0f, 1e-4f, NULL);
	zassert_within(profile.c[4], 0.0f, 1e-4f, NULL);
	zassert_within(profile.c[5], 0.0f, 1e-4f, NULL);

	for (int i = 0; i < 10; i++) {
		motion_profile_quintic_step(&profile);
		zassert_within(profile.velocity_rad_s, omega, 1e-4f, NULL);
		zassert_within(profile.acceleration_rad_s2, 0.0f, 2e-3f, NULL);
	}
}

ZTEST(motion_profile, test_limit_check_validation_and_violation)
{
	struct motion_profile_quintic profile;
	float32_t peak_v = 0.0f;
	float32_t peak_a = 0.0f;
	struct motion_profile_quintic invalid_profile = {0};

	zassert_equal(motion_profile_quintic_check_limits(NULL, 1.0f, 1.0f, 8U, NULL, NULL),
		      -EINVAL, NULL);
	zassert_equal(
		motion_profile_quintic_check_limits(&invalid_profile, 1.0f, 1.0f, 8U, NULL, NULL),
		-EINVAL, NULL);

	motion_profile_quintic_init(&profile, 0.0005f);

	zassert_ok(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f,
					       5.0f, 0.0f, 0.0f, 0.01f), NULL);

	zassert_equal(motion_profile_quintic_check_limits(&profile, 10.0f, 1000.0f, 128U,
							  &peak_v, &peak_a),
		      -ERANGE, NULL);
	zassert_true(peak_v > 10.0f || peak_a > 1000.0f, NULL);
	zassert_ok(motion_profile_quintic_check_limits(&profile, 10000.0f, 1000000.0f, 128U,
						       &peak_v, &peak_a), NULL);

	zassert_ok(motion_profile_quintic_check_limits(&profile, 0.0f, 0.0f, 0U,
						       &peak_v, &peak_a), NULL);
	zassert_true(peak_v > 0.0f, NULL);
	zassert_true(peak_a >= 0.0f, NULL);
	zassert_ok(motion_profile_quintic_check_limits(&profile, 0.0f, 0.0f, 1U, NULL, NULL), NULL);
}

ZTEST(motion_profile, test_cancel_holds_position)
{
	struct motion_profile_quintic profile;
	motion_profile_quintic_init(&profile, 0.001f);
	zassert_ok(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f,
					       1.0f, 0.0f, 0.0f, 0.2f), NULL);

	motion_profile_quintic_step(&profile);
	motion_profile_quintic_cancel(&profile, 0.7f);

	zassert_false(profile.active, NULL);
	zassert_false(profile.valid, NULL);
	zassert_within(profile.position_rad, 0.7f, 1e-6f, NULL);
	zassert_within(profile.velocity_rad_s, 0.0f, 1e-6f, NULL);
	zassert_within(profile.acceleration_rad_s2, 0.0f, 1e-6f, NULL);
}

ZTEST(motion_profile, test_null_safe_noop_calls)
{
	motion_profile_quintic_init(NULL, 0.001f);
	motion_profile_quintic_step(NULL);
	motion_profile_quintic_cancel(NULL, 0.0f);
}

ZTEST_SUITE(motion_profile, NULL, NULL, NULL, NULL, NULL);
