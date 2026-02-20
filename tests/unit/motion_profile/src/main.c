/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <zephyr/ztest.h>

#include "math_constants.h"
#include "motion_profile.h"

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
}

ZTEST(motion_profile, test_stop_to_stop_endpoints)
{
	struct motion_profile_quintic profile;
	const float32_t end_pos = 0.5f * PI_F32;
	motion_profile_quintic_init(&profile, 0.001f);

	zassert_ok(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f,
					       end_pos, 0.0f, 0.0f, 0.100f), NULL);

	for (int i = 0; i < 200; i++) {
		motion_profile_quintic_step(&profile);
	}

	zassert_false(motion_profile_quintic_is_active(&profile), NULL);
	zassert_true(profile.valid, NULL);
	zassert_within(profile.position_rad, end_pos, 1e-4f, NULL);
	zassert_within(profile.velocity_rad_s, 0.0f, 1e-3f, NULL);
	zassert_within(profile.acceleration_rad_s2, 0.0f, 5e-2f, NULL);
}

ZTEST(motion_profile, test_limit_check_detects_violation)
{
	struct motion_profile_quintic profile;
	float32_t peak_v = 0.0f;
	float32_t peak_a = 0.0f;
	motion_profile_quintic_init(&profile, 0.0005f);

	zassert_ok(motion_profile_quintic_plan(&profile, 0.0f, 0.0f, 0.0f,
					       5.0f, 0.0f, 0.0f, 0.01f), NULL);

	zassert_equal(motion_profile_quintic_check_limits(&profile, 10.0f, 1000.0f, 128U,
							  &peak_v, &peak_a),
		      -ERANGE, NULL);
	zassert_true(peak_v > 10.0f || peak_a > 1000.0f, NULL);
	zassert_ok(motion_profile_quintic_check_limits(&profile, 10000.0f, 1000000.0f, 128U,
						       &peak_v, &peak_a), NULL);
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

ZTEST_SUITE(motion_profile, NULL, NULL, NULL, NULL, NULL);
