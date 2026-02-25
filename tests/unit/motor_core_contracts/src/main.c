/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/ztest.h>

#include "motor/math/angle_wrap.h"
#include "motor/motion/motion_planner.h"

ZTEST(motor_core_contracts, test_motion_segment_handoff_has_no_boundary_discontinuity)
{
	struct motion_profile_quintic profile = {0};
	float32_t measured_pos_rad = 0.30f;
	float32_t target_wrapped_rad = 0.0f;
	float32_t position_error_rad = 0.0f;
	float32_t velocity_ff_rad_s = 0.0f;
	bool completed = false;

	motion_profile_quintic_init(&profile, 0.001f);

	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
							      measured_pos_rad,
							      0.0f,
							      1.20f,
							      0.0f,
							      0.050f,
							      100.0f,
							      3000.0f),
		   NULL);

	for (int i = 0; i < 200; i++) {
		zassert_true(motor_position_move_resolve(&profile, true, measured_pos_rad,
							 &target_wrapped_rad,
							 &position_error_rad,
							 &velocity_ff_rad_s),
			     NULL);

		if (!motion_profile_quintic_is_active(&profile)) {
			completed = true;
			zassert_within(velocity_ff_rad_s, 0.0f, 1e-6f, NULL);
			break;
		}

		measured_pos_rad = target_wrapped_rad;
	}

	zassert_true(completed, NULL);

	float32_t first_segment_end_wrapped = target_wrapped_rad;
	float32_t first_segment_end_pos = profile.position_rad;
	float32_t first_segment_end_vel = profile.velocity_rad_s;

	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
							      first_segment_end_pos,
							      first_segment_end_vel,
							      1.80f,
							      0.0f,
							      0.040f,
							      100.0f,
							      3000.0f),
		   NULL);

	zassert_true(motor_position_move_resolve(&profile, false, measured_pos_rad,
						 &target_wrapped_rad,
						 &position_error_rad,
						 &velocity_ff_rad_s),
		     NULL);
	zassert_within(wrap_rad_pi(target_wrapped_rad - first_segment_end_wrapped), 0.0f, 1e-5f,
		       NULL);

	zassert_true(motor_position_move_resolve(&profile, true, measured_pos_rad,
						 &target_wrapped_rad,
						 &position_error_rad,
						 &velocity_ff_rad_s),
		     NULL);
	zassert_true(wrap_rad_pi(target_wrapped_rad - first_segment_end_wrapped) >= -1e-5f, NULL);
}

ZTEST_SUITE(motor_core_contracts, NULL, NULL, NULL, NULL, NULL);
