/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor/motion/motion_planner.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"

ZTEST(motor_motion_modules, test_velocity_plan_limits_delta)
{
	struct traj_f32 traj = {0};
	const float32_t dt = 0.01f;
	const float32_t max_vel = 10.0f;
	const float32_t max_acc = 20.0f;
	float32_t target = 0.0f;
	float32_t ref = 0.0f;
	float32_t prev_ref = 0.0f;

	motor_velocity_plan_init(&traj, max_vel, max_acc, dt, 0.0f);
	traj_set_target_value(&traj, 5.0f);

	for (int i = 0; i < 40; i++) {
		motor_velocity_plan_step(&traj, &target, &ref);
		zassert_true(fabsf(ref - prev_ref) <= (max_acc * dt + 1e-6f), NULL);
		prev_ref = ref;
	}

	zassert_within(target, 5.0f, 1e-6f, NULL);
	zassert_true(ref > 0.0f, NULL);
	zassert_true(ref <= 5.0f + 1e-6f, NULL);
}

ZTEST(motor_motion_modules, test_velocity_plan_update_limits_clamps_state)
{
	struct traj_f32 traj = {0};

	motor_velocity_plan_init(&traj, 20.0f, 40.0f, 0.001f, 0.0f);
	traj_set_target_value(&traj, 12.0f);
	traj_set_int_value(&traj, 8.0f);

	motor_velocity_plan_update_limits(&traj, 5.0f, 10.0f, 0.001f);

	zassert_within(traj_get_target_value(&traj), 5.0f, 1e-6f, NULL);
	zassert_within(traj_get_int_value(&traj), 5.0f, 1e-6f, NULL);
}

ZTEST(motor_motion_modules, test_position_sequence_take_next_non_loop)
{
	const float32_t points[] = {0.1f, 0.2f, 0.3f};
	uint16_t idx = 0U;
	float32_t target = 0.0f;
	bool complete = false;

	zassert_ok(motor_position_sequence_take_next(points, 3U, false, &idx, &target, &complete),
		   NULL);
	zassert_within(target, 0.1f, 1e-6f, NULL);
	zassert_false(complete, NULL);
	zassert_equal(idx, 1U, NULL);

	zassert_ok(motor_position_sequence_take_next(points, 3U, false, &idx, &target, &complete),
		   NULL);
	zassert_within(target, 0.2f, 1e-6f, NULL);
	zassert_false(complete, NULL);
	zassert_equal(idx, 2U, NULL);

	zassert_ok(motor_position_sequence_take_next(points, 3U, false, &idx, &target, &complete),
		   NULL);
	zassert_within(target, 0.3f, 1e-6f, NULL);
	zassert_true(complete, NULL);
	zassert_equal(idx, 3U, NULL);

	zassert_equal(motor_position_sequence_take_next(points, 3U, false, &idx, &target, &complete),
		      -ENOENT, NULL);
}

ZTEST(motor_motion_modules, test_position_sequence_take_next_loop_wraps)
{
	const float32_t points[] = {1.0f, 2.0f};
	uint16_t idx = 2U;
	float32_t target = 0.0f;
	bool complete = true;

	zassert_ok(motor_position_sequence_take_next(points, 2U, true, &idx, &target, &complete),
		   NULL);
	zassert_within(target, 1.0f, 1e-6f, NULL);
	zassert_false(complete, NULL);
	zassert_equal(idx, 1U, NULL);
}

ZTEST(motor_motion_modules, test_position_move_plan_wraps_shortest_path)
{
	struct motion_profile_quintic profile = {0};
	const float32_t start = 6.20f;
	const float32_t target_wrapped = 0.05f;

	motion_profile_quintic_init(&profile, 0.001f);

	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
					   start,
					   0.0f,
					   target_wrapped,
					   0.0f,
					   0.3f,
					   20.0f,
					   200.0f), NULL);

	float32_t expected_delta = wrap_rad_pi(target_wrapped - start);
	zassert_within(profile.end_position_rad - start, expected_delta, 1e-5f, NULL);
	zassert_true(profile.active, NULL);
	zassert_true(profile.valid, NULL);
}

ZTEST(motor_motion_modules, test_position_move_resolve_active_and_complete)
{
	struct motion_profile_quintic profile = {0};
	float32_t target_wrapped = 0.0f;
	float32_t position_error = 0.0f;
	float32_t velocity_ff = 0.0f;

	motion_profile_quintic_init(&profile, 0.001f);
	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
					   0.2f,
					   0.0f,
					   1.0f,
					   0.0f,
					   0.2f,
					   100.0f,
					   2000.0f), NULL);

	zassert_true(motor_position_move_resolve(&profile, true, 0.2f,
					   &target_wrapped,
					   &position_error,
					   &velocity_ff), NULL);
	zassert_true(velocity_ff > 0.0f, NULL);

	for (int i = 0; i < 200; i++) {
		(void)motor_position_move_resolve(&profile, true, target_wrapped,
						 &target_wrapped,
						 &position_error,
						 &velocity_ff);
	}

	zassert_false(motion_profile_quintic_is_active(&profile), NULL);
	zassert_true(profile.valid, NULL);
	zassert_true(motor_position_move_resolve(&profile, false, target_wrapped,
					   &target_wrapped,
					   &position_error,
					   &velocity_ff), NULL);
	zassert_within(velocity_ff, 0.0f, 1e-6f, NULL);

	motion_profile_quintic_cancel(&profile, target_wrapped);
	zassert_false(motor_position_move_resolve(&profile, false, target_wrapped,
					    &target_wrapped,
					    &position_error,
					    &velocity_ff), NULL);
}

ZTEST(motor_motion_modules, test_position_move_resolve_zeroes_ff_on_completion_step)
{
	struct motion_profile_quintic profile = {0};
	float32_t target_wrapped = 0.0f;
	float32_t position_error = 0.0f;
	float32_t velocity_ff = 0.0f;

	motion_profile_quintic_init(&profile, 0.01f);
	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
					   0.0f,
					   0.5f,
					   0.02f,
					   0.3f,
					   0.01f,
					   100.0f,
					   10000.0f), NULL);

	zassert_true(motor_position_move_resolve(&profile, true, 0.0f,
					   &target_wrapped,
					   &position_error,
					   &velocity_ff), NULL);
	zassert_false(motion_profile_quintic_is_active(&profile), NULL);
	zassert_within(velocity_ff, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_motion_modules, test_profile_sequence_can_drive_generated_angle)
{
	const float32_t points[] = {0.5f, -0.25f};
	struct motion_profile_quintic profile = {0};
	angle_gen_t angle_gen = {0};
	uint16_t idx = 0U;
	float32_t target_wrapped = 0.0f;
	bool complete = false;

	motion_profile_quintic_init(&profile, 0.001f);
	angle_gen_init(&angle_gen, 0.001f);
	angle_gen_set_angle(&angle_gen, 0.0f);

	zassert_ok(motor_position_sequence_take_next(points, 2U, false, &idx, &target_wrapped,
						    &complete), NULL);
	zassert_false(complete, NULL);
	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
					   angle_gen_get_angle(&angle_gen),
					   0.0f,
					   target_wrapped,
					   0.0f,
					   0.05f,
					   100.0f,
					   10000.0f), NULL);

	while (motion_profile_quintic_is_active(&profile)) {
		motion_profile_quintic_step(&profile);
		angle_gen_set_angle(&angle_gen, motion_profile_quintic_get_position(&profile));
	}

	zassert_within(angle_gen_get_angle(&angle_gen), 0.5f, 1e-5f, NULL);
	zassert_ok(motor_position_sequence_take_next(points, 2U, false, &idx, &target_wrapped,
						    &complete), NULL);
	zassert_true(complete, NULL);
	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
					   angle_gen_get_angle(&angle_gen),
					   0.0f,
					   target_wrapped,
					   0.0f,
					   0.05f,
					   100.0f,
					   10000.0f), NULL);

	while (motion_profile_quintic_is_active(&profile)) {
		motion_profile_quintic_step(&profile);
		angle_gen_set_angle(&angle_gen, motion_profile_quintic_get_position(&profile));
	}

	zassert_within(wrap_rad_pi(angle_gen_get_angle(&angle_gen) - (-0.25f)), 0.0f, 1e-5f,
		       NULL);
	zassert_equal(idx, 2U, NULL);
}

ZTEST_SUITE(motor_motion_modules, NULL, NULL, NULL, NULL, NULL);
