/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/control/current_slew.h"
#include "motor/motion/traj.h"

ZTEST(traj, test_init_zeros_state)
{
	struct traj_f32 traj = {
		.target_value = 1.0f,
		.int_value = -2.0f,
		.min_value = -10.0f,
		.max_value = 10.0f,
		.max_delta = 0.5f,
	};

	traj_init(&traj);

	zassert_within(traj.target_value, 0.0f, 1e-8f, NULL);
	zassert_within(traj.int_value, 0.0f, 1e-8f, NULL);
	zassert_within(traj.min_value, 0.0f, 1e-8f, NULL);
	zassert_within(traj.max_value, 0.0f, 1e-8f, NULL);
	zassert_within(traj.max_delta, 0.0f, 1e-8f, NULL);
}

ZTEST(traj, test_setter_getter_accessors)
{
	struct traj_f32 traj = {0};

	traj_set_target_value(&traj, 1.5f);
	traj_set_int_value(&traj, -0.3f);
	traj_set_min_value(&traj, -2.0f);
	traj_set_max_value(&traj, 3.0f);
	traj_set_max_delta(&traj, 0.2f);

	zassert_within(traj_get_target_value(&traj), 1.5f, 1e-8f, NULL);
	zassert_within(traj_get_int_value(&traj), -0.3f, 1e-8f, NULL);
	zassert_within(traj_get_min_value(&traj), -2.0f, 1e-8f, NULL);
	zassert_within(traj_get_max_value(&traj), 3.0f, 1e-8f, NULL);
	zassert_within(traj_get_max_delta(&traj), 0.2f, 1e-8f, NULL);
}

ZTEST(traj, test_run_ramps_toward_target_with_delta_limit)
{
	struct traj_f32 traj = {0};
	traj_set_min_value(&traj, -5.0f);
	traj_set_max_value(&traj, 5.0f);
	traj_set_max_delta(&traj, 0.25f);
	traj_set_target_value(&traj, 1.0f);

	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), 0.25f, 1e-8f, NULL);
	zassert_false(traj_is_at_target(&traj), NULL);

	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), 0.50f, 1e-8f, NULL);

	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), 0.75f, 1e-8f, NULL);

	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), 1.0f, 1e-8f, NULL);
	zassert_true(traj_is_at_target(&traj), NULL);
}

ZTEST(traj, test_run_ramps_in_negative_direction)
{
	struct traj_f32 traj = {0};
	traj_set_min_value(&traj, -5.0f);
	traj_set_max_value(&traj, 5.0f);
	traj_set_max_delta(&traj, 0.4f);
	traj_set_target_value(&traj, -1.0f);

	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), -0.4f, 1e-8f, NULL);
	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), -0.8f, 1e-8f, NULL);
	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), -1.0f, 1e-8f, NULL);
	zassert_true(traj_is_at_target(&traj), NULL);
}

ZTEST(traj, test_run_clamps_to_min_max_bounds)
{
	struct traj_f32 traj = {0};
	traj_set_min_value(&traj, -0.5f);
	traj_set_max_value(&traj, 0.5f);
	traj_set_max_delta(&traj, 10.0f);

	traj_set_target_value(&traj, 2.0f);
	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), 0.5f, 1e-8f, NULL);

	traj_set_target_value(&traj, -2.0f);
	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), -0.5f, 1e-8f, NULL);
}

ZTEST(traj, test_zero_delta_prevents_motion)
{
	struct traj_f32 traj = {0};
	traj_set_int_value(&traj, 0.2f);
	traj_set_target_value(&traj, 1.0f);
	traj_set_min_value(&traj, -5.0f);
	traj_set_max_value(&traj, 5.0f);
	traj_set_max_delta(&traj, 0.0f);

	traj_run(&traj);
	zassert_within(traj_get_int_value(&traj), 0.2f, 1e-8f, NULL);
	zassert_false(traj_is_at_target(&traj), NULL);
}

ZTEST(traj, test_out_of_bounds_target_saturates_without_reaching_target)
{
	struct traj_f32 traj = {0};
	traj_set_min_value(&traj, -1.0f);
	traj_set_max_value(&traj, 1.0f);
	traj_set_max_delta(&traj, 0.6f);
	traj_set_target_value(&traj, 2.0f);

	for (int i = 0; i < 8; i++) {
		traj_run(&traj);
	}

	zassert_within(traj_get_int_value(&traj), 1.0f, 1e-8f, NULL);
	zassert_false(traj_is_at_target(&traj), NULL);
}

ZTEST(traj, test_current_slew_delta_uses_ramp_time_and_control_rate)
{
	zassert_within(motor_current_slew_delta_a_per_tick(2.0f, 0.1f, 1000.0f),
		       0.02f, 1e-8f, NULL);
	zassert_within(motor_current_slew_delta_a_per_tick(0.0f, 0.1f, 1000.0f),
		       1.0e-6f, 1e-10f, NULL);
	zassert_within(motor_current_slew_delta_a_per_tick(2.0f, 0.0f, 1000.0f),
		       1.0e-6f, 1e-10f, NULL);
}

ZTEST(traj, test_current_slew_pair_sets_targets_and_ramps_both_axes)
{
	struct traj_f32 id = {0};
	struct traj_f32 iq = {0};
	struct motor_current_slew_pair slew = {
		.id = &id,
		.iq = &iq,
	};
	float32_t id_ref = 0.0f;
	float32_t iq_ref = 0.0f;

	motor_current_slew_pair_init(&slew, 1.0f, 0.25f);
	motor_current_slew_set_target(&slew, 0.5f, -0.75f);

	motor_current_slew_run(&slew, &id_ref, &iq_ref);
	zassert_within(id_ref, 0.25f, 1e-8f, NULL);
	zassert_within(iq_ref, -0.25f, 1e-8f, NULL);
	zassert_false(motor_current_slew_at_target(&slew), NULL);

	motor_current_slew_run(&slew, &id_ref, &iq_ref);
	motor_current_slew_run(&slew, &id_ref, &iq_ref);
	zassert_within(id_ref, 0.5f, 1e-8f, NULL);
	zassert_within(iq_ref, -0.75f, 1e-8f, NULL);
	zassert_true(motor_current_slew_at_target(&slew), NULL);
}

ZTEST(traj, test_current_slew_force_zero_resets_integrated_refs)
{
	struct traj_f32 id = {0};
	struct traj_f32 iq = {0};
	struct motor_current_slew_pair slew = {
		.id = &id,
		.iq = &iq,
	};

	motor_current_slew_pair_init(&slew, 2.0f, 0.1f);
	motor_current_slew_force(&slew, 1.0f, -1.0f);
	zassert_within(traj_get_int_value(&id), 1.0f, 1e-8f, NULL);
	zassert_within(traj_get_int_value(&iq), -1.0f, 1e-8f, NULL);

	motor_current_slew_force_zero(&slew);
	zassert_within(traj_get_target_value(&id), 0.0f, 1e-8f, NULL);
	zassert_within(traj_get_target_value(&iq), 0.0f, 1e-8f, NULL);
	zassert_within(traj_get_int_value(&id), 0.0f, 1e-8f, NULL);
	zassert_within(traj_get_int_value(&iq), 0.0f, 1e-8f, NULL);
}

ZTEST_SUITE(traj, NULL, NULL, NULL, NULL, NULL);
