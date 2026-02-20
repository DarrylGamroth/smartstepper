/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "traj.h"

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

ZTEST_SUITE(traj, NULL, NULL, NULL, NULL, NULL);
