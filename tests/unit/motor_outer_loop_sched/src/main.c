/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/motion/outer_loop_sched.h"

ZTEST(motor_outer_loop_sched, test_decimation_tick_returns_true_for_phase_null)
{
	zassert_true(motor_outer_loop_decimation_tick(NULL, 4U), NULL);
}

ZTEST(motor_outer_loop_sched, test_decimation_tick_returns_true_for_decimation_le_one)
{
	uint32_t phase = 5U;

	zassert_true(motor_outer_loop_decimation_tick(&phase, 1U), NULL);
	zassert_equal(phase, 0U, NULL);
	phase = 8U;
	zassert_true(motor_outer_loop_decimation_tick(&phase, 0U), NULL);
	zassert_equal(phase, 0U, NULL);
}

ZTEST(motor_outer_loop_sched, test_decimation_tick_periodic_behavior)
{
	uint32_t phase = 0U;

	zassert_true(motor_outer_loop_decimation_tick(&phase, 3U), NULL);
	zassert_equal(phase, 2U, NULL);
	zassert_false(motor_outer_loop_decimation_tick(&phase, 3U), NULL);
	zassert_equal(phase, 1U, NULL);
	zassert_false(motor_outer_loop_decimation_tick(&phase, 3U), NULL);
	zassert_equal(phase, 0U, NULL);
	zassert_true(motor_outer_loop_decimation_tick(&phase, 3U), NULL);
	zassert_equal(phase, 2U, NULL);
}

ZTEST(motor_outer_loop_sched, test_sched_inactive_resets_phases_and_updates_false)
{
	struct motor_outer_loop_sched_input in = {
		.position_active = false,
		.velocity_active = false,
		.position_decimation = 3U,
		.velocity_decimation = 4U,
	};
	struct motor_outer_loop_sched_state state = {
		.position_phase = 2U,
		.velocity_phase = 1U,
	};
	struct motor_outer_loop_sched_output out = {
		.position_update = true,
		.velocity_update = true,
	};

	motor_outer_loop_sched_step(&in, &state, &out);

	zassert_false(out.position_update, NULL);
	zassert_false(out.velocity_update, NULL);
	zassert_equal(state.position_phase, 0U, NULL);
	zassert_equal(state.velocity_phase, 0U, NULL);
}

ZTEST(motor_outer_loop_sched, test_sched_updates_with_independent_decimations)
{
	struct motor_outer_loop_sched_input in = {
		.position_active = true,
		.velocity_active = true,
		.position_decimation = 2U,
		.velocity_decimation = 3U,
	};
	struct motor_outer_loop_sched_state state = {
		.position_phase = 0U,
		.velocity_phase = 0U,
	};
	struct motor_outer_loop_sched_output out = {0};

	motor_outer_loop_sched_step(&in, &state, &out);
	zassert_true(out.position_update, NULL);
	zassert_true(out.velocity_update, NULL);

	motor_outer_loop_sched_step(&in, &state, &out);
	zassert_false(out.position_update, NULL);
	zassert_false(out.velocity_update, NULL);

	motor_outer_loop_sched_step(&in, &state, &out);
	zassert_true(out.position_update, NULL);
	zassert_false(out.velocity_update, NULL);

	motor_outer_loop_sched_step(&in, &state, &out);
	zassert_false(out.position_update, NULL);
	zassert_true(out.velocity_update, NULL);
}

ZTEST(motor_outer_loop_sched, test_sched_handles_null_inputs_without_side_effects)
{
	struct motor_outer_loop_sched_state state = {
		.position_phase = 5U,
		.velocity_phase = 6U,
	};
	struct motor_outer_loop_sched_output out = {
		.position_update = true,
		.velocity_update = true,
	};

	motor_outer_loop_sched_step(NULL, &state, &out);
	zassert_equal(state.position_phase, 5U, NULL);
	zassert_equal(state.velocity_phase, 6U, NULL);
	zassert_true(out.position_update, NULL);
	zassert_true(out.velocity_update, NULL);

	motor_outer_loop_sched_step(&(struct motor_outer_loop_sched_input){0}, NULL, &out);
	motor_outer_loop_sched_step(&(struct motor_outer_loop_sched_input){0}, &state, NULL);
}

ZTEST_SUITE(motor_outer_loop_sched, NULL, NULL, NULL, NULL, NULL);
