/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor/control/position_regulator.h"
#include "motor/control/velocity_regulator.h"

ZTEST(pi_controller, test_position_regulator_step_includes_ff_and_pi_terms)
{
	struct motor_position_regulator_config cfg = {
		.kp_rad_s_per_rad = 2.0f,
		.ki_rad_s2_per_rad = 4.0f,
		.integrator_limit_rad_s = 5.0f,
		.output_limit_rad_s = 10.0f,
	};
	struct motor_position_regulator_state state = {0};
	float32_t cmd = 0.0f;

	zassert_ok(motor_position_regulator_step(&cfg, &state, 0.5f, 1.0f, 0.1f, &cmd), NULL);
	zassert_within(state.integrator_rad_s, 0.2f, 1e-6f, NULL);
	zassert_within(cmd, 2.2f, 1e-6f, NULL);
}

ZTEST(pi_controller, test_position_regulator_clamps_integrator_and_output)
{
	struct motor_position_regulator_config cfg = {
		.kp_rad_s_per_rad = 0.0f,
		.ki_rad_s2_per_rad = 20.0f,
		.integrator_limit_rad_s = 0.3f,
		.output_limit_rad_s = 0.2f,
	};
	struct motor_position_regulator_state state = {0};
	float32_t cmd = 0.0f;

	for (int i = 0; i < 8; i++) {
		zassert_ok(motor_position_regulator_step(&cfg, &state, 1.0f, 0.0f, 0.1f, &cmd),
			   NULL);
	}

	zassert_within(state.integrator_rad_s, 0.3f, 1e-6f, NULL);
	zassert_within(cmd, 0.2f, 1e-6f, NULL);
}

ZTEST(pi_controller, test_velocity_regulator_step_and_unwind)
{
	struct motor_velocity_regulator_config cfg = {
		.kp_a_per_rad_s = 1.0f,
		.ki_a_per_rad = 5.0f,
		.integrator_limit_a = 2.0f,
		.output_limit_a = 2.0f,
	};
	struct motor_velocity_regulator_state state = {0};
	float32_t cmd = 0.0f;

	zassert_ok(motor_velocity_regulator_step(&cfg, &state, 0.4f, 0.1f, &cmd), NULL);
	zassert_within(state.integrator_a, 0.2f, 1e-6f, NULL);
	zassert_within(cmd, 0.6f, 1e-6f, NULL);

	zassert_ok(motor_velocity_regulator_step(&cfg, &state, -0.4f, 0.1f, &cmd), NULL);
	zassert_within(state.integrator_a, 0.0f, 1e-6f, NULL);
	zassert_within(cmd, -0.4f, 1e-6f, NULL);
}

ZTEST(pi_controller, test_velocity_regulator_limits_iq_command)
{
	struct motor_velocity_regulator_config cfg = {
		.kp_a_per_rad_s = 10.0f,
		.ki_a_per_rad = 10.0f,
		.integrator_limit_a = 3.0f,
		.output_limit_a = 0.5f,
	};
	struct motor_velocity_regulator_state state = {0};
	float32_t cmd = 0.0f;

	zassert_ok(motor_velocity_regulator_step(&cfg, &state, 1.0f, 0.1f, &cmd), NULL);
	zassert_within(cmd, 0.5f, 1e-6f, NULL);
}

ZTEST(pi_controller, test_regulator_validate_rejects_invalid_inputs)
{
	struct motor_position_regulator_config pos_cfg = {
		.kp_rad_s_per_rad = 1.0f,
		.ki_rad_s2_per_rad = 1.0f,
		.integrator_limit_rad_s = -1.0f,
		.output_limit_rad_s = 1.0f,
	};
	struct motor_velocity_regulator_config vel_cfg = {
		.kp_a_per_rad_s = 1.0f,
		.ki_a_per_rad = 1.0f,
		.integrator_limit_a = 1.0f,
		.output_limit_a = 0.0f,
	};

	zassert_equal(motor_position_regulator_validate(&pos_cfg), -EINVAL, NULL);
	zassert_equal(motor_velocity_regulator_validate(&vel_cfg), -EINVAL, NULL);
}

ZTEST(pi_controller, test_regulator_step_rejects_nonfinite_or_bad_dt)
{
	struct motor_position_regulator_config pos_cfg = {
		.kp_rad_s_per_rad = 1.0f,
		.ki_rad_s2_per_rad = 1.0f,
		.integrator_limit_rad_s = 1.0f,
		.output_limit_rad_s = 1.0f,
	};
	struct motor_velocity_regulator_config vel_cfg = {
		.kp_a_per_rad_s = 1.0f,
		.ki_a_per_rad = 1.0f,
		.integrator_limit_a = 1.0f,
		.output_limit_a = 1.0f,
	};
	struct motor_position_regulator_state pos_state = {0};
	struct motor_velocity_regulator_state vel_state = {0};
	float32_t out = 0.0f;

	zassert_equal(motor_position_regulator_step(&pos_cfg, &pos_state, NAN, 0.0f, 0.001f, &out),
		      -EINVAL, NULL);
	zassert_equal(motor_velocity_regulator_step(&vel_cfg, &vel_state, 0.1f, 0.0f, &out),
		      -EINVAL, NULL);
}

ZTEST_SUITE(pi_controller, NULL, NULL, NULL, NULL, NULL);
