/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor_current_ref_policy_core.h"

static struct motor_current_ref_policy_core_input base_input(void)
{
	struct motor_current_ref_policy_core_input in = {
		.online_control_state = true,
		.control_armed = true,
		.feature_angle_gen = false,
		.feature_use_commanded_currents = false,
		.feedback_valid = true,
		.id_meas_a = -0.40f,
		.iq_meas_a = 0.15f,
		.id_setpoint_a = 0.50f,
		.iq_setpoint_a = -0.70f,
		.id_ref_in_a = 0.21f,
		.iq_ref_in_a = -0.33f,
	};

	return in;
}

ZTEST(motor_current_ref_policy_core, test_default_passthrough_when_no_command_override)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.feature_use_commanded_currents = false;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_ref_in_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_ref_in_a, 1e-6f, NULL);
	zassert_false(out.reset_current_pi, NULL);
	zassert_false(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_commanded_currents_use_setpoints_when_valid)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.feature_use_commanded_currents = true;
	in.feedback_valid = true;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_setpoint_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_setpoint_a, 1e-6f, NULL);
	zassert_false(out.reset_current_pi, NULL);
	zassert_false(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_commanded_currents_hold_measured_if_closed_loop_feedback_invalid)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.feature_use_commanded_currents = true;
	in.feature_angle_gen = false;
	in.feedback_valid = false;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_meas_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_meas_a, 1e-6f, NULL);
	zassert_true(out.reset_current_pi, NULL);
	zassert_false(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_commanded_currents_ignore_feedback_if_angle_gen_active)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.feature_use_commanded_currents = true;
	in.feature_angle_gen = true;
	in.feedback_valid = false;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_setpoint_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_setpoint_a, 1e-6f, NULL);
	zassert_false(out.reset_current_pi, NULL);
	zassert_false(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_commanded_currents_ignore_feedback_if_offline)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.online_control_state = false;
	in.feature_use_commanded_currents = true;
	in.feature_angle_gen = false;
	in.feedback_valid = false;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_setpoint_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_setpoint_a, 1e-6f, NULL);
	zassert_false(out.reset_current_pi, NULL);
	zassert_false(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_disarmed_interlock_always_holds_measured)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.control_armed = false;
	in.feature_use_commanded_currents = false;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_meas_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_meas_a, 1e-6f, NULL);
	zassert_true(out.reset_current_pi, NULL);
	zassert_true(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_disarmed_interlock_overrides_commanded_invalid_path)
{
	struct motor_current_ref_policy_core_input in = base_input();
	struct motor_current_ref_policy_core_output out = {0};

	in.control_armed = false;
	in.feature_use_commanded_currents = true;
	in.feedback_valid = false;
	motor_current_ref_policy_core_apply(&in, &out);

	zassert_within(out.id_ref_a, in.id_meas_a, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, in.iq_meas_a, 1e-6f, NULL);
	zassert_true(out.reset_current_pi, NULL);
	zassert_true(out.disarmed_interlock_active, NULL);
}

ZTEST(motor_current_ref_policy_core, test_null_arguments_are_noop)
{
	struct motor_current_ref_policy_core_output out = {
		.id_ref_a = 1.0f,
		.iq_ref_a = -1.0f,
		.reset_current_pi = true,
		.disarmed_interlock_active = true,
	};

	motor_current_ref_policy_core_apply(NULL, &out);
	zassert_within(out.id_ref_a, 1.0f, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, -1.0f, 1e-6f, NULL);
	zassert_true(out.reset_current_pi, NULL);
	zassert_true(out.disarmed_interlock_active, NULL);

	motor_current_ref_policy_core_apply(&(struct motor_current_ref_policy_core_input){0}, NULL);
}

ZTEST_SUITE(motor_current_ref_policy_core, NULL, NULL, NULL, NULL, NULL);
