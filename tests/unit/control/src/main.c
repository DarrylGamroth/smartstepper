/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/protection/interlocks.h"
#include "motor/runtime/command_arbitration.h"

ZTEST(control_ref_path, test_arbitration_passthrough_when_commanded_currents_disabled)
{
	struct motor_command_arbitration_input in = {
		.online_control_state = true,
		.feature_angle_gen = false,
		.feature_use_commanded_currents = false,
		.feedback_valid = false,
		.id_meas_a = 0.4f,
		.iq_meas_a = 0.5f,
		.id_setpoint_a = 0.6f,
		.iq_setpoint_a = 0.7f,
		.id_ref_in_a = 0.1f,
		.iq_ref_in_a = 0.2f,
	};
	struct motor_command_arbitration_output out = {0};

	motor_command_arbitration_apply(&in, &out);
	zassert_within(out.id_ref_a, 0.1f, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, 0.2f, 1e-6f, NULL);
	zassert_false(out.reset_current_pi, NULL);
}

ZTEST(control_ref_path, test_arbitration_uses_setpoints_when_feedback_valid)
{
	struct motor_command_arbitration_input in = {
		.online_control_state = true,
		.feature_angle_gen = false,
		.feature_use_commanded_currents = true,
		.feedback_valid = true,
		.id_meas_a = 0.0f,
		.iq_meas_a = 0.0f,
		.id_setpoint_a = 0.3f,
		.iq_setpoint_a = 0.4f,
		.id_ref_in_a = 0.0f,
		.iq_ref_in_a = 0.0f,
	};
	struct motor_command_arbitration_output out = {0};

	motor_command_arbitration_apply(&in, &out);
	zassert_within(out.id_ref_a, 0.3f, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, 0.4f, 1e-6f, NULL);
	zassert_false(out.reset_current_pi, NULL);
}

ZTEST(control_ref_path, test_arbitration_holds_measured_on_invalid_encoder_feedback)
{
	struct motor_command_arbitration_input in = {
		.online_control_state = true,
		.feature_angle_gen = false,
		.feature_use_commanded_currents = true,
		.feedback_valid = false,
		.id_meas_a = -0.2f,
		.iq_meas_a = 0.9f,
		.id_setpoint_a = 0.3f,
		.iq_setpoint_a = 0.4f,
		.id_ref_in_a = 0.0f,
		.iq_ref_in_a = 0.0f,
	};
	struct motor_command_arbitration_output out = {0};

	motor_command_arbitration_apply(&in, &out);
	zassert_within(out.id_ref_a, -0.2f, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, 0.9f, 1e-6f, NULL);
	zassert_true(out.reset_current_pi, NULL);
}

ZTEST(control_ref_path, test_disarmed_interlock_forces_measured_currents)
{
	struct motor_current_interlock_input in = {
		.online_control_state = true,
		.control_armed = false,
		.id_meas_a = 0.11f,
		.iq_meas_a = -0.22f,
		.id_ref_in_a = 0.7f,
		.iq_ref_in_a = 0.8f,
	};
	struct motor_current_interlock_output out = {0};

	motor_interlocks_apply_current(&in, &out);
	zassert_within(out.id_ref_a, 0.11f, 1e-6f, NULL);
	zassert_within(out.iq_ref_a, -0.22f, 1e-6f, NULL);
	zassert_true(out.reset_current_pi, NULL);
	zassert_true(out.disarmed_interlock_active, NULL);
}

ZTEST(control_ref_path, test_timeout_interlock_disarms_when_threshold_exceeded)
{
	struct motor_timeout_interlock_input in = {
		.online_control_state = true,
		.control_armed = true,
		.autonomous_keepalive = false,
		.command_timeout_ms = 1000U,
		.now_ms = 2501U,
		.last_command_update_ms = 1500U,
	};
	struct motor_timeout_interlock_output out = {0};

	motor_interlocks_eval_timeout(&in, &out);
	zassert_true(out.disarm_control, NULL);
}

ZTEST(control_ref_path, test_timeout_interlock_does_not_disarm_with_keepalive_or_boundary)
{
	struct motor_timeout_interlock_input in = {
		.online_control_state = true,
		.control_armed = true,
		.autonomous_keepalive = true,
		.command_timeout_ms = 1000U,
		.now_ms = 2600U,
		.last_command_update_ms = 1500U,
	};
	struct motor_timeout_interlock_output out = {0};

	motor_interlocks_eval_timeout(&in, &out);
	zassert_false(out.disarm_control, NULL);

	in.autonomous_keepalive = false;
	in.now_ms = 2500U; /* elapsed == timeout, should not disarm */
	motor_interlocks_eval_timeout(&in, &out);
	zassert_false(out.disarm_control, NULL);
}

ZTEST(control_ref_path, test_reference_path_priority_disarm_overrides_setpoint_path)
{
	struct motor_command_arbitration_input arb_in = {
		.online_control_state = true,
		.feature_angle_gen = true,
		.feature_use_commanded_currents = true,
		.feedback_valid = true,
		.id_meas_a = 0.4f,
		.iq_meas_a = 0.5f,
		.id_setpoint_a = 0.8f,
		.iq_setpoint_a = 0.9f,
		.id_ref_in_a = 0.0f,
		.iq_ref_in_a = 0.0f,
	};
	struct motor_command_arbitration_output arb_out = {0};
	struct motor_current_interlock_input int_in = {0};
	struct motor_current_interlock_output int_out = {0};

	motor_command_arbitration_apply(&arb_in, &arb_out);
	zassert_within(arb_out.id_ref_a, 0.8f, 1e-6f, NULL);
	zassert_within(arb_out.iq_ref_a, 0.9f, 1e-6f, NULL);

	int_in.online_control_state = true;
	int_in.control_armed = false;
	int_in.id_meas_a = 0.4f;
	int_in.iq_meas_a = 0.5f;
	int_in.id_ref_in_a = arb_out.id_ref_a;
	int_in.iq_ref_in_a = arb_out.iq_ref_a;
	motor_interlocks_apply_current(&int_in, &int_out);

	zassert_within(int_out.id_ref_a, 0.4f, 1e-6f, NULL);
	zassert_within(int_out.iq_ref_a, 0.5f, 1e-6f, NULL);
	zassert_true(int_out.disarmed_interlock_active, NULL);
}

ZTEST_SUITE(control_ref_path, NULL, NULL, NULL, NULL, NULL);
