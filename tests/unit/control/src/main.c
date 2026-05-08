/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/ztest.h>

#include "motor/control/current_loop.h"
#include "motor/control/dq_decoupling.h"
#include "motor/control/pwm_synthesis.h"
#include "motor/control/speed_limit.h"
#include "motor/control/transforms.h"
#include "motor/protection/interlocks.h"
#include "motor/runtime/command_arbitration.h"
#include "motor/runtime/control_kernel.h"
#include "motor/observers/feedback_quality.h"

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

ZTEST(control_ref_path, test_foc_current_loop_saturates_vq_after_vd_headroom)
{
	struct pi_f32 pi_id = {0};
	struct pi_f32 pi_iq = {0};
	struct motor_current_loop_input in = {
		.id_ref_a = 20.0f,
		.iq_ref_a = 20.0f,
		.id_a = 0.0f,
		.iq_a = 0.0f,
		.max_voltage_magnitude_v = 1.0f,
		.vd_ff_v = 0.0f,
		.vq_ff_v = 0.0f,
	};
	struct motor_current_loop_output out = {0};

	pi_init(&pi_id);
	pi_init(&pi_iq);
	pi_set_gains(&pi_id, 2.0f, 0.0f);
	pi_set_gains(&pi_iq, 2.0f, 0.0f);
	zassert_ok(motor_current_loop_step(&pi_id, &pi_iq, &in, &out), NULL);

	zassert_within(out.vd_v, 1.0f, 1e-6f, NULL);
	zassert_within(out.vq_limit_v, 0.0f, 1e-6f, NULL);
	zassert_within(out.vq_v, 0.0f, 1e-6f, NULL);
}

ZTEST(control_ref_path, test_decoupling_disable_path_is_deterministic_zero_ff)
{
	struct motor_dq_decoupling_feedforward_input in = {
		.enabled = false,
		.electrical_speed_rad_s = NAN,
		.ld_h = NAN,
		.lq_h = NAN,
		.flux_linkage_wb = NAN,
		.id_a = NAN,
		.iq_a = NAN,
		.max_voltage_magnitude_v = NAN,
	};
	struct motor_dq_decoupling_feedforward_output out = {
		.vd_ff_v = 123.0f,
		.vq_ff_v = 456.0f,
	};

	zassert_ok(motor_dq_decoupling_feedforward_step(&in, &out), NULL);
	zassert_within(out.vd_ff_v, 0.0f, 1e-6f, NULL);
	zassert_within(out.vq_ff_v, 0.0f, 1e-6f, NULL);
}

ZTEST(control_ref_path, test_decoupling_enable_gate_requires_all_conditions)
{
	struct motor_dq_decoupling_enable_input in = {
		.feature_enabled = true,
		.online_control_state = true,
		.control_armed = true,
		.current_encoder_mode_state = false,
		.min_speed_reached = true,
		.flux_valid = true,
		.speed_valid = true,
		.feedback_valid = true,
	};

	zassert_true(motor_dq_decoupling_is_enabled(&in), NULL);
	in.speed_valid = false;
	zassert_false(motor_dq_decoupling_is_enabled(&in), NULL);
}

ZTEST(control_ref_path, test_foc_transforms_roundtrip_is_finite)
{
	float32_t id_a = 0.0f;
	float32_t iq_a = 0.0f;
	float32_t va_v = 0.0f;
	float32_t vb_v = 0.0f;
	float32_t angle = 1.0f;

	zassert_ok(motor_transforms_park(0.4f, -0.1f, angle, &id_a, &iq_a), NULL);
	zassert_ok(motor_transforms_inv_park(id_a, iq_a, angle, &va_v, &vb_v), NULL);
	zassert_true(isfinite(va_v), NULL);
	zassert_true(isfinite(vb_v), NULL);
}

ZTEST(control_ref_path, test_pwm_synthesis_braking_clamps_to_unit_interval)
{
	struct motor_pwm_synthesis_input in = {
		.va_v = 5.0f,
		.vb_v = -5.0f,
		.vbus_v = 10.0f,
		.braking_enabled = true,
		.braking_iq_ref_a = -1.0f,
		.braking_speed_rad_s = 10.0f,
		.braking_vbus_limit_v = 5.0f,
		.braking_vbus_margin_inv = 0.5f,
	};
	struct motor_pwm_synthesis_output out = {0};

	zassert_ok(motor_pwm_synthesis_step(&in, &out), NULL);
	zassert_true(out.da_hb1_pu >= 0.0f && out.da_hb1_pu <= 1.0f, NULL);
	zassert_true(out.da_hb2_pu >= 0.0f && out.da_hb2_pu <= 1.0f, NULL);
	zassert_true(out.db_hb1_pu >= 0.0f && out.db_hb1_pu <= 1.0f, NULL);
	zassert_true(out.db_hb2_pu >= 0.0f && out.db_hb2_pu <= 1.0f, NULL);
}

ZTEST(control_ref_path, test_voltage_speed_limit_uses_flux_and_pole_pairs)
{
	const struct motor_voltage_speed_limit_input in = {
		.vbus_v = 23.3f,
		.max_modulation_index = 0.95f,
		.resistance_ohm = 2.26f,
		.inductance_h = 0.002756f,
		.flux_linkage_wb = 0.004418f,
		.current_limit_a = 0.225f,
		.pole_pairs = 50U,
		.safety_factor = 0.75f,
	};
	struct motor_voltage_speed_limit_result out = {0};

	zassert_ok(motor_voltage_speed_limit_compute(&in, &out), NULL);
	zassert_true(out.valid, NULL);
	zassert_true(out.max_mech_hz > 10.0f, "limit=%f", (double)out.max_mech_hz);
	zassert_true(out.max_mech_hz < 13.0f, "limit=%f", (double)out.max_mech_hz);
	zassert_within(out.voltage_limit_v, 16.60125f, 0.001f, NULL);
}

ZTEST(control_ref_path, test_voltage_speed_limit_decreases_with_lower_bus)
{
	struct motor_voltage_speed_limit_input in = {
		.vbus_v = 24.0f,
		.max_modulation_index = 0.95f,
		.resistance_ohm = 2.26f,
		.inductance_h = 0.002756f,
		.flux_linkage_wb = 0.004418f,
		.current_limit_a = 0.225f,
		.pole_pairs = 50U,
		.safety_factor = 0.75f,
	};
	struct motor_voltage_speed_limit_result high = {0};
	struct motor_voltage_speed_limit_result low = {0};

	zassert_ok(motor_voltage_speed_limit_compute(&in, &high), NULL);
	in.vbus_v = 12.0f;
	zassert_ok(motor_voltage_speed_limit_compute(&in, &low), NULL);
	zassert_true(low.max_mech_hz < high.max_mech_hz, NULL);
}

ZTEST(control_ref_path, test_voltage_speed_limit_rejects_unknown_flux)
{
	const struct motor_voltage_speed_limit_input in = {
		.vbus_v = 23.3f,
		.max_modulation_index = 0.95f,
		.resistance_ohm = 2.26f,
		.inductance_h = 0.002756f,
		.flux_linkage_wb = 0.0f,
		.current_limit_a = 0.225f,
		.pole_pairs = 50U,
		.safety_factor = 0.75f,
	};
	struct motor_voltage_speed_limit_result out = {0};

	zassert_not_ok(motor_voltage_speed_limit_compute(&in, &out), NULL);
	zassert_false(out.valid, NULL);
}

ZTEST(control_ref_path, test_feedback_sanity_allows_direct_current_motion_above_profile_limit)
{
	struct motor_control_policy policy = {
		.motion_source = MOTOR_MOTION_SOURCE_HOLD,
		.feedback_source = MOTOR_FEEDBACK_ENCODER,
		.angle_source = MOTOR_ANGLE_SOURCE_ENCODER,
		.current_source = MOTOR_CURRENT_SOURCE_COMMANDED,
		.encoder_required_for_control = true,
	};
	struct motor_feedback_ref feedback = {
		.source = MOTOR_FEEDBACK_ENCODER,
		.input_source = MOTOR_ANGLE_INPUT_SRC_ENCODER,
		.velocity_filtered_rad_s = 20.0f * 2.0f * PI_F32,
		.quality_flags = MOTOR_FEEDBACK_QUALITY_VALID | MOTOR_FEEDBACK_QUALITY_FRESH,
		.error = false,
	};

	zassert_true(motor_control_kernel_feedback_sane(&policy, &feedback,
						       5.0f * 2.0f * PI_F32),
		     NULL);

	policy.motion_source = MOTOR_MOTION_SOURCE_VELOCITY_TRAJ;
	policy.current_source = MOTOR_CURRENT_SOURCE_VELOCITY_LOOP;
	zassert_false(motor_control_kernel_feedback_sane(&policy, &feedback,
							5.0f * 2.0f * PI_F32),
		      NULL);
}

ZTEST_SUITE(control_ref_path, NULL, NULL, NULL, NULL, NULL);
