/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor/math/math_constants.h"
#include "motor/observers/feedback.h"
#include "motor/observers/feedback_quality.h"
#include "motor/runtime/control_kernel.h"
#include "motor/runtime/control_policy.h"

static struct motor_control_policy make_policy(enum motor_control_policy_mode mode)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = mode,
		.features = {
			.encoder_read_enabled = true,
			.angle_gen_enabled = true,
			.velocity_traj_enabled = true,
			.commanded_currents_enabled = true,
			.current_loop_enabled = true,
		},
	};

	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	return policy;
}

static struct motor_feedback_ref valid_encoder_feedback(void)
{
	return (struct motor_feedback_ref){
		.source = MOTOR_FEEDBACK_ENCODER,
		.input_source = MOTOR_ANGLE_INPUT_SRC_ENCODER,
		.quality_flags = MOTOR_FEEDBACK_QUALITY_VALID | MOTOR_FEEDBACK_QUALITY_FRESH,
		.trust_state = MOTOR_FEEDBACK_TRUST_TRUSTED,
		.fresh = true,
		.position_rad = 1.0f,
		.electrical_angle_rad = 2.0f,
		.predicted_electrical_angle_rad = 2.1f,
		.velocity_rad_s = 3.0f,
		.velocity_filtered_rad_s = 3.0f,
	};
}

static struct motor_motion_ref motion_ref(void)
{
	return (struct motor_motion_ref){
		.position_rad = 1.0f,
		.velocity_ref_rad_s = 2.0f,
		.acceleration_rad_s2 = 3.0f,
	};
}

static struct motor_current_ref current_ref(float32_t id_a, float32_t iq_a)
{
	return (struct motor_current_ref){
		.id_ref_a = id_a,
		.iq_ref_a = iq_a,
		.id_meas_a = 0.01f,
		.iq_meas_a = -0.02f,
	};
}

ZTEST(motor_control_kernel, test_generated_mode_builds_foc_current_actuator)
{
	struct motor_control_policy policy = make_policy(MOTOR_CONTROL_POLICY_MODE_VELOCITY_GENERATED);
	struct motor_motion_ref motion = motion_ref();
	struct motor_feedback_ref feedback = valid_encoder_feedback();
	struct motor_current_ref current = current_ref(0.10f, 0.20f);
	struct motor_control_kernel_input in = {
		.policy = &policy,
		.motion_ref = &motion,
		.feedback_ref = &feedback,
		.current_ref = &current,
		.current_loop_enabled = true,
		.feedback_stale_limit = 3U,
		.profile_max_velocity_rad_s = 10.0f,
	};
	struct motor_control_kernel_output out = {0};

	zassert_ok(motor_control_kernel_step_fast(&in, &out), NULL);
	zassert_true(out.feedback_valid, NULL);
	zassert_true(out.feedback_sane, NULL);
	zassert_ok(out.actuator_status, NULL);
	zassert_true(out.servo_ref.enabled, NULL);
	zassert_true(out.actuator_ref.enabled, NULL);
	zassert_equal(out.actuator_ref.kind, MOTOR_ACTUATOR_FOC_CURRENT, NULL);
	zassert_within(out.actuator_ref.id_ref_a, 0.10f, 1.0e-6f, NULL);
	zassert_within(out.actuator_ref.iq_ref_a, 0.20f, 1.0e-6f, NULL);
}

ZTEST(motor_control_kernel, test_disabled_current_loop_disables_actuator)
{
	struct motor_control_policy policy = make_policy(MOTOR_CONTROL_POLICY_MODE_VELOCITY_GENERATED);
	struct motor_motion_ref motion = motion_ref();
	struct motor_feedback_ref feedback = valid_encoder_feedback();
	struct motor_current_ref current = current_ref(0.10f, 0.20f);
	struct motor_control_kernel_input in = {
		.policy = &policy,
		.motion_ref = &motion,
		.feedback_ref = &feedback,
		.current_ref = &current,
		.current_loop_enabled = false,
		.profile_max_velocity_rad_s = 10.0f,
	};
	struct motor_control_kernel_output out = {0};

	zassert_ok(motor_control_kernel_step_fast(&in, &out), NULL);
	zassert_false(out.servo_ref.enabled, NULL);
	zassert_false(out.actuator_ref.enabled, NULL);
	zassert_ok(out.actuator_status, NULL);
}

ZTEST(motor_control_kernel, test_encoder_modes_accept_short_propagated_window)
{
	struct motor_control_policy policy = make_policy(MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER);
	struct motor_feedback_ref feedback = valid_encoder_feedback();

	feedback.input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;
	feedback.source = MOTOR_FEEDBACK_ENCODER;
	feedback.quality_flags = MOTOR_FEEDBACK_QUALITY_VALID;
	feedback.trust_state = MOTOR_FEEDBACK_TRUST_PREDICTED;
	feedback.fresh = false;

	zassert_true(motor_control_kernel_feedback_valid(&policy, &feedback, 2U, 3U), NULL);
	zassert_false(motor_control_kernel_feedback_valid(&policy, &feedback, 4U, 3U), NULL);
}

ZTEST(motor_control_kernel, test_encoder_modes_reject_error_feedback)
{
	struct motor_control_policy policy = make_policy(MOTOR_CONTROL_POLICY_MODE_CURRENT_ENCODER);
	struct motor_feedback_ref feedback = valid_encoder_feedback();

	feedback.error = true;
	zassert_false(motor_control_kernel_feedback_valid(&policy, &feedback, 0U, 3U), NULL);
}

ZTEST(motor_control_kernel, test_encoder_modes_reject_unreasonable_speed)
{
	struct motor_control_policy policy = make_policy(MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER);
	struct motor_feedback_ref feedback = valid_encoder_feedback();

	feedback.velocity_filtered_rad_s = 100.0f;
	zassert_false(motor_control_kernel_feedback_sane(&policy, &feedback, 5.0f), NULL);
	feedback.velocity_filtered_rad_s = 2.0f * PI_F32;
	zassert_true(motor_control_kernel_feedback_sane(&policy, &feedback, 5.0f), NULL);
}

ZTEST(motor_control_kernel, test_unsupported_actuator_pairing_reports_status)
{
	struct motor_control_policy policy = make_policy(MOTOR_CONTROL_POLICY_MODE_STEP_DIR_PROFILE);
	struct motor_motion_ref motion = motion_ref();
	struct motor_feedback_ref feedback = valid_encoder_feedback();
	struct motor_current_ref current = current_ref(0.10f, 0.20f);
	struct motor_control_kernel_input in = {
		.policy = &policy,
		.motion_ref = &motion,
		.feedback_ref = &feedback,
		.current_ref = &current,
		.current_loop_enabled = true,
		.profile_max_velocity_rad_s = 10.0f,
	};
	struct motor_control_kernel_output out = {0};

	zassert_ok(motor_control_kernel_step_fast(&in, &out), NULL);
	zassert_equal(out.actuator_status, -ENOTSUP, NULL);
	zassert_false(out.actuator_ref.enabled, NULL);
}

ZTEST_SUITE(motor_control_kernel, NULL, NULL, NULL, NULL, NULL);
