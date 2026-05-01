/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/runtime/control_policy.h"

static const struct motor_control_policy_features foc_open_features = {
	.encoder_read_enabled = false,
	.angle_gen_enabled = true,
	.velocity_traj_enabled = true,
	.commanded_currents_enabled = true,
	.current_loop_enabled = true,
};

static const struct motor_control_policy_features foc_closed_features = {
	.encoder_read_enabled = true,
	.angle_gen_enabled = false,
	.velocity_traj_enabled = true,
	.commanded_currents_enabled = false,
	.current_loop_enabled = true,
};

static void expect_policy_valid(const struct motor_control_policy *policy)
{
	struct motor_actuator_caps caps = motor_actuator_caps_for_kind(policy->actuator_kind);

	zassert_true(motor_control_policy_is_valid(policy, &caps), NULL);
}

ZTEST(motor_control_policy, test_velocity_open_maps_to_generated_angle_foc_current)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_OPEN,
		.features = foc_open_features,
	};

	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	zassert_equal(policy.motion_source, MOTOR_MOTION_SOURCE_VELOCITY_TRAJ, NULL);
	zassert_equal(policy.feedback_source, MOTOR_FEEDBACK_GENERATED_MODEL, NULL);
	zassert_equal(policy.angle_source, MOTOR_ANGLE_SOURCE_GENERATED, NULL);
	zassert_equal(policy.generated_angle_mode, MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN, NULL);
	zassert_equal(policy.current_source, MOTOR_CURRENT_SOURCE_COMMANDED, NULL);
	zassert_equal(policy.actuator_kind, MOTOR_ACTUATOR_FOC_CURRENT, NULL);
	zassert_false(policy.encoder_required_for_control, NULL);
	zassert_true(policy.current_loop_enabled, NULL);
	zassert_false(policy.generated_angle_position_driven, NULL);
	expect_policy_valid(&policy);
}

ZTEST(motor_control_policy, test_profile_open_maps_to_position_driven_generated_angle)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_PROFILE_OPEN,
		.features = foc_open_features,
		.profile_sequence_active = true,
	};

	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	zassert_equal(policy.motion_source, MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE, NULL);
	zassert_equal(policy.feedback_source, MOTOR_FEEDBACK_GENERATED_MODEL, NULL);
	zassert_equal(policy.angle_source, MOTOR_ANGLE_SOURCE_GENERATED, NULL);
	zassert_equal(policy.generated_angle_mode, MOTOR_GENERATED_ANGLE_POSITION_DRIVEN, NULL);
	zassert_equal(policy.current_source, MOTOR_CURRENT_SOURCE_COMMANDED, NULL);
	zassert_false(policy.encoder_required_for_control, NULL);
	zassert_true(policy.generated_angle_position_driven, NULL);
	expect_policy_valid(&policy);
}

ZTEST(motor_control_policy, test_profile_open_does_not_require_encoder_when_encoder_disabled)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_PROFILE_OPEN,
		.features = foc_open_features,
		.profile_sequence_active = false,
	};

	in.features.encoder_read_enabled = false;
	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	zassert_equal(policy.motion_source, MOTOR_MOTION_SOURCE_PROFILE, NULL);
	zassert_equal(policy.feedback_source, MOTOR_FEEDBACK_GENERATED_MODEL, NULL);
	zassert_equal(policy.angle_source, MOTOR_ANGLE_SOURCE_GENERATED, NULL);
	zassert_equal(policy.generated_angle_mode, MOTOR_GENERATED_ANGLE_POSITION_DRIVEN, NULL);
	zassert_false(policy.encoder_read_enabled, NULL);
	zassert_false(policy.encoder_required_for_control, NULL);
	zassert_true(policy.generated_angle_position_driven, NULL);
	expect_policy_valid(&policy);
}

ZTEST(motor_control_policy, test_torque_maps_to_encoder_required_commanded_current)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_TORQUE,
		.features = foc_closed_features,
	};

	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	zassert_equal(policy.motion_source, MOTOR_MOTION_SOURCE_HOLD, NULL);
	zassert_equal(policy.feedback_source, MOTOR_FEEDBACK_ENCODER, NULL);
	zassert_equal(policy.angle_source, MOTOR_ANGLE_SOURCE_ENCODER, NULL);
	zassert_equal(policy.generated_angle_mode, MOTOR_GENERATED_ANGLE_NONE, NULL);
	zassert_equal(policy.current_source, MOTOR_CURRENT_SOURCE_COMMANDED, NULL);
	zassert_true(policy.encoder_required_for_control, NULL);
	expect_policy_valid(&policy);
}

ZTEST(motor_control_policy, test_closed_loop_without_encoder_read_still_requires_control_feedback)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_CLOSED,
		.features = foc_closed_features,
	};

	in.features.encoder_read_enabled = false;
	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	zassert_equal(policy.feedback_source, MOTOR_FEEDBACK_ENCODER, NULL);
	zassert_equal(policy.angle_source, MOTOR_ANGLE_SOURCE_PROPAGATED, NULL);
	zassert_true(policy.encoder_required_for_control, NULL);
	expect_policy_valid(&policy);
}

ZTEST(motor_control_policy, test_closed_loop_modes_require_encoder_feedback)
{
	struct motor_control_policy vel = {0};
	struct motor_control_policy pos = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_CLOSED,
		.features = foc_closed_features,
	};

	zassert_ok(motor_control_policy_derive(&in, &vel), NULL);
	zassert_equal(vel.motion_source, MOTOR_MOTION_SOURCE_VELOCITY_TRAJ, NULL);
	zassert_equal(vel.feedback_source, MOTOR_FEEDBACK_ENCODER, NULL);
	zassert_equal(vel.angle_source, MOTOR_ANGLE_SOURCE_ENCODER, NULL);
	zassert_equal(vel.current_source, MOTOR_CURRENT_SOURCE_VELOCITY_LOOP, NULL);
	zassert_true(vel.encoder_required_for_control, NULL);
	expect_policy_valid(&vel);

	in.mode = MOTOR_CONTROL_POLICY_MODE_POSITION;
	zassert_ok(motor_control_policy_derive(&in, &pos), NULL);
	zassert_equal(pos.motion_source, MOTOR_MOTION_SOURCE_PROFILE, NULL);
	zassert_equal(pos.feedback_source, MOTOR_FEEDBACK_ENCODER, NULL);
	zassert_equal(pos.angle_source, MOTOR_ANGLE_SOURCE_ENCODER, NULL);
	zassert_equal(pos.current_source, MOTOR_CURRENT_SOURCE_POSITION_LOOP, NULL);
	zassert_true(pos.encoder_required_for_control, NULL);
	expect_policy_valid(&pos);
}

ZTEST(motor_control_policy, test_step_dir_profile_rejects_current_and_angle_pairings)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_STEP_DIR_PROFILE,
		.features = {
			.encoder_read_enabled = false,
			.current_loop_enabled = false,
		},
		.profile_sequence_active = true,
	};
	struct motor_actuator_caps caps;

	zassert_ok(motor_control_policy_derive(&in, &policy), NULL);
	zassert_equal(policy.motion_source, MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE, NULL);
	zassert_equal(policy.feedback_source, MOTOR_FEEDBACK_NONE, NULL);
	zassert_equal(policy.angle_source, MOTOR_ANGLE_SOURCE_NONE, NULL);
	zassert_equal(policy.current_source, MOTOR_CURRENT_SOURCE_ZERO, NULL);
	zassert_equal(policy.actuator_kind, MOTOR_ACTUATOR_STEP_DIR, NULL);
	expect_policy_valid(&policy);

	caps = motor_actuator_caps_for_kind(MOTOR_ACTUATOR_STEP_DIR);
	policy.current_source = MOTOR_CURRENT_SOURCE_COMMANDED;
	zassert_false(motor_control_policy_is_valid(&policy, &caps), NULL);

	policy.current_source = MOTOR_CURRENT_SOURCE_ZERO;
	policy.angle_source = MOTOR_ANGLE_SOURCE_GENERATED;
	zassert_false(motor_control_policy_is_valid(&policy, &caps), NULL);
}

ZTEST(motor_control_policy, test_foc_current_requires_commutation_angle)
{
	struct motor_control_policy policy = {
		.motion_source = MOTOR_MOTION_SOURCE_HOLD,
		.feedback_source = MOTOR_FEEDBACK_ENCODER,
		.angle_source = MOTOR_ANGLE_SOURCE_NONE,
		.current_source = MOTOR_CURRENT_SOURCE_COMMANDED,
		.actuator_kind = MOTOR_ACTUATOR_FOC_CURRENT,
		.encoder_required_for_control = true,
		.current_loop_enabled = true,
	};
	struct motor_actuator_caps caps = motor_actuator_caps_for_kind(policy.actuator_kind);

	zassert_false(motor_control_policy_is_valid(&policy, &caps), NULL);
	policy.angle_source = MOTOR_ANGLE_SOURCE_ENCODER;
	zassert_true(motor_control_policy_is_valid(&policy, &caps), NULL);
}

ZTEST(motor_control_policy, test_string_helpers_cover_public_enums)
{
	zassert_str_equal(motor_motion_source_to_string(MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE),
			  "profile_sequence", NULL);
	zassert_str_equal(motor_feedback_source_to_string(MOTOR_FEEDBACK_GENERATED_MODEL),
			  "generated_model", NULL);
	zassert_str_equal(motor_angle_source_to_string(MOTOR_ANGLE_SOURCE_PROPAGATED),
			  "propagated", NULL);
	zassert_str_equal(motor_generated_angle_mode_to_string(MOTOR_GENERATED_ANGLE_POSITION_DRIVEN),
			  "position", NULL);
	zassert_str_equal(motor_current_source_to_string(MOTOR_CURRENT_SOURCE_POSITION_LOOP),
			  "position_loop", NULL);
	zassert_str_equal(motor_actuator_kind_to_string(MOTOR_ACTUATOR_STEP_DIR),
			  "step_dir", NULL);
}

ZTEST(motor_control_policy, test_invalid_arguments_are_rejected)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input in = {
		.mode = MOTOR_CONTROL_POLICY_MODE_DISABLED,
	};
	struct motor_actuator_caps caps = motor_actuator_caps_for_kind(MOTOR_ACTUATOR_FOC_CURRENT);

	zassert_equal(motor_control_policy_derive(NULL, &policy), -EINVAL, NULL);
	zassert_equal(motor_control_policy_derive(&in, NULL), -EINVAL, NULL);
	in.mode = (enum motor_control_policy_mode)UINT8_MAX;
	zassert_equal(motor_control_policy_derive(&in, &policy), -EINVAL, NULL);
	zassert_false(motor_control_policy_is_valid(NULL, &caps), NULL);
	zassert_false(motor_control_policy_is_valid(&policy, NULL), NULL);
}

ZTEST_SUITE(motor_control_policy, NULL, NULL, NULL, NULL, NULL);
