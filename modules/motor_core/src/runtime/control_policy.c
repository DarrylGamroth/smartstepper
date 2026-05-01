/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/runtime/control_policy.h"

#include <errno.h>
#include <stddef.h>

static void motor_control_policy_set_disabled(struct motor_control_policy *policy)
{
	*policy = (struct motor_control_policy){
		.motion_source = MOTOR_MOTION_SOURCE_HOLD,
		.feedback_source = MOTOR_FEEDBACK_NONE,
		.angle_source = MOTOR_ANGLE_SOURCE_NONE,
		.generated_angle_mode = MOTOR_GENERATED_ANGLE_NONE,
		.current_source = MOTOR_CURRENT_SOURCE_ZERO,
		.actuator_kind = MOTOR_ACTUATOR_FOC_CURRENT,
		.encoder_read_enabled = false,
		.encoder_required_for_control = false,
		.current_loop_enabled = false,
		.generated_angle_position_driven = false,
	};
}

int motor_control_policy_derive(const struct motor_control_policy_input *in,
				struct motor_control_policy *policy)
{
	if (in == NULL || policy == NULL) {
		return -EINVAL;
	}

	motor_control_policy_set_disabled(policy);
	policy->encoder_read_enabled = in->features.encoder_read_enabled;
	policy->current_loop_enabled = in->features.current_loop_enabled;

	switch (in->mode) {
	case MOTOR_CONTROL_POLICY_MODE_DISABLED:
		break;
	case MOTOR_CONTROL_POLICY_MODE_VELOCITY_OPEN:
		policy->motion_source = MOTOR_MOTION_SOURCE_VELOCITY_TRAJ;
		policy->feedback_source = MOTOR_FEEDBACK_GENERATED_MODEL;
		policy->angle_source = MOTOR_ANGLE_SOURCE_GENERATED;
		policy->generated_angle_mode = MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN;
		policy->current_source = MOTOR_CURRENT_SOURCE_COMMANDED;
		policy->encoder_read_enabled = in->features.encoder_read_enabled;
		policy->encoder_required_for_control = false;
		policy->generated_angle_position_driven = false;
		break;
	case MOTOR_CONTROL_POLICY_MODE_PROFILE_OPEN:
		policy->motion_source = in->profile_sequence_active ?
					       MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE :
					       MOTOR_MOTION_SOURCE_PROFILE;
		policy->feedback_source = MOTOR_FEEDBACK_GENERATED_MODEL;
		policy->angle_source = MOTOR_ANGLE_SOURCE_GENERATED;
		policy->generated_angle_mode = MOTOR_GENERATED_ANGLE_POSITION_DRIVEN;
		policy->current_source = MOTOR_CURRENT_SOURCE_COMMANDED;
		policy->encoder_required_for_control = false;
		policy->generated_angle_position_driven = true;
		break;
	case MOTOR_CONTROL_POLICY_MODE_TORQUE:
		policy->motion_source = MOTOR_MOTION_SOURCE_HOLD;
		policy->feedback_source = MOTOR_FEEDBACK_ENCODER;
		policy->angle_source = in->features.encoder_read_enabled ?
					       MOTOR_ANGLE_SOURCE_ENCODER :
					       MOTOR_ANGLE_SOURCE_PROPAGATED;
		policy->current_source = MOTOR_CURRENT_SOURCE_COMMANDED;
		policy->encoder_required_for_control = true;
		break;
	case MOTOR_CONTROL_POLICY_MODE_VELOCITY_CLOSED:
		policy->motion_source = MOTOR_MOTION_SOURCE_VELOCITY_TRAJ;
		policy->feedback_source = MOTOR_FEEDBACK_ENCODER;
		policy->angle_source = in->features.encoder_read_enabled ?
					       MOTOR_ANGLE_SOURCE_ENCODER :
					       MOTOR_ANGLE_SOURCE_PROPAGATED;
		policy->current_source = MOTOR_CURRENT_SOURCE_VELOCITY_LOOP;
		policy->encoder_required_for_control = true;
		break;
	case MOTOR_CONTROL_POLICY_MODE_POSITION:
		policy->motion_source = MOTOR_MOTION_SOURCE_PROFILE;
		policy->feedback_source = MOTOR_FEEDBACK_ENCODER;
		policy->angle_source = in->features.encoder_read_enabled ?
					       MOTOR_ANGLE_SOURCE_ENCODER :
					       MOTOR_ANGLE_SOURCE_PROPAGATED;
		policy->current_source = MOTOR_CURRENT_SOURCE_POSITION_LOOP;
		policy->encoder_required_for_control = true;
		break;
	case MOTOR_CONTROL_POLICY_MODE_STEP_DIR_PROFILE:
		policy->motion_source = in->profile_sequence_active ?
					       MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE :
					       MOTOR_MOTION_SOURCE_PROFILE;
		policy->feedback_source = in->features.encoder_read_enabled ?
						  MOTOR_FEEDBACK_ENCODER :
						  MOTOR_FEEDBACK_NONE;
		policy->angle_source = MOTOR_ANGLE_SOURCE_NONE;
		policy->current_source = MOTOR_CURRENT_SOURCE_ZERO;
		policy->actuator_kind = MOTOR_ACTUATOR_STEP_DIR;
		policy->encoder_required_for_control = false;
		policy->current_loop_enabled = false;
		break;
	case MOTOR_CONTROL_POLICY_MODE_CALIBRATION:
		policy->motion_source = MOTOR_MOTION_SOURCE_HOLD;
		policy->feedback_source = in->features.encoder_read_enabled ?
						  MOTOR_FEEDBACK_ENCODER :
						  MOTOR_FEEDBACK_GENERATED_MODEL;
		policy->angle_source = in->features.angle_gen_enabled ?
					       MOTOR_ANGLE_SOURCE_GENERATED :
					       MOTOR_ANGLE_SOURCE_ENCODER;
		policy->generated_angle_mode = in->features.angle_gen_enabled ?
						       MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN :
						       MOTOR_GENERATED_ANGLE_NONE;
		policy->current_source = MOTOR_CURRENT_SOURCE_CALIBRATION;
		policy->encoder_required_for_control = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct motor_actuator_caps motor_actuator_caps_for_kind(enum motor_actuator_kind kind)
{
	switch (kind) {
	case MOTOR_ACTUATOR_FOC_CURRENT:
		return (struct motor_actuator_caps){
			.accepts_torque = true,
			.accepts_current = true,
			.requires_commutation_angle = true,
			.supports_open_loop = true,
			.supports_closed_loop = true,
		};
	case MOTOR_ACTUATOR_BRUSHED_CURRENT:
		return (struct motor_actuator_caps){
			.accepts_torque = true,
			.accepts_current = true,
			.requires_commutation_angle = false,
			.supports_open_loop = true,
			.supports_closed_loop = true,
		};
	case MOTOR_ACTUATOR_BRUSHED_VOLTAGE:
		return (struct motor_actuator_caps){
			.accepts_torque = true,
			.accepts_voltage = true,
			.requires_commutation_angle = false,
			.supports_open_loop = true,
			.supports_closed_loop = true,
		};
	case MOTOR_ACTUATOR_STEP_DIR:
		return (struct motor_actuator_caps){
			.accepts_position = true,
			.accepts_velocity = true,
			.accepts_step_dir = true,
			.requires_commutation_angle = false,
			.supports_open_loop = true,
			.supports_closed_loop = true,
		};
	default:
		return (struct motor_actuator_caps){0};
	}
}

bool motor_control_policy_is_valid(const struct motor_control_policy *policy,
				   const struct motor_actuator_caps *caps)
{
	if (policy == NULL || caps == NULL) {
		return false;
	}

	if (caps->requires_commutation_angle &&
	    policy->angle_source == MOTOR_ANGLE_SOURCE_NONE) {
		return false;
	}

	switch (policy->current_source) {
	case MOTOR_CURRENT_SOURCE_ZERO:
		break;
	case MOTOR_CURRENT_SOURCE_COMMANDED:
	case MOTOR_CURRENT_SOURCE_CALIBRATION:
		if (!caps->accepts_current) {
			return false;
		}
		break;
	case MOTOR_CURRENT_SOURCE_VELOCITY_LOOP:
	case MOTOR_CURRENT_SOURCE_POSITION_LOOP:
		if (!caps->accepts_current && !caps->accepts_torque &&
		    !caps->accepts_voltage) {
			return false;
		}
		break;
	default:
		return false;
	}

	if (policy->actuator_kind == MOTOR_ACTUATOR_STEP_DIR &&
	    policy->current_source != MOTOR_CURRENT_SOURCE_ZERO) {
		return false;
	}

	if (policy->angle_source != MOTOR_ANGLE_SOURCE_NONE &&
	    !caps->requires_commutation_angle &&
	    policy->actuator_kind == MOTOR_ACTUATOR_STEP_DIR) {
		return false;
	}

	if (policy->encoder_required_for_control &&
	    policy->feedback_source != MOTOR_FEEDBACK_ENCODER &&
	    policy->feedback_source != MOTOR_FEEDBACK_SENSORLESS_OBSERVER) {
		return false;
	}

	return true;
}

const char *motor_motion_source_to_string(enum motor_motion_source source)
{
	switch (source) {
	case MOTOR_MOTION_SOURCE_HOLD: return "hold";
	case MOTOR_MOTION_SOURCE_VELOCITY_TRAJ: return "velocity_traj";
	case MOTOR_MOTION_SOURCE_PROFILE: return "profile";
	case MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE: return "profile_sequence";
	default: return "unknown";
	}
}

const char *motor_feedback_source_to_string(enum motor_feedback_source source)
{
	switch (source) {
	case MOTOR_FEEDBACK_NONE: return "none";
	case MOTOR_FEEDBACK_ENCODER: return "encoder";
	case MOTOR_FEEDBACK_SENSORLESS_OBSERVER: return "sensorless_observer";
	case MOTOR_FEEDBACK_GENERATED_MODEL: return "generated_model";
	default: return "unknown";
	}
}

const char *motor_angle_source_to_string(enum motor_angle_source source)
{
	switch (source) {
	case MOTOR_ANGLE_SOURCE_NONE: return "none";
	case MOTOR_ANGLE_SOURCE_GENERATED: return "generated";
	case MOTOR_ANGLE_SOURCE_ENCODER: return "encoder";
	case MOTOR_ANGLE_SOURCE_PROPAGATED: return "propagated";
	default: return "unknown";
	}
}

const char *motor_generated_angle_mode_to_string(enum motor_generated_angle_mode mode)
{
	switch (mode) {
	case MOTOR_GENERATED_ANGLE_NONE: return "none";
	case MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN: return "velocity";
	case MOTOR_GENERATED_ANGLE_POSITION_DRIVEN: return "position";
	default: return "unknown";
	}
}

const char *motor_current_source_to_string(enum motor_current_source source)
{
	switch (source) {
	case MOTOR_CURRENT_SOURCE_ZERO: return "zero";
	case MOTOR_CURRENT_SOURCE_COMMANDED: return "commanded";
	case MOTOR_CURRENT_SOURCE_VELOCITY_LOOP: return "velocity_loop";
	case MOTOR_CURRENT_SOURCE_POSITION_LOOP: return "position_loop";
	case MOTOR_CURRENT_SOURCE_CALIBRATION: return "calibration";
	default: return "unknown";
	}
}

const char *motor_actuator_kind_to_string(enum motor_actuator_kind kind)
{
	switch (kind) {
	case MOTOR_ACTUATOR_FOC_CURRENT: return "foc_current";
	case MOTOR_ACTUATOR_BRUSHED_CURRENT: return "brushed_current";
	case MOTOR_ACTUATOR_BRUSHED_VOLTAGE: return "brushed_voltage";
	case MOTOR_ACTUATOR_STEP_DIR: return "step_dir";
	default: return "unknown";
	}
}
