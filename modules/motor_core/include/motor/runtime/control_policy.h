/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_CONTROL_POLICY_H_
#define MOTOR_RUNTIME_CONTROL_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_control_policy_mode {
	MOTOR_CONTROL_POLICY_MODE_DISABLED = 0,
	MOTOR_CONTROL_POLICY_MODE_VELOCITY_OPEN,
	MOTOR_CONTROL_POLICY_MODE_PROFILE_OPEN,
	MOTOR_CONTROL_POLICY_MODE_TORQUE,
	MOTOR_CONTROL_POLICY_MODE_VELOCITY_CLOSED,
	MOTOR_CONTROL_POLICY_MODE_POSITION,
	MOTOR_CONTROL_POLICY_MODE_STEP_DIR_PROFILE,
	MOTOR_CONTROL_POLICY_MODE_CALIBRATION,
};

enum motor_motion_source {
	MOTOR_MOTION_SOURCE_HOLD = 0,
	MOTOR_MOTION_SOURCE_VELOCITY_TRAJ,
	MOTOR_MOTION_SOURCE_PROFILE,
	MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE,
};

enum motor_angle_source {
	MOTOR_ANGLE_SOURCE_NONE = 0,
	MOTOR_ANGLE_SOURCE_GENERATED,
	MOTOR_ANGLE_SOURCE_ENCODER,
	MOTOR_ANGLE_SOURCE_PROPAGATED,
};

enum motor_generated_angle_mode {
	MOTOR_GENERATED_ANGLE_NONE = 0,
	MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN,
	MOTOR_GENERATED_ANGLE_POSITION_DRIVEN,
};

enum motor_feedback_source {
	MOTOR_FEEDBACK_NONE = 0,
	MOTOR_FEEDBACK_ENCODER,
	MOTOR_FEEDBACK_SENSORLESS_OBSERVER,
	MOTOR_FEEDBACK_GENERATED_MODEL,
};

enum motor_current_source {
	MOTOR_CURRENT_SOURCE_ZERO = 0,
	MOTOR_CURRENT_SOURCE_COMMANDED,
	MOTOR_CURRENT_SOURCE_VELOCITY_LOOP,
	MOTOR_CURRENT_SOURCE_POSITION_LOOP,
	MOTOR_CURRENT_SOURCE_CALIBRATION,
};

enum motor_actuator_kind {
	MOTOR_ACTUATOR_FOC_CURRENT = 0,
	MOTOR_ACTUATOR_BRUSHED_CURRENT,
	MOTOR_ACTUATOR_BRUSHED_VOLTAGE,
	MOTOR_ACTUATOR_STEP_DIR,
};

struct motor_control_policy_features {
	bool encoder_read_enabled;
	bool angle_gen_enabled;
	bool velocity_traj_enabled;
	bool commanded_currents_enabled;
	bool current_loop_enabled;
};

struct motor_control_policy_input {
	enum motor_control_policy_mode mode;
	struct motor_control_policy_features features;
	bool profile_sequence_active;
};

struct motor_control_policy {
	enum motor_motion_source motion_source;
	enum motor_feedback_source feedback_source;
	enum motor_angle_source angle_source;
	enum motor_generated_angle_mode generated_angle_mode;
	enum motor_current_source current_source;
	enum motor_actuator_kind actuator_kind;
	bool encoder_read_enabled;
	bool encoder_required_for_control;
	bool current_loop_enabled;
	bool generated_angle_position_driven;
};

struct motor_actuator_caps {
	bool accepts_position;
	bool accepts_velocity;
	bool accepts_torque;
	bool accepts_current;
	bool accepts_voltage;
	bool accepts_step_dir;
	bool requires_commutation_angle;
	bool supports_open_loop;
	bool supports_closed_loop;
};

int motor_control_policy_derive(const struct motor_control_policy_input *in,
				struct motor_control_policy *policy);

struct motor_actuator_caps motor_actuator_caps_for_kind(enum motor_actuator_kind kind);

bool motor_control_policy_is_valid(const struct motor_control_policy *policy,
				   const struct motor_actuator_caps *caps);

const char *motor_motion_source_to_string(enum motor_motion_source source);
const char *motor_feedback_source_to_string(enum motor_feedback_source source);
const char *motor_angle_source_to_string(enum motor_angle_source source);
const char *motor_generated_angle_mode_to_string(enum motor_generated_angle_mode mode);
const char *motor_current_source_to_string(enum motor_current_source source);
const char *motor_actuator_kind_to_string(enum motor_actuator_kind kind);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RUNTIME_CONTROL_POLICY_H_ */
