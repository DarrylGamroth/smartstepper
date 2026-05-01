/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_CONTROL_REFS_H_
#define MOTOR_RUNTIME_CONTROL_REFS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/runtime/control_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

struct motor_motion_ref {
	float32_t position_rad;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t velocity_rad_s;
	float32_t acceleration_rad_s2;
};

struct motor_feedback_ref {
	enum motor_feedback_source source;
	uint8_t input_source;
	uint8_t quality_flags;
	uint8_t status;
	bool fresh;
	bool warning;
	bool error;
	float32_t angle_control_deg;
	float32_t observer_input_rad;
	float32_t position_rad;
	float32_t velocity_rad_s;
	float32_t acceleration_rad_s2;
	float32_t velocity_filtered_rad_s;
};

enum motor_actuator_effort_kind {
	MOTOR_ACTUATOR_EFFORT_NONE = 0,
	MOTOR_ACTUATOR_EFFORT_CURRENT_DQ,
	MOTOR_ACTUATOR_EFFORT_CURRENT_SCALAR,
	MOTOR_ACTUATOR_EFFORT_VOLTAGE_SCALAR,
	MOTOR_ACTUATOR_EFFORT_NORMALIZED,
	MOTOR_ACTUATOR_EFFORT_STEP_DIR,
};

enum motor_servo_effort_kind {
	MOTOR_SERVO_EFFORT_NONE = 0,
	MOTOR_SERVO_EFFORT_TORQUE,
	MOTOR_SERVO_EFFORT_CURRENT,
	MOTOR_SERVO_EFFORT_VOLTAGE,
	MOTOR_SERVO_EFFORT_NORMALIZED,
	/*
	 * Transitional FOC-current command path. The servo/actuator boundary is
	 * explicit, but existing current-mode operation still commands D/Q axes.
	 */
	MOTOR_SERVO_EFFORT_CURRENT_DQ,
};

struct motor_servo_ref {
	bool enabled;
	enum motor_servo_effort_kind effort_kind;
	float32_t position_rad;
	float32_t velocity_rad_s;
	float32_t acceleration_rad_s2;
	float32_t torque_nm;
	float32_t current_a;
	float32_t voltage_v;
	float32_t normalized_effort;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

struct motor_actuator_ref {
	enum motor_actuator_kind kind;
	enum motor_actuator_effort_kind effort_kind;
	bool enabled;
	float32_t position_rad;
	float32_t velocity_rad_s;
	float32_t acceleration_rad_s2;
	float32_t torque_nm;
	float32_t current_a;
	float32_t voltage_v;
	float32_t normalized_effort;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

static inline void motor_servo_ref_clear(struct motor_servo_ref *ref)
{
	if (ref == NULL) {
		return;
	}

	*ref = (struct motor_servo_ref){0};
}

static inline void motor_servo_ref_set_motion(struct motor_servo_ref *ref,
					      bool enabled,
					      float32_t position_rad,
					      float32_t velocity_rad_s,
					      float32_t acceleration_rad_s2)
{
	if (ref == NULL) {
		return;
	}

	*ref = (struct motor_servo_ref){
		.enabled = enabled,
		.effort_kind = MOTOR_SERVO_EFFORT_NONE,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
	};
}

static inline void motor_servo_ref_set_dq_current(struct motor_servo_ref *ref,
						  bool enabled,
						  float32_t position_rad,
						  float32_t velocity_rad_s,
						  float32_t acceleration_rad_s2,
						  float32_t id_ref_a,
						  float32_t iq_ref_a)
{
	if (ref == NULL) {
		return;
	}

	*ref = (struct motor_servo_ref){
		.enabled = enabled,
		.effort_kind = MOTOR_SERVO_EFFORT_CURRENT_DQ,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.id_ref_a = id_ref_a,
		.iq_ref_a = iq_ref_a,
	};
}

static inline void motor_servo_ref_set_current(struct motor_servo_ref *ref,
					       bool enabled,
					       float32_t position_rad,
					       float32_t velocity_rad_s,
					       float32_t acceleration_rad_s2,
					       float32_t current_a)
{
	if (ref == NULL) {
		return;
	}

	*ref = (struct motor_servo_ref){
		.enabled = enabled,
		.effort_kind = MOTOR_SERVO_EFFORT_CURRENT,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.current_a = current_a,
	};
}

static inline void motor_servo_ref_set_voltage(struct motor_servo_ref *ref,
					       bool enabled,
					       float32_t position_rad,
					       float32_t velocity_rad_s,
					       float32_t acceleration_rad_s2,
					       float32_t voltage_v)
{
	if (ref == NULL) {
		return;
	}

	*ref = (struct motor_servo_ref){
		.enabled = enabled,
		.effort_kind = MOTOR_SERVO_EFFORT_VOLTAGE,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.voltage_v = voltage_v,
	};
}

struct motor_angle_ref {
	enum motor_angle_source source;
	float32_t electrical_angle_rad;
	float32_t predicted_electrical_angle_rad;
	float32_t electrical_speed_rad_s;
};

struct motor_current_ref {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	float32_t id_meas_a;
	float32_t iq_meas_a;
};

struct motor_commutation_ref {
	float32_t vd_v;
	float32_t vq_v;
	float32_t va_v;
	float32_t vb_v;
	float32_t max_voltage_magnitude_v;
	float32_t da_hb1_pu;
	float32_t da_hb2_pu;
	float32_t db_hb1_pu;
	float32_t db_hb2_pu;
	bool voltage_saturated;
};

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RUNTIME_CONTROL_REFS_H_ */
