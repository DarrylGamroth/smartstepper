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

static inline void motor_actuator_ref_clear(struct motor_actuator_ref *ref,
					    enum motor_actuator_kind kind)
{
	if (ref == NULL) {
		return;
	}

	*ref = (struct motor_actuator_ref){
		.kind = kind,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_NONE,
		.enabled = false,
	};
}

static inline void motor_actuator_ref_set_foc_current(struct motor_actuator_ref *ref,
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

	*ref = (struct motor_actuator_ref){
		.kind = MOTOR_ACTUATOR_FOC_CURRENT,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_CURRENT_DQ,
		.enabled = enabled,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.id_ref_a = id_ref_a,
		.iq_ref_a = iq_ref_a,
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
