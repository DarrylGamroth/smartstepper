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

struct motor_actuator_ref {
	enum motor_actuator_kind kind;
	bool enabled;
};

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
