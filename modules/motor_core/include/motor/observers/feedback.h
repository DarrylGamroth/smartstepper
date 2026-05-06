/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_FEEDBACK_H_
#define MOTOR_FEEDBACK_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

/**
 * @brief Compact control feedback payload consumed by ISR control logic.
 */
struct motor_control_feedback {
	bool sample_enabled;
	bool sample_available;
	bool fresh;
	bool warning;
	bool error;
	bool io_fault;
	uint8_t status;
	uint8_t input_source;
	uint8_t trust_state;
	float32_t angle_sensor_deg;
	float32_t angle_control_deg;
	float32_t generated_mech_rad;
	float32_t generated_elec_rad;
	float32_t observer_input_rad;
	float32_t observer_mech_rad;
	float32_t observer_elec_rad;
	float32_t observer_elec_pred_rad;
	float32_t observer_elec_speed_rad_s;
	float32_t position_mech_rad;
	float32_t electrical_angle_rad;
	float32_t predicted_electrical_angle_rad;
	float32_t electrical_speed_rad_s;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;
	float32_t observer_delay_samples;
	float32_t prediction_age_samples;
};

#endif /* MOTOR_FEEDBACK_H_ */
