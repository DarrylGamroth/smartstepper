/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ANGLE_PATH_H_
#define MOTOR_ANGLE_PATH_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/observers/angle_observer.h"
#include "motor/observers/encoder_feedback_core.h"
#include "motor/observers/feedback_quality.h"

/**
 * @brief Minimal control-facing encoder/angle sample consumed by control logic.
 */
struct motor_encoder_control_sample {
	float32_t position_mech_rad;
	float32_t electrical_angle_rad;
	float32_t predicted_electrical_angle_rad;
	float32_t electrical_speed_rad_s;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;
	uint8_t input_source;
	uint8_t quality_flags;
};

/**
 * @brief Input contract for a single angle-path step.
 */
struct motor_angle_path_input {
	bool feature_angle_gen;
	bool sample_enabled;
	bool sample_fresh;
	bool sample_warning;
	bool sample_error;
	bool sample_io_fault;
	bool propagated_valid;
	uint8_t previous_input_source;
	float32_t sample_angle_deg;
	float32_t encoder_direction_sign;
	float32_t generated_mech_rad;
	float32_t encoder_delay_samples;
};

/**
 * @brief Output contract for a single angle-path step.
 */
struct motor_angle_path_output {
	float32_t observer_input_rad;
	float32_t observer_mech_rad;
	float32_t observer_elec_rad;
	float32_t observer_elec_pred_rad;
	float32_t observer_elec_speed_rad_s;
	float32_t angle_sensor_deg;
	float32_t angle_control_deg;
	struct motor_encoder_control_sample control;
};

int motor_angle_path_step(struct angle_observer_state *observer,
			  const struct motor_angle_path_input *in,
			  struct motor_angle_path_output *out);

#endif /* MOTOR_ANGLE_PATH_H_ */
