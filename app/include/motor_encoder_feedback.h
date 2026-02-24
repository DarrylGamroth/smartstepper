/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_FEEDBACK_H_
#define MOTOR_ENCODER_FEEDBACK_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_parameters;
struct motor_control_encoder_sample;

/**
 * @brief Normalized encoder/observer/position feedback for one control ISR step.
 */
struct motor_encoder_feedback {
	bool sample_enabled;
	bool sample_available;
	bool fresh;
	bool warning;
	bool error;
	bool io_fault;
	uint8_t status;
	uint8_t input_source;

	float32_t angle_sensor_deg;
	float32_t angle_control_deg;
	float32_t observer_input_rad;
	float32_t observer_mech_rad;
	float32_t observer_elec_rad;

	float32_t position_mech_rad;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;

	/* Capture/debug fields */
	float32_t capture_angle_deg;
	float32_t capture_angle_rad;
	float32_t capture_encoder_mech_rad;
	float32_t capture_encoder_elec_rad;
	float32_t capture_observer_mech_rad;
	float32_t capture_observer_elec_rad;
	float32_t capture_generated_mech_rad;
	float32_t capture_generated_elec_rad;
	float32_t capture_mech_error_rad;
	float32_t capture_elec_error_rad;
	bool capture_compare_valid;
	uint8_t capture_input_source;
};

/**
 * @brief Update encoder source arbitration, observer handoff/update, and position conversion.
 *
 * Also updates encoder-related counters and quality/status fields in @p params.
 *
 * @return 0 on success, -EIO when encoder fault threshold is exceeded, -EINVAL on invalid args.
 */
int motor_encoder_feedback_update(struct motor_parameters *params,
				  const struct motor_control_encoder_sample *encoder_sample,
				  bool feature_angle_gen,
				  struct motor_encoder_feedback *feedback);

#endif /* MOTOR_ENCODER_FEEDBACK_H_ */
