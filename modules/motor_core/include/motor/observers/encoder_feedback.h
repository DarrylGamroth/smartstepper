/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_OBSERVERS_ENCODER_FEEDBACK_H_
#define MOTOR_OBSERVERS_ENCODER_FEEDBACK_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/observers/angle_path.h"
#include "motor/telemetry/capture.h"

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

	struct motor_encoder_control_sample control;

	float32_t position_mech_rad;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;
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

int motor_encoder_feedback_prepare_capture(const struct motor_parameters *params,
					   const struct motor_encoder_feedback *feedback,
					   struct motor_capture_feedback *capture);

#endif /* MOTOR_OBSERVERS_ENCODER_FEEDBACK_H_ */
