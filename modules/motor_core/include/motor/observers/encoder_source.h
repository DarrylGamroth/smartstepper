/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_SOURCE_H_
#define MOTOR_ENCODER_SOURCE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/observers/motor_encoder_feedback_core.h"

struct motor_encoder_source_sample {
	bool sample_enabled;
	bool sample_available;
	bool fresh;
	bool warning;
	bool error;
	bool io_fault;
	uint8_t status;
	float32_t angle_sensor_deg;
	float32_t angle_control_deg;
};

void motor_encoder_source_from_raw(bool raw_sample_present,
				   bool raw_sample_enabled,
				   bool raw_fresh,
				   bool raw_warning,
				   bool raw_error,
				   bool raw_io_fault,
				   uint8_t raw_status,
				   float32_t raw_angle_deg,
				   bool encoder_capture_enabled,
				   float32_t encoder_direction_sign,
				   struct motor_encoder_source_sample *sample);

uint8_t motor_encoder_source_select(bool feature_angle_gen,
				    bool sample_enabled,
				    bool fresh);

float32_t motor_encoder_source_resolve_angle_rad(uint8_t source,
						 const struct motor_encoder_source_sample *sample,
						 float32_t observer_mech_rad,
						 float32_t generated_mech_rad);

#endif /* MOTOR_ENCODER_SOURCE_H_ */
