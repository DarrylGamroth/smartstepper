/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/observers/encoder_source.h"

#include <stddef.h>
#include <string.h>

#include "motor/math/math_constants.h"

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
				   struct motor_encoder_source_sample *sample)
{
	if (sample == NULL) {
		return;
	}

	memset(sample, 0, sizeof(*sample));
	if (!raw_sample_present) {
		return;
	}

	if (!(raw_sample_enabled || encoder_capture_enabled)) {
		return;
	}

	sample->sample_enabled = raw_sample_enabled;
	sample->sample_available = true;
	sample->fresh = raw_fresh;
	sample->warning = raw_warning;
	sample->error = raw_error;
	sample->io_fault = raw_io_fault;
	sample->status = raw_status;
	sample->angle_sensor_deg = raw_angle_deg;
	sample->angle_control_deg = sample->angle_sensor_deg * encoder_direction_sign;
}

uint8_t motor_encoder_source_select(bool feature_angle_gen,
				    bool sample_enabled,
				    bool fresh)
{
	return motor_encoder_feedback_select_source(feature_angle_gen, sample_enabled, fresh);
}

float32_t motor_encoder_source_resolve_angle_rad(uint8_t source,
						 const struct motor_encoder_source_sample *sample,
						 float32_t observer_mech_rad,
						 float32_t generated_mech_rad)
{
	if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED) {
		return generated_mech_rad;
	}
	if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER && sample != NULL) {
		return sample->angle_control_deg * (PI_F32 / 180.0f);
	}
	return observer_mech_rad;
}
