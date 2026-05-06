/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CAPTURE_H_
#define MOTOR_CAPTURE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

/**
 * @brief Extended capture/debug payload for optional telemetry/capture paths.
 */
struct motor_capture_feedback {
	float32_t angle_deg;
	float32_t angle_rad;
	float32_t encoder_mech_rad;
	float32_t encoder_elec_rad;
	float32_t observer_mech_rad;
	float32_t observer_elec_rad;
	float32_t observer_delay_samples;
	float32_t prediction_age_samples;
	float32_t generated_mech_rad;
	float32_t generated_elec_rad;
	float32_t mech_error_rad;
	float32_t elec_error_rad;
	bool compare_valid;
	bool sample_enabled;
	bool sample_fresh;
	bool sample_warning;
	bool sample_error;
	uint8_t status;
	uint8_t input_source;
};

#endif /* MOTOR_CAPTURE_H_ */
