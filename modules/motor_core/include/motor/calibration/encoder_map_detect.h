/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CALIBRATION_ENCODER_MAP_DETECT_H_
#define MOTOR_CALIBRATION_ENCODER_MAP_DETECT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_encoder_map_detect_sample_flag {
	MOTOR_ENCODER_MAP_SAMPLE_WARNING = 1U << 0,
	MOTOR_ENCODER_MAP_SAMPLE_ERROR = 1U << 1,
};

struct motor_encoder_map_detect_config {
	float32_t pole_pairs;
	float32_t min_mech_motion_rad;
	float32_t max_offset_residual_rad;
	float32_t max_direction_residual_rad;
	float32_t min_direction_correlation;
	bool estimate_ratio;
};

struct motor_encoder_map_detect_sample {
	float32_t generated_elec_rad;
	float32_t encoder_mech_rad;
	uint32_t flags;
};

struct motor_encoder_map_detect_result {
	bool valid;
	bool direction_valid;
	bool offset_valid;
	bool ratio_valid;
	int8_t direction_sign;
	float32_t offset_mech_rad;
	float32_t offset_elec_rad;
	float32_t ratio;
	float32_t direction_corr;
	float32_t direction_residual_rad;
	float32_t offset_residual_rad;
	float32_t mech_motion_rad;
	uint32_t sample_count;
	uint32_t rejected_samples;
	uint32_t encoder_error_count;
	uint32_t encoder_warning_count;
};

int motor_encoder_map_detect_compute(const struct motor_encoder_map_detect_config *cfg,
				     const struct motor_encoder_map_detect_sample *samples,
				     uint32_t sample_count,
				     struct motor_encoder_map_detect_result *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CALIBRATION_ENCODER_MAP_DETECT_H_ */
