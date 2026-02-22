/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_POSITION_CONVERT_H_
#define MOTOR_POSITION_CONVERT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_position_convert_quality {
	MOTOR_POSITION_CONVERT_QUALITY_VALID = BIT(0),
	MOTOR_POSITION_CONVERT_QUALITY_FRESH = BIT(1),
	MOTOR_POSITION_CONVERT_QUALITY_STALE = BIT(2),
	MOTOR_POSITION_CONVERT_QUALITY_WARNING = BIT(3),
	MOTOR_POSITION_CONVERT_QUALITY_ERROR = BIT(4),
	MOTOR_POSITION_CONVERT_QUALITY_GLITCH = BIT(5),
	MOTOR_POSITION_CONVERT_QUALITY_JITTER = BIT(6),
	MOTOR_POSITION_CONVERT_QUALITY_GENERATED = BIT(7),
};

struct motor_position_convert_config {
	float32_t dt_s;
	float32_t velocity_lpf_hz;
	float32_t accel_lpf_hz;
	float32_t max_step_rad;
	float32_t latency_samples_default;
	float32_t jitter_threshold_rad;
	uint16_t stale_threshold_samples;
};

struct motor_position_convert_input {
	bool sample_valid;
	bool sample_fresh;
	bool source_generated;
	bool warning;
	bool error;
	float32_t measurement_wrapped_rad;
	float32_t latency_samples;
};

struct motor_position_convert_state {
	bool initialized;
	bool stale_latched;
	float32_t prev_meas_wrapped_rad;
	float32_t position_wrapped_rad;
	float32_t position_unwrapped_rad;
	float32_t velocity_rad_s;
	float32_t accel_rad_s2;
	float32_t innovation_rad;
	uint8_t quality_flags;
	uint16_t stale_count;
	uint32_t sample_count;
	uint32_t stale_event_count;
	uint32_t glitch_count;
	uint32_t jitter_count;
};

int motor_position_convert_validate(const struct motor_position_convert_config *cfg);

void motor_position_convert_init(struct motor_position_convert_state *state,
				 const struct motor_position_convert_config *cfg,
				 float32_t initial_wrapped_rad);

void motor_position_convert_reset(struct motor_position_convert_state *state,
			  float32_t initial_wrapped_rad);

int motor_position_convert_update(struct motor_position_convert_state *state,
			   const struct motor_position_convert_config *cfg,
			   const struct motor_position_convert_input *input);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_POSITION_CONVERT_H_ */
