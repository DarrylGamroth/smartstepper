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
#include "motor/observers/angle_observer.h"
#include "motor/motion/angle_gen.h"
#include "motor/runtime/io.h"
#include "motor/telemetry/capture.h"

struct motor_encoder_feedback_ctx {
	uint32_t *fault_counter;
	uint32_t *warning_count;
	uint32_t *error_count;
	uint8_t *sample_fresh;
	uint8_t *sample_warning;
	uint8_t *sample_error;
	uint8_t *last_status;
	int8_t encoder_direction_sign;
	angle_gen_t *angle_gen;
	struct angle_observer_state *observer;
	float32_t *observer_input_rad;
	uint8_t *encoder_input_source;
	float32_t *encoder_raw_deg;
	float32_t *encoder_raw_rad;
	uint16_t *position_stale_count;
	uint32_t *position_stale_events;
	uint32_t *position_glitch_count;
	uint32_t *position_jitter_count;
	uint8_t *position_quality_flags;
	uint8_t *position_trust_state;
	uint32_t encoder_fault_threshold;
	uint8_t encoder_delay_samples;
};

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

	struct motor_encoder_control_sample control;

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

/**
 * @brief Update encoder source arbitration, observer handoff/update, and position conversion.
 *
 * Also updates encoder-related counters and quality/status fields in @p ctx.
 *
 * @return 0 on success, -EIO when encoder fault threshold is exceeded, -EINVAL on invalid args.
 */
int motor_encoder_feedback_update(struct motor_encoder_feedback_ctx *ctx,
				  const struct motor_control_encoder_sample *encoder_sample,
				  bool feature_angle_gen,
				  struct motor_encoder_feedback *feedback);

int motor_encoder_feedback_prepare_capture(const struct motor_encoder_feedback_ctx *ctx,
					   const struct motor_encoder_feedback *feedback,
					   struct motor_capture_feedback *capture);

#endif /* MOTOR_OBSERVERS_ENCODER_FEEDBACK_H_ */
