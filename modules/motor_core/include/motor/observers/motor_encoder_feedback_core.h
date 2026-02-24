/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_FEEDBACK_CORE_H_
#define MOTOR_ENCODER_FEEDBACK_CORE_H_

#include <stdbool.h>
#include <stdint.h>

enum motor_encoder_feedback_source {
	MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED = 0U,
	MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER = 1U,
	MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED = 2U,
};

struct motor_encoder_feedback_core_input {
	bool feature_angle_gen;
	bool sample_enabled;
	bool sample_available;
	bool fresh;
	bool warning;
	bool error;
	bool io_fault;
	uint8_t status;
	uint32_t fault_threshold;
};

struct motor_encoder_feedback_core_state {
	uint32_t fault_counter;
	uint32_t warning_count;
	uint32_t error_count;
	uint8_t sample_fresh;
	uint8_t sample_warning;
	uint8_t sample_error;
	uint8_t last_status;
};

/**
 * @brief Resolve control-path angle source selection.
 */
uint8_t motor_encoder_feedback_select_source(bool feature_angle_gen,
					     bool sample_enabled,
					     bool fresh);

/**
 * @brief Update encoder counters/flags and report fault-threshold state.
 *
 * @return true when fault_counter exceeds fault_threshold.
 */
bool motor_encoder_feedback_update_state(const struct motor_encoder_feedback_core_input *in,
					 struct motor_encoder_feedback_core_state *state);

#endif /* MOTOR_ENCODER_FEEDBACK_CORE_H_ */
