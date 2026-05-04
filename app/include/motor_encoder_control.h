/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_CONTROL_H_
#define MOTOR_ENCODER_CONTROL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_states.h"
#include "motor_encoder_pipeline.h"

struct motor_encoder_control_status {
	bool device_ready;
	bool mapping_complete;
	bool pipeline_idle;
	bool injection_disabled;
	bool protocol_checked;
	bool protocol_ok;
	bool ready;
	int protocol_error;
	uint8_t config0;
	uint8_t config7;
	uint8_t config9;
	struct motor_encoder_pipeline_stats pipeline_stats;
};

bool motor_encoder_control_mode_requires_encoder(enum motor_state state);

int motor_encoder_control_get_status(const struct motor_parameters *params,
				     bool check_registers,
				     struct motor_encoder_control_status *status,
				     char *reason,
				     size_t reason_len);

bool motor_encoder_control_ready_for_mode(const struct motor_parameters *params,
					  enum motor_state state,
					  bool check_registers,
					  char *reason,
					  size_t reason_len);

bool motor_encoder_control_ready_for_transition(const struct motor_parameters *params,
						enum motor_state current_state,
						enum motor_state target_state,
						bool check_registers,
						char *reason,
						size_t reason_len);

#endif /* MOTOR_ENCODER_CONTROL_H_ */
