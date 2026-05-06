/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_STATE_TRANSITION_H_
#define MOTOR_STATE_TRANSITION_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_events.h"

struct motor_parameters;

enum motor_transition_result {
	MOTOR_TRANSITION_RESULT_IDLE = 0,
	MOTOR_TRANSITION_RESULT_REQUESTED,
	MOTOR_TRANSITION_RESULT_ACCEPTED,
	MOTOR_TRANSITION_RESULT_COMPLETED,
	MOTOR_TRANSITION_RESULT_REJECTED,
	MOTOR_TRANSITION_RESULT_FALLBACK,
	MOTOR_TRANSITION_RESULT_FAULT,
	MOTOR_TRANSITION_RESULT_TIMEOUT,
};

#define MOTOR_TRANSITION_REASON_LEN 96U

struct motor_transition_status {
	uint32_t request_sequence;
	enum motor_event_type request_event;
	int requested_state;
	int source_state;
	int final_state;
	int fallback_state;
	enum motor_transition_result result;
	uint32_t error_code;
	uint32_t timestamp_ms;
	uint32_t loop_count;
	char reason[MOTOR_TRANSITION_REASON_LEN];
};

const char *motor_transition_result_to_string(enum motor_transition_result result);

void motor_transition_status_init(struct motor_transition_status *status);

void motor_transition_status_update(struct motor_parameters *params,
				    enum motor_event_type event,
				    int requested,
				    int source,
				    int final,
				    int fallback,
				    enum motor_transition_result result,
				    uint32_t error_code,
				    const char *reason);

int motor_state_resolve_requested_online_mode(
	const struct motor_parameters *params,
	bool check_encoder_registers,
	bool *fallback_used,
	char *reason,
	size_t reason_len);

#endif /* MOTOR_STATE_TRANSITION_H_ */
