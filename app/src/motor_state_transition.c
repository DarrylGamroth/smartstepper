/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_state_transition.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "config.h"
#include "motor_encoder_control.h"
#include "motor_state_utils.h"
#include "motor_states.h"

const char *motor_transition_result_to_string(enum motor_transition_result result)
{
	switch (result) {
	case MOTOR_TRANSITION_RESULT_IDLE: return "idle";
	case MOTOR_TRANSITION_RESULT_REQUESTED: return "requested";
	case MOTOR_TRANSITION_RESULT_ACCEPTED: return "accepted";
	case MOTOR_TRANSITION_RESULT_COMPLETED: return "completed";
	case MOTOR_TRANSITION_RESULT_REJECTED: return "rejected";
	case MOTOR_TRANSITION_RESULT_FALLBACK: return "fallback";
	case MOTOR_TRANSITION_RESULT_FAULT: return "fault";
	case MOTOR_TRANSITION_RESULT_TIMEOUT: return "timeout";
	default: return "unknown";
	}
}

void motor_transition_status_init(struct motor_transition_status *status)
{
	if (status == NULL) {
		return;
	}

	memset(status, 0, sizeof(*status));
	status->request_event = MOTOR_EVENT_NONE;
	status->requested_state = MOTOR_STATE_IDLE;
	status->source_state = MOTOR_STATE_IDLE;
	status->final_state = MOTOR_STATE_IDLE;
	status->fallback_state = MOTOR_STATE_IDLE;
	status->result = MOTOR_TRANSITION_RESULT_IDLE;
	(void)snprintf(status->reason, sizeof(status->reason), "none");
}

void motor_transition_status_update(struct motor_parameters *params,
				    enum motor_event_type event,
				    int requested,
				    int source,
				    int final,
				    int fallback,
				    enum motor_transition_result result,
				    uint32_t error_code,
				    const char *reason)
{
	if (params == NULL) {
		return;
	}

	struct motor_transition_status *status = &params->transition_status;

	status->request_sequence++;
	status->request_event = event;
	status->requested_state = requested;
	status->source_state = source;
	status->final_state = final;
	status->fallback_state = fallback;
	status->result = result;
	status->error_code = error_code;
	status->timestamp_ms = k_uptime_get_32();
	status->loop_count = params->control_loop_count;
	(void)snprintf(status->reason, sizeof(status->reason), "%s",
		       (reason != NULL) ? reason : "none");
}

int motor_state_resolve_requested_online_mode(
	const struct motor_parameters *params,
	bool check_encoder_registers,
	bool *fallback_used,
	char *reason,
	size_t reason_len)
{
	enum motor_state mode = MOTOR_STATE_ONLINE_VELOCITY_GENERATED;

	if (fallback_used != NULL) {
		*fallback_used = false;
	}
	if (reason != NULL && reason_len > 0U) {
		reason[0] = '\0';
	}

	if (params != NULL) {
		mode = (enum motor_state)params->calibration.requested_online_mode;
	}

	if (!motor_state_is_online_submode(mode)) {
		if (fallback_used != NULL) {
			*fallback_used = true;
		}
		if (reason != NULL && reason_len > 0U) {
			(void)snprintf(reason, reason_len, "requested mode is not an online submode");
		}
		return MOTOR_STATE_ONLINE_VELOCITY_GENERATED;
	}

	if (motor_encoder_control_mode_requires_encoder(mode)) {
		char local_reason[96] = {0};

		if (!motor_encoder_control_ready_for_mode(params, mode,
							  check_encoder_registers,
							  local_reason,
							  sizeof(local_reason))) {
			if (fallback_used != NULL) {
				*fallback_used = true;
			}
			if (reason != NULL && reason_len > 0U) {
				(void)snprintf(reason, reason_len, "%s", local_reason);
			}
			return MOTOR_STATE_ONLINE_VELOCITY_GENERATED;
		}
	}

	if (reason != NULL && reason_len > 0U) {
		(void)snprintf(reason, reason_len, "ready");
	}
	return mode;
}
