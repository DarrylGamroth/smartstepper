/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/observers/encoder_feedback_core.h"

#include <stddef.h>

uint8_t motor_encoder_feedback_select_source(bool feature_angle_gen,
					     bool sample_enabled,
					     bool fresh,
					     bool warning,
					     bool error,
					     bool io_fault)
{
	if (feature_angle_gen) {
		return MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED;
	}
	(void)warning;
	if (sample_enabled && fresh && !error && !io_fault) {
		return MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER;
	}
	return MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED;
}

bool motor_encoder_feedback_update_state(const struct motor_encoder_feedback_core_input *in,
					 struct motor_encoder_feedback_core_state *state)
{
	if (in == NULL || state == NULL) {
		return false;
	}

	if (in->sample_available && (in->fresh || in->warning || in->error)) {
		state->last_status = in->status;
	}

	if (in->sample_enabled) {
		if (!in->fresh) {
			if (in->io_fault) {
				state->fault_counter++;
			}
			if (in->warning) {
				state->warning_count++;
			}
			if (in->error) {
				state->error_count++;
			}
		} else {
			state->fault_counter = 0U;
			if (in->warning) {
				state->warning_count++;
			}
		}

		state->sample_fresh = in->fresh ? 1U : 0U;
		state->sample_warning = in->warning ? 1U : 0U;
		state->sample_error = in->error ? 1U : 0U;
		return state->fault_counter > in->fault_threshold;
	}

	state->fault_counter = 0U;
	state->sample_fresh = 0U;
	state->sample_warning = 0U;
	state->sample_error = 0U;
	return false;
}
