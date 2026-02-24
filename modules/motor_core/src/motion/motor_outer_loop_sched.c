/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/motion/motor_outer_loop_sched.h"

#include <stddef.h>

bool motor_outer_loop_decimation_tick(uint32_t *phase, uint32_t decimation)
{
	if (phase == NULL || decimation <= 1U) {
		if (phase != NULL) {
			*phase = 0U;
		}
		return true;
	}

	if (*phase == 0U) {
		*phase = decimation - 1U;
		return true;
	}

	(*phase)--;
	return false;
}

void motor_outer_loop_sched_step(const struct motor_outer_loop_sched_input *in,
				 struct motor_outer_loop_sched_state *state,
				 struct motor_outer_loop_sched_output *out)
{
	if (in == NULL || state == NULL || out == NULL) {
		return;
	}

	if (in->position_active) {
		out->position_update =
			motor_outer_loop_decimation_tick(&state->position_phase,
						       in->position_decimation);
	} else {
		state->position_phase = 0U;
		out->position_update = false;
	}

	if (in->velocity_active) {
		out->velocity_update =
			motor_outer_loop_decimation_tick(&state->velocity_phase,
						       in->velocity_decimation);
	} else {
		state->velocity_phase = 0U;
		out->velocity_update = false;
	}
}
