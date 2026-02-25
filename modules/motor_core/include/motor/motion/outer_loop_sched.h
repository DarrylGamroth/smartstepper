/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_OUTER_LOOP_SCHED_H_
#define MOTOR_OUTER_LOOP_SCHED_H_

#include <stdbool.h>
#include <stdint.h>

bool motor_outer_loop_decimation_tick(uint32_t *phase, uint32_t decimation);

struct motor_outer_loop_sched_input {
	bool position_active;
	bool velocity_active;
	uint32_t position_decimation;
	uint32_t velocity_decimation;
};

struct motor_outer_loop_sched_state {
	uint32_t position_phase;
	uint32_t velocity_phase;
};

struct motor_outer_loop_sched_output {
	bool position_update;
	bool velocity_update;
};

void motor_outer_loop_sched_step(const struct motor_outer_loop_sched_input *in,
				 struct motor_outer_loop_sched_state *state,
				 struct motor_outer_loop_sched_output *out);

#endif /* MOTOR_OUTER_LOOP_SCHED_H_ */
