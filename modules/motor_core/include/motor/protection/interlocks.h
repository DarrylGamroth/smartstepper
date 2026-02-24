/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_INTERLOCKS_H_
#define MOTOR_INTERLOCKS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_timeout_interlock_input {
	bool online_control_state;
	bool control_armed;
	bool autonomous_keepalive;
	uint32_t command_timeout_ms;
	uint32_t now_ms;
	uint32_t last_command_update_ms;
};

struct motor_timeout_interlock_output {
	bool disarm_control;
};

void motor_interlocks_eval_timeout(const struct motor_timeout_interlock_input *in,
				   struct motor_timeout_interlock_output *out);

struct motor_current_interlock_input {
	bool online_control_state;
	bool control_armed;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t id_ref_in_a;
	float32_t iq_ref_in_a;
};

struct motor_current_interlock_output {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	bool reset_current_pi;
	bool disarmed_interlock_active;
};

void motor_interlocks_apply_current(const struct motor_current_interlock_input *in,
				    struct motor_current_interlock_output *out);

#endif /* MOTOR_INTERLOCKS_H_ */
