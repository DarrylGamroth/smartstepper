/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/protection/interlocks.h"

#include <stddef.h>

void motor_interlocks_eval_timeout(const struct motor_timeout_interlock_input *in,
				   struct motor_timeout_interlock_output *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	out->disarm_control = false;
	if (!in->online_control_state || !in->control_armed || in->autonomous_keepalive ||
	    in->command_timeout_ms == 0U) {
		return;
	}

	uint32_t elapsed_ms = in->now_ms - in->last_command_update_ms;
	out->disarm_control = elapsed_ms > in->command_timeout_ms;
}

void motor_interlocks_apply_current(const struct motor_current_interlock_input *in,
				    struct motor_current_interlock_output *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	out->id_ref_a = in->id_ref_in_a;
	out->iq_ref_a = in->iq_ref_in_a;
	out->reset_current_pi = false;
	out->disarmed_interlock_active = false;

	if (in->online_control_state && !in->control_armed) {
		out->id_ref_a = in->id_meas_a;
		out->iq_ref_a = in->iq_meas_a;
		out->reset_current_pi = true;
		out->disarmed_interlock_active = true;
	}
}
