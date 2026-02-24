/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/runtime/command_arbitration.h"

#include <stddef.h>

void motor_command_arbitration_apply(
	const struct motor_command_arbitration_input *in,
	struct motor_command_arbitration_output *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	out->id_ref_a = in->id_ref_in_a;
	out->iq_ref_a = in->iq_ref_in_a;
	out->reset_current_pi = false;

	if (!in->feature_use_commanded_currents) {
		return;
	}

	bool commanded_current_needs_encoder_feedback =
		in->online_control_state && !in->feature_angle_gen;
	if (commanded_current_needs_encoder_feedback && !in->feedback_valid) {
		out->id_ref_a = in->id_meas_a;
		out->iq_ref_a = in->iq_meas_a;
		out->reset_current_pi = true;
		return;
	}

	out->id_ref_a = in->id_setpoint_a;
	out->iq_ref_a = in->iq_setpoint_a;
}
