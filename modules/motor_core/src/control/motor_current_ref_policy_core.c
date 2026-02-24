/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/motor_current_ref_policy_core.h"

#include <stddef.h>

void motor_current_ref_policy_core_apply(
	const struct motor_current_ref_policy_core_input *in,
	struct motor_current_ref_policy_core_output *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	out->id_ref_a = in->id_ref_in_a;
	out->iq_ref_a = in->iq_ref_in_a;
	out->reset_current_pi = false;
	out->disarmed_interlock_active = false;

	if (in->feature_use_commanded_currents) {
		bool commanded_current_needs_encoder_feedback =
			in->online_control_state && !in->feature_angle_gen;
		if (commanded_current_needs_encoder_feedback && !in->feedback_valid) {
			out->id_ref_a = in->id_meas_a;
			out->iq_ref_a = in->iq_meas_a;
			out->reset_current_pi = true;
		} else {
			out->id_ref_a = in->id_setpoint_a;
			out->iq_ref_a = in->iq_setpoint_a;
		}
	}

	if (in->online_control_state && !in->control_armed) {
		out->id_ref_a = in->id_meas_a;
		out->iq_ref_a = in->iq_meas_a;
		out->reset_current_pi = true;
		out->disarmed_interlock_active = true;
	}
}
