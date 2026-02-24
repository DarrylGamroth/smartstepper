/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_current_ref_policy.h"

#include <errno.h>

#include "config.h"
#include "motor_control_quality.h"
#include "motor/filters/pi.h"
#include "motor/motion/traj.h"
#include "motor/motion/angle_gen.h"
#include "motor/control/motor_mpr.h"
#include "motor/control/motor_dob.h"
#include "motor/protection/interlocks.h"
#include "motor/runtime/command_arbitration.h"

int motor_current_ref_apply_policy(struct motor_parameters *params,
				   const struct motor_current_ref_policy_inputs *in,
				   struct motor_current_ref_policy_outputs *out)
{
	if (params == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}

	out->velocity_target_rad_s = in->velocity_target_rad_s;
	out->velocity_ref_rad_s = in->velocity_ref_rad_s;
	out->id_ref_a = in->id_ref_a;
	out->iq_ref_a = in->iq_ref_a;

	bool feedback_valid = motor_velocity_feedback_is_valid(params->position_quality_flags);
	struct motor_command_arbitration_input arb_in = {
		.online_control_state = in->online_control_state,
		.feature_angle_gen = in->feature_angle_gen,
		.feature_use_commanded_currents = in->feature_use_commanded_currents,
		.feedback_valid = feedback_valid,
		.id_meas_a = in->id_meas_a,
		.iq_meas_a = in->iq_meas_a,
		.id_setpoint_a = params->Id_setpoint_A,
		.iq_setpoint_a = params->Iq_setpoint_A,
		.id_ref_in_a = in->id_ref_a,
		.iq_ref_in_a = in->iq_ref_a,
	};
	struct motor_command_arbitration_output arb_out = {0};
	motor_command_arbitration_apply(&arb_in, &arb_out);
	out->id_ref_a = arb_out.id_ref_a;
	out->iq_ref_a = arb_out.iq_ref_a;

	struct motor_current_interlock_input interlock_in = {
		.online_control_state = in->online_control_state,
		.control_armed = in->control_armed,
		.id_meas_a = in->id_meas_a,
		.iq_meas_a = in->iq_meas_a,
		.id_ref_in_a = out->id_ref_a,
		.iq_ref_in_a = out->iq_ref_a,
	};
	struct motor_current_interlock_output interlock_out = {0};
	motor_interlocks_apply_current(&interlock_in, &interlock_out);
	out->id_ref_a = interlock_out.id_ref_a;
	out->iq_ref_a = interlock_out.iq_ref_a;

	if (arb_out.reset_current_pi || interlock_out.reset_current_pi) {
		pi_set_ui(&params->pi_Id, 0.0f);
		pi_set_ui(&params->pi_Iq, 0.0f);
	}

	if (interlock_out.disarmed_interlock_active) {
		params->Id_setpoint_A = 0.0f;
		params->Iq_setpoint_A = 0.0f;
		out->velocity_target_rad_s = 0.0f;
		out->velocity_ref_rad_s = 0.0f;
		params->velocity_target_rad_s = 0.0f;
		params->velocity_ref_rad_s = 0.0f;
		params->velocity_cl_i_term_A = 0.0f;
		params->position_cl_i_term_rad_s = 0.0f;
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		angle_gen_set_velocity(&params->angle_gen, 0.0f);
		motor_mpr_velocity_reset(&params->velocity_mpr_state,
					 in->speed_mech_filtered_rad_s, 0.0f);
		motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
		motor_dob_reset(&params->velocity_dob_state, in->speed_mech_filtered_rad_s);
		params->velocity_dob_iq_ff_a = 0.0f;
		params->velocity_dob_disturbance_nm = 0.0f;
		params->velocity_dob_residual_rad_s = 0.0f;
	}

	return 0;
}
