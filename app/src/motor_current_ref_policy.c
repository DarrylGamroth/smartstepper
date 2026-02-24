/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_current_ref_policy.h"

#include <errno.h>

#include "config.h"
#include "motor_control_quality.h"
#include "pi.h"
#include "traj.h"
#include "angle_gen.h"
#include "motor_mpr.h"
#include "motor_dob.h"

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

	/* Select current references based on mode */
	if (in->feature_use_commanded_currents) {
		bool commanded_current_needs_encoder_feedback =
			in->online_control_state && !in->feature_angle_gen;
		bool commanded_current_feedback_valid =
			motor_velocity_feedback_is_valid(params->position_quality_flags);

		/* In encoder-based current control, hold a neutral current-loop command
		 * until position/speed feedback quality is valid. This avoids large
		 * transients when torque mode is entered before valid encoder feedback.
		 */
		if (commanded_current_needs_encoder_feedback &&
		    !commanded_current_feedback_valid) {
			/* Keep current loop neutral while encoder feedback is invalid. */
			out->id_ref_a = in->id_meas_a;
			out->iq_ref_a = in->iq_meas_a;
			pi_set_ui(&params->pi_Id, 0.0f);
			pi_set_ui(&params->pi_Iq, 0.0f);
		} else {
			/* Normal FOC operation: use commanded current references */
			out->id_ref_a = params->Id_setpoint_A;
			out->iq_ref_a = params->Iq_setpoint_A;
		}
	}

	/* Arm/disarm interlock only applies in ONLINE control states. */
	if (in->online_control_state && !in->control_armed) {
		out->id_ref_a = in->id_meas_a;
		out->iq_ref_a = in->iq_meas_a;
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
		pi_set_ui(&params->pi_Id, 0.0f);
		pi_set_ui(&params->pi_Iq, 0.0f);
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
