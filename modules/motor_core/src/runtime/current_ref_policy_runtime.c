/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/runtime/current_ref_policy_runtime.h"

#include <errno.h>

#include "motor/control/current_ref_policy_core.h"
#include "motor/runtime/feedback_quality.h"

int motor_current_ref_apply_policy(struct motor_current_ref_policy_ctx *ctx,
				   const struct motor_current_ref_policy_inputs *in,
				   struct motor_current_ref_policy_outputs *out)
{
	if (ctx == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}

	out->velocity_target_rad_s = in->velocity_target_rad_s;
	out->velocity_ref_rad_s = in->velocity_ref_rad_s;
	out->id_ref_a = in->id_ref_a;
	out->iq_ref_a = in->iq_ref_a;
	out->reset_current_pi = false;
	out->disarmed_interlock_active = false;

	bool feedback_valid = motor_velocity_feedback_is_valid(ctx->position_quality_flags);
	struct motor_current_ref_policy_core_input core_in = {
		.online_control_state = in->online_control_state,
		.control_armed = in->control_armed,
		.feature_angle_gen = in->feature_angle_gen,
		.feature_use_commanded_currents = in->feature_use_commanded_currents,
		.feedback_valid = feedback_valid,
		.id_meas_a = in->id_meas_a,
		.iq_meas_a = in->iq_meas_a,
		.id_setpoint_a = *ctx->id_setpoint_a,
		.iq_setpoint_a = *ctx->iq_setpoint_a,
		.id_ref_in_a = in->id_ref_a,
		.iq_ref_in_a = in->iq_ref_a,
	};
	struct motor_current_ref_policy_core_output core_out = {0};
	motor_current_ref_policy_core_apply(&core_in, &core_out);
	out->id_ref_a = core_out.id_ref_a;
	out->iq_ref_a = core_out.iq_ref_a;
	out->reset_current_pi = core_out.reset_current_pi;
	out->disarmed_interlock_active = core_out.disarmed_interlock_active;

	if (out->reset_current_pi) {
		pi_set_ui(ctx->pi_id, 0.0f);
		pi_set_ui(ctx->pi_iq, 0.0f);
	}

	if (out->disarmed_interlock_active) {
		*ctx->id_setpoint_a = 0.0f;
		*ctx->iq_setpoint_a = 0.0f;
		out->velocity_target_rad_s = 0.0f;
		out->velocity_ref_rad_s = 0.0f;
		*ctx->live_velocity_target_rad_s = 0.0f;
		*ctx->live_velocity_ref_rad_s = 0.0f;
		*ctx->velocity_cl_i_term_a = 0.0f;
		*ctx->position_cl_i_term_rad_s = 0.0f;
		motor_velocity_regulator_reset(ctx->velocity_reg_state, 0.0f);
		motor_position_regulator_reset(ctx->position_reg_state, 0.0f);
		motor_mpr_velocity_reset(ctx->velocity_mpr_state, in->speed_mech_filtered_rad_s, 0.0f);
		motor_mpr_position_reset(ctx->position_mpr_state, 0.0f);
		motor_dob_reset(ctx->velocity_dob_state, in->speed_mech_filtered_rad_s);
		*ctx->live_velocity_dob_iq_ff_a = 0.0f;
		*ctx->live_velocity_dob_disturbance_nm = 0.0f;
		*ctx->live_velocity_dob_residual_rad_s = 0.0f;
	}

	return 0;
}
