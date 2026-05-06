/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_operating_mode.h"

#include <stddef.h>

#include <zephyr/sys/util.h>

#include "config.h"
#include "motor/filters/pi.h"
#include "motor/filters/filter_so.h"
#include "motor/motion/angle_gen.h"
#include "motor/motion/motion_planner.h"
#include "motor/math/angle_wrap.h"
#include "motor/control/dob.h"

static const struct motor_operating_mode_descriptor mode_descriptors[] = {
	{
		.state = MOTOR_STATE_ONLINE_CURRENT_ENCODER,
		.name = "current_encoder",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_CURRENT_ENCODER,
		.feature_mask = BIT(MOTOR_FEATURE_ENCODER_READ),
		.requires_encoder_setup = true,
		.generated_mode = false,
		.entry_reset = MOTOR_MODE_RESET_DIRECT_CURRENT,
		.exit_reset = MOTOR_MODE_RESET_DIRECT_CURRENT,
	},
	{
		.state = MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
		.name = "velocity_generated",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_GENERATED,
		.feature_mask = BIT(MOTOR_FEATURE_ANGLE_GEN) |
				BIT(MOTOR_FEATURE_VELOCITY_TRAJ),
		.requires_encoder_setup = false,
		.generated_mode = true,
		.entry_reset = MOTOR_MODE_RESET_GENERATED_VELOCITY,
		.exit_reset = MOTOR_MODE_RESET_GENERATED_VELOCITY,
	},
	{
		.state = MOTOR_STATE_ONLINE_POSITION_GENERATED,
		.name = "position_generated",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_POSITION_GENERATED,
		.feature_mask = BIT(MOTOR_FEATURE_ANGLE_GEN),
		.requires_encoder_setup = false,
		.generated_mode = true,
		.entry_reset = MOTOR_MODE_RESET_GENERATED_POSITION,
		.exit_reset = MOTOR_MODE_RESET_GENERATED_POSITION,
	},
	{
		.state = MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
		.name = "velocity_encoder",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER,
		.feature_mask = BIT(MOTOR_FEATURE_ENCODER_READ) |
				BIT(MOTOR_FEATURE_VELOCITY_TRAJ),
		.requires_encoder_setup = true,
		.generated_mode = false,
		.entry_reset = MOTOR_MODE_RESET_ENCODER_VELOCITY,
		.exit_reset = MOTOR_MODE_RESET_ENCODER_VELOCITY,
	},
	{
		.state = MOTOR_STATE_ONLINE_POSITION_ENCODER,
		.name = "position_encoder",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_POSITION_ENCODER,
		.feature_mask = BIT(MOTOR_FEATURE_ENCODER_READ) |
				BIT(MOTOR_FEATURE_VELOCITY_TRAJ),
		.requires_encoder_setup = true,
		.generated_mode = false,
		.entry_reset = MOTOR_MODE_RESET_ENCODER_POSITION,
		.exit_reset = MOTOR_MODE_RESET_ENCODER_POSITION,
	},
};

const struct motor_operating_mode_descriptor *
motor_operating_mode_descriptor_get(enum motor_state state)
{
	for (size_t i = 0U; i < ARRAY_SIZE(mode_descriptors); i++) {
		if (mode_descriptors[i].state == state) {
			return &mode_descriptors[i];
		}
	}

	return NULL;
}

atomic_val_t motor_operating_mode_feature_mask(enum motor_state state)
{
	const struct motor_operating_mode_descriptor *desc =
		motor_operating_mode_descriptor_get(state);

	return (desc != NULL) ? desc->feature_mask : 0;
}

static void motor_operating_mode_reset_common_outer(struct motor_parameters *params,
						    float32_t speed_mech_rad_s)
{
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	motor_mpr_velocity_reset(&params->velocity_mpr_state, speed_mech_rad_s, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state, speed_mech_rad_s);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;
}

void motor_operating_mode_apply_entry_policy(struct motor_parameters *params,
					     enum motor_state state)
{
	if (params == NULL) {
		return;
	}

	const struct motor_operating_mode_descriptor *desc =
		motor_operating_mode_descriptor_get(state);
	if (desc == NULL) {
		return;
	}

	float32_t speed_mech_rad_s = params->live.velocity_rad_s;

	switch (desc->entry_reset) {
	case MOTOR_MODE_RESET_DIRECT_CURRENT:
		params->Id_setpoint_A = 0.0f;
		params->Iq_setpoint_A = 0.0f;
		pi_set_ui(&params->pi_Id, 0.0f);
		pi_set_ui(&params->pi_Iq, 0.0f);
		params->live.velocity_target_rad_s = 0.0f;
		params->live.velocity_ref_rad_s = 0.0f;
		motor_operating_mode_reset_common_outer(params, speed_mech_rad_s);
		break;
	case MOTOR_MODE_RESET_GENERATED_VELOCITY:
		motor_velocity_plan_init(&params->traj_velocity,
					 params->profile_max_velocity_rad_s,
					 params->profile_max_accel_rad_s2,
					 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
					 0.0f);
		params->live.velocity_target_rad_s = 0.0f;
		params->live.velocity_ref_rad_s = 0.0f;
		motor_operating_mode_reset_common_outer(params, 0.0f);
		break;
	case MOTOR_MODE_RESET_GENERATED_POSITION:
		params->position_target_rad = wrap_rad_2pi(params->live.position_rad);
		motion_profile_quintic_cancel(&params->position_profile,
					      params->live.position_rad);
		motor_operating_mode_reset_common_outer(params, 0.0f);
		break;
	case MOTOR_MODE_RESET_ENCODER_VELOCITY:
		motor_velocity_plan_init(&params->traj_velocity,
					 params->profile_max_velocity_rad_s,
					 params->profile_max_accel_rad_s2,
					 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
					 0.0f);
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		params->live.velocity_target_rad_s = 0.0f;
		params->live.velocity_ref_rad_s = 0.0f;
		params->live.Id_ref_A = params->Id_setpoint_A;
		params->live.Iq_ref_A = 0.0f;
		filter_so_prime(&params->filter_velocity_notch, speed_mech_rad_s);
		motor_operating_mode_reset_common_outer(params, speed_mech_rad_s);
		break;
	case MOTOR_MODE_RESET_ENCODER_POSITION:
		params->position_target_rad = params->live.position_rad;
		motion_profile_quintic_cancel(&params->position_profile,
					      params->live.position_rad);
		motor_velocity_plan_init(&params->traj_velocity,
					 params->profile_max_velocity_rad_s,
					 params->profile_max_accel_rad_s2,
					 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
					 speed_mech_rad_s);
		traj_set_target_value(&params->traj_velocity, 0.0f);
		params->live.velocity_target_rad_s = 0.0f;
		params->live.velocity_ref_rad_s = speed_mech_rad_s;
		params->live.Id_ref_A = params->Id_setpoint_A;
		params->live.Iq_ref_A = 0.0f;
		filter_so_prime(&params->filter_velocity_notch, speed_mech_rad_s);
		motor_operating_mode_reset_common_outer(params, speed_mech_rad_s);
		break;
	case MOTOR_MODE_RESET_NONE:
	default:
		break;
	}
}

void motor_operating_mode_apply_exit_policy(struct motor_parameters *params,
					    enum motor_state state)
{
	if (params == NULL) {
		return;
	}

	const struct motor_operating_mode_descriptor *desc =
		motor_operating_mode_descriptor_get(state);
	if (desc == NULL) {
		return;
	}

	switch (desc->exit_reset) {
	case MOTOR_MODE_RESET_GENERATED_VELOCITY:
		angle_gen_set_velocity(&params->angle_gen, 0.0f);
		angle_gen_set_angle(&params->angle_gen, 0.0f);
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		break;
	case MOTOR_MODE_RESET_GENERATED_POSITION: {
		float32_t hold_rad = angle_gen_get_angle(&params->angle_gen);
		angle_gen_set_velocity(&params->angle_gen, 0.0f);
		motion_profile_quintic_cancel(&params->position_profile, hold_rad);
		params->profile_seq.running = false;
		params->profile_seq.tick_counter = 0U;
		motor_operating_mode_reset_common_outer(params, 0.0f);
		break;
	}
	case MOTOR_MODE_RESET_ENCODER_VELOCITY:
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		params->live.velocity_target_rad_s = 0.0f;
		params->live.velocity_ref_rad_s = 0.0f;
		motor_operating_mode_reset_common_outer(params, 0.0f);
		break;
	case MOTOR_MODE_RESET_ENCODER_POSITION:
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		params->live.velocity_target_rad_s = 0.0f;
		params->live.velocity_ref_rad_s = 0.0f;
		motion_profile_quintic_cancel(&params->position_profile,
					      params->live.position_rad);
		params->profile_seq.running = false;
		params->profile_seq.tick_counter = 0U;
		motor_operating_mode_reset_common_outer(params, 0.0f);
		break;
	case MOTOR_MODE_RESET_DIRECT_CURRENT:
		motor_operating_mode_reset_common_outer(params, 0.0f);
		break;
	case MOTOR_MODE_RESET_NONE:
	default:
		break;
	}
}
