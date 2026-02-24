/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control_outer_loops.h"

#include <errno.h>
#include <math.h>

#include "config.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "angle_wrap.h"
#include "filter_so.h"
#include "traj.h"
#include "angle_gen.h"
#include "motor_motion_modules.h"
#include "motor_mpr.h"
#include "motor_dob.h"
#include "motor_torque.h"
#include "motor_control_quality.h"

static inline bool motor_outer_loop_use_mpr(const struct motor_parameters *params)
{
	return params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR;
}

static inline bool motor_outer_loop_decimation_tick(uint32_t *phase, uint32_t decimation)
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

int motor_control_outer_loops_step(struct motor_parameters *params,
				   const struct motor_outer_loop_inputs *in,
				   struct motor_outer_loop_outputs *out)
{
	if (params == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}

	out->velocity_target_rad_s = in->velocity_target_rad_s;
	out->velocity_ref_rad_s = in->velocity_ref_rad_s;
	out->speed_mech_filtered_rad_s = in->speed_mech_rad_s;
	out->id_ref_a = in->id_ref_a;
	out->iq_ref_a = in->iq_ref_a;

	/* Position cascade: generate velocity target from position error. */
	if (in->state == &motor_states[MOTOR_STATE_ONLINE_POSITION]) {
		float32_t position_error_rad;
		float32_t profile_velocity_ff_rad_s = 0.0f;
		float32_t pos_i_limit_rad_s = params->profile_max_velocity_rad_s;
		bool use_mpr = motor_outer_loop_use_mpr(params);
		bool position_loop_update = motor_outer_loop_decimation_tick(
			&params->position_loop_phase, in->position_loop_decimation);

		/* Position move module resolves profile state into target/error/feedforward. */
		bool move_active = motor_position_move_resolve(&params->position_profile,
							       atomic_get(&params->control_armed) != 0,
							       in->position_mech_rad,
							       &params->position_target_rad,
							       &position_error_rad,
							       &profile_velocity_ff_rad_s);
		if (!move_active) {
			position_error_rad =
				wrap_rad_pi(params->position_target_rad - in->position_mech_rad);
		}

		if (position_loop_update) {
			if (use_mpr) {
				params->position_mpr_cfg.dt_s = in->position_loop_dt_s;
				params->position_mpr_cfg.velocity_limit_rad_s =
					params->profile_max_velocity_rad_s;
				params->position_mpr_cfg.max_delta_velocity_rad_s =
					params->profile_max_accel_rad_s2 * in->position_loop_dt_s;
				int mpr_ret = motor_mpr_position_step(&params->position_mpr_cfg,
								      &params->position_mpr_state,
								      position_error_rad,
								      profile_velocity_ff_rad_s,
								      &out->velocity_target_rad_s);
				if (mpr_ret != 0) {
					out->velocity_target_rad_s = clampf(profile_velocity_ff_rad_s,
									    -params->profile_max_velocity_rad_s,
									    params->profile_max_velocity_rad_s);
					motor_mpr_position_reset(&params->position_mpr_state,
								 out->velocity_target_rad_s);
				}
				params->position_cl_i_term_rad_s = 0.0f;
			} else {
				float32_t position_fb_velocity_rad_s;
				float32_t pos_i_next =
					params->position_cl_i_term_rad_s +
					(params->position_cl_ki_rad_s2_per_rad *
					 position_error_rad * in->position_loop_dt_s);
				pos_i_next = clampf(pos_i_next, -pos_i_limit_rad_s, pos_i_limit_rad_s);
				params->position_cl_i_term_rad_s = pos_i_next;
				position_fb_velocity_rad_s =
					params->position_cl_kp_rad_s_per_rad * position_error_rad +
					params->position_cl_i_term_rad_s;
				out->velocity_target_rad_s =
					profile_velocity_ff_rad_s + position_fb_velocity_rad_s;
			}

			out->velocity_target_rad_s =
				clampf(out->velocity_target_rad_s, -params->profile_max_velocity_rad_s,
				       params->profile_max_velocity_rad_s);
			traj_set_target_value(&params->traj_velocity, out->velocity_target_rad_s);
		}
	} else {
		params->position_loop_phase = 0U;
	}

	/* Update velocity trajectory if enabled */
	if (in->feature_velocity_traj) {
		motor_velocity_plan_step(&params->traj_velocity,
					&out->velocity_target_rad_s,
					&out->velocity_ref_rad_s);

		/* Open-loop commutation uses the trajectory directly. */
		if (in->feature_angle_gen) {
			angle_gen_set_velocity(&params->angle_gen, out->velocity_ref_rad_s);
		}
	}

	/* Closed-loop velocity and position share the same inner velocity->Iq stage. */
	if (in->state == &motor_states[MOTOR_STATE_ONLINE_VELOCITY_CLOSED] ||
	    in->state == &motor_states[MOTOR_STATE_ONLINE_POSITION]) {
		out->speed_mech_filtered_rad_s =
			filter_so_run(&params->filter_velocity_notch, in->speed_mech_rad_s);
		bool velocity_feedback_valid =
			motor_velocity_feedback_is_valid(params->position_quality_flags);
		bool velocity_feedback_fresh =
			(params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_FRESH) != 0U;
		bool velocity_loop_update = motor_outer_loop_decimation_tick(
			&params->velocity_loop_phase, in->velocity_loop_decimation);
		if (!velocity_feedback_valid) {
			/* Hold measured dq currents and reset outer-loop observers while encoder
			 * quality is degraded. This avoids current spikes when velocity/angle
			 * feedback is stale or glitched.
			 */
			out->id_ref_a = in->id_meas_a;
			out->iq_ref_a = in->iq_meas_a;
			out->velocity_target_rad_s = 0.0f;
			out->velocity_ref_rad_s = 0.0f;
			params->velocity_target_rad_s = 0.0f;
			params->velocity_ref_rad_s = 0.0f;
			traj_set_target_value(&params->traj_velocity, 0.0f);
			motor_mpr_velocity_reset(&params->velocity_mpr_state,
						 out->speed_mech_filtered_rad_s,
						 out->iq_ref_a);
			motor_dob_reset(&params->velocity_dob_state, out->speed_mech_filtered_rad_s);
			params->velocity_dob_iq_ff_a = 0.0f;
			params->velocity_dob_disturbance_nm = 0.0f;
			params->velocity_dob_residual_rad_s = 0.0f;
		} else if (velocity_loop_update && velocity_feedback_fresh) {
			bool use_mpr = motor_outer_loop_use_mpr(params);
			bool mpr_applied = false;
			float32_t iq_cmd_pre_dob_a = 0.0f;
			float32_t torque_gain_nm_per_a = motor_torque_gain_resolve_active(params);

			if (use_mpr) {
				struct motor_mpr_velocity_model mpr_model = {
					.inertia_kgm2 = params->inertia_kgm2_active,
					.viscous_friction_nm_per_rad_s =
						params->viscous_friction_nm_per_rad_s_active,
					.coulomb_friction_nm = params->coulomb_friction_nm_active,
					.torque_constant_nm_per_a = torque_gain_nm_per_a,
				};
				float32_t iq_cmd_mpr_a = 0.0f;

				params->velocity_mpr_cfg.dt_s = in->velocity_loop_dt_s;
				params->velocity_mpr_cfg.iq_limit_a = params->velocity_cl_iq_limit_A;
				int mpr_ret = motor_mpr_velocity_step(&params->velocity_mpr_cfg, &mpr_model,
								      &params->velocity_mpr_state,
								      out->speed_mech_filtered_rad_s,
								      out->velocity_ref_rad_s,
								      &iq_cmd_mpr_a);
				if (mpr_ret == 0) {
					out->id_ref_a = params->Id_setpoint_A;
					iq_cmd_pre_dob_a = clampf(iq_cmd_mpr_a,
								  -params->velocity_cl_iq_limit_A,
								  params->velocity_cl_iq_limit_A);
					out->iq_ref_a = iq_cmd_pre_dob_a;
					params->velocity_cl_i_term_A = 0.0f;
					mpr_applied = true;
				}
			}

			if (!mpr_applied) {
				float32_t speed_error_rad_s =
					out->velocity_ref_rad_s - out->speed_mech_filtered_rad_s;
				float32_t vel_i_next =
					params->velocity_cl_i_term_A +
					(params->velocity_cl_ki_A_per_rad * speed_error_rad_s *
					 in->velocity_loop_dt_s);
				vel_i_next = clampf(vel_i_next, -params->velocity_cl_iq_limit_A,
						   params->velocity_cl_iq_limit_A);
				params->velocity_cl_i_term_A = vel_i_next;

				out->id_ref_a = params->Id_setpoint_A;
				iq_cmd_pre_dob_a =
					clampf((params->velocity_cl_kp_A_per_rad_s * speed_error_rad_s) +
					       params->velocity_cl_i_term_A,
					       -params->velocity_cl_iq_limit_A,
					       params->velocity_cl_iq_limit_A);
				out->iq_ref_a = iq_cmd_pre_dob_a;
			}

			if (isfinite(torque_gain_nm_per_a) && torque_gain_nm_per_a > 0.0f) {
				struct motor_dob_model dob_model = {
					.inertia_kgm2 = params->inertia_kgm2_active,
					.viscous_friction_nm_per_rad_s =
						params->viscous_friction_nm_per_rad_s_active,
					.coulomb_friction_nm = params->coulomb_friction_nm_active,
					.torque_constant_nm_per_a = torque_gain_nm_per_a,
				};
				struct motor_dob_config dob_cfg = params->velocity_dob_cfg;
				float32_t iq_limit = params->velocity_cl_iq_limit_A;
				float32_t auto_torque_limit = torque_gain_nm_per_a * iq_limit;

				dob_cfg.dt_s = in->velocity_loop_dt_s;
				if (!isfinite(dob_cfg.torque_limit_nm) || dob_cfg.torque_limit_nm <= 0.0f) {
					dob_cfg.torque_limit_nm = auto_torque_limit;
				}
				if (!isfinite(dob_cfg.iq_ff_limit_a) || dob_cfg.iq_ff_limit_a <= 0.0f) {
					dob_cfg.iq_ff_limit_a = iq_limit;
				}

				float32_t iq_dob_ff_a = 0.0f;
				int dob_ret = motor_dob_step(&dob_cfg, &dob_model, &params->velocity_dob_state,
							     out->speed_mech_filtered_rad_s,
							     iq_cmd_pre_dob_a,
							     &iq_dob_ff_a);
				if (dob_ret == 0) {
					params->velocity_dob_iq_ff_a = iq_dob_ff_a;
					params->velocity_dob_disturbance_nm =
						params->velocity_dob_state.disturbance_nm;
					params->velocity_dob_residual_rad_s =
						params->velocity_dob_state.residual_rad_s;
					out->iq_ref_a = clampf(iq_cmd_pre_dob_a + iq_dob_ff_a,
							      -params->velocity_cl_iq_limit_A,
							      params->velocity_cl_iq_limit_A);
				} else {
					motor_dob_reset(&params->velocity_dob_state,
							out->speed_mech_filtered_rad_s);
					params->velocity_dob_iq_ff_a = 0.0f;
					params->velocity_dob_disturbance_nm = 0.0f;
					params->velocity_dob_residual_rad_s = 0.0f;
				}
			}
		}
	} else {
		params->velocity_loop_phase = 0U;
	}

	return 0;
}
