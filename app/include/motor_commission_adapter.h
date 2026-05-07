/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMMISSION_ADAPTER_H_
#define MOTOR_COMMISSION_ADAPTER_H_

#include "config.h"

static inline void motor_commission_runtime_ctx_init(struct motor_commission_runtime_ctx *ctx,
						     struct motor_parameters *params)
{
	if (ctx == NULL || params == NULL) {
		return;
	}

	*ctx = (struct motor_commission_runtime_ctx){
		.commission = &params->commission,
		.control_loop_count = &params->control_loop_count,
		.control_loop_frequency_hz = CONTROL_LOOP_FREQUENCY_HZ,
		.motor_max_current_a = MAX(MOTOR_MAX_CURRENT_A, 0.1f),
		.pole_pairs = MOTOR_POLE_PAIRS,
		.default_flux_linkage_wb = MOTOR_FLUX_LINKAGE_WB,
		.rs_measured_ohm = &params->Rs_measured_ohm,
		.rls_ld_est_h = &params->rls.ld_est_h,
		.rls_lq_est_h = &params->rls.lq_est_h,
		.flux_linkage_wb_active = &params->flux_linkage_wb_active,
		.torque_gain_nm_per_a_active = &params->torque_gain_nm_per_a_active,
		.inertia_kgm2_active = &params->inertia_kgm2_active,
		.viscous_friction_nm_per_rad_s_active =
			&params->viscous_friction_nm_per_rad_s_active,
		.coulomb_friction_nm_active = &params->coulomb_friction_nm_active,
		.flux_model_source = &params->flux_model_source,
		.mech_model_source = &params->mech_model_source,
		.velocity_cl_kp_a_per_rad_s = &params->velocity_cl_kp_A_per_rad_s,
		.velocity_cl_ki_a_per_rad = &params->velocity_cl_ki_A_per_rad,
		.velocity_cl_iq_limit_a = &params->velocity_cl_iq_limit_A,
		.velocity_cl_i_term_a = &params->velocity_cl_i_term_A,
		.position_cl_kp_rad_s_per_rad = &params->position_cl_kp_rad_s_per_rad,
		.position_cl_ki_rad_s2_per_rad = &params->position_cl_ki_rad_s2_per_rad,
		.position_cl_i_term_rad_s = &params->position_cl_i_term_rad_s,
		.profile_max_velocity_rad_s = params->profile_max_velocity_rad_s,
		.profile_max_accel_rad_s2 = params->profile_max_accel_rad_s2,
		.velocity_mpr_cfg = &params->velocity_mpr_cfg,
		.velocity_mpr_state = &params->velocity_mpr_state,
		.position_mpr_cfg = &params->position_mpr_cfg,
		.position_mpr_state = &params->position_mpr_state,
		.velocity_dob_cfg = &params->velocity_dob_cfg,
		.velocity_dob_state = &params->velocity_dob_state,
		.live_velocity_rad_s = &params->live.velocity_rad_s,
		.live_position_rad = &params->live.position_rad,
		.live_velocity_dob_iq_ff_a = &params->live.velocity_dob_iq_ff_a,
		.live_velocity_dob_disturbance_nm = &params->live.velocity_dob_disturbance_nm,
		.live_velocity_dob_residual_rad_s = &params->live.velocity_dob_residual_rad_s,
		.detent_map_cfg = &params->detent_map_cfg,
	};
}

#endif /* MOTOR_COMMISSION_ADAPTER_H_ */
