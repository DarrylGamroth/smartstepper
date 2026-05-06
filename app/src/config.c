/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "config.h"
#include "motor/math/math_constants.h"

LOG_MODULE_REGISTER(config, CONFIG_APP_LOG_LEVEL);

void config_init_filters(struct motor_parameters *params)
{
	/* Calculate filter coefficients from offset measurement pole frequency */
	float32_t a1 = expf(-2.0f * PI_F32 * OFFSET_POLE_HZ / CONTROL_LOOP_FREQUENCY_HZ);
	float32_t b0 = 1.0f - a1;

	filter_fo_init(&params->filter_Ia);
	filter_fo_set_den_coeffs(&params->filter_Ia, a1);
	filter_fo_set_num_coeffs(&params->filter_Ia, b0, 0.0f);

	filter_fo_init(&params->filter_Ib);
	filter_fo_set_den_coeffs(&params->filter_Ib, a1);
	filter_fo_set_num_coeffs(&params->filter_Ib, b0, 0.0f);

	filter_so_init(&params->filter_velocity_notch);
	filter_so_set_passthrough(&params->filter_velocity_notch);
	if (VELOCITY_NOTCH_FILTER_ENABLED) {
		int ret = filter_so_config_notch(&params->filter_velocity_notch,
						 CONTROL_LOOP_FREQUENCY_HZ,
						 VELOCITY_NOTCH_FREQ_HZ,
						 VELOCITY_NOTCH_Q);
		if (ret == 0) {
			LOG_INF("Velocity notch enabled: f0=%.1f Hz, Q=%.3f",
				(double)VELOCITY_NOTCH_FREQ_HZ, (double)VELOCITY_NOTCH_Q);
		} else {
			filter_so_set_passthrough(&params->filter_velocity_notch);
			LOG_WRN("Velocity notch config invalid (err %d), falling back to passthrough",
				ret);
		}
	}

	LOG_DBG("Filters initialized: a1=%.6f, b0=%.6f",
		(double)a1, (double)b0);
}

void config_init_pi_controllers(struct motor_parameters *params)
{
	/* Calculate intermediate values */
	float32_t rd_over_ld_rps = MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_D_H;
	float32_t rq_over_lq_rps = MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_Q_H;
	float32_t bwc_rps = CURRENT_LOOP_BANDWIDTH_RPS;
	float32_t current_ctrl_period_sec = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;

	/* D-axis PI controller
	 * Kp_Id = Ls_d * BWc_rps
	 * Ki_Id = (Rs/Ls_d) * Ti
	 */
	float32_t kp_d = MOTOR_INDUCTANCE_D_H * bwc_rps;
	float32_t ki_d = rd_over_ld_rps * current_ctrl_period_sec;

	/* Initialize PI controller (NOT double buffered - single instance in main struct) */
	pi_init(&params->pi_Id);
	pi_set_gains(&params->pi_Id, kp_d, ki_d);

	/* Q-axis PI controller
	 * Kp_Iq = Ls_q * BWc_rps
	 * Ki_Iq = (Rs/Ls_q) * Ti
	 */
	float32_t kp_q = MOTOR_INDUCTANCE_Q_H * bwc_rps;
	float32_t ki_q = rq_over_lq_rps * current_ctrl_period_sec;

	/* Initialize PI controller (NOT double buffered - single instance in main struct) */
	pi_init(&params->pi_Iq);
	pi_set_gains(&params->pi_Iq, kp_q, ki_q);

	LOG_DBG("PI controllers initialized: Ti=%.6f s",
		(double)current_ctrl_period_sec);
	LOG_DBG("  D-axis: Kp=%.6f, Ki=%.6f (R/L=%.1f rad/s)", 
		(double)kp_d, (double)ki_d, (double)rd_over_ld_rps);
	LOG_DBG("  Q-axis: Kp=%.6f, Ki=%.6f (R/L=%.1f rad/s)", 
		(double)kp_q, (double)ki_q, (double)rq_over_lq_rps);
}

void config_init_runtime_adapters(struct motor_parameters *params)
{
	if (params == NULL) {
		return;
	}

	params->rt_adapters.outer_loop = (struct motor_outer_loop_runtime_ctx){
		.outer_loop_mode = params->outer_loop_mode,
		.position_profile = &params->position_profile,
		.control_armed = false,
		.position_target_rad = &params->position_target_rad,
		.profile_max_velocity_rad_s = params->profile_max_velocity_rad_s,
		.profile_max_accel_rad_s2 = params->profile_max_accel_rad_s2,
		.position_mpr_cfg = &params->position_mpr_cfg,
		.position_mpr_state = &params->position_mpr_state,
		.position_cl_kp_rad_s_per_rad = params->position_cl_kp_rad_s_per_rad,
		.position_cl_ki_rad_s2_per_rad = params->position_cl_ki_rad_s2_per_rad,
		.position_cl_i_term_rad_s = &params->position_cl_i_term_rad_s,
		.position_reg_state = &params->position_reg_state,
		.traj_velocity = &params->traj_velocity,
		.filter_velocity_notch = &params->filter_velocity_notch,
		.position_quality_flags = params->live.position_quality_flags,
		.live_velocity_target_rad_s = &params->live.velocity_target_rad_s,
		.live_velocity_ref_rad_s = &params->live.velocity_ref_rad_s,
		.velocity_reg_state = &params->velocity_reg_state,
		.velocity_mpr_cfg = &params->velocity_mpr_cfg,
		.velocity_mpr_state = &params->velocity_mpr_state,
		.velocity_cl_i_term_a = &params->velocity_cl_i_term_A,
		.velocity_cl_kp_a_per_rad_s = params->velocity_cl_kp_A_per_rad_s,
		.velocity_cl_ki_a_per_rad = params->velocity_cl_ki_A_per_rad,
		.velocity_cl_iq_limit_a = params->velocity_cl_iq_limit_A,
		.id_setpoint_a = params->Id_setpoint_A,
		.torque_gain_nm_per_a_active = params->torque_gain_nm_per_a_active,
		.flux_linkage_wb_active = params->flux_linkage_wb_active,
		.default_flux_linkage_wb = MOTOR_FLUX_LINKAGE_WB,
		.pole_pairs = MOTOR_POLE_PAIRS,
		.inertia_kgm2_active = params->inertia_kgm2_active,
		.viscous_friction_nm_per_rad_s_active =
			params->viscous_friction_nm_per_rad_s_active,
		.coulomb_friction_nm_active = params->coulomb_friction_nm_active,
		.velocity_dob_cfg = &params->velocity_dob_cfg,
		.velocity_dob_state = &params->velocity_dob_state,
		.live_velocity_dob_iq_ff_a = &params->live.velocity_dob_iq_ff_a,
		.live_velocity_dob_disturbance_nm = &params->live.velocity_dob_disturbance_nm,
		.live_velocity_dob_residual_rad_s = &params->live.velocity_dob_residual_rad_s,
		.detent_map_cfg = &params->detent_map_cfg,
		.detent_map_state = &params->detent_map_state,
		.live_detent_iq_ff_a = &params->live.detent_iq_ff_a,
	};

	params->rt_adapters.current_ref_policy = (struct motor_current_ref_policy_ctx){
		.position_quality_flags = params->live.position_quality_flags,
		.id_setpoint_a = &params->Id_setpoint_A,
		.iq_setpoint_a = &params->Iq_setpoint_A,
		.pi_id = &params->pi_Id,
		.pi_iq = &params->pi_Iq,
		.live_velocity_target_rad_s = &params->live.velocity_target_rad_s,
		.live_velocity_ref_rad_s = &params->live.velocity_ref_rad_s,
		.velocity_cl_i_term_a = &params->velocity_cl_i_term_A,
		.position_cl_i_term_rad_s = &params->position_cl_i_term_rad_s,
		.velocity_reg_state = &params->velocity_reg_state,
		.position_reg_state = &params->position_reg_state,
		.velocity_mpr_state = &params->velocity_mpr_state,
		.position_mpr_state = &params->position_mpr_state,
		.velocity_dob_state = &params->velocity_dob_state,
		.live_velocity_dob_iq_ff_a = &params->live.velocity_dob_iq_ff_a,
		.live_velocity_dob_disturbance_nm = &params->live.velocity_dob_disturbance_nm,
		.live_velocity_dob_residual_rad_s = &params->live.velocity_dob_residual_rad_s,
	};

	params->rt_adapters.rls = (struct motor_rls_runtime_ctx){
		.rls_feature_enabled = false,
		.control_loop_count = 0U,
		.control_loop_frequency_hz = CONTROL_LOOP_FREQUENCY_HZ,
		.rls = {
			.prbs_gen = &params->rls.prbs_gen,
			.d = &params->rls.d,
			.q = &params->rls.q,
			.decimation = params->rls.decimation,
			.stagger_offset = params->rls.stagger_offset,
			.excitation_current_a = params->rls.excitation_current_a,
			.ld_est_h = &params->rls.ld_est_h,
			.lq_est_h = &params->rls.lq_est_h,
			.id_prev_a = &params->rls.id_prev_a,
			.iq_prev_a = &params->rls.iq_prev_a,
			.d_prev_cycle = &params->rls.d_prev_cycle,
			.q_prev_cycle = &params->rls.q_prev_cycle,
			.d_prev_valid = &params->rls.d_prev_valid,
			.q_prev_valid = &params->rls.q_prev_valid,
			.min_current_a = params->rls.min_current_a,
			.min_speed_rad_s = params->rls.min_speed_rad_s,
			.max_residual_v = params->rls.max_residual_v,
			.max_voltage_v = params->rls.max_voltage_v,
		},
		.observer = &params->observer,
		.vd_v = params->Vd_V,
		.vq_v = params->Vq_V,
		.pi_id = &params->pi_Id,
		.pi_iq = &params->pi_Iq,
		.default_flux_linkage_wb = MOTOR_FLUX_LINKAGE_WB,
		.rs_measured_ohm = &params->Rs_measured_ohm,
		.thermal = {
			.model = &params->thermal.model,
			.decimation = params->thermal.decimation,
			.rs_ref_ohm = params->thermal.rs_ref_ohm,
			.rs_ref_temp_c = params->thermal.rs_ref_temp_c,
			.rs_temp_coeff = params->thermal.rs_temp_coeff,
			.t_rls_c = &params->thermal.t_rls_c,
		},
	};

	params->rt_adapters.encoder_feedback = (struct motor_encoder_feedback_ctx){
		.fault_counter = &params->encoder_fault_counter,
		.warning_count = &params->encoder_warning_count,
		.error_count = &params->encoder_error_count,
		.sample_fresh = &params->live.encoder_sample_fresh,
		.sample_warning = &params->live.encoder_sample_warning,
		.sample_error = &params->live.encoder_sample_error,
		.last_status = &params->live.encoder_last_status,
		.encoder_direction_sign = params->encoder_direction_sign,
		.angle_gen = &params->angle_gen,
		.observer = &params->observer,
		.observer_input_rad = &params->live.encoder_observer_input_rad,
		.encoder_input_source = &params->live.encoder_input_source,
		.encoder_raw_deg = &params->live.encoder_raw_deg,
		.encoder_raw_rad = &params->live.encoder_raw_rad,
		.position_stale_count = &params->live.position_stale_count,
		.position_stale_events = &params->live.position_stale_events,
		.position_glitch_count = &params->live.position_glitch_count,
		.position_jitter_count = &params->live.position_jitter_count,
		.position_quality_flags = &params->live.position_quality_flags,
		.position_trust_state = &params->live.position_trust_state,
		.encoder_fault_threshold = ENCODER_FAULT_THRESHOLD,
		.encoder_delay_samples = ENCODER_SAMPLE_DELAY_SAMPLES,
		.pole_pairs = MOTOR_POLE_PAIRS,
	};

	params->rt_adapters.commission = (struct motor_commission_runtime_ctx){
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
	};
}

void config_print_parameters(void)
{
	LOG_INF("Motor Controller Configuration");
	LOG_INF("================================");
	LOG_INF("System Parameters:");
	LOG_INF("  Nominal Voltage: %.2f V", (double)NOMINAL_VOLTAGE_V);
	LOG_INF("  PWM Frequency: %.0f Hz", (double)PWM_FREQUENCY_HZ);
	LOG_INF("  Control Loop Frequency: %.0f Hz", (double)CONTROL_LOOP_FREQUENCY_HZ);
	LOG_INF("  Current Loop Bandwidth: %.0f Hz", (double)CURRENT_LOOP_BANDWIDTH_HZ);
	LOG_INF("  Offset Filter Pole: %.0f Hz", (double)OFFSET_POLE_HZ);
	LOG_INF("  Max Voltage: %.1f%% of bus", (double)(MAX_VS_MPU * 100.0f));

	LOG_INF("Current Sensing:");
	LOG_INF("  Resistor: %.0fmOhm, Gain: %.0fx, Full Scale: %.1fA",
		(double)(CURRENT_SENSE_RESISTOR_OHM * 1000.0f),
		(double)CURRENT_SENSE_GAIN,
		(double)CURRENT_SENSE_FULL_SCALE_A);

	LOG_INF("Motor Parameters:");
	LOG_INF("  Ld=%.1fuH, Lq=%.1fuH",
		(double)(MOTOR_INDUCTANCE_D_H * 1e6f),
		(double)(MOTOR_INDUCTANCE_Q_H * 1e6f));
	LOG_INF("  R=%.1fmOhm", (double)(MOTOR_RESISTANCE_OHM * 1000.0f));
	LOG_INF("  Flux linkage=%.3fuV/Hz", (double)(MOTOR_FLUX_LINKAGE_VPH_ELEC * 1000000.0f));
	LOG_INF("  Pole pairs=%d", MOTOR_POLE_PAIRS);
	LOG_INF("  Encoder direction sign=%d", ENCODER_DIRECTION_SIGN);
	LOG_INF("  Max current=%.1fA", (double)MOTOR_MAX_CURRENT_A);
	LOG_INF("  Max speed=%.0fHz", (double)MOTOR_MAX_SPEED_HZ);
	LOG_INF("  Inertia=%.3fkgcm²", (double)(MOTOR_INERTIA_KGM2 * 10000.0f));
	LOG_INF("Control Options:");
	LOG_INF("  DQ Decoupling=%s", CURRENT_DECOUPLING_ENABLED ? "ON" : "OFF");
	LOG_INF("  Velocity notch=%s%s",
		VELOCITY_NOTCH_FILTER_ENABLED ? "ON" : "OFF",
		VELOCITY_NOTCH_FILTER_ENABLED ? "" : " (passthrough)");
	if (VELOCITY_NOTCH_FILTER_ENABLED) {
		LOG_INF("  Velocity notch f0=%.1fHz, Q=%.3f",
			(double)VELOCITY_NOTCH_FREQ_HZ, (double)VELOCITY_NOTCH_Q);
	}

	LOG_INF("Alignment:");
	LOG_INF("  Current=%.2fA, Duration=%.1fs",
		(double)ALIGN_CURRENT_A,
		(double)ALIGN_DURATION_S);
}
