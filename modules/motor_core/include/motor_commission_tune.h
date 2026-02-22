/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMMISSION_TUNE_H_
#define MOTOR_COMMISSION_TUNE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

enum motor_commission_tune_reject {
	MOTOR_COMMISSION_TUNE_REJECT_NONE = 0U,
	MOTOR_COMMISSION_TUNE_REJECT_PSI_INVALID = BIT(0),
	MOTOR_COMMISSION_TUNE_REJECT_PSI_SAMPLES = BIT(1),
	MOTOR_COMMISSION_TUNE_REJECT_PSI_R2 = BIT(2),
	MOTOR_COMMISSION_TUNE_REJECT_PSI_RMS = BIT(3),
	MOTOR_COMMISSION_TUNE_REJECT_PSI_SIGN = BIT(4),
	MOTOR_COMMISSION_TUNE_REJECT_MECH_INVALID = BIT(5),
	MOTOR_COMMISSION_TUNE_REJECT_MECH_SAMPLES = BIT(6),
	MOTOR_COMMISSION_TUNE_REJECT_MECH_R2 = BIT(7),
	MOTOR_COMMISSION_TUNE_REJECT_MECH_RMS = BIT(8),
	MOTOR_COMMISSION_TUNE_REJECT_INERTIA_SIGN = BIT(9),
	MOTOR_COMMISSION_TUNE_REJECT_VISCOUS_SIGN = BIT(10),
	MOTOR_COMMISSION_TUNE_REJECT_KT_INVALID = BIT(11),
};

struct motor_commission_fit_summary {
	bool psi_f_valid;
	float32_t psi_f_wb;
	float32_t psi_f_r2;
	float32_t psi_f_residual_rms_v;
	uint16_t psi_f_sample_count;
	bool mech_valid;
	float32_t inertia_kgm2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t mech_r2;
	float32_t mech_residual_rms_nm;
	uint16_t mech_sample_count;
};

struct motor_commission_tune_config {
	float32_t pole_pairs;
	float32_t dt_s;
	float32_t velocity_bw_hz;
	float32_t velocity_zeta;
	float32_t position_bw_ratio;
	float32_t position_zeta;
	float32_t iq_limit_a;
	float32_t max_current_a;
	float32_t profile_max_velocity_rad_s;
	float32_t profile_max_accel_rad_s2;
	float32_t min_flux_r2;
	float32_t min_mech_r2;
	float32_t max_flux_rms_v;
	float32_t max_mech_rms_nm;
	uint16_t min_flux_samples;
	uint16_t min_mech_samples;
};

struct motor_commission_tune_output {
	bool accepted;
	uint32_t reject_flags;
	float32_t kt_nm_per_a;
	float32_t velocity_bw_hz;
	float32_t position_bw_hz;

	/* PI defaults */
	float32_t velocity_kp_a_per_rad_s;
	float32_t velocity_ki_a_per_rad;
	float32_t velocity_iq_limit_a;
	float32_t position_kp_rad_s_per_rad;
	float32_t position_ki_rad_s2_per_rad;

	/* MPR defaults */
	uint16_t velocity_mpr_horizon;
	float32_t velocity_mpr_q_speed;
	float32_t velocity_mpr_r_delta_iq;
	float32_t velocity_mpr_max_delta_iq_a;
	float32_t velocity_mpr_disturbance_ki_nm_per_rad_s;
	uint16_t position_mpr_horizon;
	float32_t position_mpr_q_position;
	float32_t position_mpr_q_velocity_ff;
	float32_t position_mpr_r_delta_velocity;
	float32_t position_mpr_max_delta_velocity_rad_s;

	/* DOB defaults */
	bool velocity_dob_enable;
	float32_t velocity_dob_observer_gain_nm_per_rad_s;
	float32_t velocity_dob_torque_limit_nm;
	float32_t velocity_dob_iq_ff_limit_a;
};

int motor_commission_tune_config_default(struct motor_commission_tune_config *cfg,
					 float32_t pole_pairs,
					 float32_t dt_s,
					 float32_t max_current_a,
					 float32_t profile_max_velocity_rad_s,
					 float32_t profile_max_accel_rad_s2);

int motor_commission_tune_compute(const struct motor_commission_fit_summary *fit,
				  const struct motor_commission_tune_config *cfg,
				  struct motor_commission_tune_output *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_COMMISSION_TUNE_H_ */
