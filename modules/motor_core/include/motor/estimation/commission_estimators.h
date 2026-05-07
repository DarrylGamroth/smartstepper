/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMMISSION_ESTIMATORS_H_
#define MOTOR_COMMISSION_ESTIMATORS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/utils.h>

struct motor_flux_id_config {
	float32_t rs_ohm;
	float32_t ld_h;
	float32_t lq_h;
	float32_t min_abs_speed_rad_s;
	float32_t min_speed_span_rad_s;
	uint16_t min_samples;
	float32_t min_r2;
	bool require_positive_psi;
};

struct motor_flux_id_state {
	struct motor_flux_id_config cfg;
	bool config_valid;
	uint32_t sample_count;
	float32_t min_speed_rad_s;
	float32_t max_speed_rad_s;
	float32_t sx;
	float32_t sy;
	float32_t sxx;
	float32_t sxy;
	float32_t syy;
};

struct motor_flux_id_result {
	float32_t psi_f_wb;
	float32_t bias_v;
	float32_t residual_rms_v;
	float32_t r2;
	uint16_t sample_count;
	bool valid;
};

int motor_flux_id_validate_config(const struct motor_flux_id_config *cfg);
void motor_flux_id_init(struct motor_flux_id_state *state,
			const struct motor_flux_id_config *cfg);
bool motor_flux_id_accumulate(struct motor_flux_id_state *state,
			      float32_t elec_speed_rad_s,
			      float32_t id_a,
			      float32_t iq_a,
			      float32_t diq_dt_a_s,
			      float32_t vq_v);
int motor_flux_id_finalize(const struct motor_flux_id_state *state,
			   struct motor_flux_id_result *result);

struct motor_mech_id_config {
	float32_t kt_nm_per_a;
	float32_t sign_deadband_rad_s;
	uint16_t min_samples;
	float32_t min_r2;
	bool require_positive_inertia;
	bool require_nonnegative_viscous;
};

struct motor_mech_id_state {
	struct motor_mech_id_config cfg;
	bool config_valid;
	uint32_t sample_count;
	float32_t A[4][4];
	float32_t b[4];
	float32_t sum_z;
	float32_t sum_z2;
};

struct motor_mech_id_result {
	float32_t inertia_kgm2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
	float32_t offset_friction_nm;
	float32_t residual_rms_nm;
	float32_t r2;
	uint16_t sample_count;
	bool valid;
};

int motor_mech_id_validate_config(const struct motor_mech_id_config *cfg);
void motor_mech_id_init(struct motor_mech_id_state *state,
			const struct motor_mech_id_config *cfg);
bool motor_mech_id_accumulate(struct motor_mech_id_state *state,
			      float32_t mech_speed_rad_s,
			      float32_t mech_accel_rad_s2,
			      float32_t iq_a);
int motor_mech_id_finalize(const struct motor_mech_id_state *state,
			   struct motor_mech_id_result *result);

struct motor_mech_friction_id_config {
	float32_t kt_nm_per_a;
	float32_t sign_deadband_rad_s;
	float32_t max_abs_accel_rad_s2;
	uint16_t min_samples;
	uint16_t min_samples_per_direction;
	float32_t min_r2;
	bool require_nonnegative_viscous;
	bool require_nonnegative_coulomb;
};

struct motor_mech_friction_id_state {
	struct motor_mech_friction_id_config cfg;
	bool config_valid;
	uint32_t sample_count;
	uint32_t positive_count;
	uint32_t negative_count;
	uint32_t rejected_count;
	float32_t A[3][3];
	float32_t b[3];
	float32_t sum_z;
	float32_t sum_z2;
};

struct motor_mech_friction_id_result {
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
	float32_t offset_friction_nm;
	float32_t residual_rms_nm;
	float32_t r2;
	uint16_t sample_count;
	uint16_t positive_count;
	uint16_t negative_count;
	uint16_t rejected_count;
	bool valid;
};

int motor_mech_friction_id_validate_config(const struct motor_mech_friction_id_config *cfg);
void motor_mech_friction_id_init(struct motor_mech_friction_id_state *state,
				 const struct motor_mech_friction_id_config *cfg);
bool motor_mech_friction_id_accumulate(struct motor_mech_friction_id_state *state,
				       float32_t mech_speed_rad_s,
				       float32_t mech_accel_rad_s2,
				       float32_t iq_a,
				       float32_t detent_torque_nm);
int motor_mech_friction_id_finalize(const struct motor_mech_friction_id_state *state,
				    struct motor_mech_friction_id_result *result);

struct motor_mech_inertia_id_config {
	float32_t kt_nm_per_a;
	float32_t sign_deadband_rad_s;
	float32_t min_abs_accel_rad_s2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
	float32_t offset_friction_nm;
	float32_t fallback_inertia_kgm2;
	float32_t min_plausibility_ratio;
	float32_t max_plausibility_ratio;
	uint16_t min_samples;
	uint16_t min_samples_per_accel_direction;
	float32_t max_residual_rms_nm;
	bool require_plausible;
};

struct motor_mech_inertia_id_state {
	struct motor_mech_inertia_id_config cfg;
	bool config_valid;
	uint32_t sample_count;
	uint32_t positive_accel_count;
	uint32_t negative_accel_count;
	uint32_t rejected_count;
	float32_t sum_alpha_tau;
	float32_t sum_alpha2;
	float32_t sum_tau;
	float32_t sum_tau2;
};

struct motor_mech_inertia_id_result {
	float32_t inertia_kgm2;
	float32_t residual_rms_nm;
	float32_t r2;
	float32_t plausibility_ratio;
	uint16_t sample_count;
	uint16_t positive_accel_count;
	uint16_t negative_accel_count;
	uint16_t rejected_count;
	bool valid;
};

int motor_mech_inertia_id_validate_config(const struct motor_mech_inertia_id_config *cfg);
void motor_mech_inertia_id_init(struct motor_mech_inertia_id_state *state,
				const struct motor_mech_inertia_id_config *cfg);
bool motor_mech_inertia_id_accumulate(struct motor_mech_inertia_id_state *state,
				      float32_t mech_speed_rad_s,
				      float32_t mech_accel_rad_s2,
				      float32_t iq_a,
				      float32_t detent_torque_nm);
int motor_mech_inertia_id_finalize(const struct motor_mech_inertia_id_state *state,
				   struct motor_mech_inertia_id_result *result);

bool motor_mech_accel_window_velocity_fit(const float32_t *speed_rad_s,
					  const uint32_t *loop_count,
					  uint16_t sample_count,
					  uint16_t sample_index,
					  uint16_t half_window,
					  float32_t control_loop_frequency_hz,
					  float32_t *accel_rad_s2);

#endif /* MOTOR_COMMISSION_ESTIMATORS_H_ */
