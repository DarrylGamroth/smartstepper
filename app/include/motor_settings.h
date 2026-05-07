/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_SETTINGS_H_
#define MOTOR_SETTINGS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_parameters;

#define MOTOR_SETTINGS_GROUP_ENCODER     (1U << 0)
#define MOTOR_SETTINGS_GROUP_MODEL       (1U << 1)
#define MOTOR_SETTINGS_GROUP_CONTROLLERS (1U << 2)
#define MOTOR_SETTINGS_GROUP_DETENT      (1U << 3)
#define MOTOR_SETTINGS_GROUP_ALL \
	(MOTOR_SETTINGS_GROUP_ENCODER | MOTOR_SETTINGS_GROUP_MODEL | \
	 MOTOR_SETTINGS_GROUP_CONTROLLERS | MOTOR_SETTINGS_GROUP_DETENT)

#define MOTOR_SETTINGS_SCHEMA_VERSION 1U

struct motor_settings_snapshot {
	uint32_t schema_version;
	uint32_t generation;
	uint32_t valid_groups;

	int8_t encoder_direction_sign;
	float32_t encoder_commutation_offset_mech_rad;
	float32_t encoder_trim_elec_rad;
	float32_t encoder_mapping_correlation;
	float32_t encoder_mapping_residual_rad;

	float32_t model_rs_ohm;
	float32_t model_ld_h;
	float32_t model_lq_h;
	float32_t model_flux_linkage_wb;
	float32_t model_kt_nm_per_a;
	float32_t model_inertia_kgm2;
	float32_t model_viscous_friction_nm_per_rad_s;
	float32_t model_coulomb_friction_nm;

	float32_t ctrl_velocity_kp_a_per_rad_s;
	float32_t ctrl_velocity_ki_a_per_rad;
	float32_t ctrl_velocity_iq_limit_a;
	float32_t ctrl_position_kp_rad_s_per_rad;
	float32_t ctrl_position_ki_rad_s2_per_rad;
	float32_t ctrl_velocity_mpr_q_speed;
	float32_t ctrl_velocity_mpr_r_delta_iq;
	float32_t ctrl_velocity_mpr_max_delta_iq_a;
	float32_t ctrl_velocity_mpr_disturbance_ki_nm_per_rad_s;
	uint16_t ctrl_velocity_mpr_horizon;
	float32_t ctrl_position_mpr_q_position;
	float32_t ctrl_position_mpr_q_velocity_ff;
	float32_t ctrl_position_mpr_r_delta_velocity;
	float32_t ctrl_position_mpr_max_delta_velocity_rad_s;
	uint16_t ctrl_position_mpr_horizon;
	bool ctrl_velocity_dob_enabled;
	float32_t ctrl_velocity_dob_observer_gain_nm_per_rad_s;
	float32_t ctrl_velocity_dob_torque_limit_nm;
	float32_t ctrl_velocity_dob_iq_ff_limit_a;

	bool detent_enabled;
	uint16_t detent_bins;
	int16_t detent_phase_advance_bins;
	float32_t detent_gain;
	float32_t detent_iq_ff_limit_a;
	uint32_t detent_table_crc32;
};

int motor_settings_read(struct motor_settings_snapshot *snapshot,
				uint32_t *present_groups);
int motor_settings_save(const struct motor_parameters *params,
			uint32_t groups,
			uint32_t *saved_groups);
int motor_settings_load(struct motor_parameters *params,
			uint32_t groups,
			uint32_t *loaded_groups);
int motor_settings_clear(uint32_t groups);
int motor_settings_clear_all(void);
bool motor_settings_autoload_enabled(void);
const char *motor_settings_key_root(void);

#endif /* MOTOR_SETTINGS_H_ */
