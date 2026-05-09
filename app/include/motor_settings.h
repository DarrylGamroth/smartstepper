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

#define MOTOR_SETTINGS_GROUP_ENCODER          (1U << 0)
#define MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL (1U << 1)
#define MOTOR_SETTINGS_GROUP_CONTROLLERS      (1U << 2)
#define MOTOR_SETTINGS_GROUP_DETENT           (1U << 3)
#define MOTOR_SETTINGS_GROUP_IDENTITY         (1U << 4)
#define MOTOR_SETTINGS_GROUP_LIMITS           (1U << 5)
#define MOTOR_SETTINGS_GROUP_CHOPPER          (1U << 6)
#define MOTOR_SETTINGS_GROUP_ALL \
	(MOTOR_SETTINGS_GROUP_ENCODER | MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL | \
	 MOTOR_SETTINGS_GROUP_CONTROLLERS | MOTOR_SETTINGS_GROUP_DETENT | \
	 MOTOR_SETTINGS_GROUP_IDENTITY | MOTOR_SETTINGS_GROUP_LIMITS | \
	 MOTOR_SETTINGS_GROUP_CHOPPER)
#define MOTOR_SETTINGS_GROUP_BASELINE \
	(MOTOR_SETTINGS_GROUP_ENCODER | MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL)

#define MOTOR_SETTINGS_SCHEMA_VERSION 5U
#define MOTOR_SETTINGS_CHOPPER_MAX_CENTERS 64U

struct motor_settings_snapshot {
	uint32_t schema_version;
	uint32_t generation;
	uint32_t valid_groups;
	bool autoload_enabled;
	uint32_t autoload_groups;

	int8_t encoder_direction_sign;
	float32_t encoder_commutation_offset_mech_rad;
	float32_t encoder_trim_elec_rad;
	float32_t encoder_mapping_correlation;
	float32_t encoder_mapping_residual_rad;

	uint16_t identity_pole_pairs;

	float32_t model_rs_ohm;
	float32_t model_ld_h;
	float32_t model_lq_h;
	float32_t model_flux_linkage_wb;
	float32_t model_inertia_kgm2;
	float32_t model_viscous_friction_nm_per_rad_s;
	float32_t model_coulomb_friction_nm;

	uint8_t ctrl_outer_loop_mode;
	float32_t ctrl_velocity_bandwidth_hz;
	float32_t ctrl_position_bandwidth_hz;
	float32_t ctrl_damping_ratio;
	float32_t ctrl_velocity_iq_limit_a;
	bool ctrl_velocity_dob_enabled;
	float32_t ctrl_velocity_dob_gain_scale;

	float32_t limits_nominal_voltage_v;
	float32_t limits_max_current_a;
	float32_t limits_brake_current_a;
	float32_t limits_max_velocity_hz;
	float32_t limits_max_accel_hz_s;
	uint32_t limits_command_timeout_ms;

	bool detent_enabled;
	uint16_t detent_bins;
	int16_t detent_phase_advance_bins;
	float32_t detent_gain;
	float32_t detent_iq_ff_limit_a;
	uint32_t detent_table_crc32;

	uint16_t chopper_slots;
	uint16_t chopper_teeth;
	uint16_t chopper_edge_count;
	float32_t chopper_edges_rad[MOTOR_SETTINGS_CHOPPER_MAX_CENTERS];
	uint8_t chopper_edge_region_after[MOTOR_SETTINGS_CHOPPER_MAX_CENTERS];
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
int motor_settings_autoload_read(bool *enabled, uint32_t *groups);
int motor_settings_autoload_set(bool enabled, uint32_t groups);
int motor_settings_autoload_apply(struct motor_parameters *params,
				  uint32_t *loaded_groups);
const char *motor_settings_key_root(void);

#endif /* MOTOR_SETTINGS_H_ */
