/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_PERSISTENT_CONFIG_H_
#define MOTOR_RUNTIME_PERSISTENT_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PERSISTENT_CONFIG_MAGIC 0x4D504331U /* "MPC1" */
#define MOTOR_PERSISTENT_CONFIG_SCHEMA_V1 1U

enum motor_persistent_config_flags {
	MOTOR_PERSISTENT_CONFIG_FLAG_CURRENT_OFFSETS_VALID = (1U << 0),
	MOTOR_PERSISTENT_CONFIG_FLAG_ENCODER_MAPPING_VALID = (1U << 1),
	MOTOR_PERSISTENT_CONFIG_FLAG_MOTOR_MODEL_VALID = (1U << 2),
	MOTOR_PERSISTENT_CONFIG_FLAG_CONTROLLERS_VALID = (1U << 3),
	MOTOR_PERSISTENT_CONFIG_FLAG_DETENT_META_VALID = (1U << 4),
};

struct motor_persistent_config_header {
	uint32_t magic;
	uint16_t schema_version;
	uint16_t header_size;
	uint16_t payload_size;
	uint16_t record_size;
	uint32_t generation;
	uint32_t flags;
	uint32_t payload_crc32;
};

struct motor_persistent_current_offsets_v1 {
	float32_t ia_offset_a;
	float32_t ib_offset_a;
};

struct motor_persistent_encoder_mapping_v1 {
	int8_t direction_sign;
	uint8_t reserved0[3];
	float32_t commutation_offset_mech_rad;
	float32_t trim_elec_rad;
	float32_t mapping_correlation;
	float32_t mapping_residual_rad;
};

struct motor_persistent_motor_model_v1 {
	float32_t rs_ohm;
	float32_t ld_h;
	float32_t lq_h;
	float32_t flux_linkage_wb;
	float32_t kt_nm_per_a;
	float32_t inertia_kgm2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
};

struct motor_persistent_controller_v1 {
	float32_t velocity_kp_a_per_rad_s;
	float32_t velocity_ki_a_per_rad;
	float32_t velocity_iq_limit_a;
	float32_t position_kp_rad_s_per_rad;
	float32_t position_ki_rad_s2_per_rad;
	float32_t velocity_mpr_q_speed;
	float32_t velocity_mpr_r_delta_iq;
	float32_t velocity_mpr_max_delta_iq_a;
	uint16_t velocity_mpr_horizon;
	uint16_t position_mpr_horizon;
	float32_t position_mpr_q_position;
	float32_t position_mpr_q_velocity_ff;
	float32_t position_mpr_r_delta_velocity;
	float32_t position_mpr_max_delta_velocity_rad_s;
	bool velocity_dob_enabled;
	uint8_t reserved0[3];
	float32_t velocity_dob_observer_gain_nm_per_rad_s;
	float32_t velocity_dob_torque_limit_nm;
	float32_t velocity_dob_iq_ff_limit_a;
};

struct motor_persistent_detent_meta_v1 {
	bool enabled;
	uint8_t reserved0[3];
	uint16_t bins;
	uint16_t reserved1;
	float32_t gain;
	float32_t limit_a;
	uint32_t table_crc32;
};

struct motor_persistent_config_payload_v1 {
	struct motor_persistent_current_offsets_v1 current_offsets;
	struct motor_persistent_encoder_mapping_v1 encoder_mapping;
	struct motor_persistent_motor_model_v1 motor_model;
	struct motor_persistent_controller_v1 controllers;
	struct motor_persistent_detent_meta_v1 detent;
};

struct motor_persistent_config_v1 {
	struct motor_persistent_config_header header;
	struct motor_persistent_config_payload_v1 payload;
};

uint32_t motor_persistent_config_payload_crc32(
	const struct motor_persistent_config_payload_v1 *payload);

void motor_persistent_config_v1_prepare(struct motor_persistent_config_v1 *record,
					uint32_t generation,
					uint32_t flags);

bool motor_persistent_config_v1_validate(const struct motor_persistent_config_v1 *record);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RUNTIME_PERSISTENT_CONFIG_H_ */
