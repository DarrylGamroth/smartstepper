/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMMISSION_H_
#define MOTOR_COMMISSION_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>

struct motor_parameters;

#define MOTOR_COMMISSION_MAX_SAMPLES 512U

enum motor_commission_mode {
	MOTOR_COMMISSION_MODE_NONE = 0,
	MOTOR_COMMISSION_MODE_FLUX = 1,
	MOTOR_COMMISSION_MODE_MECH = 2,
};

enum motor_commission_stage {
	MOTOR_COMMISSION_STAGE_IDLE = 0,
	MOTOR_COMMISSION_STAGE_RUNNING = 1,
	MOTOR_COMMISSION_STAGE_COMPLETED = 2,
	MOTOR_COMMISSION_STAGE_ABORTED = 3,
};

enum motor_commission_expected_mode {
	MOTOR_COMMISSION_EXPECT_ANY = 0,
	MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED = 1,
	MOTOR_COMMISSION_EXPECT_TORQUE = 2,
};

struct motor_commission_flux_config {
	float32_t min_speed_hz;
	float32_t max_speed_hz;
	uint16_t steps;
	uint32_t settle_ms;
	uint32_t sample_ms;
	float32_t iq_limit_a;
};

struct motor_commission_mech_config {
	float32_t coast_speed_hz;
	float32_t prbs_amp_a;
	uint32_t prbs_period_ms;
	uint32_t duration_ms;
};

struct motor_commission_sample {
	uint32_t loop_count;
	float32_t mech_speed_rad_s;
	float32_t elec_speed_rad_s;
	float32_t mech_accel_rad_s2;
	float32_t id_a;
	float32_t iq_a;
	float32_t did_dt_a_s;
	float32_t diq_dt_a_s;
	float32_t vd_v;
	float32_t vq_v;
	float32_t vbus_v;
	uint8_t encoder_status;
	uint8_t flags;
};

struct motor_commission_results {
	float32_t psi_f_wb;
	float32_t psi_f_bias_v;
	float32_t psi_f_residual_rms_v;
	float32_t psi_f_r2;
	uint16_t psi_f_sample_count;
	float32_t inertia_kgm2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
	float32_t offset_friction_nm;
	float32_t mech_residual_rms_nm;
	float32_t mech_r2;
	uint16_t mech_sample_count;
	bool psi_f_valid;
	bool mech_valid;
};

struct motor_commission_observation {
	uint32_t control_loop_count;
	const struct smf_state *state;
	bool control_armed;
	bool encoder_fresh;
	bool encoder_warning;
	bool encoder_error;
	uint8_t encoder_status;
	bool fault_active;
	bool saturation;
	bool data_valid;
	float32_t vbus_v;
	float32_t id_a;
	float32_t iq_a;
	float32_t vd_v;
	float32_t vq_v;
	float32_t mech_speed_rad_s;
	float32_t elec_speed_rad_s;
};

struct motor_commission_ctx {
	bool active;
	uint8_t mode;
	uint8_t stage;
	uint8_t expected_mode;

	uint32_t start_loop_count;
	uint32_t stop_loop_count;
	uint32_t sample_decimation;
	uint32_t sample_decimation_counter;

	uint32_t accepted_samples;
	uint32_t rejected_samples;
	uint32_t reject_mode_mismatch;
	uint32_t reject_disarmed;
	uint32_t reject_encoder;
	uint32_t reject_fault;
	uint32_t reject_saturation;
	uint32_t reject_data_invalid;

	bool prev_valid;
	uint32_t prev_loop_count;
	float32_t prev_id_a;
	float32_t prev_iq_a;
	float32_t prev_speed_rad_s;
	float32_t did_dt_filt_a_s;
	float32_t diq_dt_filt_a_s;
	float32_t domega_dt_filt_rad_s2;

	struct motor_commission_flux_config flux_cfg;
	struct motor_commission_mech_config mech_cfg;
	struct motor_commission_results results;

	uint16_t sample_count;
	struct motor_commission_sample samples[MOTOR_COMMISSION_MAX_SAMPLES];

	char last_abort_reason[32];
};

void motor_commission_init(struct motor_parameters *params);
void motor_commission_reset(struct motor_parameters *params);
int motor_commission_start_flux(struct motor_parameters *params,
				const struct motor_commission_flux_config *cfg);
int motor_commission_start_mech(struct motor_parameters *params,
				const struct motor_commission_mech_config *cfg);
void motor_commission_abort(struct motor_parameters *params, const char *reason);
int motor_commission_apply_results(struct motor_parameters *params);
void motor_commission_update(struct motor_parameters *params,
			     const struct motor_commission_observation *obs);
bool motor_commission_is_active(const struct motor_parameters *params);
const char *motor_commission_mode_to_string(uint8_t mode);
const char *motor_commission_stage_to_string(uint8_t stage);

#endif /* MOTOR_COMMISSION_H_ */
