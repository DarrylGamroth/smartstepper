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

#include "motor/control/dob.h"
#include "motor/control/mpr.h"
#include "motor/runtime/commission_tune.h"

#define MOTOR_COMMISSION_MAX_SAMPLES 512U

#ifndef MOTOR_MODEL_SOURCE_FALLBACK
#define MOTOR_MODEL_SOURCE_FALLBACK 0U
#endif
#ifndef MOTOR_MODEL_SOURCE_MEASURED
#define MOTOR_MODEL_SOURCE_MEASURED 1U
#endif

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
	float32_t base_speed_hz;
	float32_t dither_speed_hz;
	uint32_t dither_period_ms;
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
	float32_t iq_move_min_pos_a;
	float32_t iq_move_min_neg_a;
	float32_t iq_move_recommended_a;
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
	float32_t inertia_stddev_kgm2;
	float32_t viscous_friction_stddev_nm_per_rad_s;
	float32_t coulomb_friction_stddev_nm;
	float32_t mech_validation_residual_rms_nm;
	float32_t mech_confidence;
	int32_t mech_finalize_error;
	uint16_t mech_sample_count;
	uint8_t mech_capture_count;
	uint8_t mech_reject_reason;
	int8_t mech_fit_torque_sign;
	float32_t mapping_direction_corr;
	float32_t mapping_offset_ratio;
	float32_t mapping_pole_pairs_est;
	float32_t mapping_confidence;
	uint16_t iq_move_pos_sample_count;
	uint16_t iq_move_neg_sample_count;
	uint16_t iq_move_warning_count;
	uint16_t iq_move_error_count;
	int8_t iq_to_mech_sign;
	bool iq_move_pos_valid;
	bool iq_move_neg_valid;
	bool iq_move_valid;
	bool psi_f_valid;
	bool mech_valid;
	bool mech_validation_valid;
	bool mech_validation_pass;
	bool mapping_direction_valid;
	bool mapping_direction_pass;
	bool mapping_offset_valid;
	bool mapping_offset_pass;
	bool mapping_pole_pairs_valid;
	bool mapping_pole_pairs_pass;
	bool mapping_valid;
	bool mapping_pass;
};

enum motor_commission_mech_reject_reason {
	MOTOR_COMMISSION_MECH_REJECT_NONE = 0,
	MOTOR_COMMISSION_MECH_REJECT_KT_INVALID = 1,
	MOTOR_COMMISSION_MECH_REJECT_SAMPLES = 2,
	MOTOR_COMMISSION_MECH_REJECT_SOLVER = 3,
	MOTOR_COMMISSION_MECH_REJECT_FINALIZE = 4,
	MOTOR_COMMISSION_MECH_REJECT_VISCOUS_NEGATIVE = 5,
	MOTOR_COMMISSION_MECH_REJECT_FIT_INVALID = 6,
};

struct motor_commission_observation {
	uint32_t control_loop_count;
	bool mode_velocity_encoder;
	bool mode_current_encoder;
	bool control_armed;
	bool encoder_fresh;
	bool encoder_warning;
	bool encoder_error;
	uint8_t encoder_status;
	bool fault_active;
	bool saturation;
	bool data_valid;
	float32_t velocity_ref_rad_s;
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
	uint32_t reject_velocity_tracking;

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
	struct motor_commission_tune_config auto_tune_cfg;
	struct motor_commission_tune_output auto_tune_staged;
	bool auto_tune_valid;
	bool auto_tune_applied;
	int32_t auto_tune_last_error;

	uint16_t sample_count;
	struct motor_commission_sample samples[MOTOR_COMMISSION_MAX_SAMPLES];

	char last_abort_reason[32];
};

struct motor_commission_runtime_ctx {
	struct motor_commission_ctx *commission;
	uint32_t *control_loop_count;
	float32_t control_loop_frequency_hz;
	float32_t motor_max_current_a;
	uint16_t pole_pairs;
	float32_t default_flux_linkage_wb;
	float32_t *rs_measured_ohm;
	float32_t *rls_ld_est_h;
	float32_t *rls_lq_est_h;
	float32_t *flux_linkage_wb_active;
	float32_t *torque_gain_nm_per_a_active;
	float32_t *inertia_kgm2_active;
	float32_t *viscous_friction_nm_per_rad_s_active;
	float32_t *coulomb_friction_nm_active;
	uint8_t *flux_model_source;
	uint8_t *mech_model_source;
	float32_t *velocity_cl_kp_a_per_rad_s;
	float32_t *velocity_cl_ki_a_per_rad;
	float32_t *velocity_cl_iq_limit_a;
	float32_t *velocity_cl_i_term_a;
	float32_t *position_cl_kp_rad_s_per_rad;
	float32_t *position_cl_ki_rad_s2_per_rad;
	float32_t *position_cl_i_term_rad_s;
	float32_t profile_max_velocity_rad_s;
	float32_t profile_max_accel_rad_s2;
	struct motor_mpr_velocity_config *velocity_mpr_cfg;
	struct motor_mpr_velocity_state *velocity_mpr_state;
	struct motor_mpr_position_config *position_mpr_cfg;
	struct motor_mpr_position_state *position_mpr_state;
	struct motor_dob_config *velocity_dob_cfg;
	struct motor_dob_state *velocity_dob_state;
	float32_t *live_velocity_rad_s;
	float32_t *live_position_rad;
	float32_t *live_velocity_dob_iq_ff_a;
	float32_t *live_velocity_dob_disturbance_nm;
	float32_t *live_velocity_dob_residual_rad_s;
};

void motor_commission_init(struct motor_commission_runtime_ctx *ctx);
void motor_commission_reset(struct motor_commission_runtime_ctx *ctx);
int motor_commission_start_flux(struct motor_commission_runtime_ctx *ctx,
				const struct motor_commission_flux_config *cfg);
int motor_commission_start_mech(struct motor_commission_runtime_ctx *ctx,
				const struct motor_commission_mech_config *cfg);
void motor_commission_abort(struct motor_commission_runtime_ctx *ctx, const char *reason);
int motor_commission_apply_results(struct motor_commission_runtime_ctx *ctx);
int motor_commission_stage_auto_tune(struct motor_commission_runtime_ctx *ctx,
				     const struct motor_commission_tune_config *cfg);
int motor_commission_apply_staged_auto_tune(struct motor_commission_runtime_ctx *ctx);
void motor_commission_update(struct motor_commission_runtime_ctx *ctx,
			     const struct motor_commission_observation *obs);
bool motor_commission_is_active(const struct motor_commission_runtime_ctx *ctx);
const char *motor_commission_mode_to_string(uint8_t mode);
const char *motor_commission_stage_to_string(uint8_t stage);

#endif /* MOTOR_COMMISSION_H_ */
