/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include <zephyr/dsp/utils.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <dsp/controller_functions.h>

#include "motor_control_loop.h"
#include "motor/math/math_constants.h"
#include "config.h"
#include "motor/filters/pi.h"
#include "motor/filters/filter_fo.h"
#include "motor/motion/traj.h"
#include "motor/observers/angle_observer.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"
#include "motor/runtime/config_snapshot.h"
#include "motor/estimation/rls_runtime.h"
#include "motor/control/foc_voltage_pwm.h"
#include "motor/runtime/commission_runtime.h"
#include "motor/control/dob.h"
#include "motor/motion/motion_planner.h"
#include "motor/calibration/align.h"
#include "motor/calibration/rl_ident.h"
#include "motor/observers/encoder_feedback.h"
#include "motor/observers/feedback.h"
#include "motor/telemetry/capture.h"
#include "motor/runtime/outer_loop_runtime.h"
#include "motor/runtime/current_ref_policy_runtime.h"
#include "motor/runtime/feedback_quality.h"
#include "motor/protection/interlocks.h"
#include "motor/control/dq_decoupling.h"
#include "motor/control/transforms.h"

/**
 * @brief Convert Q31 ADC value to current in Amperes
 *
 * @param q31_value Q31 format ADC reading (0.0 to +1.0 represented as int32)
 * @param polarity Polarity multiplier (1 or -1)
 * @return Current in Amperes (offset removal done separately)
 */
static inline float32_t adc_to_current(q31_t q31_value, int polarity)
{
	float32_t normalized = (float32_t)q31_value / (float32_t)(1U << 31);
	/* Direct scaling - offset measurement handles midpoint centering */
	return (float32_t)polarity * normalized * CURRENT_SENSE_FULL_SCALE_A * 2.0f;
}

/**
 * @brief Convert Q31 ADC value to bus voltage in Volts
 *
 * @param q31_value Q31 format ADC reading (0.0 to +1.0 represented as int32)
 * @return Bus voltage in Volts
 */
static inline float32_t adc_to_vbus_v(q31_t q31_value)
{
	float32_t normalized = (float32_t)q31_value / (float32_t)(1U << 31);
	return normalized * VBUS_FULL_SCALE_V;
}

#define VBUS_MIN_VALID_V 0.1f
#define CURRENT_DECOUPLING_MIN_MECH_SPEED_HZ 0.75f
#define CURRENT_DECOUPLING_MIN_MECH_SPEED_RAD_S \
	(2.0f * PI_F32 * CURRENT_DECOUPLING_MIN_MECH_SPEED_HZ)
#define CURRENT_DECOUPLING_MIN_FLUX_WB 1.0e-5f
#define CURRENT_DECOUPLING_MAX_FLUX_WB 1.0f
#define CURRENT_DQ_DECOUPLING_FLUX_HEADROOM_RATIO 0.60f
#define CURRENT_DQ_DECOUPLING_FF_LIMIT_RATIO 0.70f

static inline bool motor_rt_mode_active(uint32_t mode_flags, uint32_t flag)
{
	return (mode_flags & flag) != 0U;
}

static inline bool motor_is_align_injection_state(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_POS_INJECT) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_NEG_INJECT);
}

static inline bool motor_is_align_sample_state(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_POS_SAMPLE) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_NEG_SAMPLE);
}

static inline bool motor_is_align_active_state(uint32_t mode_flags)
{
	return motor_is_align_injection_state(mode_flags) || motor_is_align_sample_state(mode_flags);
}

static inline void motor_align_load_pos_accum(const struct motor_parameters *params,
					      struct motor_align_sample_accum *acc)
{
	if (params == NULL || acc == NULL) {
		return;
	}

	acc->sum_sin = params->calibration.align_pos_sum_sin;
	acc->sum_cos = params->calibration.align_pos_sum_cos;
	acc->count = params->calibration.align_pos_sample_count;
}

static inline void motor_align_store_pos_accum(struct motor_parameters *params,
					       const struct motor_align_sample_accum *acc)
{
	if (params == NULL || acc == NULL) {
		return;
	}

	params->calibration.align_pos_sum_sin = acc->sum_sin;
	params->calibration.align_pos_sum_cos = acc->sum_cos;
	params->calibration.align_pos_sample_count = acc->count;
}

static inline void motor_align_load_neg_accum(const struct motor_parameters *params,
					      struct motor_align_sample_accum *acc)
{
	if (params == NULL || acc == NULL) {
		return;
	}

	acc->sum_sin = params->calibration.align_neg_sum_sin;
	acc->sum_cos = params->calibration.align_neg_sum_cos;
	acc->count = params->calibration.align_neg_sample_count;
}

static inline void motor_align_store_neg_accum(struct motor_parameters *params,
					       const struct motor_align_sample_accum *acc)
{
	if (params == NULL || acc == NULL) {
		return;
	}

	params->calibration.align_neg_sum_sin = acc->sum_sin;
	params->calibration.align_neg_sum_cos = acc->sum_cos;
	params->calibration.align_neg_sample_count = acc->count;
}

static inline void motor_fault_snapshot_prepare(struct motor_control_step_report *report,
						float32_t encoder_angle_deg,
						float32_t observer_input_rad,
						float32_t elec_angle_rad,
						float32_t observer_elec_speed_rad_s,
						float32_t id_ref_a,
						float32_t iq_ref_a,
						float32_t id_a,
						float32_t iq_a,
						float32_t ia_a,
						float32_t ib_a,
						float32_t vd_v,
						float32_t vq_v,
						uint8_t input_source,
						bool sample_fresh,
						bool sample_warning,
						bool sample_error,
						uint8_t status,
						uint8_t position_quality_flags)
{
	if (report == NULL) {
		return;
	}

	report->fault_snapshot.valid = true;
	report->fault_snapshot.encoder_angle_deg = encoder_angle_deg;
	report->fault_snapshot.observer_input_rad = observer_input_rad;
	report->fault_snapshot.elec_angle_rad = elec_angle_rad;
	report->fault_snapshot.observer_elec_speed_rad_s = observer_elec_speed_rad_s;
	report->fault_snapshot.id_ref_a = id_ref_a;
	report->fault_snapshot.iq_ref_a = iq_ref_a;
	report->fault_snapshot.id_a = id_a;
	report->fault_snapshot.iq_a = iq_a;
	report->fault_snapshot.ia_a = ia_a;
	report->fault_snapshot.ib_a = ib_a;
	report->fault_snapshot.vd_v = vd_v;
	report->fault_snapshot.vq_v = vq_v;
	report->fault_snapshot.input_source = input_source;
	report->fault_snapshot.sample_fresh = sample_fresh ? 1U : 0U;
	report->fault_snapshot.sample_warning = sample_warning ? 1U : 0U;
	report->fault_snapshot.sample_error = sample_error ? 1U : 0U;
	report->fault_snapshot.status = status;
	report->fault_snapshot.position_quality_flags = position_quality_flags;
}

static inline void motor_step_report_post_error(struct motor_control_step_report *report,
						uint32_t error_code)
{
	if (report == NULL || report->error_pending) {
		return;
	}

	report->error_pending = true;
	report->error_code = error_code;
}

static inline void motor_runtime_fast_sync(struct motor_parameters *params,
					   uint32_t mode_flags,
					   bool control_armed)
{
	params->rt_fast.control_loop_count = params->control_loop_count;
	params->rt_fast.rls_d_prev_cycle = params->rls.d_prev_cycle;
	params->rt_fast.rls_q_prev_cycle = params->rls.q_prev_cycle;
	params->rt_fast.mode_flags_shadow = mode_flags;
	params->rt_fast.rls_d_prev_valid = params->rls.d_prev_valid;
	params->rt_fast.rls_q_prev_valid = params->rls.q_prev_valid;
	params->rt_fast.Id_setpoint_A = params->Id_setpoint_A;
	params->rt_fast.Iq_setpoint_A = params->Iq_setpoint_A;
	params->rt_fast.Vd_V = params->Vd_V;
	params->rt_fast.Vq_V = params->Vq_V;
	params->rt_fast.feature_flags_shadow = atomic_get(&params->feature_flags);
	params->rt_fast.control_armed_shadow = control_armed;
}

static inline void motor_runtime_diag_sync(struct motor_parameters *params)
{
	params->rt_diag.state_counter = params->state_counter;
	params->rt_diag.encoder_fault_counter = params->encoder_fault_counter;
	params->rt_diag.encoder_warning_count = params->encoder_warning_count;
	params->rt_diag.encoder_error_count = params->encoder_error_count;
	params->rt_diag.max_isr_cycles = params->max_isr_cycles;
	params->rt_diag.total_isr_cycles = params->total_isr_cycles;
	params->rt_diag.overrun_count = params->overrun_count;
	params->rt_diag.encoder_capture_overrun_count = params->encoder_capture.overrun_count;
	params->rt_diag.fault_snapshot_overrun_count = params->fault_snapshot.overrun_count;
	params->rt_diag.fault_snapshot_latch_loop = params->fault_snapshot.latch_loop;
	params->rt_diag.fault_snapshot_latch_error_code = params->fault_snapshot.latch_error_code;
	params->rt_diag.command_timeout_count = params->command_timeout_count;
	params->rt_diag.profile_sequence_event_drop_count = params->profile_seq.event_drop_count;
}

static inline void motor_control_feedback_from_encoder(
	const struct motor_encoder_feedback *encoder_fb,
	struct motor_control_feedback *control_fb)
{
	memset(control_fb, 0, sizeof(*control_fb));
	if (encoder_fb == NULL) {
		return;
	}

	control_fb->sample_enabled = encoder_fb->sample_enabled;
	control_fb->sample_available = encoder_fb->sample_available;
	control_fb->fresh = encoder_fb->fresh;
	control_fb->warning = encoder_fb->warning;
	control_fb->error = encoder_fb->error;
	control_fb->io_fault = encoder_fb->io_fault;
	control_fb->status = encoder_fb->status;
	control_fb->input_source = encoder_fb->input_source;
	control_fb->angle_sensor_deg = encoder_fb->angle_sensor_deg;
	control_fb->angle_control_deg = encoder_fb->angle_control_deg;
	control_fb->observer_input_rad = encoder_fb->observer_input_rad;
	control_fb->observer_mech_rad = encoder_fb->observer_mech_rad;
	control_fb->observer_elec_rad = encoder_fb->observer_elec_rad;
	control_fb->position_mech_rad = encoder_fb->control.position_mech_rad;
	control_fb->speed_mech_rad_s = encoder_fb->control.speed_mech_rad_s;
	control_fb->accel_mech_rad_s2 = encoder_fb->control.accel_mech_rad_s2;
	control_fb->speed_mech_filtered_rad_s = encoder_fb->control.speed_mech_filtered_rad_s;
	control_fb->input_source = encoder_fb->control.input_source;
}

struct motor_control_step_ctx {
	uint32_t config_epoch;
	atomic_val_t feature_flags;
	uint32_t mode_flags;
	bool feature_angle_gen;
	bool feature_pwm_output;
	bool feature_pi_control;
	bool feature_velocity_traj;
	bool feature_use_commanded_currents;
	bool feature_braking;
	bool online_control_state;
	bool control_armed;
	float32_t dt_s;
	uint32_t velocity_loop_decimation;
	uint32_t position_loop_decimation;
	float32_t velocity_loop_dt_s;
	float32_t position_loop_dt_s;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t position_mech_rad;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;
};

struct motor_encoder_stage_result {
	struct motor_encoder_feedback feedback;
	struct motor_control_feedback control_fb;
	uint8_t input_source;
	float32_t angle_control_deg;
	bool fresh;
	uint8_t frame_status;
	bool frame_warning;
	bool frame_error;
};

static inline void motor_outer_loop_runtime_ctx_refresh(struct motor_outer_loop_runtime_ctx *ctx,
							struct motor_parameters *params,
							bool control_armed)
{
	ctx->outer_loop_mode = params->outer_loop_mode;
	ctx->control_armed = control_armed;
	ctx->profile_max_velocity_rad_s = params->profile_max_velocity_rad_s;
	ctx->profile_max_accel_rad_s2 = params->profile_max_accel_rad_s2;
	ctx->position_cl_kp_rad_s_per_rad = params->position_cl_kp_rad_s_per_rad;
	ctx->position_cl_ki_rad_s2_per_rad = params->position_cl_ki_rad_s2_per_rad;
	ctx->position_quality_flags = params->live.position_quality_flags;
	ctx->velocity_cl_kp_a_per_rad_s = params->velocity_cl_kp_A_per_rad_s;
	ctx->velocity_cl_ki_a_per_rad = params->velocity_cl_ki_A_per_rad;
	ctx->velocity_cl_iq_limit_a = params->velocity_cl_iq_limit_A;
	ctx->id_setpoint_a = params->Id_setpoint_A;
	ctx->torque_gain_nm_per_a_active = params->torque_gain_nm_per_a_active;
	ctx->flux_linkage_wb_active = params->flux_linkage_wb_active;
	ctx->inertia_kgm2_active = params->inertia_kgm2_active;
	ctx->viscous_friction_nm_per_rad_s_active =
		params->viscous_friction_nm_per_rad_s_active;
	ctx->coulomb_friction_nm_active = params->coulomb_friction_nm_active;
}

static inline void motor_current_ref_policy_ctx_refresh(struct motor_current_ref_policy_ctx *ctx,
							struct motor_parameters *params)
{
	ctx->position_quality_flags = params->live.position_quality_flags;
}

static inline void motor_rls_runtime_ctx_refresh(struct motor_rls_runtime_ctx *ctx,
						 struct motor_parameters *params)
{
	ctx->rls_feature_enabled =
		atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION);
	ctx->control_loop_count = params->control_loop_count;
	ctx->vd_v = params->Vd_V;
	ctx->vq_v = params->Vq_V;
}

static inline void motor_encoder_feedback_ctx_refresh(struct motor_encoder_feedback_ctx *ctx,
						      struct motor_parameters *params)
{
	ctx->encoder_direction_sign = params->encoder_direction_sign;
}

static inline void motor_commission_runtime_ctx_refresh(struct motor_commission_runtime_ctx *ctx,
							struct motor_parameters *params)
{
	ctx->profile_max_velocity_rad_s = params->profile_max_velocity_rad_s;
	ctx->profile_max_accel_rad_s2 = params->profile_max_accel_rad_s2;
}

static inline void motor_control_step_cfg_init(struct motor_control_step_ctx *ctx,
					       const struct motor_parameters *params)
{
	memset(ctx, 0, sizeof(*ctx));
	struct motor_rt_config_snapshot cfg = {0};
	bool cfg_valid = motor_config_snapshot_read(&cfg);
	ctx->config_epoch = cfg.epoch;
	ctx->feature_flags = cfg_valid ? cfg.feature_flags : atomic_get(&params->feature_flags);
	ctx->mode_flags = cfg_valid ? cfg.mode_flags : params->rt_fast.mode_flags_shadow;
	ctx->feature_angle_gen = (ctx->feature_flags & BIT(MOTOR_FEATURE_ANGLE_GEN)) != 0;
	ctx->feature_pwm_output = (ctx->feature_flags & BIT(MOTOR_FEATURE_PWM_OUTPUT)) != 0;
	ctx->feature_pi_control = (ctx->feature_flags & BIT(MOTOR_FEATURE_PI_CONTROL)) != 0;
	ctx->feature_velocity_traj = (ctx->feature_flags & BIT(MOTOR_FEATURE_VELOCITY_TRAJ)) != 0;
	ctx->feature_use_commanded_currents =
		(ctx->feature_flags & BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS)) != 0;
	ctx->feature_braking = (ctx->feature_flags & BIT(MOTOR_FEATURE_BRAKING)) != 0;
	ctx->online_control_state = motor_rt_mode_active(ctx->mode_flags, MOTOR_RT_MODE_ONLINE_CONTROL);
	ctx->control_armed = atomic_get(&params->control_armed) != 0;
	ctx->dt_s = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;
	ctx->velocity_loop_decimation =
		CLAMP(cfg_valid ? cfg.velocity_loop_decimation : params->velocity_loop_decimation,
		      OUTER_LOOP_DECIMATION_MIN,
		      OUTER_LOOP_DECIMATION_MAX);
	ctx->position_loop_decimation =
		CLAMP(cfg_valid ? cfg.position_loop_decimation : params->position_loop_decimation,
		      OUTER_LOOP_DECIMATION_MIN,
		      OUTER_LOOP_DECIMATION_MAX);
	ctx->velocity_loop_dt_s = ctx->dt_s * (float32_t)ctx->velocity_loop_decimation;
	ctx->position_loop_dt_s = ctx->dt_s * (float32_t)ctx->position_loop_decimation;
	ctx->velocity_target_rad_s = params->live.velocity_target_rad_s;
	ctx->velocity_ref_rad_s = params->live.velocity_ref_rad_s;
	ctx->position_mech_rad = params->live.position_rad;
	ctx->speed_mech_rad_s = params->live.velocity_rad_s;
	ctx->accel_mech_rad_s2 = params->live.acceleration_rad_s2;
	ctx->speed_mech_filtered_rad_s = params->live.velocity_filtered_rad_s;
}

static inline void motor_core_step_init_pwm_output(struct motor_control_pwm_output *pwm_out)
{
	if (pwm_out == NULL) {
		return;
	}

	pwm_out->update_pwm = false;
	pwm_out->da_hb1_pu = 0.0f;
	pwm_out->da_hb2_pu = 0.0f;
	pwm_out->db_hb1_pu = 0.0f;
	pwm_out->db_hb2_pu = 0.0f;
}

static inline void motor_core_step_init_commission_obs(struct motor_commission_observation *obs,
						       uint32_t mode_flags,
						       bool control_armed)
{
	*obs = (struct motor_commission_observation){
		.control_loop_count = 0U,
		.mode_velocity_closed = motor_rt_mode_active(
			mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_CLOSED),
		.mode_torque = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_TORQUE),
		.control_armed = control_armed,
		.encoder_fresh = false,
		.encoder_warning = false,
		.encoder_error = false,
		.encoder_status = 0U,
		.fault_active = false,
		.saturation = false,
		.data_valid = false,
		.vbus_v = 0.0f,
		.id_a = 0.0f,
		.iq_a = 0.0f,
		.vd_v = 0.0f,
		.vq_v = 0.0f,
		.mech_speed_rad_s = 0.0f,
		.elec_speed_rad_s = 0.0f,
	};
}

static int motor_control_step_read_encoder(struct motor_parameters *params,
					   uint32_t mode_flags,
					   const struct motor_control_encoder_sample *encoder_sample,
					   bool feature_angle_gen,
					   struct motor_commission_observation *commission_obs,
					   struct motor_encoder_stage_result *enc_res)
{
	struct motor_encoder_feedback_ctx encoder_ctx;
	motor_encoder_feedback_ctx_refresh(&params->rt_adapters.encoder_feedback, params);
	encoder_ctx = params->rt_adapters.encoder_feedback;
	int enc_ret = motor_encoder_feedback_update(&encoder_ctx, encoder_sample, feature_angle_gen,
						    &enc_res->feedback);
	motor_control_feedback_from_encoder(&enc_res->feedback, &enc_res->control_fb);

	enc_res->input_source = enc_res->control_fb.input_source;
	enc_res->angle_control_deg = enc_res->control_fb.angle_control_deg;
	enc_res->fresh = enc_res->control_fb.fresh;
	enc_res->frame_status = enc_res->control_fb.status;
	enc_res->frame_warning = enc_res->control_fb.warning;
	enc_res->frame_error = enc_res->control_fb.error;

	if (enc_ret == -EIO) {
		return -EIO;
	}

	if (enc_res->control_fb.sample_enabled) {
		commission_obs->encoder_fresh = enc_res->fresh;
		commission_obs->encoder_warning = enc_res->frame_warning;
		commission_obs->encoder_error = enc_res->frame_error;
		commission_obs->encoder_status = enc_res->frame_status;
	}

	if (motor_is_align_sample_state(mode_flags) &&
	    enc_res->input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER &&
	    enc_res->fresh &&
	    !enc_res->frame_warning &&
	    !enc_res->frame_error) {
		float32_t align_mech_rad = enc_res->control_fb.observer_mech_rad;
		if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_POS_SAMPLE)) {
			struct motor_align_sample_accum acc = {0};
			motor_align_load_pos_accum(params, &acc);
			motor_align_accum_push(&acc, align_mech_rad);
			motor_align_store_pos_accum(params, &acc);
		} else {
			struct motor_align_sample_accum acc = {0};
			motor_align_load_neg_accum(params, &acc);
			motor_align_accum_push(&acc, align_mech_rad);
			motor_align_store_neg_accum(params, &acc);
		}
	}

	return 0;
}

static inline void motor_control_step_publish_encoder_sidework(
	struct motor_parameters *params,
	const struct motor_control_encoder_sample *encoder_sample,
	const struct motor_encoder_stage_result *enc_res,
	struct motor_control_step_report *report)
{
	if (enc_res == NULL) {
		return;
	}

	if (params->encoder_capture.enabled) {
		struct motor_capture_feedback capture_fb = {0};
		(void)motor_encoder_feedback_prepare_capture(&params->rt_adapters.encoder_feedback,
							     &enc_res->feedback,
							     &capture_fb);
		if (report != NULL) {
			report->encoder_capture_valid = true;
			report->encoder_capture = capture_fb;
		}
	}

	if (report != NULL && params->encoder_raw_trace.enabled && encoder_sample != NULL) {
		report->encoder_raw_trace_valid = true;
		report->encoder_sample = *encoder_sample;
		report->encoder_feedback = enc_res->control_fb;
		report->position_quality_flags = params->live.position_quality_flags;
	}
}

static inline void motor_core_step_finalize(struct motor_parameters *params,
					    uint32_t mode_flags,
					    bool control_armed,
					    struct motor_commission_observation *commission_obs)
{
	commission_obs->control_armed = control_armed;
	commission_obs->mode_velocity_closed = motor_rt_mode_active(
		mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_CLOSED);
	commission_obs->mode_torque = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_TORQUE);
	commission_obs->fault_active = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ERROR);
	motor_runtime_fast_sync(params, mode_flags, control_armed);
	motor_runtime_diag_sync(params);
	motor_commission_runtime_ctx_refresh(&params->rt_adapters.commission, params);
	motor_commission_update(&params->rt_adapters.commission, commission_obs);
}

void motor_control_loop_step(struct motor_parameters *params,
			     const q31_t *values,
			     uint8_t count,
			     const struct motor_control_encoder_sample *encoder_sample,
			     struct motor_control_pwm_output *pwm_out,
			     struct motor_control_step_report *report)
{
	ARG_UNUSED(count);
	if (params == NULL || values == NULL || pwm_out == NULL) {
		return;
	}

	if (report != NULL) {
		memset(report, 0, sizeof(*report));
	}

	struct motor_control_step_ctx ctx;
	motor_control_step_cfg_init(&ctx, params);

	uint32_t mode_flags = ctx.mode_flags;
	bool feature_angle_gen = ctx.feature_angle_gen;
	bool feature_pwm_output = ctx.feature_pwm_output;
	bool feature_pi_control = ctx.feature_pi_control;
	bool feature_velocity_traj = ctx.feature_velocity_traj;
	bool feature_use_commanded_currents = ctx.feature_use_commanded_currents;
	bool feature_braking = ctx.feature_braking;
	bool online_control_state = ctx.online_control_state;
	bool control_armed = ctx.control_armed;
	struct motor_commission_observation commission_obs;
	motor_core_step_init_commission_obs(&commission_obs, mode_flags, control_armed);
	motor_core_step_init_pwm_output(pwm_out);

	/* Increment control loop counter */
	params->control_loop_count++;
	commission_obs.control_loop_count = params->control_loop_count;

	float32_t angle_control_degrees = 0.0f;
	float32_t Ia_A, Ib_A;
	float32_t Vbus_V;
	float32_t Id_A, Iq_A;
	float32_t Va_V, Vb_V;
	float32_t Ua_pu, Ub_pu;
	float32_t Da_pu, Db_pu;
	float32_t Da_hb1_pu, Da_hb2_pu, Db_hb1_pu, Db_hb2_pu;
	float32_t Id_ref_A = params->live.Id_ref_A;
	float32_t Iq_ref_A = params->live.Iq_ref_A;
	float32_t Vd_V, Vq_V;
	float32_t max_voltage_magnitude_V;
	float32_t inv_park_angle_rad;
	uint32_t velocity_loop_decimation = ctx.velocity_loop_decimation;
	uint32_t position_loop_decimation = ctx.position_loop_decimation;
	float32_t velocity_loop_dt_s = ctx.velocity_loop_dt_s;
	float32_t position_loop_dt_s = ctx.position_loop_dt_s;
	float32_t velocity_target_rad_s = ctx.velocity_target_rad_s;
	float32_t velocity_ref_rad_s = ctx.velocity_ref_rad_s;
	float32_t position_mech_rad = ctx.position_mech_rad;
	float32_t speed_mech_rad_s = ctx.speed_mech_rad_s;
	float32_t accel_mech_rad_s2 = ctx.accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s = ctx.speed_mech_filtered_rad_s;
	uint8_t encoder_input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;

	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = params->velocity_dob_state.disturbance_nm;
	params->live.velocity_dob_residual_rad_s = 0.0f;

	/* Read encoder if feature is enabled */
	struct motor_encoder_stage_result enc_stage = {0};
	int enc_ret = motor_control_step_read_encoder(params, mode_flags, encoder_sample,
						      feature_angle_gen, &commission_obs,
						      &enc_stage);
	if (enc_ret == -EIO) {
		motor_step_report_post_error(report, ERROR_ENCODER_FAULT);
		goto isr_done;
	}
	motor_control_step_publish_encoder_sidework(params, encoder_sample, &enc_stage, report);
	encoder_input_source = enc_stage.input_source;
	angle_control_degrees = enc_stage.angle_control_deg;
	bool fresh_encoder_sample = enc_stage.fresh;
	uint8_t encoder_frame_status = enc_stage.frame_status;
	bool encoder_frame_warning = enc_stage.frame_warning;
	bool encoder_frame_error = enc_stage.frame_error;
	position_mech_rad = enc_stage.control_fb.position_mech_rad;
	speed_mech_rad_s = enc_stage.control_fb.speed_mech_rad_s;
	accel_mech_rad_s2 = enc_stage.control_fb.accel_mech_rad_s2;
	speed_mech_filtered_rad_s = enc_stage.control_fb.speed_mech_filtered_rad_s;

	/* Skip control if PWM output not enabled */
	if (!feature_pwm_output) {
		goto isr_done;
	}

	Ia_A = adc_to_current(values[CURRENT_SENSE_ADC_BUFFER_INDEX_0], CURRENT_SENSE_POLARITY_0);
	Ib_A = adc_to_current(values[CURRENT_SENSE_ADC_BUFFER_INDEX_1], CURRENT_SENSE_POLARITY_1);
	Vbus_V = adc_to_vbus_v(values[VBUS_ADC_BUFFER_INDEX]);
	commission_obs.vbus_v = Vbus_V;

	/* Validate bus voltage before reciprocal to avoid Inf/NaN propagation. */
	if (Vbus_V < VBUS_MIN_VALID_V) {
		motor_step_report_post_error(report, ERROR_HARDWARE_BREAK);
		goto isr_done;
	}

	/* Fault detection: Check for overvoltage */
	if (Vbus_V > VBUS_MAX_V) {
		motor_step_report_post_error(report, ERROR_OVERVOLTAGE);
		goto isr_done;
	}

	/* Handle offset measurement (no control, just filtering) */
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_OFFSET_MEAS)) {
		filter_fo_run(&params->filter_Ia, Ia_A);
		filter_fo_run(&params->filter_Ib, Ib_A);
		goto isr_done;
	}

	/* Remove offsets for current measurements */
	Ia_A -= params->Ia_offset;
	Ib_A -= params->Ib_offset;

	/* Compute dq currents before fault checks so snapshot rows include
	 * same-cycle electrical frame measurements at fault time.
	 */
	float32_t park_angle_rad = angle_observer_get_elec_angle(&params->observer);
	int park_ret = motor_transforms_park(Ia_A, Ib_A, park_angle_rad, &Id_A, &Iq_A);
	if (park_ret != 0) {
		goto isr_done;
	}

	motor_fault_snapshot_prepare(report,
				     angle_control_degrees,
				     params->live.encoder_observer_input_rad,
				     park_angle_rad,
				     angle_observer_get_elec_speed(&params->observer),
				     Id_ref_A,
				     Iq_ref_A,
				     Id_A,
				     Iq_A,
				     Ia_A,
				     Ib_A,
				     params->Vd_V,
				     params->Vq_V,
				     encoder_input_source,
				     fresh_encoder_sample,
				     encoder_frame_warning,
				     encoder_frame_error,
				     encoder_frame_status,
				     params->live.position_quality_flags);

	/* Fault detection: Check for overcurrent after offset removal */
	if (fabsf(Ia_A) > OVERCURRENT_THRESHOLD_A || fabsf(Ib_A) > OVERCURRENT_THRESHOLD_A) {
		motor_step_report_post_error(report, ERROR_OVERCURRENT);
		goto isr_done;
	}

	/* Skip PI control if not enabled */
	if (!feature_pi_control) {
		goto isr_done;
	}

	/* R/L measurement: set current reference and accumulate V/I in rotating frame */
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS)) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;

		/* Check if settling period complete using trajectory target */
		if (traj_is_at_target(&params->traj_Id)) {
			/* Accumulate for R/L extraction using previous cycle's voltage. */
			motor_roverl_accumulate_scalars(&params->roverl_accumulator_Vd_Id,
							&params->roverl_accumulator_Vq_Id,
							&params->roverl_accumulator_Id2,
							params->Vd_V, params->Vq_V, Id_A);
		}
	}

	/* Rs EST: filter V/I in d-axis for DC resistance */
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST)) {
		motor_rs_est_step_filter(&params->traj_Id,
					 &params->filter_rs_est_V,
					 &params->filter_rs_est_I,
					 params->Vd_V, Id_A, &Id_ref_A);
		Iq_ref_A = 0.0f;
	}

	/* ALIGN states must source d-axis reference from traj_Id only. */
	if (motor_is_align_active_state(mode_flags)) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;
	}

	struct motor_outer_loop_inputs outer_inputs = {
		.position_active = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_POSITION),
		.velocity_active =
			motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_CLOSED) ||
			motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_POSITION),
		.feature_angle_gen = feature_angle_gen,
		.feature_velocity_traj = feature_velocity_traj,
		.velocity_loop_decimation = velocity_loop_decimation,
		.position_loop_decimation = position_loop_decimation,
		.velocity_loop_dt_s = velocity_loop_dt_s,
		.position_loop_dt_s = position_loop_dt_s,
		.position_mech_rad = position_mech_rad,
		.speed_mech_rad_s = speed_mech_rad_s,
		.id_meas_a = Id_A,
		.iq_meas_a = Iq_A,
		.velocity_target_rad_s = velocity_target_rad_s,
		.velocity_ref_rad_s = velocity_ref_rad_s,
		.id_ref_a = Id_ref_A,
		.iq_ref_a = Iq_ref_A,
	};
	struct motor_outer_loop_outputs outer_outputs = {0};
	motor_outer_loop_runtime_ctx_refresh(&params->rt_adapters.outer_loop, params, control_armed);
	(void)motor_outer_loop_runtime_step(&params->rt_adapters.outer_loop, &outer_inputs,
					    &outer_outputs);
	velocity_target_rad_s = outer_outputs.velocity_target_rad_s;
	velocity_ref_rad_s = outer_outputs.velocity_ref_rad_s;
	speed_mech_filtered_rad_s = outer_outputs.speed_mech_filtered_rad_s;
	Id_ref_A = outer_outputs.id_ref_a;
	Iq_ref_A = outer_outputs.iq_ref_a;

	struct motor_current_ref_policy_inputs ref_policy_inputs = {
		.online_control_state = online_control_state,
		.feature_angle_gen = feature_angle_gen,
		.feature_use_commanded_currents = feature_use_commanded_currents,
		.control_armed = control_armed,
		.speed_mech_filtered_rad_s = speed_mech_filtered_rad_s,
		.id_meas_a = Id_A,
		.iq_meas_a = Iq_A,
		.velocity_target_rad_s = velocity_target_rad_s,
		.velocity_ref_rad_s = velocity_ref_rad_s,
		.id_ref_a = Id_ref_A,
		.iq_ref_a = Iq_ref_A,
	};
	struct motor_current_ref_policy_outputs ref_policy_outputs = {0};
	motor_current_ref_policy_ctx_refresh(&params->rt_adapters.current_ref_policy, params);
	(void)motor_current_ref_apply_policy(&params->rt_adapters.current_ref_policy, &ref_policy_inputs,
					     &ref_policy_outputs);
	velocity_target_rad_s = ref_policy_outputs.velocity_target_rad_s;
	velocity_ref_rad_s = ref_policy_outputs.velocity_ref_rad_s;
	Id_ref_A = ref_policy_outputs.id_ref_a;
	Iq_ref_A = ref_policy_outputs.iq_ref_a;

	struct motor_rls_runtime_state rls_runtime = {0};
	motor_rls_runtime_ctx_refresh(&params->rt_adapters.rls, params);
	Id_ref_A = motor_rls_prepare_id_reference(&params->rt_adapters.rls, online_control_state,
						 control_armed,
						 fresh_encoder_sample, encoder_frame_error,
						 encoder_input_source, Id_ref_A, &rls_runtime);

	/* Invariant: ALIGN injection/sample references are trajectory-owned.
	 * Force final refs from traj_Id after all other policy hooks.
	 */
	if (motor_is_align_active_state(mode_flags)) {
		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;
	}

	/* Advance angle generator if enabled
	 * This compensates for the fact that computed voltages will be applied in the next cycle
	 */
	if (feature_angle_gen) {
		angle_gen_run(&params->angle_gen);
	}

	inv_park_angle_rad = angle_observer_get_elec_angle_pred(&params->observer);

	float32_t observer_elec_speed_rad_s = angle_observer_get_elec_speed(&params->observer);
	float32_t decoupling_speed_limit_rad_s =
		MAX(50.0f, params->profile_max_velocity_rad_s * (float32_t)MOTOR_POLE_PAIRS * 1.5f);
	float32_t flux_linkage_wb_abs = fabsf(params->flux_linkage_wb_active);
	bool torque_mode_state = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_TORQUE);
	bool decoupling_min_speed_reached =
		fabsf(speed_mech_filtered_rad_s) >= CURRENT_DECOUPLING_MIN_MECH_SPEED_RAD_S;
	bool decoupling_flux_valid = isfinite(flux_linkage_wb_abs) &&
				     (flux_linkage_wb_abs >= CURRENT_DECOUPLING_MIN_FLUX_WB) &&
				     (flux_linkage_wb_abs <= CURRENT_DECOUPLING_MAX_FLUX_WB);
	bool decoupling_speed_valid = isfinite(observer_elec_speed_rad_s) &&
				      (fabsf(observer_elec_speed_rad_s) <=
				       decoupling_speed_limit_rad_s);
	bool decoupling_feedback_valid = feature_angle_gen ||
					 motor_velocity_feedback_is_valid(params->live.position_quality_flags);
	struct motor_dq_decoupling_enable_input decoupling_enable_in = {
		.feature_enabled = CURRENT_DECOUPLING_ENABLED,
		.online_control_state = online_control_state,
		.control_armed = control_armed,
		.torque_mode_state = torque_mode_state,
		.min_speed_reached = decoupling_min_speed_reached,
		.flux_valid = decoupling_flux_valid,
		.speed_valid = decoupling_speed_valid,
		.feedback_valid = decoupling_feedback_valid,
	};
	bool dq_decoupling_enabled = motor_dq_decoupling_is_enabled(&decoupling_enable_in);
		float32_t decoupling_speed_rad_s = decoupling_speed_valid ? observer_elec_speed_rad_s : 0.0f;

#if defined(CONFIG_MOTOR_ISR_SANITY_CHECKS) && (CONFIG_MOTOR_ISR_SANITY_CHECKS == 1)
		/* Fast-path ingress sanity gate for downstream control modules. */
		if (!isfinite(inv_park_angle_rad) || !isfinite(Id_ref_A) || !isfinite(Iq_ref_A) ||
		    !isfinite(Id_A) || !isfinite(Iq_A) || !isfinite(Vbus_V) ||
		    !isfinite(params->max_modulation_index) || params->max_modulation_index <= 0.0f) {
			goto isr_done;
		}
#endif

		struct motor_foc_voltage_pwm_inputs foc_inputs = {
			.id_ref_a = Id_ref_A,
		.iq_ref_a = Iq_ref_A,
		.id_a = Id_A,
		.iq_a = Iq_A,
		.vbus_v = Vbus_V,
		.max_modulation_index = params->max_modulation_index,
		.inv_park_angle_rad = inv_park_angle_rad,
		/* Keep decoupling/feedforward in ONLINE control only; calibration states
		 * (ALIGN/RS_EST/ROVERL) can have transient observer speed spikes.
		 */
		.dq_decoupling_enabled = dq_decoupling_enabled,
		.electrical_speed_rad_s = decoupling_speed_rad_s,
		.ld_h = params->rls.ld_est_h,
		.lq_h = params->rls.lq_est_h,
		.flux_linkage_wb = params->flux_linkage_wb_active,
		.dq_decoupling_flux_headroom_ratio = CURRENT_DQ_DECOUPLING_FLUX_HEADROOM_RATIO,
		.dq_decoupling_ff_limit_ratio = CURRENT_DQ_DECOUPLING_FF_LIMIT_RATIO,
		.braking_enabled = feature_braking,
		.braking_iq_ref_a = Iq_ref_A,
		.braking_speed_rad_s = speed_mech_rad_s,
		.braking_vbus_limit_v = VBUS_REGEN_LIMIT_V,
		.braking_vbus_margin_inv = VBUS_VOLTAGE_MARGIN_INV,
	};
	struct motor_foc_voltage_pwm_outputs foc_outputs = {0};
	int foc_ret = motor_foc_voltage_pwm_step(&params->pi_Id, &params->pi_Iq,
						 &foc_inputs, &foc_outputs);
	if (foc_ret != 0) {
		goto isr_done;
	}

	Vd_V = foc_outputs.vd_v;
	Vq_V = foc_outputs.vq_v;
	Va_V = foc_outputs.va_v;
	Vb_V = foc_outputs.vb_v;
	Ua_pu = foc_outputs.ua_pu;
	Ub_pu = foc_outputs.ub_pu;
	Da_pu = foc_outputs.da_pu;
	Db_pu = foc_outputs.db_pu;
	Da_hb1_pu = foc_outputs.da_hb1_pu;
	Da_hb2_pu = foc_outputs.da_hb2_pu;
	Db_hb1_pu = foc_outputs.db_hb1_pu;
	Db_hb2_pu = foc_outputs.db_hb2_pu;
	max_voltage_magnitude_V = foc_outputs.max_voltage_magnitude_v;
	float32_t voltage_norm_sq = Vd_V * Vd_V + Vq_V * Vq_V;
	float32_t voltage_limit = 0.98f * max_voltage_magnitude_V;
	bool voltage_saturated = max_voltage_magnitude_V > 0.0f &&
				 voltage_norm_sq >= (voltage_limit * voltage_limit);

	pwm_out->da_hb1_pu = Da_hb1_pu;
	pwm_out->da_hb2_pu = Da_hb2_pu;
	pwm_out->db_hb1_pu = Db_hb1_pu;
	pwm_out->db_hb2_pu = Db_hb2_pu;
	pwm_out->update_pwm = true;

	motor_rls_runtime_ctx_refresh(&params->rt_adapters.rls, params);
	motor_rls_update_estimators(&params->rt_adapters.rls, &rls_runtime, Id_A, Iq_A);

	/* Update telemetry snapshot (mechanical-domain feedback from observer path). */
	params->live.position_rad = position_mech_rad;
	params->live.position_unwrapped_rad = position_mech_rad;
	params->live.position_innovation_rad = 0.0f;
	params->live.velocity_rad_s = speed_mech_rad_s;
	params->live.acceleration_rad_s2 = accel_mech_rad_s2;
	params->live.velocity_filtered_rad_s = speed_mech_filtered_rad_s;
	params->live.velocity_target_rad_s = velocity_target_rad_s;
	params->live.velocity_ref_rad_s = velocity_ref_rad_s;

	params->live.Id_ref_A = Id_ref_A;
	params->live.Iq_ref_A = Iq_ref_A;
	params->live.Id_A = Id_A;
	params->live.Iq_A = Iq_A;
	params->live.Ia_A = Ia_A;
	params->live.Ib_A = Ib_A;
	params->Vd_V = Vd_V;
	params->Vq_V = Vq_V;
	params->live.Va_V = Va_V;
	params->live.Vb_V = Vb_V;
	params->max_voltage_magnitude_V = max_voltage_magnitude_V;
	params->live.elec_angle_rad = inv_park_angle_rad;
	params->live.dc_bus_voltage_V = Vbus_V;

	commission_obs.data_valid = true;
	commission_obs.id_a = Id_A;
	commission_obs.iq_a = Iq_A;
	commission_obs.vd_v = Vd_V;
	commission_obs.vq_v = Vq_V;
	commission_obs.mech_speed_rad_s = params->live.velocity_rad_s;
	commission_obs.elec_speed_rad_s = angle_observer_get_elec_speed(&params->observer);
	commission_obs.saturation = voltage_saturated;

isr_done:
	motor_core_step_finalize(params, mode_flags, control_armed, &commission_obs);
	return;
}
