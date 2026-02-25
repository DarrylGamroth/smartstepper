/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <dsp/controller_functions.h>

#include "motor/runtime/motor_core_step.h"
#include "motor_control_loop.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor/math/math_constants.h"
#include "config.h"
#include "motor/filters/pi.h"
#include "motor/filters/filter_fo.h"
#include "motor/motion/traj.h"
#include "motor/observers/angle_observer.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"
#include "motor/runtime/config_snapshot.h"
#include "motor/runtime/keepalive_policy.h"
#include "motor_rls_runtime.h"
#include "motor/control/foc_voltage_pwm.h"
#include "motor_state_utils.h"
#include "motor_commission.h"
#include "motor/control/dob.h"
#include "motor/motion/motion_planner.h"
#include "motor_torque.h"
#include "motor_encoder_feedback.h"
#include "motor/observers/feedback.h"
#include "motor/telemetry/capture.h"
#include "motor_control_outer_loops.h"
#include "motor_current_ref_policy.h"
#include "motor_control_quality.h"
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

static inline bool motor_is_align_injection_state(const struct smf_state *state)
{
	return motor_state_ptr_is_mode(state, MOTOR_STATE_ALIGN_POS_INJECT) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ALIGN_NEG_INJECT);
}

static inline bool motor_is_align_sample_state(const struct smf_state *state)
{
	return motor_state_ptr_is_mode(state, MOTOR_STATE_ALIGN_POS_SAMPLE) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ALIGN_NEG_SAMPLE);
}

static inline void motor_encoder_capture_try_store(
	struct motor_parameters *params,
	const struct motor_capture_feedback *capture)
{
	if (params == NULL || capture == NULL || !params->encoder_capture_enabled) {
		return;
	}

	uint16_t decimation = MAX((uint16_t)1U, params->encoder_capture_decimation);
	if (params->encoder_capture_phase > 0U) {
		params->encoder_capture_phase--;
		return;
	}
	params->encoder_capture_phase = decimation - 1U;

	uint16_t idx = params->encoder_capture_write_idx;
	struct motor_encoder_capture_sample *sample = &params->encoder_capture_samples[idx];
	sample->control_loop_count = params->rt_fast.control_loop_count;
	sample->angle_deg = capture->angle_deg;
	sample->angle_rad = capture->angle_rad;
	sample->encoder_mech_rad = capture->encoder_mech_rad;
	sample->encoder_elec_rad = capture->encoder_elec_rad;
	sample->observer_mech_rad = capture->observer_mech_rad;
	sample->observer_elec_rad = capture->observer_elec_rad;
	sample->generated_mech_rad = capture->generated_mech_rad;
	sample->generated_elec_rad = capture->generated_elec_rad;
	sample->mech_error_rad = capture->mech_error_rad;
	sample->elec_error_rad = capture->elec_error_rad;
	sample->compare_valid = capture->compare_valid ? 1U : 0U;
	sample->input_source = capture->input_source;
	sample->sample_enabled = capture->sample_enabled ? 1U : 0U;
	sample->sample_fresh = capture->sample_fresh ? 1U : 0U;
	sample->sample_warning = capture->sample_warning ? 1U : 0U;
	sample->sample_error = capture->sample_error ? 1U : 0U;
	sample->status = capture->status;

	params->encoder_capture_write_idx =
		(uint16_t)((idx + 1U) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	if (params->encoder_capture_count < MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
		params->encoder_capture_count++;
	} else {
		params->encoder_capture_overrun_count++;
	}
}

static inline void motor_fault_snapshot_try_store(struct motor_parameters *params,
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
	if (params == NULL) {
		return;
	}

	uint16_t idx = params->fault_snapshot_write_idx;
	struct motor_fault_snapshot_sample *sample = &params->fault_snapshot_samples[idx];

	sample->control_loop_count = params->rt_fast.control_loop_count;
	sample->encoder_angle_deg = encoder_angle_deg;
	sample->observer_input_rad = observer_input_rad;
	sample->elec_angle_rad = elec_angle_rad;
	sample->observer_elec_speed_rad_s = observer_elec_speed_rad_s;
	sample->Id_ref_A = id_ref_a;
	sample->Iq_ref_A = iq_ref_a;
	sample->Id_A = id_a;
	sample->Iq_A = iq_a;
	sample->Ia_A = ia_a;
	sample->Ib_A = ib_a;
	sample->Vd_V = vd_v;
	sample->Vq_V = vq_v;
	sample->input_source = input_source;
	sample->sample_fresh = sample_fresh ? 1U : 0U;
	sample->sample_warning = sample_warning ? 1U : 0U;
	sample->sample_error = sample_error ? 1U : 0U;
	sample->status = status;
	sample->position_quality_flags = position_quality_flags;

	params->fault_snapshot_write_idx =
		(uint16_t)((idx + 1U) % MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	if (params->fault_snapshot_count < MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES) {
		params->fault_snapshot_count++;
	} else {
		params->fault_snapshot_overrun_count++;
	}
}

static inline void motor_post_error_with_snapshot(struct motor_parameters *params,
						  uint32_t error_code)
{
	if (params != NULL) {
		params->fault_snapshot_latched = 1U;
		params->fault_snapshot_latch_error_code = error_code;
		params->fault_snapshot_latch_loop = params->rt_fast.control_loop_count;
	}

	motor_api_post_error(error_code);
}

static inline void motor_runtime_fast_sync(struct motor_parameters *params, bool control_armed)
{
	params->rt_fast.control_loop_count = params->control_loop_count;
	params->rt_fast.rls_d_prev_cycle = params->rls_d_prev_cycle;
	params->rt_fast.rls_q_prev_cycle = params->rls_q_prev_cycle;
	params->rt_fast.rls_d_prev_valid = params->rls_d_prev_valid;
	params->rt_fast.rls_q_prev_valid = params->rls_q_prev_valid;
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
	params->rt_diag.encoder_capture_overrun_count = params->encoder_capture_overrun_count;
	params->rt_diag.fault_snapshot_overrun_count = params->fault_snapshot_overrun_count;
	params->rt_diag.fault_snapshot_latch_loop = params->fault_snapshot_latch_loop;
	params->rt_diag.fault_snapshot_latch_error_code = params->fault_snapshot_latch_error_code;
	params->rt_diag.command_timeout_count = params->command_timeout_count;
	params->rt_diag.profile_sequence_event_drop_count = params->profile_sequence_event_drop_count;
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
	control_fb->position_mech_rad = encoder_fb->position_mech_rad;
	control_fb->speed_mech_rad_s = encoder_fb->speed_mech_rad_s;
	control_fb->accel_mech_rad_s2 = encoder_fb->accel_mech_rad_s2;
	control_fb->speed_mech_filtered_rad_s = encoder_fb->speed_mech_filtered_rad_s;
}

static inline void motor_capture_feedback_from_encoder(
	const struct motor_encoder_feedback *encoder_fb,
	struct motor_capture_feedback *capture_fb)
{
	memset(capture_fb, 0, sizeof(*capture_fb));
	if (encoder_fb == NULL) {
		return;
	}

	capture_fb->angle_deg = encoder_fb->capture_angle_deg;
	capture_fb->angle_rad = encoder_fb->capture_angle_rad;
	capture_fb->encoder_mech_rad = encoder_fb->capture_encoder_mech_rad;
	capture_fb->encoder_elec_rad = encoder_fb->capture_encoder_elec_rad;
	capture_fb->observer_mech_rad = encoder_fb->capture_observer_mech_rad;
	capture_fb->observer_elec_rad = encoder_fb->capture_observer_elec_rad;
	capture_fb->generated_mech_rad = encoder_fb->capture_generated_mech_rad;
	capture_fb->generated_elec_rad = encoder_fb->capture_generated_elec_rad;
	capture_fb->mech_error_rad = encoder_fb->capture_mech_error_rad;
	capture_fb->elec_error_rad = encoder_fb->capture_elec_error_rad;
	capture_fb->compare_valid = encoder_fb->capture_compare_valid;
	capture_fb->sample_enabled = encoder_fb->sample_available;
	capture_fb->sample_fresh = encoder_fb->fresh;
	capture_fb->sample_warning = encoder_fb->warning;
	capture_fb->sample_error = encoder_fb->error;
	capture_fb->status = encoder_fb->status;
	capture_fb->input_source = encoder_fb->capture_input_source;
}

struct motor_control_step_ctx {
	uint32_t config_epoch;
	const struct smf_state *state;
	atomic_val_t feature_flags;
	bool feature_angle_gen;
	bool feature_pwm_output;
	bool feature_pi_control;
	bool feature_velocity_traj;
	bool feature_use_commanded_currents;
	bool feature_braking;
	bool profile_sequence_running;
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
	struct motor_encoder_feedback encoder_fb;
};

static inline void motor_control_step_ctx_init(struct motor_control_step_ctx *ctx,
					       const struct motor_parameters *params)
{
	memset(ctx, 0, sizeof(*ctx));
	struct motor_rt_config_snapshot cfg = {0};
	bool cfg_valid = motor_config_snapshot_read(&cfg);
	ctx->config_epoch = cfg.epoch;
	ctx->state = cfg_valid ? cfg.state : params->state_for_isr;
	ctx->feature_flags = cfg_valid ? cfg.feature_flags : atomic_get(&params->feature_flags);
	ctx->feature_angle_gen = (ctx->feature_flags & BIT(MOTOR_FEATURE_ANGLE_GEN)) != 0;
	ctx->feature_pwm_output = (ctx->feature_flags & BIT(MOTOR_FEATURE_PWM_OUTPUT)) != 0;
	ctx->feature_pi_control = (ctx->feature_flags & BIT(MOTOR_FEATURE_PI_CONTROL)) != 0;
	ctx->feature_velocity_traj = (ctx->feature_flags & BIT(MOTOR_FEATURE_VELOCITY_TRAJ)) != 0;
	ctx->feature_use_commanded_currents =
		(ctx->feature_flags & BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS)) != 0;
	ctx->feature_braking = (ctx->feature_flags & BIT(MOTOR_FEATURE_BRAKING)) != 0;
	ctx->profile_sequence_running = cfg_valid ? cfg.profile_sequence_running :
					      params->profile_sequence_running;
	ctx->online_control_state = motor_state_ptr_is_online_control_state(ctx->state);
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
	ctx->velocity_target_rad_s = params->velocity_target_rad_s;
	ctx->velocity_ref_rad_s = params->velocity_ref_rad_s;
	ctx->position_mech_rad = params->position_rad;
	ctx->speed_mech_rad_s = params->velocity_rad_s;
	ctx->accel_mech_rad_s2 = params->acceleration_rad_s2;
	ctx->speed_mech_filtered_rad_s = params->velocity_filtered_rad_s;
}

void motor_core_step_fast(struct motor_parameters *params,
			     const q31_t *values,
			     uint8_t count,
			     const struct motor_control_encoder_sample *encoder_sample,
			     struct motor_control_pwm_output *pwm_out)
{
	ARG_UNUSED(count);
	if (params == NULL || values == NULL || pwm_out == NULL) {
		return;
	}

	struct motor_control_step_ctx ctx;
	motor_control_step_ctx_init(&ctx, params);

	const struct smf_state *state = ctx.state;
	bool feature_angle_gen = ctx.feature_angle_gen;
	bool feature_pwm_output = ctx.feature_pwm_output;
	bool feature_pi_control = ctx.feature_pi_control;
	bool feature_velocity_traj = ctx.feature_velocity_traj;
	bool feature_use_commanded_currents = ctx.feature_use_commanded_currents;
	bool feature_braking = ctx.feature_braking;
	bool profile_sequence_running = ctx.profile_sequence_running;
	bool online_control_state = ctx.online_control_state;
	bool control_armed = ctx.control_armed;
	bool autonomous_keepalive = false;
	struct motor_commission_observation commission_obs = {
		.control_loop_count = 0U,
		.state = state,
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

	if (pwm_out != NULL) {
		pwm_out->update_pwm = false;
		pwm_out->da_hb1_pu = 0.0f;
		pwm_out->da_hb2_pu = 0.0f;
		pwm_out->db_hb1_pu = 0.0f;
		pwm_out->db_hb2_pu = 0.0f;
	}

	/* Increment control loop counter */
	params->control_loop_count++;
	motor_runtime_fast_sync(params, control_armed);
	commission_obs.control_loop_count = params->control_loop_count;

	float32_t angle_control_degrees = 0.0f;
	float32_t sin_theta, cos_theta;
	float32_t Ia_A, Ib_A;
	float32_t Vbus_V;
	float32_t Id_A, Iq_A;
	float32_t Va_V, Vb_V;
	float32_t Ua_pu, Ub_pu;
	float32_t Da_pu, Db_pu;
	float32_t Da_hb1_pu, Da_hb2_pu, Db_hb1_pu, Db_hb2_pu;
	float32_t Id_ref_A = params->Id_ref_A;
	float32_t Iq_ref_A = params->Iq_ref_A;
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
	uint32_t now_ms = 0U;
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_OPEN) ||
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_CLOSED) ||
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_POSITION);

	params->velocity_dob_iq_ff_a = 0.0f;
	params->velocity_dob_disturbance_nm = params->velocity_dob_state.disturbance_nm;
	params->velocity_dob_residual_rad_s = 0.0f;

	autonomous_keepalive =
		motor_keepalive_policy_should_keepalive(control_armed, autonomous_mode_active,
						 profile_sequence_running,
						 params->chopper_cal_active,
						 motion_profile_quintic_is_active(&params->position_profile));
	if (autonomous_keepalive) {
		now_ms = k_uptime_get_32();
		params->last_command_update_ms = now_ms;
		params->command_timeout_latched = false;
	}

	/* Timeout interlock disarms output commands when command updates stop. */
	if (params->command_timeout_ms > 0U) {
		now_ms = k_uptime_get_32();
		struct motor_timeout_interlock_input timeout_in = {
			.online_control_state = online_control_state,
			.control_armed = control_armed,
			.autonomous_keepalive = autonomous_keepalive,
			.command_timeout_ms = params->command_timeout_ms,
			.now_ms = now_ms,
			.last_command_update_ms = params->last_command_update_ms,
		};
		struct motor_timeout_interlock_output timeout_out = {0};
		motor_interlocks_eval_timeout(&timeout_in, &timeout_out);
		if (timeout_out.disarm_control) {
			control_armed = false;
			atomic_set(&params->control_armed, 0);
			if (!params->command_timeout_latched) {
				params->command_timeout_latched = true;
				params->command_timeout_count++;
			}
		}
	}

	/* Read encoder if feature is enabled */
	int enc_ret = motor_encoder_feedback_update(params, encoder_sample, feature_angle_gen,
						      &ctx.encoder_fb);
	struct motor_control_feedback control_fb = {0};
	struct motor_capture_feedback capture_fb = {0};
	motor_control_feedback_from_encoder(&ctx.encoder_fb, &control_fb);
	motor_capture_feedback_from_encoder(&ctx.encoder_fb, &capture_fb);
	encoder_input_source = control_fb.input_source;
	angle_control_degrees = control_fb.angle_control_deg;
	bool fresh_encoder_sample = control_fb.fresh;
	uint8_t encoder_frame_status = control_fb.status;
	bool encoder_frame_warning = control_fb.warning;
	bool encoder_frame_error = control_fb.error;

	if (control_fb.sample_enabled) {
		commission_obs.encoder_fresh = fresh_encoder_sample;
		commission_obs.encoder_warning = encoder_frame_warning;
		commission_obs.encoder_error = encoder_frame_error;
		commission_obs.encoder_status = encoder_frame_status;
	} else {
		commission_obs.encoder_fresh = false;
		commission_obs.encoder_warning = false;
		commission_obs.encoder_error = false;
	}

	if (enc_ret == -EIO) {
		motor_post_error_with_snapshot(params, ERROR_ENCODER_FAULT);
		goto isr_done;
	}

	/* ALIGN sample phases only accept fresh, warning-free encoder samples.
	 * Accumulate circular means in ISR so state thread can validate sample quality.
	 */
	if (motor_is_align_sample_state(state) &&
	    encoder_input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER &&
	    fresh_encoder_sample &&
	    !encoder_frame_warning &&
	    !encoder_frame_error) {
		float32_t align_mech_rad = control_fb.observer_mech_rad;
		arm_sin_cos_f32(align_mech_rad, &sin_theta, &cos_theta);

		if (motor_state_ptr_is_mode(state, MOTOR_STATE_ALIGN_POS_SAMPLE)) {
			params->align_pos_sum_sin += sin_theta;
			params->align_pos_sum_cos += cos_theta;
			params->align_pos_sample_count++;
		} else {
			params->align_neg_sum_sin += sin_theta;
			params->align_neg_sum_cos += cos_theta;
			params->align_neg_sample_count++;
		}
	}

	motor_encoder_capture_try_store(params, &capture_fb);

	position_mech_rad = control_fb.position_mech_rad;
	speed_mech_rad_s = control_fb.speed_mech_rad_s;
	accel_mech_rad_s2 = control_fb.accel_mech_rad_s2;
	speed_mech_filtered_rad_s = control_fb.speed_mech_filtered_rad_s;

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
		motor_post_error_with_snapshot(params, ERROR_HARDWARE_BREAK);
		goto isr_done;
	}

	/* Fault detection: Check for overvoltage */
	if (Vbus_V > VBUS_MAX_V) {
		motor_post_error_with_snapshot(params, ERROR_OVERVOLTAGE);
		goto isr_done;
	}

	/* Handle offset measurement (no control, just filtering) */
	if (state == &motor_states[MOTOR_STATE_OFFSET_MEAS]) {
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

	motor_fault_snapshot_try_store(params,
				      angle_control_degrees,
				      params->encoder_observer_input_rad,
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
				      params->position_quality_flags);

	/* Fault detection: Check for overcurrent after offset removal */
	if (fabsf(Ia_A) > OVERCURRENT_THRESHOLD_A || fabsf(Ib_A) > OVERCURRENT_THRESHOLD_A) {
		motor_post_error_with_snapshot(params, ERROR_OVERCURRENT);
		goto isr_done;
	}

	/* Skip PI control if not enabled */
	if (!feature_pi_control) {
		goto isr_done;
	}

	/* R/L measurement: set current reference and accumulate V/I in rotating frame */
	if (state == &motor_states[MOTOR_STATE_ROVERL_MEAS]) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;

		/* Check if settling period complete using trajectory target */
		if (traj_is_at_target(&params->traj_Id)) {
			/* Accumulate for R/L extraction using previous cycle's voltage */
			params->roverl_accumulator_Vd_Id += params->Vd_V * Id_A;
			params->roverl_accumulator_Vq_Id += params->Vq_V * Id_A;
			params->roverl_accumulator_Id2 += Id_A * Id_A;
		}
	}

	/* Rs EST: filter V/I in d-axis for DC resistance */
	if (state == &motor_states[MOTOR_STATE_RS_EST]) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;

		/* After rampup complete: filter voltage and current measurements using previous cycle's voltage */
		if (traj_is_at_target(&params->traj_Id)) {
			filter_fo_run(&params->filter_rs_est_V, params->Vd_V);
			filter_fo_run(&params->filter_rs_est_I, Id_A);
		}
	}

	/* ALIGN child states: ramp/hold calibration d-axis current target. */
	if (motor_is_align_injection_state(state) || motor_is_align_sample_state(state)) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;
	}

	struct motor_outer_loop_inputs outer_inputs = {
		.state = state,
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
	(void)motor_control_outer_loops_step(params, &outer_inputs, &outer_outputs);
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
	(void)motor_current_ref_apply_policy(params, &ref_policy_inputs, &ref_policy_outputs);
	velocity_target_rad_s = ref_policy_outputs.velocity_target_rad_s;
	velocity_ref_rad_s = ref_policy_outputs.velocity_ref_rad_s;
	Id_ref_A = ref_policy_outputs.id_ref_a;
	Iq_ref_A = ref_policy_outputs.iq_ref_a;

	struct motor_rls_runtime_state rls_runtime = {0};
	Id_ref_A = motor_rls_prepare_id_reference(params, online_control_state, control_armed,
						 fresh_encoder_sample, encoder_frame_error,
						 encoder_input_source, Id_ref_A, &rls_runtime);

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
	bool torque_mode_state = motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_TORQUE);
	bool decoupling_min_speed_reached =
		fabsf(speed_mech_filtered_rad_s) >= CURRENT_DECOUPLING_MIN_MECH_SPEED_RAD_S;
	bool decoupling_flux_valid = isfinite(flux_linkage_wb_abs) &&
				     (flux_linkage_wb_abs >= CURRENT_DECOUPLING_MIN_FLUX_WB) &&
				     (flux_linkage_wb_abs <= CURRENT_DECOUPLING_MAX_FLUX_WB);
	bool decoupling_speed_valid = isfinite(observer_elec_speed_rad_s) &&
				      (fabsf(observer_elec_speed_rad_s) <=
				       decoupling_speed_limit_rad_s);
	bool decoupling_feedback_valid = feature_angle_gen ||
					 motor_velocity_feedback_is_valid(params->position_quality_flags);
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
		.ld_h = params->Ld_est,
		.lq_h = params->Lq_est,
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

	motor_rls_update_estimators(params, &rls_runtime, Id_A, Iq_A);

	/* Update telemetry snapshot (mechanical-domain feedback from observer path). */
	params->position_rad = position_mech_rad;
	params->position_unwrapped_rad = position_mech_rad;
	params->position_innovation_rad = 0.0f;
	params->velocity_rad_s = speed_mech_rad_s;
	params->acceleration_rad_s2 = accel_mech_rad_s2;
	params->velocity_filtered_rad_s = speed_mech_filtered_rad_s;
	params->velocity_target_rad_s = velocity_target_rad_s;
	params->velocity_ref_rad_s = velocity_ref_rad_s;

	params->Id_ref_A = Id_ref_A;
	params->Iq_ref_A = Iq_ref_A;
	params->Id_A = Id_A;
	params->Iq_A = Iq_A;
	params->Ia_A = Ia_A;
	params->Ib_A = Ib_A;
	params->Vd_V = Vd_V;
	params->Vq_V = Vq_V;
	params->Va_V = Va_V;
	params->Vb_V = Vb_V;
	params->max_voltage_magnitude_V = max_voltage_magnitude_V;
	params->elec_angle_rad = inv_park_angle_rad;
	params->dc_bus_voltage_V = Vbus_V;

	commission_obs.data_valid = true;
	commission_obs.id_a = Id_A;
	commission_obs.iq_a = Iq_A;
	commission_obs.vd_v = Vd_V;
	commission_obs.vq_v = Vq_V;
	commission_obs.mech_speed_rad_s = params->velocity_rad_s;
	commission_obs.elec_speed_rad_s = angle_observer_get_elec_speed(&params->observer);
	commission_obs.saturation = voltage_saturated;

isr_done:
	commission_obs.control_armed = control_armed;
	commission_obs.state = state;
	commission_obs.fault_active = motor_state_ptr_is_mode(state, MOTOR_STATE_ERROR);
	motor_runtime_fast_sync(params, control_armed);
	motor_runtime_diag_sync(params);
	motor_commission_update(params, &commission_obs);
	return;
}
