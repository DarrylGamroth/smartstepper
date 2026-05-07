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
#include "motor/control/current_loop.h"
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
#include "motor/runtime/control_kernel.h"
#include "motor/runtime/control_refs.h"
#include "motor/motion/outer_loop_sched.h"
#include "motor/protection/interlocks.h"
#include "motor/control/dq_decoupling.h"
#include "motor/control/pwm_synthesis.h"
#include "motor/control/transforms.h"
#include "motor_encoder_fault_reason.h"
#include "motor_current_slew.h"

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

#define MOTOR_ISR_STAGE_NOINLINE __attribute__((noinline))

static inline bool motor_rt_mode_active(uint32_t mode_flags, uint32_t flag)
{
	return (mode_flags & flag) != 0U;
}

static inline bool motor_is_align_injection_state(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_POS_INJECT);
}

static inline bool motor_is_align_sample_state(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ALIGN_POS_SAMPLE);
}

static inline bool motor_is_align_active_state(uint32_t mode_flags)
{
	return motor_is_align_injection_state(mode_flags) || motor_is_align_sample_state(mode_flags);
}

static inline bool motor_calibration_owns_angle_generator(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST) ||
	       motor_is_align_active_state(mode_flags);
}

static inline bool motor_current_slew_calibration_target_active(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST) ||
	       motor_is_align_active_state(mode_flags);
}

static inline float32_t motor_current_ref_default_delta_a(void)
{
	return fmaxf(MOTOR_MAX_CURRENT_A /
		     (CURRENT_COMMAND_RAMP_S * CONTROL_LOOP_FREQUENCY_HZ),
		     1.0e-6f);
}

static inline void motor_current_slew_set_online_targets(struct motor_parameters *params,
							const struct motor_current_ref *current_ref)
{
	const float32_t current_ref_delta_a = motor_current_ref_default_delta_a();

	struct motor_current_slew_pair slew = motor_current_slew_from_params(params);

	motor_current_slew_set_delta(&slew, current_ref_delta_a);
	motor_current_slew_set_target(&slew, current_ref->id_ref_a, current_ref->iq_ref_a);
}

static inline void motor_current_slew_set_calibration_targets(struct motor_parameters *params)
{
	struct motor_current_slew_pair slew = motor_current_slew_from_params(params);

	traj_set_max_delta(slew.iq, motor_current_ref_default_delta_a());
	traj_set_target_value(slew.iq, 0.0f);
}

static inline void motor_control_current_slew_run(struct motor_parameters *params,
						  struct motor_current_ref *current_ref)
{
	struct motor_current_slew_pair slew = motor_current_slew_from_params(params);

	motor_current_slew_run(&slew, &current_ref->id_ref_a, &current_ref->iq_ref_a);
}

static inline void motor_current_slew_accumulate_calibration(
	struct motor_parameters *params,
	uint32_t mode_flags,
	const struct motor_control_measurements *meas)
{
	if (meas == NULL || !traj_is_at_target(&params->traj_Id)) {
		return;
	}
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS) &&
	    traj_is_at_target(&params->traj_Id)) {
		motor_roverl_accumulate_scalars(&params->roverl_accumulator_Vd_Id,
						&params->roverl_accumulator_Vq_Id,
						&params->roverl_accumulator_Id2,
						params->Vd_V, params->Vq_V, meas->id_a);
	}

	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST)) {
		motor_rs_est_accumulate(&params->filter_rs_est_V,
					&params->filter_rs_est_I,
					params->Vd_V, meas->id_a);
	}
}

static inline void motor_control_step_current_slew_stage(struct motor_parameters *params,
							 uint32_t mode_flags,
							 bool use_commanded_currents,
							 const struct motor_control_measurements *meas,
							 struct motor_current_ref *current_ref)
{
	if (params == NULL || current_ref == NULL) {
		return;
	}

	if (motor_current_slew_calibration_target_active(mode_flags)) {
		motor_current_slew_set_calibration_targets(params);
	} else if (!use_commanded_currents) {
		motor_current_slew_set_online_targets(params, current_ref);
	}
	motor_control_current_slew_run(params, current_ref);
	motor_current_slew_accumulate_calibration(params, mode_flags, meas);
}

static inline bool motor_vbus_fault_required(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_CONTROL) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_OFFSET_MEAS) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS) ||
	       motor_is_align_active_state(mode_flags);
}

static inline bool motor_current_fault_required(uint32_t mode_flags)
{
	return motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_CONTROL) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST) ||
	       motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS) ||
	       motor_is_align_active_state(mode_flags);
}

static inline enum motor_control_policy_mode
motor_control_policy_mode_from_rt_flags(uint32_t mode_flags)
{
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_GENERATED)) {
		return MOTOR_CONTROL_POLICY_MODE_VELOCITY_GENERATED;
	}
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_POSITION_GENERATED)) {
		return MOTOR_CONTROL_POLICY_MODE_POSITION_GENERATED;
	}
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_CURRENT_ENCODER)) {
		return MOTOR_CONTROL_POLICY_MODE_CURRENT_ENCODER;
	}
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_ENCODER)) {
		return MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER;
	}
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_POSITION_ENCODER)) {
		return MOTOR_CONTROL_POLICY_MODE_POSITION_ENCODER;
	}
	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_OFFSET_MEAS) ||
	    motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST) ||
	    motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS) ||
	    motor_is_align_active_state(mode_flags)) {
		return MOTOR_CONTROL_POLICY_MODE_CALIBRATION;
	}

	return MOTOR_CONTROL_POLICY_MODE_DISABLED;
}

static inline void motor_align_load_accum(const struct motor_parameters *params,
					  struct motor_align_sample_accum *acc)
{
	if (params == NULL || acc == NULL) {
		return;
	}

	acc->sum_sin = params->calibration.align_sum_sin;
	acc->sum_cos = params->calibration.align_sum_cos;
	acc->count = params->calibration.align_sample_count;
}

static inline void motor_align_store_accum(struct motor_parameters *params,
					   const struct motor_align_sample_accum *acc)
{
	if (params == NULL || acc == NULL) {
		return;
	}

	params->calibration.align_sum_sin = acc->sum_sin;
	params->calibration.align_sum_cos = acc->sum_cos;
	params->calibration.align_sample_count = acc->count;
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
	report->fault_snapshot.encoder_fault_reason = report->encoder_fault_reason;
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

static inline void motor_step_report_post_error_with_encoder_reason(
	struct motor_control_step_report *report,
	uint32_t error_code,
	uint8_t encoder_fault_reason)
{
	if (report == NULL || report->error_pending) {
		return;
	}

	report->error_pending = true;
	report->error_code = error_code;
	report->encoder_fault_reason = encoder_fault_reason;
	report->fault_snapshot.encoder_fault_reason = encoder_fault_reason;
}

static inline void motor_step_report_post_error(struct motor_control_step_report *report,
						uint32_t error_code)
{
	motor_step_report_post_error_with_encoder_reason(
		report, error_code, MOTOR_ENCODER_FAULT_REASON_NONE);
}

static inline uint8_t motor_encoder_fault_reason_for_invalid_feedback(
	const struct motor_feedback_ref *feedback_ref,
	uint16_t stale_count,
	uint32_t stale_limit)
{
	if (feedback_ref == NULL || feedback_ref->source == MOTOR_FEEDBACK_NONE) {
		return MOTOR_ENCODER_FAULT_REASON_NO_FEEDBACK;
	}
	if (feedback_ref->error) {
		return MOTOR_ENCODER_FAULT_REASON_FRAME_OR_IO_ERROR;
	}
	if (feedback_ref->input_source == MOTOR_ANGLE_INPUT_SRC_PROPAGATED &&
	    stale_count > stale_limit) {
		return MOTOR_ENCODER_FAULT_REASON_STALE;
	}
	if (!motor_feedback_quality_is_usable(feedback_ref->quality_flags)) {
		return MOTOR_ENCODER_FAULT_REASON_QUALITY;
	}
	if (feedback_ref->input_source == MOTOR_ANGLE_INPUT_SRC_PROPAGATED) {
		return MOTOR_ENCODER_FAULT_REASON_PROPAGATED;
	}

	return MOTOR_ENCODER_FAULT_REASON_FEEDBACK_INVALID;
}

static inline uint8_t motor_encoder_fault_reason_for_insane_feedback(
	const struct motor_feedback_ref *feedback_ref)
{
	if (feedback_ref == NULL || !isfinite(feedback_ref->velocity_filtered_rad_s)) {
		return MOTOR_ENCODER_FAULT_REASON_VELOCITY_NAN;
	}

	return MOTOR_ENCODER_FAULT_REASON_VELOCITY_SPIKE;
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
	if (encoder_fb == NULL || control_fb == NULL) {
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
	control_fb->trust_state = encoder_fb->trust_state;
	control_fb->angle_sensor_deg = encoder_fb->angle_sensor_deg;
	control_fb->angle_control_deg = encoder_fb->angle_control_deg;
	control_fb->generated_mech_rad = encoder_fb->generated_mech_rad;
	control_fb->generated_elec_rad = encoder_fb->generated_elec_rad;
	control_fb->observer_input_rad = encoder_fb->observer_input_rad;
	control_fb->observer_mech_rad = encoder_fb->observer_mech_rad;
	control_fb->observer_elec_rad = encoder_fb->observer_elec_rad;
	control_fb->observer_elec_pred_rad = encoder_fb->observer_elec_pred_rad;
	control_fb->observer_elec_speed_rad_s = encoder_fb->observer_elec_speed_rad_s;
	control_fb->position_mech_rad = encoder_fb->control.position_mech_rad;
	control_fb->electrical_angle_rad = encoder_fb->control.electrical_angle_rad;
	control_fb->predicted_electrical_angle_rad =
		encoder_fb->control.predicted_electrical_angle_rad;
	control_fb->electrical_speed_rad_s = encoder_fb->control.electrical_speed_rad_s;
	control_fb->speed_mech_rad_s = encoder_fb->control.speed_mech_rad_s;
	control_fb->accel_mech_rad_s2 = encoder_fb->control.accel_mech_rad_s2;
	control_fb->speed_mech_filtered_rad_s = encoder_fb->control.speed_mech_filtered_rad_s;
	control_fb->observer_delay_samples = encoder_fb->control.observer_delay_samples;
	control_fb->prediction_age_samples = encoder_fb->control.prediction_age_samples;
	control_fb->input_source = encoder_fb->control.input_source;
	control_fb->trust_state = encoder_fb->control.trust_state;
}

static inline void motor_control_publish_encoder_live(
	struct motor_parameters *params,
	const struct motor_control_feedback *control_fb,
	uint8_t position_quality_flags)
{
	if (params == NULL || control_fb == NULL) {
		return;
	}

	params->live.encoder_raw_deg = control_fb->angle_sensor_deg;
	params->live.encoder_raw_rad = control_fb->angle_sensor_deg * (PI_F32 / 180.0f);
	params->live.encoder_observer_input_rad = control_fb->observer_input_rad;
	params->live.observer_mech_rad = control_fb->observer_mech_rad;
	params->live.observer_elec_rad = control_fb->observer_elec_rad;
	params->live.observer_elec_pred_rad = control_fb->observer_elec_pred_rad;
	params->live.observer_elec_speed_rad_s = control_fb->observer_elec_speed_rad_s;
	params->live.observer_delay_samples = control_fb->observer_delay_samples;
	params->live.observer_prediction_age_samples = control_fb->prediction_age_samples;
	params->live.encoder_input_source = control_fb->input_source;
	params->live.position_rad = control_fb->position_mech_rad;
	params->live.position_unwrapped_rad = control_fb->position_mech_rad;
	params->live.velocity_rad_s = control_fb->speed_mech_rad_s;
	params->live.acceleration_rad_s2 = control_fb->accel_mech_rad_s2;
	params->live.velocity_filtered_rad_s = control_fb->speed_mech_filtered_rad_s;
	params->live.position_quality_flags = position_quality_flags;
	params->live.position_trust_state = control_fb->trust_state;
}

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
	ctx->live_detent_iq_ff_a = &params->live.detent_iq_ff_a;
}

static inline void motor_current_ref_policy_ctx_refresh(struct motor_current_ref_policy_ctx *ctx,
							struct motor_parameters *params)
{
	ctx->position_quality_flags = params->live.position_quality_flags;
}

static inline void motor_rls_runtime_ctx_refresh(struct motor_rls_runtime_ctx *ctx,
						 struct motor_parameters *params)
{
#if defined(CONFIG_RLS_PARAMETER_ESTIMATION) && (CONFIG_RLS_PARAMETER_ESTIMATION == 1)
	ctx->rls_feature_enabled =
		atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION);
#else
	ctx->rls_feature_enabled = false;
#endif
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

static inline bool motor_control_step_decimation_tick(bool active,
						      uint32_t *phase,
						      uint32_t decimation)
{
	if (!active) {
		if (phase != NULL) {
			*phase = 0U;
		}
		return false;
	}

	return motor_outer_loop_decimation_tick(phase, decimation);
}

static inline void motor_rt_control_ctx_refresh(struct motor_rt_control_ctx *ctx,
						const struct motor_parameters *params)
{
	struct motor_rt_config_snapshot *cfg = &ctx->cfg_snapshot;
	bool cfg_valid = motor_config_snapshot_read(cfg);
	atomic_val_t feature_flags =
		cfg_valid ? cfg->feature_flags : atomic_get(&params->feature_flags);

	ctx->mode_flags = cfg_valid ? cfg->mode_flags : params->rt_fast.mode_flags_shadow;
	ctx->feature_angle_gen = (feature_flags & BIT(MOTOR_FEATURE_ANGLE_GEN)) != 0;
	ctx->feature_pwm_output = (feature_flags & BIT(MOTOR_FEATURE_PWM_OUTPUT)) != 0;
	ctx->feature_pi_control = (feature_flags & BIT(MOTOR_FEATURE_PI_CONTROL)) != 0;
	ctx->feature_velocity_traj = (feature_flags & BIT(MOTOR_FEATURE_VELOCITY_TRAJ)) != 0;
	ctx->feature_use_commanded_currents =
		(feature_flags & BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS)) != 0;
	ctx->feature_braking = (feature_flags & BIT(MOTOR_FEATURE_BRAKING)) != 0;
	ctx->online_control_state = motor_rt_mode_active(ctx->mode_flags, MOTOR_RT_MODE_ONLINE_CONTROL);
	ctx->control_armed = atomic_get(&params->control_armed) != 0;
	ctx->velocity_loop_decimation =
		CLAMP(cfg_valid ? cfg->velocity_loop_decimation : params->velocity_loop_decimation,
		      OUTER_LOOP_DECIMATION_MIN,
		      OUTER_LOOP_DECIMATION_MAX);
	ctx->position_loop_decimation =
		CLAMP(cfg_valid ? cfg->position_loop_decimation : params->position_loop_decimation,
		      OUTER_LOOP_DECIMATION_MIN,
		      OUTER_LOOP_DECIMATION_MAX);
	ctx->velocity_loop_dt_s =
		(1.0f / CONTROL_LOOP_FREQUENCY_HZ) * (float32_t)ctx->velocity_loop_decimation;
	ctx->position_loop_dt_s =
		(1.0f / CONTROL_LOOP_FREQUENCY_HZ) * (float32_t)ctx->position_loop_decimation;
	ctx->velocity_target_rad_s = params->live.velocity_target_rad_s;
	ctx->velocity_ref_rad_s = params->live.velocity_ref_rad_s;
	ctx->position_mech_rad = params->live.position_rad;
	ctx->speed_mech_rad_s = params->live.velocity_rad_s;
	ctx->accel_mech_rad_s2 = params->live.acceleration_rad_s2;
	ctx->speed_mech_filtered_rad_s = params->live.velocity_filtered_rad_s;

	if (cfg_valid && cfg->control_policy_valid) {
		ctx->policy_input = cfg->control_policy_input;
		ctx->policy = cfg->control_policy;
	} else {
		ctx->policy_input = (struct motor_control_policy_input){
			.mode = motor_control_policy_mode_from_rt_flags(ctx->mode_flags),
			.features = {
				.encoder_read_enabled =
					(feature_flags & BIT(MOTOR_FEATURE_ENCODER_READ)) != 0,
				.angle_gen_enabled = ctx->feature_angle_gen,
				.velocity_traj_enabled = ctx->feature_velocity_traj,
				.commanded_currents_enabled = ctx->feature_use_commanded_currents,
				.current_loop_enabled = ctx->feature_pi_control,
			},
			.profile_sequence_active = cfg_valid ? cfg->profile_sequence_running :
								params->profile_seq.running,
		};
		if (motor_control_policy_derive(&ctx->policy_input, &ctx->policy) != 0) {
			ctx->policy_input = (struct motor_control_policy_input){
				.mode = MOTOR_CONTROL_POLICY_MODE_DISABLED,
			};
			(void)motor_control_policy_derive(&ctx->policy_input, &ctx->policy);
		}
	}

	ctx->meas.observer_input_rad = 0.0f;
	ctx->meas.position_mech_rad = ctx->position_mech_rad;
	ctx->meas.electrical_angle_rad = 0.0f;
	ctx->meas.predicted_electrical_angle_rad = 0.0f;
	ctx->meas.electrical_speed_rad_s = 0.0f;
	ctx->meas.speed_mech_rad_s = ctx->speed_mech_rad_s;
	ctx->meas.accel_mech_rad_s2 = ctx->accel_mech_rad_s2;
	ctx->meas.speed_mech_filtered_rad_s = ctx->speed_mech_filtered_rad_s;
	ctx->meas.observer_delay_samples = 0.0f;
	ctx->meas.prediction_age_samples = 0.0f;
	ctx->meas.encoder_input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;

	ctx->motion_ref.position_rad = ctx->position_mech_rad;
	ctx->motion_ref.velocity_target_rad_s = ctx->velocity_target_rad_s;
	ctx->motion_ref.velocity_ref_rad_s = ctx->velocity_ref_rad_s;
	ctx->motion_ref.velocity_rad_s = ctx->speed_mech_rad_s;
	ctx->motion_ref.acceleration_rad_s2 = ctx->accel_mech_rad_s2;

	ctx->feedback_ref.source = MOTOR_FEEDBACK_NONE;
	ctx->feedback_ref.input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;
	ctx->feedback_ref.position_rad = ctx->position_mech_rad;
	ctx->feedback_ref.electrical_angle_rad = 0.0f;
	ctx->feedback_ref.predicted_electrical_angle_rad = 0.0f;
	ctx->feedback_ref.electrical_speed_rad_s = 0.0f;
	ctx->feedback_ref.velocity_rad_s = ctx->speed_mech_rad_s;
	ctx->feedback_ref.acceleration_rad_s2 = ctx->accel_mech_rad_s2;
	ctx->feedback_ref.velocity_filtered_rad_s = ctx->speed_mech_filtered_rad_s;
	ctx->feedback_ref.observer_delay_samples = 0.0f;
	ctx->feedback_ref.prediction_age_samples = 0.0f;

	ctx->angle_ref.source = ctx->policy.angle_source;
	ctx->current_ref.id_ref_a = params->live.Id_ref_A;
	ctx->current_ref.iq_ref_a = params->live.Iq_ref_A;
}

static inline void motor_control_step_prepare_commission_obs(
	struct motor_commission_observation *obs, uint32_t mode_flags, bool control_armed)
{
	obs->mode_velocity_encoder =
		motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_ENCODER);
	obs->mode_current_encoder = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_CURRENT_ENCODER);
	obs->control_armed = control_armed;
	obs->encoder_fresh = false;
	obs->encoder_warning = false;
	obs->encoder_error = false;
	obs->encoder_status = 0U;
	obs->fault_active = false;
	obs->saturation = false;
	obs->data_valid = false;
	obs->velocity_ref_rad_s = 0.0f;
	obs->mech_position_rad = 0.0f;
}

static MOTOR_ISR_STAGE_NOINLINE int motor_control_step_read_encoder(struct motor_parameters *params,
					   uint32_t mode_flags,
					   const struct motor_control_encoder_sample *encoder_sample,
					   bool feature_angle_gen,
					   struct motor_commission_observation *commission_obs,
					   struct motor_encoder_stage_result *enc_res)
{
	motor_encoder_feedback_ctx_refresh(&params->rt_adapters.encoder_feedback, params);
	int enc_ret = motor_encoder_feedback_update(&params->rt_adapters.encoder_feedback,
						    encoder_sample,
						    feature_angle_gen,
						    &enc_res->feedback);
	motor_control_feedback_from_encoder(&enc_res->feedback, &enc_res->control_fb);

	enc_res->input_source = enc_res->control_fb.input_source;
	enc_res->angle_control_deg = enc_res->control_fb.angle_control_deg;
	enc_res->observer_mech_rad = enc_res->control_fb.observer_mech_rad;
	enc_res->observer_elec_rad = enc_res->control_fb.observer_elec_rad;
	enc_res->observer_elec_pred_rad = enc_res->control_fb.observer_elec_pred_rad;
	enc_res->observer_elec_speed_rad_s = enc_res->control_fb.observer_elec_speed_rad_s;
	enc_res->observer_delay_samples = enc_res->control_fb.observer_delay_samples;
	enc_res->prediction_age_samples = enc_res->control_fb.prediction_age_samples;
	enc_res->fresh = enc_res->control_fb.fresh;
	enc_res->frame_status = enc_res->control_fb.status;
	enc_res->frame_warning = enc_res->control_fb.warning;
	enc_res->frame_error = enc_res->control_fb.error;
	enc_res->io_fault = enc_res->control_fb.io_fault;
	enc_res->position_quality_flags = enc_res->feedback.control.quality_flags;
	enc_res->position_trust_state = enc_res->feedback.control.trust_state;
	motor_control_publish_encoder_live(params, &enc_res->control_fb,
					   enc_res->position_quality_flags);

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
	    encoder_sample != NULL &&
	    encoder_sample->enabled &&
	    enc_res->fresh &&
	    !enc_res->frame_error) {
		float32_t align_mech_rad = enc_res->angle_control_deg * (PI_F32 / 180.0f);
		struct motor_align_sample_accum acc = {0};

		motor_align_load_accum(params, &acc);
		motor_align_accum_push(&acc, align_mech_rad);
		motor_align_store_accum(params, &acc);
	}

	return 0;
}

static inline void motor_control_step_prepare_encoder_reports(
	struct motor_parameters *params,
	const struct motor_control_encoder_sample *encoder_sample,
	const struct motor_encoder_stage_result *enc_res,
	struct motor_control_step_report *report)
{
	if (enc_res == NULL) {
		return;
	}

	if (IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_CAPTURE) &&
	    report != NULL && params->encoder_capture.enabled) {
		if (motor_encoder_feedback_prepare_capture(&params->rt_adapters.encoder_feedback,
							   &enc_res->feedback,
							   &report->encoder_capture) == 0) {
			report->encoder_capture_valid = true;
		}
	}

	if (IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_RAW_TRACE) &&
	    report != NULL && params->encoder_raw_trace.enabled && encoder_sample != NULL) {
		report->encoder_raw_trace_valid = true;
		report->encoder_sample = *encoder_sample;
		report->encoder_feedback = enc_res->control_fb;
		report->position_quality_flags = params->live.position_quality_flags;
	}
}

static inline void motor_control_step_finalize(struct motor_parameters *params,
					       uint32_t mode_flags,
					       bool control_armed,
					       struct motor_commission_observation *commission_obs)
{
	commission_obs->control_armed = control_armed;
	commission_obs->mode_velocity_encoder = motor_rt_mode_active(
		mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_ENCODER);
	commission_obs->mode_current_encoder = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_CURRENT_ENCODER);
	commission_obs->fault_active = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ERROR);
	motor_runtime_fast_sync(params, mode_flags, control_armed);
	motor_runtime_diag_sync(params);
	if (IS_ENABLED(CONFIG_MOTOR_ISR_COMMISSION_CAPTURE)) {
		motor_commission_runtime_ctx_refresh(&params->rt_adapters.commission, params);
		motor_commission_update(&params->rt_adapters.commission, commission_obs);
	}
}

static inline uint16_t motor_detent_capture_bin_from_angle(float32_t mech_angle_rad)
{
	float32_t wrapped = wrap_rad_2pi(mech_angle_rad);
	float32_t scaled = wrapped * ((float32_t)MOTOR_DETENT_MAP_BINS / (2.0f * PI_F32));
	uint16_t bin = (uint16_t)floorf(scaled);

	return (bin >= MOTOR_DETENT_MAP_BINS) ? 0U : bin;
}

static inline void motor_control_step_detent_capture(
	struct motor_parameters *params,
	const struct motor_feedback_ref *feedback_ref,
	const struct motor_current_ref *current_ref)
{
	struct motor_detent_capture_ctx *cap = &params->detent_capture;

	if (!cap->active) {
		return;
	}

	uint32_t decimation = (cap->decimation == 0U) ? 1U : cap->decimation;
	cap->decimation_counter++;
	if (cap->decimation_counter < decimation) {
		return;
	}
	cap->decimation_counter = 0U;

	float32_t omega = feedback_ref->velocity_filtered_rad_s;
	if (!motor_feedback_quality_is_trusted(feedback_ref->quality_flags) ||
	    feedback_ref->error ||
	    !isfinite(feedback_ref->position_rad) ||
	    !isfinite(omega) ||
	    !isfinite(feedback_ref->acceleration_rad_s2) ||
	    !isfinite(current_ref->iq_ref_a) ||
	    !isfinite(cap->kt_nm_per_a) ||
	    cap->kt_nm_per_a <= 0.0f) {
		cap->rejected_quality++;
		cap->rejected_samples++;
		return;
	}
	if (fabsf(omega) < 0.1f ||
	    (cap->target_speed_rad_s != 0.0f &&
	     fabsf(omega - cap->target_speed_rad_s) > cap->velocity_band_rad_s)) {
		cap->rejected_velocity++;
		cap->rejected_samples++;
		return;
	}
	if (cap->accel_limit_rad_s2 > 0.0f &&
	    fabsf(feedback_ref->acceleration_rad_s2) > cap->accel_limit_rad_s2) {
		cap->rejected_accel++;
		cap->rejected_samples++;
		return;
	}
	if (cap->iq_saturation_limit_a > 0.0f &&
	    fabsf(current_ref->iq_ref_a) > cap->iq_saturation_limit_a) {
		cap->rejected_saturation++;
		cap->rejected_samples++;
		return;
	}

	float32_t sign_term = (omega >= 0.0f) ? 1.0f : -1.0f;
	float32_t model_torque_nm =
		(cap->inertia_kgm2 * feedback_ref->acceleration_rad_s2) +
		(cap->viscous_friction_nm_per_rad_s * omega) +
		(cap->coulomb_friction_nm * sign_term);
	float32_t residual_iq_a = current_ref->iq_ref_a - (model_torque_nm / cap->kt_nm_per_a);

	if (!isfinite(residual_iq_a)) {
		cap->rejected_samples++;
		return;
	}

	uint16_t bin = motor_detent_capture_bin_from_angle(feedback_ref->position_rad);
	if (cap->bin_counts[bin] != UINT16_MAX) {
		cap->sum_iq_a[bin] += residual_iq_a;
		cap->bin_counts[bin]++;
		cap->sample_count++;
	}
	if (omega >= 0.0f) {
		if (cap->bin_counts_forward[bin] != UINT16_MAX) {
			cap->sum_iq_forward_a[bin] += residual_iq_a;
			cap->bin_counts_forward[bin]++;
			cap->accepted_forward++;
		}
	} else if (cap->bin_counts_reverse[bin] != UINT16_MAX) {
		cap->sum_iq_reverse_a[bin] += residual_iq_a;
		cap->bin_counts_reverse[bin]++;
		cap->accepted_reverse++;
	}
}

static inline void motor_control_measurements_from_encoder(
	struct motor_control_measurements *meas,
	const struct motor_encoder_stage_result *enc_stage)
{
	if (meas == NULL || enc_stage == NULL) {
		return;
	}

	meas->angle_control_degrees = enc_stage->angle_control_deg;
	meas->encoder_input_source = enc_stage->input_source;
	meas->position_quality_flags = enc_stage->position_quality_flags;
	meas->position_trust_state = enc_stage->position_trust_state;
	meas->fresh_encoder_sample = enc_stage->fresh;
	meas->encoder_frame_status = enc_stage->frame_status;
	meas->encoder_frame_warning = enc_stage->frame_warning;
	meas->encoder_frame_error = enc_stage->frame_error;
	meas->encoder_io_fault = enc_stage->io_fault;
	meas->observer_input_rad = enc_stage->control_fb.observer_input_rad;
	meas->position_mech_rad = enc_stage->control_fb.position_mech_rad;
	meas->electrical_angle_rad = enc_stage->control_fb.electrical_angle_rad;
	meas->predicted_electrical_angle_rad =
		enc_stage->control_fb.predicted_electrical_angle_rad;
	meas->electrical_speed_rad_s = enc_stage->control_fb.electrical_speed_rad_s;
	meas->speed_mech_rad_s = enc_stage->control_fb.speed_mech_rad_s;
	meas->accel_mech_rad_s2 = enc_stage->control_fb.accel_mech_rad_s2;
	meas->speed_mech_filtered_rad_s = enc_stage->control_fb.speed_mech_filtered_rad_s;
	meas->observer_delay_samples = enc_stage->control_fb.observer_delay_samples;
	meas->prediction_age_samples = enc_stage->control_fb.prediction_age_samples;
}

static inline void motor_feedback_ref_from_measurements(
	struct motor_feedback_ref *feedback_ref,
	const struct motor_control_measurements *meas)
{
	if (feedback_ref == NULL || meas == NULL) {
		return;
	}

	switch (meas->encoder_input_source) {
	case MOTOR_ANGLE_INPUT_SRC_ENCODER:
		feedback_ref->source = MOTOR_FEEDBACK_ENCODER;
		break;
	case MOTOR_ANGLE_INPUT_SRC_GENERATED:
		feedback_ref->source = MOTOR_FEEDBACK_GENERATED_REFERENCE;
		break;
	default:
		feedback_ref->source = MOTOR_FEEDBACK_NONE;
		break;
	}
	feedback_ref->input_source = meas->encoder_input_source;
	feedback_ref->quality_flags = meas->position_quality_flags;
	feedback_ref->trust_state = meas->position_trust_state;
	feedback_ref->status = meas->encoder_frame_status;
	feedback_ref->fresh = meas->fresh_encoder_sample;
	feedback_ref->warning = meas->encoder_frame_warning;
	feedback_ref->error = meas->encoder_frame_error || meas->encoder_io_fault;
	feedback_ref->angle_control_deg = meas->angle_control_degrees;
	feedback_ref->observer_input_rad = meas->observer_input_rad;
	feedback_ref->position_rad = meas->position_mech_rad;
	feedback_ref->electrical_angle_rad = meas->electrical_angle_rad;
	feedback_ref->predicted_electrical_angle_rad = meas->predicted_electrical_angle_rad;
	feedback_ref->electrical_speed_rad_s = meas->electrical_speed_rad_s;
	feedback_ref->velocity_rad_s = meas->speed_mech_rad_s;
	feedback_ref->acceleration_rad_s2 = meas->accel_mech_rad_s2;
	feedback_ref->velocity_filtered_rad_s = meas->speed_mech_filtered_rad_s;
	feedback_ref->observer_delay_samples = meas->observer_delay_samples;
	feedback_ref->prediction_age_samples = meas->prediction_age_samples;
}

static MOTOR_ISR_STAGE_NOINLINE bool motor_control_step_measure_stage(struct motor_parameters *params,
					     uint32_t mode_flags,
					     const q31_t *values,
					     const struct motor_current_ref *current_ref,
					     struct motor_current_ref *current_meas_ref,
					     struct motor_control_measurements *meas,
					     struct motor_commission_observation *commission_obs,
					     struct motor_control_step_report *report)
{
	meas->ia_a = adc_to_current(values[CURRENT_SENSE_ADC_BUFFER_INDEX_0], CURRENT_SENSE_POLARITY_0);
	meas->ib_a = adc_to_current(values[CURRENT_SENSE_ADC_BUFFER_INDEX_1], CURRENT_SENSE_POLARITY_1);
	meas->vbus_v = adc_to_vbus_v(values[VBUS_ADC_BUFFER_INDEX]);
	commission_obs->vbus_v = meas->vbus_v;
	params->live.dc_bus_voltage_V = meas->vbus_v;
	params->live.Ia_A = meas->ia_a;
	params->live.Ib_A = meas->ib_a;

	bool vbus_fault_required = motor_vbus_fault_required(mode_flags);
	if (vbus_fault_required && meas->vbus_v < VBUS_MIN_VALID_V) {
		motor_step_report_post_error(report, ERROR_HARDWARE_BREAK);
		return true;
	}

	if (vbus_fault_required && meas->vbus_v > VBUS_MAX_V) {
		motor_step_report_post_error(report, ERROR_OVERVOLTAGE);
		return true;
	}

	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_OFFSET_MEAS)) {
		filter_fo_run(&params->filter_Ia, meas->ia_a);
		filter_fo_run(&params->filter_Ib, meas->ib_a);
		return true;
	}

	meas->ia_a -= params->Ia_offset;
	meas->ib_a -= params->Ib_offset;
	meas->park_angle_rad = meas->electrical_angle_rad;

	if (motor_transforms_park(meas->ia_a, meas->ib_a, meas->park_angle_rad, &meas->id_a,
				  &meas->iq_a) != 0) {
		return true;
	}
	current_meas_ref->id_meas_a = meas->id_a;
	current_meas_ref->iq_meas_a = meas->iq_a;

	if (IS_ENABLED(CONFIG_MOTOR_ISR_FAULT_SNAPSHOT) && params->fault_snapshot.enabled) {
		motor_fault_snapshot_prepare(report,
					     meas->angle_control_degrees,
					     meas->observer_input_rad,
					     meas->park_angle_rad,
					     meas->electrical_speed_rad_s,
					     current_ref->id_ref_a,
					     current_ref->iq_ref_a,
					     meas->id_a,
					     meas->iq_a,
					     meas->ia_a,
					     meas->ib_a,
					     params->Vd_V,
					     params->Vq_V,
					     meas->encoder_input_source,
					     meas->fresh_encoder_sample,
					     meas->encoder_frame_warning,
					     meas->encoder_frame_error,
					     meas->encoder_frame_status,
					     params->live.position_quality_flags);
	}

	if (motor_current_fault_required(mode_flags) &&
	    (fabsf(meas->ia_a) > OVERCURRENT_THRESHOLD_A ||
	     fabsf(meas->ib_a) > OVERCURRENT_THRESHOLD_A)) {
		motor_step_report_post_error(report, ERROR_OVERCURRENT);
		return true;
	}

	return false;
}

static inline void motor_control_step_position_generated_motion(struct motor_parameters *params,
							  bool advance,
							  struct motor_motion_ref *motion_ref,
							  struct motor_feedback_ref *feedback_ref)
{
	if (params == NULL || motion_ref == NULL || feedback_ref == NULL) {
		return;
	}

	if (advance && motion_profile_quintic_is_active(&params->position_profile)) {
		motion_profile_quintic_step(&params->position_profile);
	}

	if (!params->position_profile.valid) {
		return;
	}

	float32_t position_rad = motion_profile_quintic_get_position(&params->position_profile);
	float32_t velocity_rad_s = motion_profile_quintic_is_active(&params->position_profile) ?
					   motion_profile_quintic_get_velocity(&params->position_profile) :
					   0.0f;
	float32_t accel_rad_s2 = motion_profile_quintic_is_active(&params->position_profile) ?
					 motion_profile_quintic_get_accel(&params->position_profile) :
					 0.0f;

	angle_gen_set_angle(&params->angle_gen, position_rad);
	angle_gen_set_velocity(&params->angle_gen, velocity_rad_s);
	motion_ref->position_rad = position_rad;
	motion_ref->velocity_target_rad_s = velocity_rad_s;
	motion_ref->velocity_ref_rad_s = velocity_rad_s;
	motion_ref->velocity_rad_s = velocity_rad_s;
	motion_ref->acceleration_rad_s2 = accel_rad_s2;
	feedback_ref->position_rad = position_rad;
	feedback_ref->velocity_rad_s = velocity_rad_s;
	feedback_ref->acceleration_rad_s2 = accel_rad_s2;
	feedback_ref->velocity_filtered_rad_s = velocity_rad_s;
	params->position_target_rad = wrap_rad_2pi(position_rad);
}

static MOTOR_ISR_STAGE_NOINLINE void motor_control_step_reference_stage(struct motor_parameters *params,
					       struct motor_rt_control_ctx *ctx,
					       struct motor_control_measurements *meas,
					       struct motor_motion_ref *motion_ref,
					       struct motor_feedback_ref *feedback_ref,
					       struct motor_current_ref *current_ref,
					       struct motor_rls_runtime_state *rls_runtime)
{
	uint32_t mode_flags = ctx->mode_flags;

	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ROVERL_MEAS)) {
		current_ref->id_ref_a = traj_get_target_value(&params->traj_Id);
		current_ref->iq_ref_a = 0.0f;
	}

	if (motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_RS_EST)) {
		current_ref->id_ref_a = traj_get_target_value(&params->traj_Id);
		current_ref->iq_ref_a = 0.0f;
	}

	if (motor_is_align_active_state(mode_flags)) {
		current_ref->id_ref_a = traj_get_target_value(&params->traj_Id);
		current_ref->iq_ref_a = 0.0f;
	}

	bool generated_angle_position_driven =
		ctx->policy.generated_angle_mode == MOTOR_GENERATED_ANGLE_POSITION_DRIVEN;
	if (generated_angle_position_driven) {
		motor_control_step_position_generated_motion(params, ctx->control_armed, motion_ref,
						       feedback_ref);
		meas->position_mech_rad = feedback_ref->position_rad;
		meas->speed_mech_rad_s = feedback_ref->velocity_rad_s;
		meas->accel_mech_rad_s2 = feedback_ref->acceleration_rad_s2;
		meas->speed_mech_filtered_rad_s = feedback_ref->velocity_filtered_rad_s;
	}

	bool position_loop_active = motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_POSITION_ENCODER);
	bool velocity_loop_active =
		motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_VELOCITY_ENCODER) ||
		motor_rt_mode_active(mode_flags, MOTOR_RT_MODE_ONLINE_POSITION_ENCODER);
	bool position_loop_update =
		motor_control_step_decimation_tick(position_loop_active,
						   &params->position_loop_phase,
						   ctx->position_loop_decimation);
	bool velocity_loop_update =
		motor_control_step_decimation_tick(velocity_loop_active,
						   &params->velocity_loop_phase,
						   ctx->velocity_loop_decimation);

	ctx->outer_inputs = (struct motor_outer_loop_inputs){
		.position_active = position_loop_active,
		.velocity_active = velocity_loop_active,
		.feature_velocity_traj = ctx->feature_velocity_traj,
		.velocity_loop_update = velocity_loop_update,
		.position_loop_update = position_loop_update,
		.velocity_loop_dt_s = ctx->velocity_loop_dt_s,
		.position_loop_dt_s = ctx->position_loop_dt_s,
		.position_mech_rad = feedback_ref->position_rad,
		.speed_mech_rad_s = feedback_ref->velocity_rad_s,
		.id_meas_a = current_ref->id_meas_a,
		.iq_meas_a = current_ref->iq_meas_a,
		.velocity_target_rad_s = motion_ref->velocity_target_rad_s,
		.velocity_ref_rad_s = motion_ref->velocity_ref_rad_s,
		.id_ref_a = current_ref->id_ref_a,
		.iq_ref_a = current_ref->iq_ref_a,
	};
	ctx->outer_outputs = (struct motor_outer_loop_outputs){0};

	motor_outer_loop_runtime_ctx_refresh(&params->rt_adapters.outer_loop, params,
						 ctx->control_armed);
	(void)motor_outer_loop_runtime_step(&params->rt_adapters.outer_loop, &ctx->outer_inputs,
					    &ctx->outer_outputs);
	motion_ref->velocity_target_rad_s = ctx->outer_outputs.velocity_target_rad_s;
	motion_ref->velocity_ref_rad_s = ctx->outer_outputs.velocity_ref_rad_s;
	meas->speed_mech_filtered_rad_s = ctx->outer_outputs.speed_mech_filtered_rad_s;
	feedback_ref->velocity_filtered_rad_s = ctx->outer_outputs.speed_mech_filtered_rad_s;
	current_ref->id_ref_a = ctx->outer_outputs.id_ref_a;
	current_ref->iq_ref_a = ctx->outer_outputs.iq_ref_a;

	ctx->ref_policy_inputs = (struct motor_current_ref_policy_inputs){
		.online_control_state = ctx->online_control_state,
		.feature_angle_gen = ctx->feature_angle_gen,
		.feature_use_commanded_currents = ctx->feature_use_commanded_currents,
		.control_armed = ctx->control_armed,
		.speed_mech_filtered_rad_s = meas->speed_mech_filtered_rad_s,
		.id_meas_a = current_ref->id_meas_a,
		.iq_meas_a = current_ref->iq_meas_a,
		.velocity_target_rad_s = motion_ref->velocity_target_rad_s,
		.velocity_ref_rad_s = motion_ref->velocity_ref_rad_s,
		.id_ref_a = current_ref->id_ref_a,
		.iq_ref_a = current_ref->iq_ref_a,
	};
	ctx->ref_policy_outputs = (struct motor_current_ref_policy_outputs){0};

	motor_current_ref_policy_ctx_refresh(&params->rt_adapters.current_ref_policy, params);
	(void)motor_current_ref_apply_policy(&params->rt_adapters.current_ref_policy,
					     &ctx->ref_policy_inputs,
					     &ctx->ref_policy_outputs);
	motion_ref->velocity_target_rad_s = ctx->ref_policy_outputs.velocity_target_rad_s;
	motion_ref->velocity_ref_rad_s = ctx->ref_policy_outputs.velocity_ref_rad_s;
	current_ref->id_ref_a = ctx->ref_policy_outputs.id_ref_a;
	current_ref->iq_ref_a = ctx->ref_policy_outputs.iq_ref_a;

	if (ctx->ref_policy_outputs.disarmed_interlock_active) {
		motor_current_slew_params_force_zero(params);
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		angle_gen_set_velocity(&params->angle_gen, 0.0f);
	}

	/* Calibration states preconfigure angle_gen in their entry actions.  In
	 * particular, ROVERL_MEAS depends on a fixed rotating excitation; replacing
	 * that velocity with the online motion reference collapses L estimation.
	 */
	if (ctx->policy.generated_angle_mode == MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN &&
	    !motor_calibration_owns_angle_generator(mode_flags)) {
		angle_gen_set_velocity(&params->angle_gen, motion_ref->velocity_ref_rad_s);
	}

	motor_rls_runtime_ctx_refresh(&params->rt_adapters.rls, params);
	current_ref->id_ref_a = motor_rls_prepare_id_reference(&params->rt_adapters.rls,
							       ctx->online_control_state,
							       ctx->control_armed,
							       feedback_ref->fresh,
							       feedback_ref->error,
							       feedback_ref->input_source,
							       current_ref->id_ref_a,
							       rls_runtime);

	if (motor_is_align_active_state(mode_flags)) {
		current_ref->id_ref_a = traj_get_target_value(&params->traj_Id);
		current_ref->iq_ref_a = 0.0f;
	}

	motor_control_step_current_slew_stage(params, mode_flags,
					      ctx->feature_use_commanded_currents,
					      meas, current_ref);

	if (ctx->policy.generated_angle_mode == MOTOR_GENERATED_ANGLE_VELOCITY_DRIVEN) {
		angle_gen_run(&params->angle_gen);
	}
}

static MOTOR_ISR_STAGE_NOINLINE bool motor_control_step_foc_stage(struct motor_parameters *params,
					 const struct motor_rt_control_ctx *ctx,
					 const struct motor_control_measurements *meas,
					 const struct motor_motion_ref *motion_ref,
					 const struct motor_feedback_ref *feedback_ref,
					 const struct motor_actuator_ref *actuator_ref,
					 struct motor_angle_ref *angle_ref,
					 const struct motor_current_ref *current_ref,
					 struct motor_commutation_ref *commutation_ref,
					 struct motor_control_pwm_output *pwm_out)
{
	ARG_UNUSED(motion_ref);

	angle_ref->electrical_angle_rad = feedback_ref->electrical_angle_rad;
	angle_ref->predicted_electrical_angle_rad =
		feedback_ref->predicted_electrical_angle_rad;
	angle_ref->electrical_speed_rad_s = feedback_ref->electrical_speed_rad_s;
	angle_ref->source = ctx->policy.angle_source;

	float32_t decoupling_speed_limit_rad_s =
		MAX(50.0f, params->profile_max_velocity_rad_s * (float32_t)MOTOR_POLE_PAIRS * 1.5f);
	float32_t flux_linkage_wb_abs = fabsf(params->flux_linkage_wb_active);
	bool current_encoder_mode_state = motor_rt_mode_active(ctx->mode_flags, MOTOR_RT_MODE_ONLINE_CURRENT_ENCODER);
	bool decoupling_min_speed_reached =
		fabsf(feedback_ref->velocity_filtered_rad_s) >=
		CURRENT_DECOUPLING_MIN_MECH_SPEED_RAD_S;
	bool decoupling_flux_valid = isfinite(flux_linkage_wb_abs) &&
				     (flux_linkage_wb_abs >= CURRENT_DECOUPLING_MIN_FLUX_WB) &&
				     (flux_linkage_wb_abs <= CURRENT_DECOUPLING_MAX_FLUX_WB);
	bool decoupling_speed_valid = isfinite(angle_ref->electrical_speed_rad_s) &&
				      (fabsf(angle_ref->electrical_speed_rad_s) <=
				       decoupling_speed_limit_rad_s);
	bool decoupling_feedback_valid = ctx->policy.angle_source == MOTOR_ANGLE_SOURCE_GENERATED ||
					 motor_velocity_feedback_is_valid(params->live.position_quality_flags);
	struct motor_dq_decoupling_enable_input decoupling_enable_in = {
		.feature_enabled = CURRENT_DECOUPLING_ENABLED,
		.online_control_state = ctx->online_control_state,
		.control_armed = ctx->control_armed,
		.current_encoder_mode_state = current_encoder_mode_state,
		.min_speed_reached = decoupling_min_speed_reached,
		.flux_valid = decoupling_flux_valid,
		.speed_valid = decoupling_speed_valid,
		.feedback_valid = decoupling_feedback_valid,
	};
	bool dq_decoupling_enabled = motor_dq_decoupling_is_enabled(&decoupling_enable_in);
	float32_t decoupling_speed_rad_s =
		decoupling_speed_valid ? angle_ref->electrical_speed_rad_s : 0.0f;

#if defined(CONFIG_MOTOR_ISR_SANITY_CHECKS) && (CONFIG_MOTOR_ISR_SANITY_CHECKS == 1)
	if (!isfinite(angle_ref->predicted_electrical_angle_rad) ||
	    !isfinite(actuator_ref->id_ref_a) || !isfinite(actuator_ref->iq_ref_a) ||
	    !isfinite(current_ref->id_meas_a) || !isfinite(current_ref->iq_meas_a) ||
	    !isfinite(meas->vbus_v) || !isfinite(params->max_modulation_index) ||
	    params->max_modulation_index <= 0.0f) {
		return true;
	}
#endif

	float32_t max_voltage_magnitude_v = params->max_modulation_index * meas->vbus_v;
	if (max_voltage_magnitude_v <= 0.0f) {
		return true;
	}

	float32_t vd_ff_v = 0.0f;
	float32_t vq_ff_v = 0.0f;
	if (motor_dq_decoupling_feedforward_step_fast_values(
		    dq_decoupling_enabled,
		    decoupling_speed_rad_s,
		    params->Ld_measured_H,
		    params->Lq_measured_H,
		    params->flux_linkage_wb_active,
		    current_ref->id_meas_a,
		    current_ref->iq_meas_a,
		    max_voltage_magnitude_v,
		    CURRENT_DQ_DECOUPLING_FLUX_HEADROOM_RATIO,
		    CURRENT_DQ_DECOUPLING_FF_LIMIT_RATIO,
		    &vd_ff_v,
		    &vq_ff_v) != 0) {
		return true;
	}

	float32_t vq_limit_v = 0.0f;
	if (fabsf(actuator_ref->id_ref_a) <= 1.0e-5f &&
	    fabsf(actuator_ref->iq_ref_a) <= 1.0e-5f) {
		pi_set_ui(&params->pi_Id, 0.0f);
		pi_set_ui(&params->pi_Iq, 0.0f);
	}
	if (motor_current_loop_step_fast_values(&params->pi_Id,
						&params->pi_Iq,
						actuator_ref->id_ref_a,
						actuator_ref->iq_ref_a,
						current_ref->id_meas_a,
						current_ref->iq_meas_a,
						max_voltage_magnitude_v,
						vd_ff_v,
						vq_ff_v,
						&commutation_ref->vd_v,
						&commutation_ref->vq_v,
						&vq_limit_v) != 0) {
		return true;
	}

	if (motor_transforms_inv_park(commutation_ref->vd_v,
				      commutation_ref->vq_v,
				      angle_ref->predicted_electrical_angle_rad,
				      &commutation_ref->va_v,
				      &commutation_ref->vb_v) != 0) {
		return true;
	}

	float32_t ua_pu;
	float32_t ub_pu;
	float32_t da_pu;
	float32_t db_pu;
	if (motor_pwm_synthesis_step_fast_values(commutation_ref->va_v,
						 commutation_ref->vb_v,
						 meas->vbus_v,
						 ctx->feature_braking,
						 actuator_ref->iq_ref_a,
						 meas->speed_mech_rad_s,
						 VBUS_REGEN_LIMIT_V,
						 VBUS_VOLTAGE_MARGIN_INV,
						 &ua_pu,
						 &ub_pu,
						 &da_pu,
						 &db_pu,
						 &commutation_ref->da_hb1_pu,
						 &commutation_ref->da_hb2_pu,
						 &commutation_ref->db_hb1_pu,
						 &commutation_ref->db_hb2_pu) != 0) {
		return true;
	}
	ARG_UNUSED(ua_pu);
	ARG_UNUSED(ub_pu);
	ARG_UNUSED(da_pu);
	ARG_UNUSED(db_pu);
	ARG_UNUSED(vq_limit_v);

	commutation_ref->max_voltage_magnitude_v = max_voltage_magnitude_v;

	float32_t voltage_norm_sq = commutation_ref->vd_v * commutation_ref->vd_v +
				    commutation_ref->vq_v * commutation_ref->vq_v;
	float32_t voltage_limit = 0.98f * commutation_ref->max_voltage_magnitude_v;
	commutation_ref->voltage_saturated = commutation_ref->max_voltage_magnitude_v > 0.0f &&
					 voltage_norm_sq >= (voltage_limit * voltage_limit);

	pwm_out->da_hb1_pu = commutation_ref->da_hb1_pu;
	pwm_out->da_hb2_pu = commutation_ref->da_hb2_pu;
	pwm_out->db_hb1_pu = commutation_ref->db_hb1_pu;
	pwm_out->db_hb2_pu = commutation_ref->db_hb2_pu;
	pwm_out->update_pwm = true;

	return false;
}

static inline bool motor_control_step_electrical_id_voltage_active(
	const struct motor_electrical_id_capture_ctx *cap)
{
	return cap != NULL && cap->active && cap->direct_voltage_enabled &&
	       (cap->mode == MOTOR_ELECTRICAL_ID_CAPTURE_LD ||
		cap->mode == MOTOR_ELECTRICAL_ID_CAPTURE_LQ);
}

static MOTOR_ISR_STAGE_NOINLINE bool motor_control_step_electrical_id_voltage_stage(
	struct motor_parameters *params,
	const struct motor_rt_control_ctx *ctx,
	const struct motor_control_measurements *meas,
	const struct motor_feedback_ref *feedback_ref,
	struct motor_angle_ref *angle_ref,
	struct motor_commutation_ref *commutation_ref,
	struct motor_control_pwm_output *pwm_out)
{
	struct motor_electrical_id_capture_ctx *cap = &params->electrical_id_capture;

	angle_ref->electrical_angle_rad = feedback_ref->electrical_angle_rad;
	angle_ref->predicted_electrical_angle_rad =
		feedback_ref->predicted_electrical_angle_rad;
	angle_ref->electrical_speed_rad_s = feedback_ref->electrical_speed_rad_s;
	angle_ref->source = ctx->policy.angle_source;

	float32_t max_voltage_magnitude_v = params->max_modulation_index * meas->vbus_v;
	if (max_voltage_magnitude_v <= 0.0f || cap->voltage_limit_v <= 0.0f) {
		return true;
	}

	float32_t voltage_limit_v = fminf(cap->voltage_limit_v, max_voltage_magnitude_v);
	commutation_ref->vd_v = clampf(cap->vd_cmd_v, -voltage_limit_v, voltage_limit_v);
	commutation_ref->vq_v = clampf(cap->vq_cmd_v, -voltage_limit_v, voltage_limit_v);

	if (motor_transforms_inv_park(commutation_ref->vd_v,
				      commutation_ref->vq_v,
				      angle_ref->predicted_electrical_angle_rad,
				      &commutation_ref->va_v,
				      &commutation_ref->vb_v) != 0) {
		return true;
	}

	float32_t ua_pu;
	float32_t ub_pu;
	float32_t da_pu;
	float32_t db_pu;
	if (motor_pwm_synthesis_step_fast_values(commutation_ref->va_v,
						 commutation_ref->vb_v,
						 meas->vbus_v,
						 ctx->feature_braking,
						 0.0f,
						 0.0f,
						 VBUS_REGEN_LIMIT_V,
						 VBUS_VOLTAGE_MARGIN_INV,
						 &ua_pu,
						 &ub_pu,
						 &da_pu,
						 &db_pu,
						 &commutation_ref->da_hb1_pu,
						 &commutation_ref->da_hb2_pu,
						 &commutation_ref->db_hb1_pu,
						 &commutation_ref->db_hb2_pu) != 0) {
		return true;
	}
	ARG_UNUSED(ua_pu);
	ARG_UNUSED(ub_pu);
	ARG_UNUSED(da_pu);
	ARG_UNUSED(db_pu);

	commutation_ref->max_voltage_magnitude_v = max_voltage_magnitude_v;
	commutation_ref->voltage_saturated =
		(fabsf(commutation_ref->vd_v) >= voltage_limit_v) ||
		(fabsf(commutation_ref->vq_v) >= voltage_limit_v);

	pwm_out->da_hb1_pu = commutation_ref->da_hb1_pu;
	pwm_out->da_hb2_pu = commutation_ref->da_hb2_pu;
	pwm_out->db_hb1_pu = commutation_ref->db_hb1_pu;
	pwm_out->db_hb2_pu = commutation_ref->db_hb2_pu;
	pwm_out->update_pwm = true;

	return false;
}

static MOTOR_ISR_STAGE_NOINLINE bool motor_control_step_actuator_stage(
	struct motor_parameters *params,
	const struct motor_rt_control_ctx *ctx,
	const struct motor_control_measurements *meas,
	const struct motor_motion_ref *motion_ref,
	const struct motor_feedback_ref *feedback_ref,
	const struct motor_actuator_ref *actuator_ref,
	struct motor_angle_ref *angle_ref,
	const struct motor_current_ref *current_ref,
	struct motor_commutation_ref *commutation_ref,
	struct motor_control_pwm_output *pwm_out)
{
	if (actuator_ref == NULL || !actuator_ref->enabled) {
		return false;
	}

	if (motor_control_step_electrical_id_voltage_active(&params->electrical_id_capture)) {
		return motor_control_step_electrical_id_voltage_stage(params, ctx, meas,
								     feedback_ref, angle_ref,
								     commutation_ref, pwm_out);
	}

	switch (actuator_ref->kind) {
	case MOTOR_ACTUATOR_FOC_CURRENT:
		if (actuator_ref->effort_kind != MOTOR_ACTUATOR_EFFORT_CURRENT_DQ) {
			return true;
		}
		return motor_control_step_foc_stage(params, ctx, meas, motion_ref, feedback_ref,
						    actuator_ref, angle_ref, current_ref,
						    commutation_ref, pwm_out);
	default:
		return true;
	}
}

static MOTOR_ISR_STAGE_NOINLINE void motor_control_step_publish_stage(
	struct motor_parameters *params,
	const struct motor_control_measurements *meas,
	const struct motor_motion_ref *motion_ref,
	const struct motor_feedback_ref *feedback_ref,
	const struct motor_angle_ref *angle_ref,
	const struct motor_current_ref *current_ref,
	const struct motor_commutation_ref *commutation_ref,
	const struct motor_rls_runtime_state *rls_runtime,
	struct motor_commission_observation *commission_obs)
{
	motor_rls_runtime_ctx_refresh(&params->rt_adapters.rls, params);
	motor_rls_update_estimators(&params->rt_adapters.rls, rls_runtime, meas->id_a, meas->iq_a);

	params->live.position_rad = feedback_ref->position_rad;
	params->live.position_unwrapped_rad = feedback_ref->position_rad;
	params->live.position_innovation_rad = 0.0f;
	params->live.observer_mech_rad = feedback_ref->position_rad;
	params->live.observer_elec_rad = feedback_ref->electrical_angle_rad;
	params->live.observer_elec_pred_rad = feedback_ref->predicted_electrical_angle_rad;
	params->live.observer_elec_speed_rad_s = feedback_ref->electrical_speed_rad_s;
	params->live.velocity_rad_s = feedback_ref->velocity_rad_s;
	params->live.acceleration_rad_s2 = feedback_ref->acceleration_rad_s2;
	params->live.velocity_filtered_rad_s = feedback_ref->velocity_filtered_rad_s;
	params->live.velocity_target_rad_s = motion_ref->velocity_target_rad_s;
	params->live.velocity_ref_rad_s = motion_ref->velocity_ref_rad_s;

	motor_control_step_detent_capture(params, feedback_ref, current_ref);

	params->live.Id_ref_A = current_ref->id_ref_a;
	params->live.Iq_ref_A = current_ref->iq_ref_a;
	params->live.Id_A = current_ref->id_meas_a;
	params->live.Iq_A = current_ref->iq_meas_a;
	params->live.Ia_A = meas->ia_a;
	params->live.Ib_A = meas->ib_a;
	params->Vd_V = commutation_ref->vd_v;
	params->Vq_V = commutation_ref->vq_v;
	params->live.Va_V = commutation_ref->va_v;
	params->live.Vb_V = commutation_ref->vb_v;
	params->max_voltage_magnitude_V = commutation_ref->max_voltage_magnitude_v;
	params->live.elec_angle_rad = angle_ref->predicted_electrical_angle_rad;
	params->live.dc_bus_voltage_V = meas->vbus_v;

	commission_obs->data_valid = true;
	commission_obs->id_a = current_ref->id_meas_a;
	commission_obs->iq_a = current_ref->iq_meas_a;
	commission_obs->vd_v = commutation_ref->vd_v;
	commission_obs->vq_v = commutation_ref->vq_v;
	commission_obs->mech_position_rad = params->live.position_rad;
	commission_obs->mech_speed_rad_s = params->live.velocity_rad_s;
	commission_obs->elec_speed_rad_s = angle_ref->electrical_speed_rad_s;
	commission_obs->velocity_ref_rad_s = motion_ref->velocity_ref_rad_s;
	commission_obs->saturation = commutation_ref->voltage_saturated;
}

static inline void motor_control_step_electrical_id_capture(
	struct motor_parameters *params,
	const struct motor_current_ref *current_ref,
	const struct motor_commutation_ref *commutation_ref)
{
	struct motor_electrical_id_capture_ctx *cap = &params->electrical_id_capture;

	if (!cap->active || cap->done || cap->target_samples == 0U) {
		return;
	}

	cap->update_count++;
	int ret = 0;
	switch (cap->mode) {
	case MOTOR_ELECTRICAL_ID_CAPTURE_RS:
		ret = motor_electrical_id_rs_add(&cap->rs_accum, &cap->rs_cfg,
						 commutation_ref->vd_v,
						 current_ref->id_meas_a);
		break;
	case MOTOR_ELECTRICAL_ID_CAPTURE_LD:
	case MOTOR_ELECTRICAL_ID_CAPTURE_LQ: {
		bool q_axis = cap->mode == MOTOR_ELECTRICAL_ID_CAPTURE_LQ;
		float32_t current = q_axis ? current_ref->iq_meas_a : current_ref->id_meas_a;
		float32_t voltage = q_axis ? commutation_ref->vq_v : commutation_ref->vd_v;

		if (cap->direct_voltage_enabled) {
			if (cap->l_segment_active) {
				if (cap->l_segment_latch_pending) {
					cap->l_segment_start_current_a = current;
					cap->l_segment_last_current_a = current;
					cap->l_segment_flux_vs = 0.0f;
					cap->l_segment_samples = 0U;
					cap->l_segment_latch_pending = false;
				} else {
					float32_t v_eff = voltage - (cap->rs_ohm * current);

					cap->l_segment_flux_vs += v_eff *
								  cap->l_cfg.dt_s;
					cap->l_segment_last_current_a = current;
					cap->l_segment_samples++;
				}
			}
			return;
		}

		if (!cap->prev_valid) {
			cap->prev_current_a = current;
			cap->prev_valid = true;
			return;
		}
		ret = motor_electrical_id_l_add(&cap->l_accum, &cap->l_cfg,
						voltage, current,
						cap->prev_current_a,
						cap->rs_ohm);
		cap->prev_current_a = current;
		break;
	}
	case MOTOR_ELECTRICAL_ID_CAPTURE_VALIDATE_ID: {
		float32_t err = fabsf(cap->validation_target_a - current_ref->id_meas_a);
		cap->validation_sum_abs_error_a += err;
		cap->validation_max_abs_error_a = fmaxf(cap->validation_max_abs_error_a, err);
		ret = 0;
		break;
	}
	default:
		cap->active = false;
		cap->done = true;
		cap->valid = false;
		return;
	}

	if (ret == 0) {
		cap->sample_count++;
	} else if (ret > 0) {
		cap->rejected_samples++;
	} else {
		cap->rejected_samples++;
	}

	if (cap->sample_count >= cap->target_samples) {
		cap->active = false;
		cap->done = true;
		cap->valid = true;
	}
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

	struct motor_rt_control_ctx *ctx = &params->rt_control;
	motor_rt_control_ctx_refresh(ctx, params);

	uint32_t mode_flags = ctx->mode_flags;
	bool generated_angle_active = ctx->policy.angle_source == MOTOR_ANGLE_SOURCE_GENERATED;
	bool feature_pwm_output = ctx->feature_pwm_output;
	bool feature_pi_control = ctx->feature_pi_control;
	bool control_armed = ctx->control_armed;
	struct motor_commission_observation *commission_obs = &ctx->commission_obs;
	motor_control_step_prepare_commission_obs(commission_obs, mode_flags, control_armed);
	pwm_out->update_pwm = false;

	/* Increment control loop counter */
	params->control_loop_count++;
	commission_obs->control_loop_count = params->control_loop_count;
	struct motor_control_measurements *meas = &ctx->meas;
	struct motor_motion_ref *motion_ref = &ctx->motion_ref;
	struct motor_feedback_ref *feedback_ref = &ctx->feedback_ref;
	struct motor_servo_ref *servo_ref = &ctx->servo_ref;
	struct motor_actuator_ref *actuator_ref = &ctx->actuator_ref;
	struct motor_angle_ref *angle_ref = &ctx->angle_ref;
	struct motor_current_ref *current_ref = &ctx->current_ref;
	struct motor_commutation_ref *commutation_ref = &ctx->commutation_ref;
	struct motor_rls_runtime_state *rls_runtime = &ctx->rls_runtime;

	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = params->velocity_dob_state.disturbance_nm;
	params->live.velocity_dob_residual_rad_s = 0.0f;

	/* Read encoder if feature is enabled */
	struct motor_encoder_stage_result *enc_stage = &ctx->enc_stage;
	int enc_ret = motor_control_step_read_encoder(params, mode_flags, encoder_sample,
						      generated_angle_active, commission_obs,
						      enc_stage);
	if (enc_ret == -EIO) {
		motor_step_report_post_error_with_encoder_reason(
			report, ERROR_ENCODER_FAULT, MOTOR_ENCODER_FAULT_REASON_READ_EIO);
		goto isr_done;
	}
	motor_control_step_prepare_encoder_reports(params, encoder_sample, enc_stage, report);
	motor_control_measurements_from_encoder(meas, enc_stage);
	motor_feedback_ref_from_measurements(feedback_ref, meas);
	if (!motor_control_kernel_feedback_valid(&ctx->policy, feedback_ref,
						 params->live.position_stale_count,
						 ENCODER_FAULT_THRESHOLD)) {
		motor_step_report_post_error_with_encoder_reason(
			report, ERROR_ENCODER_FAULT,
			motor_encoder_fault_reason_for_invalid_feedback(
				feedback_ref, params->live.position_stale_count,
				ENCODER_FAULT_THRESHOLD));
		goto isr_done;
	}
	if (!motor_control_kernel_feedback_sane(&ctx->policy, feedback_ref,
						params->profile_max_velocity_rad_s)) {
		motor_step_report_post_error_with_encoder_reason(
			report, ERROR_ENCODER_FAULT,
			motor_encoder_fault_reason_for_insane_feedback(feedback_ref));
		goto isr_done;
	}

	if (motor_control_step_measure_stage(params, mode_flags, values, current_ref, current_ref,
					     meas, commission_obs, report)) {
		goto isr_done;
	}

	/* ADC sampling/telemetry is independent of PWM output. */
	if (!feature_pwm_output) {
		goto isr_done;
	}

	/* Skip PI control if not enabled */
	if (!feature_pi_control) {
		goto isr_done;
	}

	motor_control_step_reference_stage(params, ctx, meas, motion_ref, feedback_ref,
					   current_ref, rls_runtime);
	ctx->kernel_input = (struct motor_control_kernel_input){
		.policy = &ctx->policy,
		.motion_ref = motion_ref,
		.feedback_ref = feedback_ref,
		.current_ref = current_ref,
		.current_loop_enabled = ctx->feature_pi_control,
		.feedback_stale_count = params->live.position_stale_count,
		.feedback_stale_limit = ENCODER_FAULT_THRESHOLD,
		.profile_max_velocity_rad_s = params->profile_max_velocity_rad_s,
	};
	if (motor_control_kernel_step_fast(&ctx->kernel_input, &ctx->kernel_output) != 0) {
		goto isr_done;
	}
	*servo_ref = ctx->kernel_output.servo_ref;
	*actuator_ref = ctx->kernel_output.actuator_ref;

	if (!actuator_ref->enabled) {
		goto isr_done;
	}

	if (motor_control_step_actuator_stage(params, ctx, meas, motion_ref, feedback_ref,
					      actuator_ref, angle_ref, current_ref,
					      commutation_ref, pwm_out)) {
		goto isr_done;
	}

	motor_control_step_publish_stage(params, meas, motion_ref, feedback_ref, angle_ref,
					 current_ref, commutation_ref, rls_runtime, commission_obs);
	motor_control_step_electrical_id_capture(params, current_ref, commutation_ref);

isr_done:
	motor_control_step_finalize(params, mode_flags, control_armed, commission_obs);
	return;
}
