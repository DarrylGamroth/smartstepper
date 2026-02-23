/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <dsp/controller_functions.h>

#include "motor_control_loop.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "math_constants.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "angle_observer.h"
#include "angle_gen.h"
#include "angle_wrap.h"
#include "motor_autonomy.h"
#include "motor_rls_runtime.h"
#include "motor_foc_voltage_pwm.h"
#include "motor_state_utils.h"
#include "motor_commission.h"
#include "motor_dob.h"
#include "motor_motion_modules.h"
#include "motor_torque.h"

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

static inline bool motor_outer_loop_use_mpr(const struct motor_parameters *params)
{
	return params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR;
}

static inline bool motor_velocity_feedback_is_valid(uint8_t quality_flags)
{
	const uint8_t required = MOTOR_POSITION_CONVERT_QUALITY_FRESH;
	const uint8_t forbidden = MOTOR_POSITION_CONVERT_QUALITY_ERROR |
				  MOTOR_POSITION_CONVERT_QUALITY_GLITCH;

	return ((quality_flags & required) != 0U) &&
	       ((quality_flags & forbidden) == 0U);
}

static inline bool motor_outer_loop_decimation_tick(uint32_t *phase, uint32_t decimation)
{
	if (phase == NULL || decimation <= 1U) {
		if (phase != NULL) {
			*phase = 0U;
		}
		return true;
	}

	if (*phase == 0U) {
		*phase = decimation - 1U;
		return true;
	}

	(*phase)--;
	return false;
}

static inline void motor_encoder_capture_try_store(struct motor_parameters *params,
						   float32_t angle_deg,
						   float32_t angle_rad,
						   bool sample_enabled,
						   bool sample_fresh,
						   bool sample_warning,
						   bool sample_error,
						   uint8_t status,
						   uint8_t input_source)
{
	if (params == NULL || !params->encoder_capture_enabled) {
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
	sample->control_loop_count = params->control_loop_count;
	sample->angle_deg = angle_deg;
	sample->angle_rad = angle_rad;
	sample->input_source = input_source;
	sample->sample_enabled = sample_enabled ? 1U : 0U;
	sample->sample_fresh = sample_fresh ? 1U : 0U;
	sample->sample_warning = sample_warning ? 1U : 0U;
	sample->sample_error = sample_error ? 1U : 0U;
	sample->status = status;

	params->encoder_capture_write_idx =
		(uint16_t)((idx + 1U) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	if (params->encoder_capture_count < MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
		params->encoder_capture_count++;
	} else {
		params->encoder_capture_overrun_count++;
	}
}

void motor_control_loop_step(struct motor_parameters *params,
			     const q31_t *values,
			     uint8_t count,
			     const struct motor_control_encoder_sample *encoder_sample,
			     struct motor_control_pwm_output *pwm_out)
{
	ARG_UNUSED(count);
	if (params == NULL || values == NULL || pwm_out == NULL) {
		return;
	}

	const struct smf_state *state = params->state_for_isr;
	atomic_val_t feature_flags = atomic_get(&params->feature_flags);
	bool feature_angle_gen = (feature_flags & BIT(MOTOR_FEATURE_ANGLE_GEN)) != 0;
	bool feature_pwm_output = (feature_flags & BIT(MOTOR_FEATURE_PWM_OUTPUT)) != 0;
	bool feature_pi_control = (feature_flags & BIT(MOTOR_FEATURE_PI_CONTROL)) != 0;
	bool feature_velocity_traj = (feature_flags & BIT(MOTOR_FEATURE_VELOCITY_TRAJ)) != 0;
	bool feature_use_commanded_currents =
		(feature_flags & BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS)) != 0;
	bool feature_braking = (feature_flags & BIT(MOTOR_FEATURE_BRAKING)) != 0;
	bool online_control_state = motor_state_ptr_is_online_control_state(state);
	bool control_armed = atomic_get(&params->control_armed) != 0;
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
	commission_obs.control_loop_count = params->control_loop_count;

	float32_t angle_sensor_degrees = 0.0f;
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
	float32_t dt_s = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t encoder_direction_sign =
		(params->encoder_direction_sign >= 0) ? 1.0f : -1.0f;
	uint32_t velocity_loop_decimation =
		CLAMP(params->velocity_loop_decimation, OUTER_LOOP_DECIMATION_MIN,
		      OUTER_LOOP_DECIMATION_MAX);
	uint32_t position_loop_decimation =
		CLAMP(params->position_loop_decimation, OUTER_LOOP_DECIMATION_MIN,
		      OUTER_LOOP_DECIMATION_MAX);
	float32_t velocity_loop_dt_s = dt_s * (float32_t)velocity_loop_decimation;
	float32_t position_loop_dt_s = dt_s * (float32_t)position_loop_decimation;
	float32_t velocity_target_rad_s = params->velocity_target_rad_s;
	float32_t velocity_ref_rad_s = params->velocity_ref_rad_s;
	float32_t position_mech_rad = params->position_rad;
	float32_t speed_mech_rad_s = params->velocity_rad_s;
	float32_t accel_mech_rad_s2 = params->acceleration_rad_s2;
	float32_t speed_mech_filtered_rad_s = params->velocity_filtered_rad_s;
	uint8_t prev_encoder_input_source = params->encoder_input_source;
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
		motor_autonomy_should_keepalive(control_armed, autonomous_mode_active,
						 params->profile_sequence_running,
						 params->chopper_cal_active,
						 motion_profile_quintic_is_active(&params->position_profile));
	if (autonomous_keepalive) {
		now_ms = k_uptime_get_32();
		params->last_command_update_ms = now_ms;
		params->command_timeout_latched = false;
	}

	/* Timeout disarms output commands when command updates stop. */
	if (online_control_state && control_armed && !autonomous_keepalive &&
	    params->command_timeout_ms > 0U) {
		now_ms = k_uptime_get_32();
		uint32_t elapsed_ms = now_ms - params->last_command_update_ms;

		if (elapsed_ms > params->command_timeout_ms) {
			control_armed = false;
			atomic_set(&params->control_armed, 0);
			if (!params->command_timeout_latched) {
				params->command_timeout_latched = true;
				params->command_timeout_count++;
			}
		}
	}

	/* Read encoder if feature is enabled */
	float32_t angle_raw_rad;
	bool encoder_sample_enabled = false;
	bool encoder_sample_available = false;
	bool fresh_encoder_sample = false;
	uint8_t encoder_frame_status = 0U;
	bool encoder_frame_warning = false;
	bool encoder_frame_error = false;
	if (encoder_sample != NULL &&
	    (encoder_sample->enabled || params->encoder_capture_enabled)) {
		encoder_sample_enabled = encoder_sample->enabled;
		encoder_sample_available = true;
		angle_sensor_degrees = encoder_sample->angle_deg;
		angle_control_degrees = angle_sensor_degrees * encoder_direction_sign;
		encoder_frame_status = encoder_sample->status;
		encoder_frame_warning = encoder_sample->warning;
		encoder_frame_error = encoder_sample->error;
		fresh_encoder_sample = encoder_sample->fresh;
		if (fresh_encoder_sample || encoder_frame_warning || encoder_frame_error) {
			params->encoder_last_status = encoder_frame_status;
		}
	}

	if (encoder_sample != NULL && encoder_sample->enabled) {
		if (!fresh_encoder_sample) {
			if (encoder_sample->io_fault) {
				params->encoder_fault_counter++;
			}
			if (encoder_frame_warning) {
				params->encoder_warning_count++;
			}
			if (encoder_frame_error) {
				params->encoder_error_count++;
			}
		} else {
			/* Reset fault counter on successful read. */
			params->encoder_fault_counter = 0;
			if (encoder_frame_warning) {
				params->encoder_warning_count++;
			}
		}

		params->encoder_sample_fresh = fresh_encoder_sample ? 1U : 0U;
		params->encoder_sample_warning = encoder_frame_warning ? 1U : 0U;
		params->encoder_sample_error = encoder_frame_error ? 1U : 0U;
		commission_obs.encoder_fresh = fresh_encoder_sample;
		commission_obs.encoder_warning = encoder_frame_warning;
		commission_obs.encoder_error = encoder_frame_error;
		commission_obs.encoder_status = encoder_frame_status;

		/* Fault detection: too many consecutive encoder failures. */
		if (params->encoder_fault_counter > ENCODER_FAULT_THRESHOLD) {
			motor_api_post_error(ERROR_ENCODER_FAULT);
			goto isr_done;
		}
	} else {
		/* Encoder not active - reset fault counter */
		params->encoder_fault_counter = 0;
		params->encoder_sample_fresh = 0U;
		params->encoder_sample_warning = 0U;
		params->encoder_sample_error = 0U;
		commission_obs.encoder_fresh = false;
		commission_obs.encoder_warning = false;
		commission_obs.encoder_error = false;
	}
	
	/* Select angle source based on feature flag */
	if (feature_angle_gen) {
		/* Calibration/open-loop: use generated angle (no delay) */
		angle_raw_rad = angle_gen_get_angle(&params->angle_gen);
		angle_observer_set_delay(&params->observer, 0.0f);
		encoder_input_source = MOTOR_ANGLE_INPUT_SRC_GENERATED;
	} else if (encoder_sample_enabled && fresh_encoder_sample) {
		/* Normal operation: use fresh encoder reading (1-cycle pipelined delay) */
		angle_raw_rad = angle_control_degrees * (PI_F32 / 180.0f);
		angle_observer_set_delay(&params->observer, 1.0f);
		encoder_input_source = MOTOR_ANGLE_INPUT_SRC_ENCODER;
		params->encoder_raw_deg = angle_sensor_degrees;
		params->encoder_raw_rad = angle_sensor_degrees * (PI_F32 / 180.0f);
	} else {
		/* No fresh encoder sample: propagate using prior estimate only. */
		angle_raw_rad = angle_observer_get_mech_angle(&params->observer);
		angle_observer_set_delay(&params->observer, 0.0f);
		encoder_input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;
	}

	if (encoder_input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER &&
	    prev_encoder_input_source == MOTOR_ANGLE_INPUT_SRC_GENERATED &&
	    fresh_encoder_sample) {
		motor_position_convert_reset(&params->position_convert, wrap_rad_2pi(angle_raw_rad));
	}
	
	/* Update observer with angle (encoder or generated) */
	angle_observer_update(&params->observer, angle_raw_rad);
	params->encoder_observer_input_rad = angle_raw_rad;
	params->encoder_input_source = encoder_input_source;

	float32_t capture_angle_rad = encoder_sample_available ?
					     (angle_control_degrees * (PI_F32 / 180.0f)) :
					     angle_raw_rad;
	float32_t capture_angle_deg = encoder_sample_available ?
					     angle_control_degrees :
					     (angle_raw_rad * (180.0f / PI_F32));
	uint8_t capture_input_source = encoder_sample_available ?
					      MOTOR_ANGLE_INPUT_SRC_ENCODER :
					      encoder_input_source;
	motor_encoder_capture_try_store(params, capture_angle_deg, capture_angle_rad,
					encoder_sample_available, fresh_encoder_sample,
					encoder_frame_warning, encoder_frame_error,
					encoder_frame_status, capture_input_source);

	struct motor_position_convert_input pos_input = {
		.sample_valid = false,
		.sample_fresh = false,
		.source_generated = false,
		.warning = encoder_frame_warning,
		.error = encoder_frame_error,
		.measurement_wrapped_rad = 0.0f,
		.latency_samples = 0.0f,
	};
	switch (encoder_input_source) {
	case MOTOR_ANGLE_INPUT_SRC_GENERATED:
		pos_input.sample_valid = true;
		pos_input.sample_fresh = true;
		pos_input.source_generated = true;
		pos_input.measurement_wrapped_rad = wrap_rad_2pi(angle_raw_rad);
		pos_input.latency_samples = 0.0f;
		break;
	case MOTOR_ANGLE_INPUT_SRC_ENCODER:
		pos_input.sample_valid = true;
		pos_input.sample_fresh = fresh_encoder_sample;
		pos_input.source_generated = false;
		pos_input.measurement_wrapped_rad = wrap_rad_2pi(angle_raw_rad);
		pos_input.latency_samples = 1.0f;
		break;
	case MOTOR_ANGLE_INPUT_SRC_PROPAGATED:
	default:
		pos_input.sample_valid = false;
		pos_input.sample_fresh = false;
		pos_input.source_generated = false;
		pos_input.measurement_wrapped_rad = 0.0f;
		pos_input.latency_samples = 0.0f;
		break;
	}

	int pos_ret = motor_position_convert_update(&params->position_convert,
						    &params->position_convert_cfg,
						    &pos_input);
	if (pos_ret != 0) {
		motor_position_convert_reset(&params->position_convert, wrap_rad_2pi(angle_raw_rad));
	}
	params->position_quality_flags = params->position_convert.quality_flags;
	params->position_stale_count = params->position_convert.stale_count;
	params->position_stale_events = params->position_convert.stale_event_count;
	params->position_glitch_count = params->position_convert.glitch_count;
	params->position_jitter_count = params->position_convert.jitter_count;

	position_mech_rad = params->position_convert.position_wrapped_rad;
	speed_mech_rad_s = params->position_convert.velocity_rad_s;
	accel_mech_rad_s2 = params->position_convert.accel_rad_s2;
	speed_mech_filtered_rad_s = speed_mech_rad_s;

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
		motor_api_post_error(ERROR_HARDWARE_BREAK);
		goto isr_done;
	}

	/* Fault detection: Check for overvoltage */
	if (Vbus_V > VBUS_MAX_V) {
		motor_api_post_error(ERROR_OVERVOLTAGE);
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

	/* Fault detection: Check for overcurrent after offset removal */
	if (fabsf(Ia_A) > OVERCURRENT_THRESHOLD_A || fabsf(Ib_A) > OVERCURRENT_THRESHOLD_A) {
		motor_api_post_error(ERROR_OVERCURRENT);
		goto isr_done;
	}

	/* Skip PI control if not enabled */
	if (!feature_pi_control) {
		goto isr_done;
	}

	/* Select control frame angle based on state and transform currents */
	float32_t park_angle_rad = angle_observer_get_elec_angle(&params->observer);

	/* Park transform to dq frame - convert to degrees for arm_sin_cos_f32 */
	float32_t park_angle_deg = park_angle_rad * (180.0f / PI_F32);
	arm_sin_cos_f32(park_angle_deg, &sin_theta, &cos_theta);
	arm_park_f32(Ia_A, Ib_A, &Id_A, &Iq_A, sin_theta, cos_theta);

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

	/* ALIGN: ramp/hold alignment current */
	if (state == &motor_states[MOTOR_STATE_ALIGN] ||
	    state == &motor_states[MOTOR_STATE_ALIGN_SAMPLE]) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;
	}

	/* Position cascade: generate velocity target from position error. */
	if (state == &motor_states[MOTOR_STATE_ONLINE_POSITION]) {
		float32_t position_error_rad;
		float32_t profile_velocity_ff_rad_s = 0.0f;
		float32_t pos_i_limit_rad_s = params->profile_max_velocity_rad_s;
		bool use_mpr = motor_outer_loop_use_mpr(params);
		bool position_loop_update = motor_outer_loop_decimation_tick(
			&params->position_loop_phase, position_loop_decimation);

		/* Position move module resolves profile state into target/error/feedforward. */
		bool move_active = motor_position_move_resolve(&params->position_profile,
							       control_armed,
							       position_mech_rad,
							       &params->position_target_rad,
							       &position_error_rad,
							       &profile_velocity_ff_rad_s);
		if (!move_active) {
			position_error_rad =
				wrap_rad_pi(params->position_target_rad - position_mech_rad);
		}

		if (position_loop_update) {
			if (use_mpr) {
				params->position_mpr_cfg.dt_s = position_loop_dt_s;
				params->position_mpr_cfg.velocity_limit_rad_s =
					params->profile_max_velocity_rad_s;
				params->position_mpr_cfg.max_delta_velocity_rad_s =
					params->profile_max_accel_rad_s2 * position_loop_dt_s;
				int mpr_ret = motor_mpr_position_step(&params->position_mpr_cfg,
								      &params->position_mpr_state,
								      position_error_rad,
								      profile_velocity_ff_rad_s,
								      &velocity_target_rad_s);
				if (mpr_ret != 0) {
					velocity_target_rad_s = clampf(profile_velocity_ff_rad_s,
							      -params->profile_max_velocity_rad_s,
							      params->profile_max_velocity_rad_s);
					motor_mpr_position_reset(&params->position_mpr_state,
								 velocity_target_rad_s);
				}
				params->position_cl_i_term_rad_s = 0.0f;
			} else {
				float32_t position_fb_velocity_rad_s;
				float32_t pos_i_next =
					params->position_cl_i_term_rad_s +
					(params->position_cl_ki_rad_s2_per_rad *
					 position_error_rad * position_loop_dt_s);
				pos_i_next = clampf(pos_i_next, -pos_i_limit_rad_s, pos_i_limit_rad_s);
				params->position_cl_i_term_rad_s = pos_i_next;
				position_fb_velocity_rad_s =
					params->position_cl_kp_rad_s_per_rad * position_error_rad +
					params->position_cl_i_term_rad_s;
				velocity_target_rad_s =
					profile_velocity_ff_rad_s + position_fb_velocity_rad_s;
			}

			velocity_target_rad_s =
				clampf(velocity_target_rad_s, -params->profile_max_velocity_rad_s,
				       params->profile_max_velocity_rad_s);
			traj_set_target_value(&params->traj_velocity, velocity_target_rad_s);
		}
	} else {
		params->position_loop_phase = 0U;
	}

	/* Update velocity trajectory if enabled */
	if (feature_velocity_traj) {
		motor_velocity_plan_step(&params->traj_velocity,
					&velocity_target_rad_s,
					&velocity_ref_rad_s);

		/* Open-loop commutation uses the trajectory directly. */
		if (feature_angle_gen) {
			angle_gen_set_velocity(&params->angle_gen, velocity_ref_rad_s);
		}
	}

	/* Closed-loop velocity and position share the same inner velocity->Iq stage. */
	if (state == &motor_states[MOTOR_STATE_ONLINE_VELOCITY_CLOSED] ||
	    state == &motor_states[MOTOR_STATE_ONLINE_POSITION]) {
		speed_mech_filtered_rad_s = filter_so_run(&params->filter_velocity_notch,
							 speed_mech_rad_s);
		bool velocity_feedback_valid =
			motor_velocity_feedback_is_valid(params->position_quality_flags);
		bool velocity_loop_update = motor_outer_loop_decimation_tick(
			&params->velocity_loop_phase, velocity_loop_decimation);
		if (velocity_loop_update) {
			if (!velocity_feedback_valid) {
				/* Hold prior current command and reset observers while encoder quality
				 * is degraded to avoid direction flips from propagated/glitched speed.
				 */
				Iq_ref_A = clampf(Iq_ref_A,
						 -params->velocity_cl_iq_limit_A,
						 params->velocity_cl_iq_limit_A);
				motor_mpr_velocity_reset(&params->velocity_mpr_state,
							 speed_mech_filtered_rad_s,
							 Iq_ref_A);
				motor_dob_reset(&params->velocity_dob_state, speed_mech_filtered_rad_s);
				params->velocity_dob_iq_ff_a = 0.0f;
				params->velocity_dob_disturbance_nm = 0.0f;
				params->velocity_dob_residual_rad_s = 0.0f;
			} else {
				bool use_mpr = motor_outer_loop_use_mpr(params);
				bool mpr_applied = false;
				float32_t iq_cmd_pre_dob_a = 0.0f;
				float32_t torque_gain_nm_per_a = motor_torque_gain_resolve_active(params);

				if (use_mpr) {
					struct motor_mpr_velocity_model mpr_model = {
						.inertia_kgm2 = params->inertia_kgm2_active,
						.viscous_friction_nm_per_rad_s =
							params->viscous_friction_nm_per_rad_s_active,
						.coulomb_friction_nm = params->coulomb_friction_nm_active,
						.torque_constant_nm_per_a = torque_gain_nm_per_a,
					};
					float32_t iq_cmd_mpr_a = 0.0f;

					params->velocity_mpr_cfg.dt_s = velocity_loop_dt_s;
					params->velocity_mpr_cfg.iq_limit_a = params->velocity_cl_iq_limit_A;
					int mpr_ret = motor_mpr_velocity_step(&params->velocity_mpr_cfg, &mpr_model,
									      &params->velocity_mpr_state,
									      speed_mech_filtered_rad_s,
									      velocity_ref_rad_s,
									      &iq_cmd_mpr_a);
					if (mpr_ret == 0) {
						Id_ref_A = params->Id_setpoint_A;
						iq_cmd_pre_dob_a = clampf(iq_cmd_mpr_a,
									  -params->velocity_cl_iq_limit_A,
									  params->velocity_cl_iq_limit_A);
						Iq_ref_A = iq_cmd_pre_dob_a;
						params->velocity_cl_i_term_A = 0.0f;
						mpr_applied = true;
					}
				}

				if (!mpr_applied) {
					float32_t speed_error_rad_s = velocity_ref_rad_s - speed_mech_filtered_rad_s;
					float32_t vel_i_next =
						params->velocity_cl_i_term_A +
						(params->velocity_cl_ki_A_per_rad * speed_error_rad_s *
						 velocity_loop_dt_s);
					vel_i_next = clampf(vel_i_next, -params->velocity_cl_iq_limit_A,
							   params->velocity_cl_iq_limit_A);
					params->velocity_cl_i_term_A = vel_i_next;

					Id_ref_A = params->Id_setpoint_A;
					iq_cmd_pre_dob_a =
						clampf((params->velocity_cl_kp_A_per_rad_s * speed_error_rad_s) +
						       params->velocity_cl_i_term_A,
						       -params->velocity_cl_iq_limit_A,
						       params->velocity_cl_iq_limit_A);
					Iq_ref_A = iq_cmd_pre_dob_a;
				}

				if (isfinite(torque_gain_nm_per_a) && torque_gain_nm_per_a > 0.0f) {
					struct motor_dob_model dob_model = {
						.inertia_kgm2 = params->inertia_kgm2_active,
						.viscous_friction_nm_per_rad_s =
							params->viscous_friction_nm_per_rad_s_active,
						.coulomb_friction_nm = params->coulomb_friction_nm_active,
						.torque_constant_nm_per_a = torque_gain_nm_per_a,
					};
					struct motor_dob_config dob_cfg = params->velocity_dob_cfg;
					float32_t iq_limit = params->velocity_cl_iq_limit_A;
					float32_t auto_torque_limit = torque_gain_nm_per_a * iq_limit;

					dob_cfg.dt_s = velocity_loop_dt_s;
					if (!isfinite(dob_cfg.torque_limit_nm) || dob_cfg.torque_limit_nm <= 0.0f) {
						dob_cfg.torque_limit_nm = auto_torque_limit;
					}
					if (!isfinite(dob_cfg.iq_ff_limit_a) || dob_cfg.iq_ff_limit_a <= 0.0f) {
						dob_cfg.iq_ff_limit_a = iq_limit;
					}

					float32_t iq_dob_ff_a = 0.0f;
					int dob_ret = motor_dob_step(&dob_cfg, &dob_model, &params->velocity_dob_state,
								     speed_mech_filtered_rad_s,
								     iq_cmd_pre_dob_a,
								     &iq_dob_ff_a);
					if (dob_ret == 0) {
						params->velocity_dob_iq_ff_a = iq_dob_ff_a;
						params->velocity_dob_disturbance_nm =
							params->velocity_dob_state.disturbance_nm;
						params->velocity_dob_residual_rad_s =
							params->velocity_dob_state.residual_rad_s;
						Iq_ref_A = clampf(iq_cmd_pre_dob_a + iq_dob_ff_a,
								 -params->velocity_cl_iq_limit_A,
								 params->velocity_cl_iq_limit_A);
					} else {
						motor_dob_reset(&params->velocity_dob_state, speed_mech_filtered_rad_s);
						params->velocity_dob_iq_ff_a = 0.0f;
						params->velocity_dob_disturbance_nm = 0.0f;
						params->velocity_dob_residual_rad_s = 0.0f;
					}
				}
			}
		}
	} else {
		params->velocity_loop_phase = 0U;
	}

	/* Select current references based on mode */
	if (feature_use_commanded_currents) {
		/* Normal FOC operation: use commanded current references */
		Id_ref_A = params->Id_setpoint_A;
		Iq_ref_A = params->Iq_setpoint_A;
	}

	/* Arm/disarm interlock only applies in ONLINE control states. */
	if (online_control_state && !control_armed) {
		Id_ref_A = Id_A;
		Iq_ref_A = Iq_A;
		params->Id_setpoint_A = 0.0f;
		params->Iq_setpoint_A = 0.0f;
		velocity_target_rad_s = 0.0f;
		velocity_ref_rad_s = 0.0f;
		params->velocity_target_rad_s = 0.0f;
		params->velocity_ref_rad_s = 0.0f;
		params->velocity_cl_i_term_A = 0.0f;
		params->position_cl_i_term_rad_s = 0.0f;
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		angle_gen_set_velocity(&params->angle_gen, 0.0f);
		pi_set_ui(&params->pi_Id, 0.0f);
		pi_set_ui(&params->pi_Iq, 0.0f);
		motor_mpr_velocity_reset(&params->velocity_mpr_state,
					 speed_mech_filtered_rad_s, 0.0f);
		motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
		motor_dob_reset(&params->velocity_dob_state, speed_mech_filtered_rad_s);
		params->velocity_dob_iq_ff_a = 0.0f;
		params->velocity_dob_disturbance_nm = 0.0f;
		params->velocity_dob_residual_rad_s = 0.0f;
	}

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
		.decoupling_enabled = CURRENT_DECOUPLING_ENABLED &&
				     online_control_state && control_armed,
		.electrical_speed_rad_s = angle_observer_get_elec_speed(&params->observer),
		.ld_h = params->Ld_est,
		.lq_h = params->Lq_est,
		.flux_linkage_wb = params->flux_linkage_wb_active,
		.braking_enabled =
			feature_braking,
		.braking_iq_ref_a = params->Iq_ref_A,
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

	/* Update telemetry snapshot (position_convert provides mechanical domain signals). */
	params->position_rad = position_mech_rad;
	params->position_unwrapped_rad = params->position_convert.position_unwrapped_rad;
	params->position_innovation_rad = params->position_convert.innovation_rad;
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
	motor_commission_update(params, &commission_obs);
	return;
}
