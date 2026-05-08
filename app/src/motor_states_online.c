/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <drivers/gate_driver/ti_drv8328.h>

#include "motor_states.h"
#include "motor_states_online.h"
#include "motor_control_api.h"
#include "config.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/motion_planner.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"
#include "motor_encoder_control.h"
#include "motor_operating_mode.h"
#include "motor_state_transition.h"

LOG_MODULE_DECLARE(motor_states, CONFIG_APP_LOG_LEVEL);

static inline void motor_enable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next |= mask;
}

static inline void motor_disable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next &= ~mask;
}

static inline void motor_online_reset_feedback_quality(struct motor_parameters *params)
{
	params->live.position_quality_flags = 0U;
	params->live.position_trust_state = MOTOR_FEEDBACK_TRUST_FAULT;
	params->live.position_stale_count = 0U;
	params->live.position_stale_events = 0U;
	params->live.position_glitch_count = 0U;
	params->live.position_jitter_count = 0U;
}

static inline enum motor_state motor_online_current_substate(const struct motor_parameters *params)
{
	if (motor_state_ptr_is_mode(params->smf.current, MOTOR_STATE_ONLINE_CURRENT_ENCODER)) {
		return MOTOR_STATE_ONLINE_CURRENT_ENCODER;
	}
	if (motor_state_ptr_is_mode(params->smf.current, MOTOR_STATE_ONLINE_VELOCITY_ENCODER)) {
		return MOTOR_STATE_ONLINE_VELOCITY_ENCODER;
	}
	if (motor_state_ptr_is_mode(params->smf.current, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		return MOTOR_STATE_ONLINE_POSITION_ENCODER;
	}
	if (motor_state_ptr_is_mode(params->smf.current, MOTOR_STATE_ONLINE_VELOCITY_GENERATED)) {
		return MOTOR_STATE_ONLINE_VELOCITY_GENERATED;
	}
	if (motor_state_ptr_is_mode(params->smf.current, MOTOR_STATE_ONLINE_POSITION_GENERATED)) {
		return MOTOR_STATE_ONLINE_POSITION_GENERATED;
	}
	return MOTOR_STATE_ONLINE;
}

static int motor_position_plan_sequence_move(struct motor_parameters *params, float32_t target_wrapped_rad)
{
	int ret = motor_position_move_plan_sequence_segment(&params->position_profile,
							    params->live.position_rad,
							    params->live.velocity_rad_s,
							    target_wrapped_rad,
							    params->profile_seq.end_velocity_rad_s,
							    params->profile_seq.move_duration_s,
							    params->profile_max_velocity_rad_s,
							    params->profile_max_accel_rad_s2);
	if (ret != 0) {
		return ret;
	}

	params->position_target_rad = wrap_rad_2pi(params->live.position_rad);
	params->last_command_update_ms = k_uptime_get_32();
	params->last_command_update_loop = params->control_loop_count;
	params->command_timeout_latched = false;

	return 0;
}

static enum smf_state_result motor_profile_sequence_run(struct motor_parameters *params)
{
	if (params == NULL) {
		return SMF_EVENT_PROPAGATE;
	}

	switch (params->event.type) {
	case MOTOR_EVENT_PROFILE_SEQ_TICK: {
		if (!params->profile_seq.running) {
			return SMF_EVENT_HANDLED;
		}
		if (atomic_get(&params->control_armed) == 0) {
			return SMF_EVENT_HANDLED;
		}

		float32_t target_wrapped_rad = 0.0f;
		bool complete_after_take = false;
		int ret = motor_position_sequence_take_next(params->profile_seq.points_rad,
							    params->profile_seq.count,
							    params->profile_seq.loop,
							    &params->profile_seq.next_idx,
							    &target_wrapped_rad,
							    &complete_after_take);
		if (ret == -ENOENT) {
			params->profile_seq.running = false;
			params->profile_seq.tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}
		if (ret != 0) {
			LOG_ERR("Profile sequence index update failed (%d), stopping", ret);
			params->profile_seq.running = false;
			params->profile_seq.tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}

		ret = motor_position_plan_sequence_move(params, target_wrapped_rad);
		if (ret != 0) {
			LOG_ERR("Profile sequence move planning failed (%d), stopping", ret);
			params->profile_seq.running = false;
			params->profile_seq.tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}

		if (complete_after_take) {
			params->profile_seq.running = false;
			params->profile_seq.tick_counter = 0U;
			LOG_INF("Profile sequence completed");
		}
		return SMF_EVENT_HANDLED;
	}

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

void motor_state_online_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE state");
	LOG_INF("ONLINE observer offset: base=%.3f deg stored=%.3f deg",
		(double)(params->observer_alignment_offset_rad * (180.0f / PI_F32)),
		(double)(params->observer.mech_angle_offset_rad * (180.0f / PI_F32)));

	/* ONLINE modes require power-stage channels enabled.
	 * IDLE entry disables them, so re-enable on every ONLINE entry.
	 */
	if (motor_hardware_restart_pwm_adc_trigger() < 0) {
		LOG_ERR("Failed to restart PWM/ADC trigger path");
		motor_api_post_error(ERROR_HARDWARE_BREAK);
		return;
	}
	drv8328_enable_channel(gate_driver_a, 0);
	drv8328_enable_channel(gate_driver_a, 1);
	drv8328_enable_channel(gate_driver_b, 0);
	drv8328_enable_channel(gate_driver_b, 1);

	/* ONLINE baseline requirements (shared by all ONLINE substates). */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_BRAKING) |
				     BIT(MOTOR_FEATURE_PWM_OUTPUT) |
				     BIT(MOTOR_FEATURE_PI_CONTROL) |
				     BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
#if defined(CONFIG_RLS_PARAMETER_ESTIMATION) && (CONFIG_RLS_PARAMETER_ESTIMATION == 1)
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_RLS_ESTIMATION));
#endif
}

void motor_state_online_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE state");

	/* Clear ONLINE baseline requirements on exit. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_BRAKING) |
				      BIT(MOTOR_FEATURE_PWM_OUTPUT) |
				      BIT(MOTOR_FEATURE_PI_CONTROL) |
				      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
#if defined(CONFIG_RLS_PARAMETER_ESTIMATION) && (CONFIG_RLS_PARAMETER_ESTIMATION == 1)
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_RLS_ESTIMATION));
#endif
}

enum smf_state_result motor_state_online_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_IDLE:
		LOG_INF("IDLE request received, transitioning to IDLE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_ERROR:
		LOG_WRN("%s event received, code: %s, transitioning to ERROR",
			motor_event_to_string(params->event.type),
			motor_error_to_string(params->event.error_code));
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_CALIBRATE_REQUEST:
		LOG_INF("Calibrate request received from ONLINE, running boot calibration");
		params->calibration.mode = MOTOR_CALIBRATION_MODE_BOOT;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_COMMISSION_REQUEST:
		LOG_INF("Commission request received from ONLINE, running commissioning sequence");
		params->calibration.mode = MOTOR_CALIBRATION_MODE_COMMISSIONING;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_MODE_CHANGE:
		/* Transition to requested control mode substate */
		LOG_INF("%s event: changing to state %s",
			motor_event_to_string(params->event.type),
			motor_state_to_string(params->event.target_mode));

		/* Validate target is an ONLINE substate */
		if (!motor_state_is_online_submode(params->event.target_mode)) {
			LOG_ERR("Invalid mode change target: %s",
				motor_state_to_string(params->event.target_mode));
			motor_transition_status_update(params, MOTOR_EVENT_MODE_CHANGE,
						       params->event.target_mode,
						       motor_online_current_substate(params),
						       motor_online_current_substate(params),
						       motor_online_current_substate(params),
						       MOTOR_TRANSITION_RESULT_REJECTED,
						       ERROR_NONE,
						       "target is not an online submode");
			return SMF_EVENT_HANDLED;
		}

		if (motor_encoder_control_mode_requires_encoder(params->event.target_mode)) {
			char reason[96] = {0};

			if (!motor_encoder_control_ready_for_transition(
				    params, motor_online_current_substate(params),
				    params->event.target_mode, true, reason,
				    sizeof(reason))) {
				LOG_ERR("Encoder mode %s rejected: %s",
					motor_state_to_string(params->event.target_mode),
					reason);
				motor_transition_status_update(params, MOTOR_EVENT_MODE_CHANGE,
							       params->event.target_mode,
							       motor_online_current_substate(params),
							       motor_online_current_substate(params),
							       motor_online_current_substate(params),
							       MOTOR_TRANSITION_RESULT_REJECTED,
							       ERROR_NONE, reason);
				return SMF_EVENT_HANDLED;
			}
		}

		motor_transition_status_update(params, MOTOR_EVENT_MODE_CHANGE,
					       params->event.target_mode,
					       motor_online_current_substate(params),
					       params->event.target_mode,
					       params->event.target_mode,
					       MOTOR_TRANSITION_RESULT_COMPLETED,
					       ERROR_NONE, "mode changed");
		smf_set_state(SMF_CTX(params), &motor_states[params->event.target_mode]);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

/* Substate: ONLINE_CURRENT_ENCODER - encoder-commutated direct Id/Iq control */
void motor_state_online_current_encoder_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE_CURRENT_ENCODER substate (direct Id/Iq control)");

	/* Encoder-based control: add encoder read; ONLINE provides the baseline. */
	motor_enable_isr_feature_flags(params,
				       motor_operating_mode_feature_mask(
					       MOTOR_STATE_ONLINE_CURRENT_ENCODER));
	motor_online_reset_feedback_quality(params);
	motor_operating_mode_apply_entry_policy(params, MOTOR_STATE_ONLINE_CURRENT_ENCODER);
}

void motor_state_online_current_encoder_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_CURRENT_ENCODER substate");

	/* Clear this substate's additional requirements. */
	motor_operating_mode_apply_exit_policy(params, MOTOR_STATE_ONLINE_CURRENT_ENCODER);
	motor_disable_isr_feature_flags(params,
					motor_operating_mode_feature_mask(
						MOTOR_STATE_ONLINE_CURRENT_ENCODER));
}

enum smf_state_result motor_state_online_current_encoder_run(void *obj)
{
	ARG_UNUSED(obj);
	/* Direct Id/Iq control - no additional processing needed.
	 * Id/Iq setpoints are controlled via shell commands.
	 * Parent ONLINE state handles stop/error events
	 */
	return SMF_EVENT_PROPAGATE;
}

/* Substate: ONLINE_VELOCITY_GENERATED - generated-angle velocity control */
void motor_state_online_velocity_generated_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t mech_angle_rad = params->live.position_rad;

	LOG_INF("Entering ONLINE_VELOCITY_GENERATED substate");

	/* Generated-angle velocity control uses angle generator and velocity trajectory.
	 * ONLINE provides the baseline.
	 */
	motor_enable_isr_feature_flags(params,
				       motor_operating_mode_feature_mask(
					       MOTOR_STATE_ONLINE_VELOCITY_GENERATED));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));

	/* Initialize angle generator for open-loop mode */
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	/* Preserve commutation frame across mode transitions. */
	angle_gen_set_angle(&params->angle_gen, mech_angle_rad);

	motor_operating_mode_apply_entry_policy(params,
						MOTOR_STATE_ONLINE_VELOCITY_GENERATED);

	LOG_INF("Generated-angle velocity mode initialized: max=%.1f Hz, accel=%.1f Hz/s",
		(double)(params->profile_max_velocity_rad_s / (2.0f * PI_F32)),
		(double)(params->profile_max_accel_rad_s2 / (2.0f * PI_F32)));
}

enum smf_state_result motor_state_online_velocity_generated_run(void *obj)
{
	ARG_UNUSED(obj);
	/* Velocity control happens in motor_isr based on active substate
	 * This handler just maintains state and propagates events to parent
	 */
	return SMF_EVENT_PROPAGATE;
}

void motor_state_online_velocity_generated_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_VELOCITY_GENERATED substate");

	motor_operating_mode_apply_exit_policy(params,
					       MOTOR_STATE_ONLINE_VELOCITY_GENERATED);

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params,
					motor_operating_mode_feature_mask(
						MOTOR_STATE_ONLINE_VELOCITY_GENERATED));
}

/* Substate: ONLINE_POSITION_GENERATED - generated-angle position/profile control */
void motor_state_online_position_generated_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t mech_angle_rad = params->live.position_rad;

	LOG_INF("Entering ONLINE_POSITION_GENERATED substate");

	/* Generated-angle position/profile control drives mechanical position directly.
	 * Encoder reads remain disabled; capture telemetry can still request samples.
	 */
	motor_enable_isr_feature_flags(params,
				       motor_operating_mode_feature_mask(
					       MOTOR_STATE_ONLINE_POSITION_GENERATED));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
						      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));

	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, mech_angle_rad);
	motor_operating_mode_apply_entry_policy(params,
						MOTOR_STATE_ONLINE_POSITION_GENERATED);

	LOG_INF("Generated-angle position/profile mode initialized at %.2f deg",
		(double)(mech_angle_rad * 180.0f / PI_F32));
}

enum smf_state_result motor_state_online_position_generated_run(void *obj)
{
	return motor_profile_sequence_run((struct motor_parameters *)obj);
}

void motor_state_online_position_generated_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	LOG_INF("Exiting ONLINE_POSITION_GENERATED substate");

	motor_operating_mode_apply_exit_policy(params,
					       MOTOR_STATE_ONLINE_POSITION_GENERATED);

	motor_disable_isr_feature_flags(params,
					motor_operating_mode_feature_mask(
						MOTOR_STATE_ONLINE_POSITION_GENERATED));
}

/* Substate: ONLINE_VELOCITY_ENCODER - encoder-feedback velocity control */
void motor_state_online_velocity_encoder_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE_VELOCITY_ENCODER substate");

	/* Encoder-feedback velocity uses measured speed and acceleration-limited velocity profile. */
	motor_enable_isr_feature_flags(params,
				       motor_operating_mode_feature_mask(
					       MOTOR_STATE_ONLINE_VELOCITY_ENCODER));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
						      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
	motor_online_reset_feedback_quality(params);

	motor_operating_mode_apply_entry_policy(params,
						MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
}

enum smf_state_result motor_state_online_velocity_encoder_run(void *obj)
{
	ARG_UNUSED(obj);
	return SMF_EVENT_PROPAGATE;
}

void motor_state_online_velocity_encoder_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_VELOCITY_ENCODER substate");

	/* Clear this substate's additional requirements. */
	motor_operating_mode_apply_exit_policy(params,
					       MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	motor_disable_isr_feature_flags(params,
					motor_operating_mode_feature_mask(
						MOTOR_STATE_ONLINE_VELOCITY_ENCODER));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

/* Substate: ONLINE_POSITION_ENCODER - encoder-feedback position/profile control */
void motor_state_online_position_encoder_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE_POSITION_ENCODER substate");

	motor_enable_isr_feature_flags(params,
				       motor_operating_mode_feature_mask(
					       MOTOR_STATE_ONLINE_POSITION_ENCODER));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
						      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
	motor_online_reset_feedback_quality(params);

	motor_operating_mode_apply_entry_policy(params,
						MOTOR_STATE_ONLINE_POSITION_ENCODER);
}

enum smf_state_result motor_state_online_position_encoder_run(void *obj)
{
	return motor_profile_sequence_run((struct motor_parameters *)obj);
}

void motor_state_online_position_encoder_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_POSITION_ENCODER substate");

	motor_operating_mode_apply_exit_policy(params,
					       MOTOR_STATE_ONLINE_POSITION_ENCODER);

	motor_disable_isr_feature_flags(params,
					motor_operating_mode_feature_mask(
						MOTOR_STATE_ONLINE_POSITION_ENCODER));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}
