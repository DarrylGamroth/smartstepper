/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "motor_states.h"
#include "motor_states_online.h"
#include "motor_control_api.h"
#include "config.h"
#include "pi.h"
#include "traj.h"
#include "angle_observer.h"
#include "angle_gen.h"
#include "angle_wrap.h"
#include "motor_state_utils.h"

LOG_MODULE_DECLARE(motor_states, CONFIG_APP_LOG_LEVEL);

static inline void motor_enable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next |= mask;
}

static inline void motor_disable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next &= ~mask;
}

static int motor_position_plan_sequence_move(struct motor_parameters *params, float32_t target_wrapped_rad)
{
	float32_t start_pos_rad = params->position_rad;
	float32_t start_vel_rad_s = params->velocity_rad_s;
	float32_t delta_rad = wrap_rad_pi(target_wrapped_rad - start_pos_rad);
	float32_t end_pos_rad = start_pos_rad + delta_rad;

	int ret = motion_profile_quintic_plan(&params->position_profile,
					      start_pos_rad, start_vel_rad_s, 0.0f,
					      end_pos_rad, params->profile_sequence_end_velocity_rad_s,
					      0.0f, params->profile_sequence_move_duration_s);
	if (ret != 0) {
		return ret;
	}

	ret = motion_profile_quintic_check_limits(&params->position_profile,
						  params->profile_max_velocity_rad_s,
						  params->profile_max_accel_rad_s2, 64U,
						  NULL, NULL);
	if (ret != 0) {
		motion_profile_quintic_cancel(&params->position_profile, start_pos_rad);
		return ret;
	}

	params->position_target_rad = wrap_rad_2pi(start_pos_rad);
	params->last_command_update_ms = k_uptime_get_32();
	params->command_timeout_latched = false;

	return 0;
}

void motor_state_online_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE state");

	/* ONLINE baseline requirements (shared by all ONLINE substates). */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_BRAKING) |
				     BIT(MOTOR_FEATURE_PWM_OUTPUT) |
				     BIT(MOTOR_FEATURE_PI_CONTROL) |
				     BIT(MOTOR_FEATURE_RLS_ESTIMATION) |
				     BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

void motor_state_online_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE state");

	/* Clear ONLINE baseline requirements on exit. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_BRAKING) |
				      BIT(MOTOR_FEATURE_PWM_OUTPUT) |
				      BIT(MOTOR_FEATURE_PI_CONTROL) |
				      BIT(MOTOR_FEATURE_RLS_ESTIMATION) |
				      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
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

	case MOTOR_EVENT_MODE_CHANGE:
		/* Transition to requested control mode substate */
		LOG_INF("%s event: changing to state %s",
			motor_event_to_string(params->event.type),
			motor_state_to_string(params->event.target_mode));

		/* Validate target is an ONLINE substate */
		if (!motor_state_is_online_submode(params->event.target_mode)) {
			LOG_ERR("Invalid mode change target: %s",
				motor_state_to_string(params->event.target_mode));
			return SMF_EVENT_HANDLED;
		}

		smf_set_state(SMF_CTX(params), &motor_states[params->event.target_mode]);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

/* Substate: ONLINE_TORQUE - Direct Id/Iq control mode */
void motor_state_online_torque_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE_TORQUE substate (direct Id/Iq control)");

	/* Encoder-based control: add encoder read; ONLINE provides the baseline. */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));
	/* Start torque mode from a neutral current command for bumpless handover. */
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
}

void motor_state_online_torque_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_TORQUE substate");

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));
}

enum smf_state_result motor_state_online_torque_run(void *obj)
{
	ARG_UNUSED(obj);
	/* Direct torque control - no additional processing needed
	 * Id/Iq setpoints are controlled via shell commands
	 * Parent ONLINE state handles stop/error events
	 */
	return SMF_EVENT_PROPAGATE;
}

/* Substate: ONLINE_VELOCITY_OPEN - Open-loop velocity control */
void motor_state_online_velocity_open_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t mech_angle_rad = angle_observer_get_mech_angle(&params->observer);

	LOG_INF("Entering ONLINE_VELOCITY_OPEN substate");

	/* Open-loop velocity control uses angle generator and velocity trajectory.
	 * ONLINE provides the baseline.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));

	/* Initialize angle generator for open-loop mode */
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	/* Preserve commutation frame across mode transitions. */
	angle_gen_set_angle(&params->angle_gen, mech_angle_rad);

	/* Initialize velocity trajectory */
	traj_init(&params->traj_velocity);
	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;

	LOG_INF("Open-loop velocity mode initialized: max=%.1f Hz, accel=%.1f Hz/s",
		(double)(params->profile_max_velocity_rad_s / (2.0f * PI_F32)),
		(double)(params->profile_max_accel_rad_s2 / (2.0f * PI_F32)));
}

enum smf_state_result motor_state_online_velocity_open_run(void *obj)
{
	ARG_UNUSED(obj);
	/* Velocity control happens in motor_isr based on active substate
	 * This handler just maintains state and propagates events to parent
	 */
	return SMF_EVENT_PROPAGATE;
}

void motor_state_online_velocity_open_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_VELOCITY_OPEN substate");

	/* Reset angle generator and trajectory */
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
}

/* Substate: ONLINE_VELOCITY_CLOSED - Closed-loop velocity control */
void motor_state_online_velocity_closed_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t speed_mech_rad_s = angle_observer_get_mech_speed(&params->observer);

	LOG_INF("Entering ONLINE_VELOCITY_CLOSED substate");

	/* Closed-loop velocity uses measured speed and acceleration-limited velocity profile. */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
					      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));

	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, speed_mech_rad_s);
	traj_set_int_value(&params->traj_velocity, speed_mech_rad_s);
	params->velocity_target_rad_s = speed_mech_rad_s;
	params->velocity_ref_rad_s = speed_mech_rad_s;
	params->velocity_cl_i_term_A = 0.0f;
	filter_so_prime(&params->filter_velocity_notch, speed_mech_rad_s);
}

enum smf_state_result motor_state_online_velocity_closed_run(void *obj)
{
	ARG_UNUSED(obj);
	return SMF_EVENT_PROPAGATE;
}

void motor_state_online_velocity_closed_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_VELOCITY_CLOSED substate");

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

/* Substate: ONLINE_POSITION - Cascaded position->velocity->current scaffold */
void motor_state_online_position_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t speed_mech_rad_s = angle_observer_get_mech_speed(&params->observer);
	float32_t position_mech_rad = angle_observer_get_mech_angle(&params->observer);

	LOG_INF("Entering ONLINE_POSITION substate");

	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
					      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));

	/* Use current angle as initial target for bumpless mode entry. */
	params->position_target_rad = position_mech_rad;
	motion_profile_quintic_cancel(&params->position_profile, position_mech_rad);

	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, speed_mech_rad_s);
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = speed_mech_rad_s;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	filter_so_prime(&params->filter_velocity_notch, speed_mech_rad_s);
}

enum smf_state_result motor_state_online_position_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	switch (params->event.type) {
	case MOTOR_EVENT_PROFILE_SEQ_TICK: {
		if (!params->profile_sequence_running) {
			return SMF_EVENT_HANDLED;
		}
		if (params->profile_sequence_count == 0U) {
			params->profile_sequence_running = false;
			params->profile_sequence_tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}
		if (atomic_get(&params->control_armed) == 0) {
			return SMF_EVENT_HANDLED;
		}

		uint16_t idx = params->profile_sequence_next_idx;
		if (idx >= params->profile_sequence_count) {
			if (params->profile_sequence_loop) {
				idx = 0U;
			} else {
				params->profile_sequence_running = false;
				params->profile_sequence_tick_counter = 0U;
				return SMF_EVENT_HANDLED;
			}
		}

		int ret = motor_position_plan_sequence_move(params, params->profile_sequence_points_rad[idx]);
		if (ret != 0) {
			LOG_ERR("Profile sequence move %u failed (%d), stopping", idx, ret);
			params->profile_sequence_running = false;
			params->profile_sequence_tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}

		idx++;
		if (idx >= params->profile_sequence_count) {
			if (params->profile_sequence_loop) {
				idx = 0U;
			} else {
				params->profile_sequence_running = false;
				params->profile_sequence_tick_counter = 0U;
				LOG_INF("Profile sequence completed");
			}
		}
		params->profile_sequence_next_idx = idx;
		return SMF_EVENT_HANDLED;
	}

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

void motor_state_online_position_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_POSITION substate");

	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	params->velocity_target_rad_s = 0.0f;
	motion_profile_quintic_cancel(&params->position_profile,
				      angle_observer_get_mech_angle(&params->observer));
	params->velocity_ref_rad_s = 0.0f;
	params->profile_sequence_running = false;
	params->profile_sequence_tick_counter = 0U;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;

	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}
