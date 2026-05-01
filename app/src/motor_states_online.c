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
#include "motor/filters/pi.h"
#include "motor/motion/traj.h"
#include "motor/motion/angle_gen.h"
#include "motor/observers/angle_observer.h"
#include "motor/math/angle_wrap.h"
#include "motor/control/dob.h"
#include "motor/motion/motion_planner.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"

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
	params->live.position_stale_count = 0U;
	params->live.position_stale_events = 0U;
	params->live.position_glitch_count = 0U;
	params->live.position_jitter_count = 0U;
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

	/* ONLINE modes require power-stage channels enabled.
	 * IDLE entry disables them, so re-enable on every ONLINE entry.
	 */
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
	motor_online_reset_feedback_quality(params);
	/* Start torque mode from a neutral current command for bumpless handover. */
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	params->live.velocity_target_rad_s = 0.0f;
	params->live.velocity_ref_rad_s = 0.0f;
	motor_mpr_velocity_reset(&params->velocity_mpr_state,
				 params->live.velocity_rad_s,
				 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state,
			params->live.velocity_rad_s);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;
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
	float32_t mech_angle_rad = params->live.position_rad;

	LOG_INF("Entering ONLINE_VELOCITY_OPEN substate");

	/* Open-loop velocity control uses angle generator and velocity trajectory.
	 * ONLINE provides the baseline.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));

	/* Initialize angle generator for open-loop mode */
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	/* Preserve commutation frame across mode transitions. */
	angle_gen_set_angle(&params->angle_gen, mech_angle_rad);

	/* Initialize velocity trajectory */
	motor_velocity_plan_init(&params->traj_velocity,
				 params->profile_max_velocity_rad_s,
				 params->profile_max_accel_rad_s2,
				 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
				 0.0f);
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	motor_mpr_velocity_reset(&params->velocity_mpr_state, 0.0f, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state, 0.0f);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;

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

/* Substate: ONLINE_PROFILE_OPEN - Open-loop generated-angle profile control */
void motor_state_online_profile_open_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t mech_angle_rad = params->live.position_rad;

	LOG_INF("Entering ONLINE_PROFILE_OPEN substate");

	/* Open-loop profile control drives generated mechanical position directly.
	 * Encoder reads remain disabled; capture telemetry can still request samples.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
						      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));

	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, mech_angle_rad);
	motion_profile_quintic_cancel(&params->position_profile, mech_angle_rad);
	params->position_target_rad = wrap_rad_2pi(mech_angle_rad);

	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	motor_mpr_velocity_reset(&params->velocity_mpr_state, 0.0f, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state, 0.0f);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;

	LOG_INF("Open-loop profile mode initialized at %.2f deg",
		(double)(mech_angle_rad * 180.0f / PI_F32));
}

enum smf_state_result motor_state_online_profile_open_run(void *obj)
{
	return motor_profile_sequence_run((struct motor_parameters *)obj);
}

void motor_state_online_profile_open_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t hold_rad = angle_gen_get_angle(&params->angle_gen);

	LOG_INF("Exiting ONLINE_PROFILE_OPEN substate");

	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	motion_profile_quintic_cancel(&params->position_profile, hold_rad);
	params->live.velocity_target_rad_s = 0.0f;
	params->live.velocity_ref_rad_s = 0.0f;
	params->profile_seq.running = false;
	params->profile_seq.tick_counter = 0U;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	motor_mpr_velocity_reset(&params->velocity_mpr_state, 0.0f, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state, 0.0f);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;

	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN));
}

/* Substate: ONLINE_VELOCITY_CLOSED - Closed-loop velocity control */
void motor_state_online_velocity_closed_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t speed_mech_rad_s = params->live.velocity_rad_s;

	LOG_INF("Entering ONLINE_VELOCITY_CLOSED substate");

	/* Closed-loop velocity uses measured speed and acceleration-limited velocity profile. */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
						     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
						      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
	motor_online_reset_feedback_quality(params);

	motor_velocity_plan_init(&params->traj_velocity,
				 params->profile_max_velocity_rad_s,
				 params->profile_max_accel_rad_s2,
				 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
				 0.0f);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	params->live.velocity_target_rad_s = 0.0f;
	params->live.velocity_ref_rad_s = 0.0f;
	params->live.Id_ref_A = params->Id_setpoint_A;
	params->live.Iq_ref_A = 0.0f;
	params->velocity_cl_i_term_A = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	filter_so_prime(&params->filter_velocity_notch, speed_mech_rad_s);
	motor_mpr_velocity_reset(&params->velocity_mpr_state, speed_mech_rad_s, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, speed_mech_rad_s);
	motor_dob_reset(&params->velocity_dob_state, speed_mech_rad_s);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;
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
	float32_t speed_mech_rad_s = params->live.velocity_rad_s;
	float32_t position_mech_rad = params->live.position_rad;

	LOG_INF("Entering ONLINE_POSITION substate");

	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
						     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
						      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
	motor_online_reset_feedback_quality(params);

	/* Use current angle as initial target for bumpless mode entry. */
	params->position_target_rad = position_mech_rad;
	motion_profile_quintic_cancel(&params->position_profile, position_mech_rad);

	motor_velocity_plan_init(&params->traj_velocity,
				 params->profile_max_velocity_rad_s,
				 params->profile_max_accel_rad_s2,
				 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
				 speed_mech_rad_s);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	params->live.velocity_target_rad_s = 0.0f;
	params->live.velocity_ref_rad_s = speed_mech_rad_s;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	filter_so_prime(&params->filter_velocity_notch, speed_mech_rad_s);
	motor_mpr_velocity_reset(&params->velocity_mpr_state, speed_mech_rad_s, params->live.Iq_ref_A);
	motor_mpr_position_reset(&params->position_mpr_state, speed_mech_rad_s);
	motor_dob_reset(&params->velocity_dob_state, speed_mech_rad_s);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;
}

enum smf_state_result motor_state_online_position_run(void *obj)
{
	return motor_profile_sequence_run((struct motor_parameters *)obj);
}

void motor_state_online_position_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_POSITION substate");

	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	params->live.velocity_target_rad_s = 0.0f;
	motion_profile_quintic_cancel(&params->position_profile,
				      params->live.position_rad);
	params->live.velocity_ref_rad_s = 0.0f;
	params->profile_seq.running = false;
	params->profile_seq.tick_counter = 0U;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	motor_mpr_velocity_reset(&params->velocity_mpr_state, 0.0f, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state, 0.0f);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;

	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}
