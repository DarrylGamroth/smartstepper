#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor state prepare */
int cmd_motor_state_prepare_online(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_prepare_online() == 0) {
		shell_print(sh, "PREPARE_ONLINE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request PREPARE_ONLINE state");
		return -EIO;
	}
}

/* motor state idle */
int cmd_motor_state_idle(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_idle() == 0) {
		shell_print(sh, "IDLE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request IDLE state");
		return -EIO;
	}
}

/* motor state online */
int cmd_motor_state_online(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_online() == 0) {
		enum motor_state requested =
			(enum motor_state)g_motor_params->calibration.requested_online_mode;
		motor_transition_status_update(g_motor_params, MOTOR_EVENT_ONLINE,
					       requested,
					       (enum motor_state)motor_api_get_state(),
					       (enum motor_state)motor_api_get_state(),
					       (enum motor_state)motor_api_get_state(),
					       MOTOR_TRANSITION_RESULT_REQUESTED,
					       ERROR_NONE,
					       "online request posted; waiting for SMF");
		shell_print(sh, "ONLINE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request ONLINE state");
		return -EIO;
	}
}

/* motor state calibrate */
int cmd_motor_state_calibrate(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (motor_api_request_calibrate() == 0) {
		shell_print(sh, "Boot current-offset calibration started");
		return 0;
	} else {
		shell_error(sh, "Failed to start calibration");
		return -EIO;
	}
}

/* motor state commission */
int cmd_motor_state_commission(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (motor_api_request_commission() == 0) {
		shell_print(sh, "Commissioning sequence started");
		return 0;
	} else {
		shell_error(sh, "Failed to start commissioning");
		return -EIO;
	}
}

/* motor state clear_error */
int cmd_motor_state_clear_error(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (motor_api_clear_error() == 0) {
		shell_print(sh, "Error clear requested");
		return 0;
	} else {
		shell_error(sh, "Failed to clear error");
		return -EIO;
	}
}


/* motor state status */
int cmd_motor_state_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	int state = motor_api_get_state();
	const char *state_str = motor_state_to_string(state);
	int error = motor_api_get_error();
	const char *error_str = motor_error_to_string(error);
	uint32_t age_ms = motor_command_age_ms(g_motor_params);
	bool control_armed = motor_control_is_armed(g_motor_params);
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_GENERATED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER);
	bool autonomous_keepalive =
		motor_keepalive_policy_should_keepalive(control_armed, autonomous_mode_active,
						 g_motor_params->profile_seq.running,
						 g_motor_params->chopper_cal.active,
						 motion_profile_quintic_is_active(
							 &g_motor_params->position_profile));
	
	shell_print(sh, "Motor Status:");
	shell_print(sh, "  State: %s (%d)", state_str, state);
	shell_print(sh, "  Error: %s (%d)", error_str, error);
	if (error == ERROR_ENCODER_FAULT) {
		shell_print(sh, "  Enc reason: %s (%u)",
			    motor_encoder_fault_reason_to_string(
				    g_motor_params->fault_snapshot.latch_encoder_fault_reason),
			    g_motor_params->fault_snapshot.latch_encoder_fault_reason);
	}
	shell_print(sh, "  Armed: %s", motor_control_is_armed(g_motor_params) ? "YES" : "NO");
	shell_print(sh, "  Command timeout: %u ms", g_motor_params->command_timeout_ms);
	shell_print(sh, "  Command age: %u ms", age_ms);
	shell_print(sh, "  Timeout latch: %s", g_motor_params->command_timeout_latched ? "SET" : "CLEAR");
	shell_print(sh, "  Timeout count: %u", g_motor_params->command_timeout_count);
	shell_print(sh, "  Auto keepalive: %s", autonomous_keepalive ? "ACTIVE" : "INACTIVE");
	shell_print(sh, "  Cal complete: %s", g_motor_params->calibration.complete ? "YES" : "NO");
	shell_print(sh, "  Offsets valid:%s",
		    g_motor_params->calibration.current_offsets_valid ? " YES" : " NO");
	shell_print(sh, "  Cal running:  %s", g_motor_params->calibration.running ? "YES" : "NO");
	shell_print(sh, "  Cal mode:     %s",
		    motor_calibration_mode_to_string(g_motor_params->calibration.mode));
	shell_print(sh, "  Commissioned: %s",
		    g_motor_params->calibration.commissioning_complete ? "YES" : "NO");
		shell_print(sh, "  Enc mapped:   %s",
			    g_motor_params->calibration.encoder_mapping_complete ? "YES" : "NO");
		shell_print(sh, "  Active mode:  %s",
			    motor_state_ptr_is_online_control_state(g_motor_params->state_for_isr) ?
				    motor_state_to_string(state) :
				    "none");
		shell_print(sh, "  Requested:    %s",
			    motor_state_to_string(g_motor_params->calibration.requested_online_mode));
	shell_print(sh, "  Transition:   #%u %s requested=%s final=%s reason=%s",
		    g_motor_params->transition_status.request_sequence,
		    motor_transition_result_to_string(g_motor_params->transition_status.result),
		    motor_state_to_string(g_motor_params->transition_status.requested_state),
		    motor_state_to_string(g_motor_params->transition_status.final_state),
		    g_motor_params->transition_status.reason);
	shell_print(sh, "  Enc dir sign: %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	struct motor_control_policy policy = {0};
	if (motor_shell_derive_control_policy(g_motor_params, &policy) == 0) {
		motor_shell_print_control_policy(sh, &policy);
	} else {
		shell_print(sh, "Control Policy:");
		shell_print(sh, "  Policy valid:     NO");
	}
	if (g_motor_params->calibration.running || motor_state_is_align_phase(state)) {
		shell_print(sh, "  Align phase:  %s", state_str);
		shell_print(sh,
			    "  Align sample: samples=%u retries=%u mean=%.2f deg",
			    g_motor_params->calibration.align_sample_count,
			    g_motor_params->calibration.align_sample_retries,
			    (double)(g_motor_params->calibration.align_mech_angle_rad *
				     (180.0f / PI_F32)));
		shell_print(sh,
			    "  Align offset: %.2f deg",
			    (double)(g_motor_params->observer_alignment_offset_rad *
				     (180.0f / PI_F32)));
	}
	
	return 0;
}

/* motor state transition */
int cmd_motor_state_transition(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct motor_transition_status *st = &g_motor_params->transition_status;

	shell_print(sh, "Transition Status:");
	shell_print(sh, "  Sequence:     %u", st->request_sequence);
	shell_print(sh, "  Event:        %s", motor_event_to_string(st->request_event));
	shell_print(sh, "  Result:       %s", motor_transition_result_to_string(st->result));
	shell_print(sh, "  Requested:    %s", motor_state_to_string(st->requested_state));
	shell_print(sh, "  Source:       %s", motor_state_to_string(st->source_state));
	shell_print(sh, "  Final:        %s", motor_state_to_string(st->final_state));
	shell_print(sh, "  Fallback:     %s", motor_state_to_string(st->fallback_state));
	shell_print(sh, "  Error:        %s (%u)", motor_error_to_string(st->error_code),
		    st->error_code);
	shell_print(sh, "  Timestamp:    %u ms", st->timestamp_ms);
	shell_print(sh, "  Loop:         %u", st->loop_count);
	shell_print(sh, "  Reason:       %s", st->reason);
	return 0;
}

static void motor_shell_print_recovery_status(const struct shell *sh,
					      const struct motor_recovery_status *st)
{
	shell_print(sh, "Recovery Status:");
	shell_print(sh, "  Sequence:          %u", st->sequence);
	shell_print(sh, "  Fault latched:     %s", st->fault_latched ? "YES" : "NO");
	shell_print(sh, "  Last error:        %s (%u)",
		    motor_error_to_string(st->last_error_code),
		    st->last_error_code);
	shell_print(sh, "  Gate reset req:    %s", st->gate_reset_required ? "YES" : "NO");
	shell_print(sh, "  Gate reset done:   %s", st->gate_reset_done ? "YES" : "NO");
	shell_print(sh, "  Encoder rec req:   %s",
		    st->encoder_recovery_required ? "YES" : "NO");
	shell_print(sh, "  Encoder rec done:  %s",
		    st->encoder_recovery_done ? "YES" : "NO");
	shell_print(sh, "  Safe idle ready:   %s", st->safe_idle_ready ? "YES" : "NO");
	shell_print(sh, "  Clear command:     motor state clear_error");
	shell_print(sh, "  Gate recovery:     motor gate reset");
	shell_print(sh, "  Encoder recovery:  motor encoder recover");
}

int cmd_motor_state_recovery(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	motor_shell_print_recovery_status(sh, &g_motor_params->recovery_status);
	return 0;
}

int cmd_motor_fault_recovery(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_motor_state_recovery(sh, argc, argv);
}

/* motor state policy */
int cmd_motor_state_policy(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	int state = motor_api_get_state();
	struct motor_control_policy policy = {0};
	int ret = motor_shell_derive_control_policy(g_motor_params, &policy);
	if (ret != 0) {
		shell_error(sh, "Failed to derive control policy (err %d)", ret);
		return ret;
	}

	shell_print(sh, "State: %s (%d)", motor_state_to_string(state), state);
	motor_shell_print_control_policy(sh, &policy);
	return 0;
}

/* Helper function for mode changes */
static int motor_request_mode_change(const struct shell *sh, enum motor_state target_state, const char *mode_name)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	int current_state = motor_api_get_state();
	if (motor_encoder_control_mode_requires_encoder(target_state)) {
		char reason[96] = {0};

		if (!motor_encoder_control_ready_for_transition(g_motor_params,
								(enum motor_state)current_state,
								target_state, true,
								reason, sizeof(reason))) {
			shell_error(sh, "Cannot enter %s: %s", mode_name, reason);
			shell_error(sh,
				    "Run 'motor commission encoder run <current_a> <mech_hz> <cycles>' then 'motor commission encoder apply'");
			return -EACCES;
		}
	}

	bool online_active = (current_state == MOTOR_STATE_ONLINE) ||
			    motor_state_is_online_submode(current_state);
	if (!online_active) {
		g_motor_params->calibration.requested_online_mode = (uint8_t)target_state;
		motor_transition_status_update(g_motor_params, MOTOR_EVENT_MODE_CHANGE,
					       target_state, (enum motor_state)current_state,
					       (enum motor_state)current_state,
					       (enum motor_state)current_state,
					       MOTOR_TRANSITION_RESULT_REQUESTED,
					       ERROR_NONE,
					       "mode staged; run motor state online to enter");
		shell_print(sh,
			    "Online mode staged: %s (run 'motor state online' to enter)",
			    mode_name);
		return 0;
	}

	/* Post mode change event to state machine */
	struct motor_event evt = {
		.type = MOTOR_EVENT_MODE_CHANGE,
		.target_mode = target_state,
	};

	int ret = motor_api_post_event(&evt);
	if (ret != 0) {
		shell_error(sh, "Failed to post mode change event: queue full");
		return ret;
	}

	shell_print(sh, "Mode change to %s requested", mode_name);
	return 0;
}

/* motor state mode current_encoder */
int cmd_motor_state_mode_current_encoder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_CURRENT_ENCODER,
					 "current_encoder");
}

/* motor state mode velocity_generated */
int cmd_motor_state_mode_velocity_generated(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
					 "velocity_generated");
}

/* motor state mode position_generated */
int cmd_motor_state_mode_position_generated(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_POSITION_GENERATED,
					 "position_generated");
}

/* motor state mode velocity_encoder */
int cmd_motor_state_mode_velocity_encoder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					 "velocity_encoder");
}

/* motor state mode position_encoder */
int cmd_motor_state_mode_position_encoder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_POSITION_ENCODER,
					 "position_encoder");
}
