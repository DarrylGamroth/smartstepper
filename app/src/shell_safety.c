#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor arm */
int cmd_motor_arm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	int state = motor_api_get_state();
	if (!motor_state_allows_arm(state)) {
		shell_error(sh, "Cannot arm while in state %s. Wait for IDLE/PREPARE_ONLINE/ONLINE.",
			    motor_state_to_string(state));
		return -EAGAIN;
	}

#if CONFIG_ENCODER_MAGNET_CHECK_ON_ARM
	{
		uint8_t status = 0U;
		bool mhi = false;
		bool mlo = false;
		int ret = motor_encoder_read_aeat_alarm(&status, &mhi, &mlo);
		if (ret == -ENOTSUP) {
			shell_error(sh, "CONFIG_ENCODER_MAGNET_CHECK_ON_ARM requires AEAT-9955 encoder1.");
			return ret;
		}
		if (ret < 0) {
			shell_error(sh, "Failed to read encoder magnet alarms (err %d)", ret);
			return ret;
		}
		if (mhi || mlo) {
			shell_error(sh,
				    "Cannot arm: encoder magnet alarm active (raw=0x%02X, MHI=%s, MLO=%s)",
				    status, mhi ? "SET" : "CLEAR", mlo ? "SET" : "CLEAR");
			return -EACCES;
		}
	}
#endif

	atomic_set(&g_motor_params->control_armed, 1);
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Control armed (state=%s)", motor_state_to_string(state));
	return 0;
}

/* motor disarm */
int cmd_motor_disarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	atomic_set(&g_motor_params->control_armed, 0);
	motor_zero_control_targets(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_api_request_idle();
	if (ret != 0) {
		shell_error(sh, "Disarmed, but failed to request IDLE (err %d)", ret);
		return ret;
	}

	shell_print(sh, "Control disarmed and IDLE requested");
	return 0;
}


/* motor safety timeout <ms> */
int cmd_motor_safety_timeout(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor safety timeout <ms>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t timeout_ms = 0U;
	if (!shell_parse_u32(argv[1], &timeout_ms)) {
		shell_error(sh, "Timeout must be a non-negative integer in milliseconds.");
		return -EINVAL;
	}

	int ret = motor_api_set_param("command_timeout_ms", (float)timeout_ms);
	if (ret != 0) {
		shell_error(sh, "Failed to set command timeout (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Command timeout set to %u ms%s",
		    timeout_ms, timeout_ms == 0 ? " (disabled)" : "");
	return 0;
}

/* motor safety pet */
int cmd_motor_safety_pet(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Command watchdog fed");
	return 0;
}

/* motor safety status */
int cmd_motor_safety_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t age_ms = motor_command_age_ms(g_motor_params);
	bool timeout_enabled = g_motor_params->command_timeout_ms > 0U;
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
	bool timeout_expired = timeout_enabled && !autonomous_keepalive &&
			       (age_ms > g_motor_params->command_timeout_ms);

	shell_print(sh, "Safety Status:");
	shell_print(sh, "  Armed:              %s",
		    motor_control_is_armed(g_motor_params) ? "YES" : "NO");
	shell_print(sh, "  Timeout enabled:    %s", timeout_enabled ? "YES" : "NO");
	shell_print(sh, "  Timeout value:      %u ms", g_motor_params->command_timeout_ms);
	shell_print(sh, "  Command age:        %u ms", age_ms);
	shell_print(sh, "  Timeout expired:    %s", timeout_expired ? "YES" : "NO");
	shell_print(sh, "  Auto keepalive:     %s", autonomous_keepalive ? "ACTIVE" : "INACTIVE");
	shell_print(sh, "  Timeout latch:      %s",
		    g_motor_params->command_timeout_latched ? "SET" : "CLEAR");
	shell_print(sh, "  Timeout count:      %u", g_motor_params->command_timeout_count);
#if CONFIG_ENCODER_MAGNET_CHECK_ON_ARM
	shell_print(sh, "  Magnet check arm:   ENABLED (Kconfig)");
#else
	shell_print(sh, "  Magnet check arm:   DISABLED (Kconfig)");
#endif

#if MOTOR_ENCODER_IS_AEAT9955
	uint8_t mag_status = 0U;
	bool mhi = false;
	bool mlo = false;
	int mag_ret = motor_encoder_read_aeat_alarm(&mag_status, &mhi, &mlo);
	if (mag_ret == 0) {
		shell_print(sh, "  Magnet raw status:  0x%02X", mag_status);
		shell_print(sh, "  Magnet MHI:         %s", mhi ? "SET" : "CLEAR");
		shell_print(sh, "  Magnet MLO:         %s", mlo ? "SET" : "CLEAR");
	} else {
		shell_print(sh, "  Magnet status err:  %d", mag_ret);
	}
#endif

	return 0;
}


