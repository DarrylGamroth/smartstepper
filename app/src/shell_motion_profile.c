/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>

#include "shell_commands_motion.h"
#include "motor_control_api.h"
#include "motor_state_utils.h"
#include "config.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/motor_motion_modules.h"
#include "shell_parse.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

int cmd_motor_profile_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor profile set <max_hz> <max_accel_hz_s>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float max_hz = 0.0f;
	float max_accel_hz_s = 0.0f;
	if (!shell_parse_finite_float(argv[1], &max_hz) ||
	    !shell_parse_finite_float(argv[2], &max_accel_hz_s)) {
		shell_error(sh, "max_hz/max_accel_hz_s must be finite numbers");
		return -EINVAL;
	}
	if (motor_api_set_param("profile_max_velocity_hz", max_hz) != 0 ||
	    motor_api_set_param("profile_max_accel_hz_s", max_accel_hz_s) != 0) {
		shell_error(sh, "Failed to update profile limits");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Profile limits set: vmax=%.2f Hz, amax=%.2f Hz/s",
		    (double)max_hz, (double)max_accel_hz_s);
	return 0;
}

/* motor profile move <target_deg> <end_vel_hz> <duration_ms> */
int cmd_motor_profile_move(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "Usage: motor profile move <target_deg> <end_vel_hz> <duration_ms>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		shell_error(sh, "Profile move requires ONLINE_POSITION mode.");
		return -EACCES;
	}

	float target_deg = 0.0f;
	float end_vel_hz = 0.0f;
	float duration_ms = 0.0f;
	if (!shell_parse_finite_float(argv[1], &target_deg) ||
	    !shell_parse_finite_float(argv[2], &end_vel_hz) ||
	    !shell_parse_finite_float(argv[3], &duration_ms)) {
		shell_error(sh, "target_deg/end_vel_hz/duration_ms must be finite numbers");
		return -EINVAL;
	}
	if (duration_ms <= 0.0f) {
		shell_error(sh, "duration_ms must be > 0");
		return -EINVAL;
	}

	float start_pos_rad = g_motor_params->position_rad;
	float start_vel_rad_s = g_motor_params->velocity_rad_s;
	float target_wrapped_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	float end_vel_rad_s = end_vel_hz * 2.0f * PI_F32;
	float duration_s = duration_ms * 0.001f;
	g_motor_params->profile_sequence_running = false;
	g_motor_params->profile_sequence_tick_counter = 0U;

	int ret = motor_position_move_plan_sequence_segment(&g_motor_params->position_profile,
							    start_pos_rad,
							    start_vel_rad_s,
							    target_wrapped_rad,
							    end_vel_rad_s,
							    duration_s,
							    g_motor_params->profile_max_velocity_rad_s,
							    g_motor_params->profile_max_accel_rad_s2);
	if (ret != 0) {
		shell_error(sh, "Failed to plan profile (err %d)", ret);
		return ret;
	}
	float end_pos_rad = g_motor_params->position_profile.end_position_rad;
	float peak_vel = 0.0f;
	float peak_acc = 0.0f;
	(void)motion_profile_quintic_check_limits(&g_motor_params->position_profile,
						  g_motor_params->profile_max_velocity_rad_s,
						  g_motor_params->profile_max_accel_rad_s2, 64U,
						  &peak_vel, &peak_acc);

	g_motor_params->position_target_rad = wrap_rad_2pi(start_pos_rad);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Profile move planned: %.2f deg -> %.2f deg in %.1f ms (vend=%.2f Hz)",
		    (double)(start_pos_rad * 180.0f / PI_F32),
		    (double)(wrap_rad_2pi(end_pos_rad) * 180.0f / PI_F32),
		    (double)duration_ms, (double)end_vel_hz);
	shell_print(sh, "  Peak estimate: %.2f Hz, %.2f Hz/s",
		    (double)(peak_vel / (2.0f * PI_F32)),
		    (double)(peak_acc / (2.0f * PI_F32)));
	return 0;
}

/* motor profile cancel */
int cmd_motor_profile_cancel(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float hold_pos_rad = g_motor_params->position_rad;
	g_motor_params->profile_sequence_running = false;
	g_motor_params->profile_sequence_tick_counter = 0U;
	motion_profile_quintic_cancel(&g_motor_params->position_profile, hold_pos_rad);
	g_motor_params->position_target_rad = wrap_rad_2pi(hold_pos_rad);
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Motion profile canceled at %.2f deg",
		    (double)(hold_pos_rad * 180.0f / PI_F32));
	return 0;
}

/* motor profile status */
int cmd_motor_profile_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Motion Profile:");
	shell_print(sh, "  Max velocity: %.2f Hz",
		    (double)(g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Max accel:    %.2f Hz/s",
		    (double)(g_motor_params->profile_max_accel_rad_s2 / (2.0f * PI_F32)));
	shell_print(sh, "  Vel target:   %.2f Hz",
		    (double)(g_motor_params->velocity_target_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Vel ref:      %.2f Hz",
		    (double)(g_motor_params->velocity_ref_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Quintic:      %s",
		    motion_profile_quintic_is_active(&g_motor_params->position_profile) ?
			    "ACTIVE" :
			    (g_motor_params->position_profile.valid ? "COMPLETE" : "IDLE"));
	shell_print(sh, "  Sequence:     %s (%u points, next=%u, drops=%u)",
		    g_motor_params->profile_sequence_running ? "RUNNING" : "STOPPED",
		    g_motor_params->profile_sequence_count,
		    g_motor_params->profile_sequence_next_idx,
		    g_motor_params->profile_sequence_event_drop_count);

	if (g_motor_params->position_profile.valid) {
		shell_print(sh, "  Segment t/T:  %.1f / %.1f ms",
			    (double)(g_motor_params->position_profile.t_s * 1000.0f),
			    (double)(g_motor_params->position_profile.duration_s * 1000.0f));
		shell_print(sh, "  Start->End:   %.2f -> %.2f deg",
			    (double)(wrap_rad_2pi(
					     g_motor_params->position_profile.start_position_rad) *
				     180.0f / PI_F32),
			    (double)(wrap_rad_2pi(g_motor_params->position_profile.end_position_rad) *
				     180.0f / PI_F32));
		shell_print(sh, "  Vend:         %.2f Hz",
			    (double)(g_motor_params->position_profile.end_velocity_rad_s /
				     (2.0f * PI_F32)));
		shell_print(sh, "  Ref pos/vel:  %.2f deg / %.2f Hz",
			    (double)(wrap_rad_2pi(
					     g_motor_params->position_profile.position_rad) *
				     180.0f / PI_F32),
			    (double)(g_motor_params->position_profile.velocity_rad_s /
				     (2.0f * PI_F32)));
	}
	return 0;
}
