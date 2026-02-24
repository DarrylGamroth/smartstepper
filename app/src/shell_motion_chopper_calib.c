/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>
#include <drivers/timer_ic.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "shell_commands_motion.h"
#include "shell_commands_motion_common.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"
#include "config.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/traj.h"
#include "shell_parse.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

#if defined(CONFIG_TIMER_IC) && DT_HAS_COMPAT_STATUS_OKAY(st_stm32_timer_ic)
#define CHOPPER_CAL_CAPTURE_AVAILABLE 1
static const struct device *const chopper_capture_dev = DEVICE_DT_GET_ANY(st_stm32_timer_ic);
#else
#define CHOPPER_CAL_CAPTURE_AVAILABLE 0
#endif

static void motor_chopper_cal_reset_buffers(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	params->chopper_cal_complete = false;
	params->chopper_cal_valid = false;
	params->chopper_cal_midpoint_count = 0U;
	params->chopper_cal_total_edges_target = 0U;
	params->chopper_cal_total_edges_captured = 0U;
	params->chopper_cal_discarded_edges = 0U;
	params->chopper_cal_last_wrapped_rad = 0.0f;
	params->chopper_cal_last_unwrapped_rad = 0.0f;
	params->chopper_cal_start_unwrapped_rad = 0.0f;

	for (uint32_t i = 0U; i < CHOPPER_CAL_MAX_EDGES; i++) {
		params->chopper_cal_edge_sum_rad[i] = 0.0f;
		params->chopper_cal_edge_count[i] = 0U;
	}
	for (uint32_t i = 0U; i < CHOPPER_CAL_MAX_SLOTS; i++) {
		params->chopper_blade_midpoints_rad[i] = 0.0f;
	}
}

static inline void motor_chopper_cal_restore_timeout(struct motor_parameters *params)
{
	if (!params) {
		return;
	}
	params->command_timeout_ms = params->chopper_cal_saved_timeout_ms;
}

static int motor_chopper_cal_compute_midpoints(struct motor_parameters *params)
{
	if (!params) {
		return -ENODEV;
	}
	if (params->chopper_cal_slots == 0U || params->chopper_cal_slots > CHOPPER_CAL_MAX_SLOTS) {
		return -EINVAL;
	}

	uint16_t edge_bins = (uint16_t)(2U * params->chopper_cal_slots);
	for (uint16_t i = 0U; i < edge_bins; i++) {
		if (params->chopper_cal_edge_count[i] == 0U) {
			params->chopper_cal_valid = false;
			params->chopper_cal_midpoint_count = 0U;
			return -ENODATA;
		}
	}

	for (uint16_t i = 0U; i < params->chopper_cal_slots; i++) {
		uint16_t rise_idx = (uint16_t)(2U * i);
		uint16_t fall_idx = (uint16_t)(rise_idx + 1U);
		float32_t rise_mean =
			params->chopper_cal_edge_sum_rad[rise_idx] /
			(float32_t)params->chopper_cal_edge_count[rise_idx];
		float32_t fall_mean =
			params->chopper_cal_edge_sum_rad[fall_idx] /
			(float32_t)params->chopper_cal_edge_count[fall_idx];
		float32_t midpoint = rise_mean + 0.5f * (fall_mean - rise_mean);

		params->chopper_blade_midpoints_rad[i] = wrap_rad_2pi(midpoint);
	}

	params->chopper_cal_midpoint_count = params->chopper_cal_slots;
	params->chopper_cal_valid = true;
	return 0;
}

static void motor_chopper_release_sequence_capture_channel(struct motor_parameters *params)
{
	if (!params || params->profile_sequence_running ||
	    !params->profile_sequence_ext_capture_enabled ||
	    params->profile_sequence_trigger_channel != CHOPPER_CAL_CAPTURE_CHANNEL) {
		return;
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (device_is_ready(chopper_capture_dev)) {
		(void)timer_ic_disable_capture(chopper_capture_dev,
					       params->profile_sequence_trigger_channel);
	}
#endif
	params->profile_sequence_ext_capture_enabled = false;
	params->profile_sequence_ext_last_capture_valid = false;
}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
static void motor_chopper_capture_callback(const struct device *dev, uint32_t channel,
					   uint32_t cycles, int status, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cycles);
	ARG_UNUSED(user_data);

	struct motor_parameters *params = g_motor_params;
	if (!params || !params->chopper_cal_active) {
		return;
	}
	if (!motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_OPEN) ||
	    atomic_get(&params->control_armed) == 0) {
		params->chopper_cal_active = false;
		params->chopper_cal_complete = false;
		params->chopper_cal_valid = false;
		motor_chopper_cal_restore_timeout(params);
		return;
	}

	if (status < 0 || channel != CHOPPER_CAL_CAPTURE_CHANNEL) {
		params->chopper_cal_discarded_edges++;
		return;
	}

	uint16_t slots = params->chopper_cal_slots;
	uint16_t edge_bins = (uint16_t)(2U * slots);
	if (edge_bins == 0U || edge_bins > CHOPPER_CAL_MAX_EDGES) {
		params->chopper_cal_discarded_edges++;
		return;
	}

	uint32_t captured = params->chopper_cal_total_edges_captured;
	float32_t wrapped_rad = wrap_rad_2pi(params->position_rad);
	float32_t unwrapped_rad = wrapped_rad;

	if (captured == 0U) {
		params->chopper_cal_start_unwrapped_rad = wrapped_rad;
		params->chopper_cal_last_wrapped_rad = wrapped_rad;
		params->chopper_cal_last_unwrapped_rad = wrapped_rad;
	} else {
		float32_t delta_rad = wrap_rad_pi(wrapped_rad - params->chopper_cal_last_wrapped_rad);
		if (fabsf(delta_rad) < params->chopper_cal_edge_min_step_rad) {
			params->chopper_cal_discarded_edges++;
			return;
		}
		unwrapped_rad = params->chopper_cal_last_unwrapped_rad + delta_rad;
		params->chopper_cal_last_wrapped_rad = wrapped_rad;
		params->chopper_cal_last_unwrapped_rad = unwrapped_rad;
	}

	uint16_t bin = (uint16_t)(captured % edge_bins);
	params->chopper_cal_edge_sum_rad[bin] += unwrapped_rad;
	params->chopper_cal_edge_count[bin]++;
	params->chopper_cal_total_edges_captured = captured + 1U;

	if (params->chopper_cal_total_edges_captured >= params->chopper_cal_total_edges_target) {
		params->chopper_cal_active = false;
		params->chopper_cal_complete = true;
		params->chopper_cal_valid = false;
		motor_chopper_cal_restore_timeout(params);
	}
}
#endif

/* motor chopper calib clear */
int cmd_motor_chopper_calib_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
#endif

	g_motor_params->chopper_cal_active = false;
	g_motor_params->chopper_cal_slots = 0U;
	g_motor_params->chopper_cal_revs_target = 0U;
	g_motor_params->chopper_cal_samples_per_edge = 0U;
	g_motor_params->chopper_cal_speed_target_rad_s = 0.0f;
	motor_chopper_cal_restore_timeout(g_motor_params);
	motor_chopper_cal_reset_buffers(g_motor_params);

	int ret = motor_hardware_set_photo_interruptor_enable(false);
	if (ret < 0) {
		shell_warn(sh, "Failed to disable photo interrupter output (%d)", ret);
	}

	shell_print(sh, "Chopper calibration buffers cleared");
	return 0;
}

/* motor chopper calib start <slots> <revs> <speed_hz> */
int cmd_motor_chopper_calib_start(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "Usage: motor chopper calib start <slots> <revs> <speed_hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

#if !CHOPPER_CAL_CAPTURE_AVAILABLE
	shell_error(sh, "Timer capture driver unavailable for chopper calibration");
	return -ENOTSUP;
#else
	if (!device_is_ready(chopper_capture_dev)) {
		shell_error(sh, "Capture device not ready");
		return -ENODEV;
	}
	motor_chopper_release_sequence_capture_channel(g_motor_params);
	if (g_motor_params->profile_sequence_ext_capture_enabled &&
	    g_motor_params->profile_sequence_trigger_channel == CHOPPER_CAL_CAPTURE_CHANNEL) {
		shell_error(sh, "Sequence external trigger is using capture channel %u",
			    CHOPPER_CAL_CAPTURE_CHANNEL);
		return -EBUSY;
	}
#endif

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_OPEN)) {
		shell_error(sh, "Calibration requires ONLINE_VELOCITY_OPEN mode.");
		return -EACCES;
	}

	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' first.");
		return -EACCES;
	}

	uint32_t slots_u32 = 0U;
	if (!shell_parse_u32(argv[1], &slots_u32) || slots_u32 == 0U ||
	    slots_u32 > CHOPPER_CAL_MAX_SLOTS) {
		shell_error(sh, "slots must be 1..%u", CHOPPER_CAL_MAX_SLOTS);
		return -EINVAL;
	}

	uint32_t revs_u32 = 0U;
	if (!shell_parse_u32(argv[2], &revs_u32) || revs_u32 == 0U || revs_u32 > 10000U) {
		shell_error(sh, "revs must be 1..10000");
		return -EINVAL;
	}

	float speed_hz = 0.0f;
	if (!shell_parse_finite_float(argv[3], &speed_hz) || speed_hz <= 0.0f) {
		shell_error(sh, "speed_hz must be a positive finite value");
		return -EINVAL;
	}

	uint16_t slots = (uint16_t)slots_u32;
	uint16_t revs = (uint16_t)revs_u32;
	uint64_t edges_target_u64 = 2ULL * (uint64_t)slots * (uint64_t)revs;
	if (edges_target_u64 > UINT32_MAX) {
		shell_error(sh, "Edge target too large");
		return -ERANGE;
	}

	float32_t speed_target_rad_s = speed_hz * 2.0f * PI_F32;
	speed_target_rad_s = clampf(speed_target_rad_s, -g_motor_params->profile_max_velocity_rad_s,
				    g_motor_params->profile_max_velocity_rad_s);
	if (fabsf(speed_target_rad_s) < 1e-4f) {
		shell_error(sh, "speed_hz too small after clamping");
		return -EINVAL;
	}

	int ret = motor_hardware_set_photo_interruptor_enable(true);
	if (ret < 0) {
		shell_warn(sh, "Failed to enable photo interrupter output (%d)", ret);
	}

	/* Ensure no prior capture is running before reconfiguration. */
#if CHOPPER_CAL_CAPTURE_AVAILABLE
	(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
#endif

	if (g_motor_params->chopper_cal_active) {
		motor_chopper_cal_restore_timeout(g_motor_params);
	}

	g_motor_params->chopper_cal_active = false;
	g_motor_params->chopper_cal_slots = slots;
	g_motor_params->chopper_cal_revs_target = revs;
	g_motor_params->chopper_cal_samples_per_edge = revs;
	g_motor_params->chopper_cal_speed_target_rad_s = speed_target_rad_s;
	g_motor_params->chopper_cal_edge_min_step_rad = CHOPPER_CAL_MIN_STEP_DEG_DEFAULT * PI_F32 / 180.0f;
	g_motor_params->chopper_cal_saved_timeout_ms = g_motor_params->command_timeout_ms;
	g_motor_params->command_timeout_ms = 0U; /* Prevent disarm while calibration runs. */
	motor_chopper_cal_reset_buffers(g_motor_params);
	g_motor_params->chopper_cal_total_edges_target = (uint32_t)edges_target_u64;
	g_motor_params->chopper_cal_active = true;

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	ret = timer_ic_configure_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL,
					 TIMER_IC_CAPTURE_EDGE_BOTH |
						 TIMER_IC_CAPTURE_MODE_CONTINUOUS,
					 motor_chopper_capture_callback, NULL);
	if (ret < 0) {
		g_motor_params->chopper_cal_active = false;
		motor_chopper_cal_restore_timeout(g_motor_params);
		shell_error(sh, "Failed to configure capture channel %u (err %d)",
			    CHOPPER_CAL_CAPTURE_CHANNEL, ret);
		return ret;
	}

	ret = timer_ic_enable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
	if (ret < 0) {
		g_motor_params->chopper_cal_active = false;
		motor_chopper_cal_restore_timeout(g_motor_params);
		shell_error(sh, "Failed to enable capture channel %u (err %d)",
			    CHOPPER_CAL_CAPTURE_CHANNEL, ret);
		return ret;
	}
#endif

	traj_set_target_value(&g_motor_params->traj_velocity, speed_target_rad_s);
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh,
		    "Chopper calibration started: slots=%u revs=%u edges=%u speed=%.3f Hz (channel=%u)",
		    slots, revs, g_motor_params->chopper_cal_total_edges_target,
		    (double)(speed_target_rad_s / (2.0f * PI_F32)),
		    CHOPPER_CAL_CAPTURE_CHANNEL);
	return 0;
}

/* motor chopper calib stop */
int cmd_motor_chopper_calib_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
#endif

	g_motor_params->chopper_cal_active = false;
	motor_chopper_cal_restore_timeout(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);
	int ret = motor_hardware_set_photo_interruptor_enable(false);
	if (ret < 0) {
		shell_warn(sh, "Failed to disable photo interrupter output (%d)", ret);
	}

	if (g_motor_params->chopper_cal_complete && !g_motor_params->chopper_cal_valid) {
		ret = motor_chopper_cal_compute_midpoints(g_motor_params);
		if (ret < 0) {
			shell_warn(sh, "Capture stopped; midpoint computation failed (err %d)", ret);
		}
	}

	shell_print(sh, "Chopper calibration stopped");
	return 0;
}

/* motor chopper calib status */
int cmd_motor_chopper_calib_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->chopper_cal_complete && !g_motor_params->chopper_cal_valid) {
		int ret = motor_chopper_cal_compute_midpoints(g_motor_params);
		if (ret < 0) {
			shell_warn(sh, "Midpoint computation incomplete (err %d)", ret);
		}
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (!g_motor_params->chopper_cal_active && g_motor_params->chopper_cal_complete) {
		(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
	}
#endif

	shell_print(sh, "Chopper Calibration:");
	shell_print(sh, "  Active:         %s", g_motor_params->chopper_cal_active ? "YES" : "NO");
	shell_print(sh, "  Complete:       %s", g_motor_params->chopper_cal_complete ? "YES" : "NO");
	shell_print(sh, "  Valid:          %s", g_motor_params->chopper_cal_valid ? "YES" : "NO");
	shell_print(sh, "  Slots:          %u", g_motor_params->chopper_cal_slots);
	shell_print(sh, "  Revolutions:    %u", g_motor_params->chopper_cal_revs_target);
	shell_print(sh, "  Samples/edge:   %u", g_motor_params->chopper_cal_samples_per_edge);
	shell_print(sh, "  Speed target:   %.3f Hz",
		    (double)(g_motor_params->chopper_cal_speed_target_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Edges:          %u / %u",
		    g_motor_params->chopper_cal_total_edges_captured,
		    g_motor_params->chopper_cal_total_edges_target);
	shell_print(sh, "  Discarded:      %u", g_motor_params->chopper_cal_discarded_edges);
	shell_print(sh, "  Midpoints:      %u", g_motor_params->chopper_cal_midpoint_count);

	if (g_motor_params->chopper_cal_valid && g_motor_params->chopper_cal_midpoint_count > 0U) {
		for (uint16_t i = 0U; i < g_motor_params->chopper_cal_midpoint_count; i++) {
			shell_print(sh, "    [%u] %.3f deg", i,
				    (double)(g_motor_params->chopper_blade_midpoints_rad[i] *
					     180.0f / PI_F32));
		}
	}

	return 0;
}

/* motor chopper calib apply */
int cmd_motor_chopper_calib_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->chopper_cal_complete && !g_motor_params->chopper_cal_valid) {
		int ret = motor_chopper_cal_compute_midpoints(g_motor_params);
		if (ret < 0) {
			shell_error(sh, "Calibration data incomplete (err %d)", ret);
			return ret;
		}
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (!g_motor_params->chopper_cal_active && g_motor_params->chopper_cal_complete) {
		(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
	}
#endif

	if (!g_motor_params->chopper_cal_valid || g_motor_params->chopper_cal_midpoint_count == 0U) {
		shell_error(sh, "No valid midpoint table. Run 'motor chopper calib start ...' first.");
		return -EINVAL;
	}

	g_motor_params->profile_sequence_running = false;
	g_motor_params->profile_sequence_tick_counter = 0U;
	g_motor_params->profile_sequence_count = g_motor_params->chopper_cal_midpoint_count;
	g_motor_params->profile_sequence_next_idx = 0U;
	for (uint16_t i = 0U; i < g_motor_params->profile_sequence_count; i++) {
		g_motor_params->profile_sequence_points_rad[i] = g_motor_params->chopper_blade_midpoints_rad[i];
	}
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Applied %u midpoint targets into profile sequence",
		    g_motor_params->profile_sequence_count);
	return 0;
}
