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
#include <string.h>

#include "shell_commands_motion.h"
#include "shell_commands_motion_common.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "config.h"
#include "angle_wrap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

#if defined(CONFIG_TIMER_IC) && DT_HAS_COMPAT_STATUS_OKAY(st_stm32_timer_ic)
#define PROFILE_SEQ_CAPTURE_AVAILABLE 1
static const struct device *const profile_seq_capture_dev = DEVICE_DT_GET_ANY(st_stm32_timer_ic);
#else
#define PROFILE_SEQ_CAPTURE_AVAILABLE 0
#endif

static inline uint32_t motor_profile_period_ms_to_ticks(uint32_t period_ms)
{
	float32_t ticks_f = (CONTROL_LOOP_FREQUENCY_HZ * (float32_t)period_ms) / 1000.0f;
	uint32_t ticks = (uint32_t)(ticks_f + 0.5f);

	return (ticks == 0U) ? 1U : ticks;
}

static const char *motor_profile_seq_trigger_source_to_string(uint8_t source)
{
	switch (source) {
	case PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL:
		return "TIMER";
	case PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL:
		return "EXTERNAL";
	default:
		return "UNKNOWN";
	}
}

static const char *motor_profile_seq_trigger_edge_to_string(uint8_t edge)
{
	switch (edge) {
	case PROFILE_SEQUENCE_TRIGGER_EDGE_RISING:
		return "RISING";
	case PROFILE_SEQUENCE_TRIGGER_EDGE_FALLING:
		return "FALLING";
	case PROFILE_SEQUENCE_TRIGGER_EDGE_BOTH:
		return "BOTH";
	default:
		return "UNKNOWN";
	}
}

static int motor_profile_seq_post_tick_event(struct motor_parameters *params, bool external_trigger)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_PROFILE_SEQ_TICK,
	};
	extern struct k_msgq motor_event_queue;
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		if (params) {
			params->profile_sequence_event_drop_count++;
		}
		return ret;
	}

	if (params && external_trigger) {
		params->profile_sequence_ext_trigger_count++;
	}

	return 0;
}

#if PROFILE_SEQ_CAPTURE_AVAILABLE
static void motor_profile_seq_external_capture_callback(const struct device *dev, uint32_t channel,
							uint32_t cycles, int status, void *user_data);

static timer_ic_flags_t motor_profile_seq_ext_edge_to_capture_flags(uint8_t edge)
{
	switch (edge) {
	case PROFILE_SEQUENCE_TRIGGER_EDGE_FALLING:
		return TIMER_IC_CAPTURE_EDGE_FALLING;
	case PROFILE_SEQUENCE_TRIGGER_EDGE_BOTH:
		return TIMER_IC_CAPTURE_EDGE_BOTH;
	case PROFILE_SEQUENCE_TRIGGER_EDGE_RISING:
	default:
		return TIMER_IC_CAPTURE_EDGE_RISING;
	}
}
#endif

static int motor_profile_seq_update_ext_min_interval_cycles(struct motor_parameters *params)
{
	if (!params) {
		return -ENODEV;
	}

#if PROFILE_SEQ_CAPTURE_AVAILABLE
	if (!device_is_ready(profile_seq_capture_dev)) {
		params->profile_sequence_ext_min_interval_cycles = 0U;
		return -ENODEV;
	}

	uint64_t cycles_per_sec = 0U;
	int ret = timer_ic_get_cycles_per_sec(profile_seq_capture_dev,
					      params->profile_sequence_trigger_channel,
					      &cycles_per_sec);
	if (ret < 0) {
		params->profile_sequence_ext_min_interval_cycles = 0U;
		return ret;
	}

	uint64_t min_cycles = (cycles_per_sec * (uint64_t)params->profile_sequence_ext_min_interval_us +
			       999999ULL) /
			      1000000ULL;
	if (min_cycles > UINT32_MAX) {
		min_cycles = UINT32_MAX;
	}
	params->profile_sequence_ext_min_interval_cycles = (uint32_t)min_cycles;
	return 0;
#else
	params->profile_sequence_ext_min_interval_cycles = 0U;
	return -ENOTSUP;
#endif
}

static int motor_profile_seq_external_capture_disable(struct motor_parameters *params)
{
	if (!params) {
		return -ENODEV;
	}

#if PROFILE_SEQ_CAPTURE_AVAILABLE
	int ret = 0;
	if (params->profile_sequence_ext_capture_enabled && device_is_ready(profile_seq_capture_dev)) {
		ret = timer_ic_disable_capture(profile_seq_capture_dev,
					       params->profile_sequence_trigger_channel);
	}
	params->profile_sequence_ext_capture_enabled = false;
	params->profile_sequence_ext_last_capture_valid = false;
	return ret;
#else
	params->profile_sequence_ext_capture_enabled = false;
	params->profile_sequence_ext_last_capture_valid = false;
	return -ENOTSUP;
#endif
}

static int motor_profile_seq_external_capture_enable(struct motor_parameters *params)
{
	if (!params) {
		return -ENODEV;
	}

#if PROFILE_SEQ_CAPTURE_AVAILABLE
	if (!device_is_ready(profile_seq_capture_dev)) {
		return -ENODEV;
	}

	timer_ic_flags_t flags = motor_profile_seq_ext_edge_to_capture_flags(
		params->profile_sequence_trigger_edge) |
			       TIMER_IC_CAPTURE_MODE_CONTINUOUS;
	int ret = timer_ic_configure_capture(profile_seq_capture_dev,
					     params->profile_sequence_trigger_channel,
					     flags,
					     motor_profile_seq_external_capture_callback,
					     NULL);
	if (ret < 0) {
		params->profile_sequence_ext_capture_enabled = false;
		params->profile_sequence_ext_last_capture_valid = false;
		return ret;
	}

	ret = timer_ic_enable_capture(profile_seq_capture_dev,
				      params->profile_sequence_trigger_channel);
	if (ret < 0) {
		params->profile_sequence_ext_capture_enabled = false;
		params->profile_sequence_ext_last_capture_valid = false;
		return ret;
	}

	params->profile_sequence_ext_capture_enabled = true;
	params->profile_sequence_ext_last_capture_valid = false;
	return 0;
#else
	params->profile_sequence_ext_capture_enabled = false;
	params->profile_sequence_ext_last_capture_valid = false;
	return -ENOTSUP;
#endif
}

#if PROFILE_SEQ_CAPTURE_AVAILABLE
static void motor_profile_seq_external_capture_callback(const struct device *dev, uint32_t channel,
							uint32_t cycles, int status, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	struct motor_parameters *params = g_motor_params;
	if (!params || !params->profile_sequence_ext_capture_enabled ||
	    params->profile_sequence_trigger_source != PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		return;
	}

	if (status < 0 || channel != params->profile_sequence_trigger_channel) {
		params->profile_sequence_ext_reject_count++;
		return;
	}

	if (!params->profile_sequence_running ||
	    atomic_get(&params->control_armed) == 0 ||
	    !motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		return;
	}

	if (params->profile_sequence_ext_min_interval_cycles > 0U &&
	    params->profile_sequence_ext_last_capture_valid) {
		uint32_t delta_cycles = cycles - params->profile_sequence_ext_last_capture_cycles;
		if (delta_cycles < params->profile_sequence_ext_min_interval_cycles) {
			params->profile_sequence_ext_reject_count++;
			return;
		}
	}

	params->profile_sequence_ext_last_capture_valid = true;
	params->profile_sequence_ext_last_capture_cycles = cycles;
	(void)motor_profile_seq_post_tick_event(params, true);
}
#endif

/* motor profile seq clear */
int cmd_motor_profile_seq_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->profile_sequence_running = false;
	(void)motor_profile_seq_external_capture_disable(g_motor_params);
	g_motor_params->profile_sequence_count = 0U;
	g_motor_params->profile_sequence_next_idx = 0U;
	g_motor_params->profile_sequence_tick_counter = 0U;
	g_motor_params->profile_sequence_event_drop_count = 0U;
	g_motor_params->profile_sequence_ext_trigger_count = 0U;
	g_motor_params->profile_sequence_ext_reject_count = 0U;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Profile sequence cleared");
	return 0;
}

/* motor profile seq add <target_deg> */
int cmd_motor_profile_seq_add(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq add <target_deg>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->profile_sequence_count >= MOTOR_PROFILE_SEQUENCE_MAX_POINTS) {
		shell_error(sh, "Sequence full (%u points max)", MOTOR_PROFILE_SEQUENCE_MAX_POINTS);
		return -ENOMEM;
	}

	float target_deg = strtof(argv[1], NULL);
	if (!isfinite(target_deg)) {
		shell_error(sh, "Invalid target_deg");
		return -EINVAL;
	}

	float target_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	uint16_t idx = g_motor_params->profile_sequence_count;
	g_motor_params->profile_sequence_points_rad[idx] = target_rad;
	g_motor_params->profile_sequence_count++;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Added seq[%u] = %.2f deg", (unsigned int)idx,
		    (double)(target_rad * 180.0f / PI_F32));
	return 0;
}

/* motor profile seq config <period_ms> <move_ms> <end_vel_hz> <loop:0|1> */
int cmd_motor_profile_seq_config(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 5) {
		shell_error(sh, "Usage: motor profile seq config <period_ms> <move_ms> <end_vel_hz> <loop:0|1>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	char *endp = NULL;
	unsigned long period_ms_ul = strtoul(argv[1], &endp, 10);
	if (endp == argv[1] || *endp != '\0' || period_ms_ul == 0UL || period_ms_ul > UINT32_MAX) {
		shell_error(sh, "period_ms must be a positive integer");
		return -EINVAL;
	}

	endp = NULL;
	unsigned long move_ms_ul = strtoul(argv[2], &endp, 10);
	if (endp == argv[2] || *endp != '\0' || move_ms_ul == 0UL || move_ms_ul > UINT32_MAX) {
		shell_error(sh, "move_ms must be a positive integer");
		return -EINVAL;
	}

	float end_vel_hz = strtof(argv[3], &endp);
	if (endp == argv[3] || *endp != '\0' || !isfinite(end_vel_hz)) {
		shell_error(sh, "end_vel_hz must be a finite number");
		return -EINVAL;
	}

	endp = NULL;
	unsigned long loop_ul = strtoul(argv[4], &endp, 10);
	if (endp == argv[4] || *endp != '\0' || loop_ul > 1UL) {
		shell_error(sh, "loop must be 0 or 1");
		return -EINVAL;
	}

	uint32_t period_ms = (uint32_t)period_ms_ul;
	uint32_t move_ms = (uint32_t)move_ms_ul;
	g_motor_params->profile_sequence_period_ms = period_ms;
	g_motor_params->profile_sequence_period_ticks = motor_profile_period_ms_to_ticks(period_ms);
	g_motor_params->profile_sequence_move_duration_s = (float32_t)move_ms * 0.001f;
	g_motor_params->profile_sequence_end_velocity_rad_s = end_vel_hz * 2.0f * PI_F32;
	g_motor_params->profile_sequence_loop = (loop_ul != 0UL);
	g_motor_params->profile_sequence_tick_counter = 0U;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Sequence config: period=%u ms (%u ticks), move=%u ms, vend=%.2f Hz, loop=%s",
		    period_ms, g_motor_params->profile_sequence_period_ticks, move_ms,
		    (double)end_vel_hz, g_motor_params->profile_sequence_loop ? "YES" : "NO");
	return 0;
}

/* motor profile seq trigger source <timer|external> */
int cmd_motor_profile_seq_trigger_source(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq trigger source <timer|external>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->profile_sequence_running) {
		shell_error(sh, "Stop sequence before changing trigger source.");
		return -EBUSY;
	}

	uint8_t source;
	if (strcmp(argv[1], "timer") == 0 || strcmp(argv[1], "internal") == 0) {
		source = PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL;
	} else if (strcmp(argv[1], "external") == 0) {
		source = PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL;
	} else {
		shell_error(sh, "Source must be 'timer' or 'external'");
		return -EINVAL;
	}

	g_motor_params->profile_sequence_trigger_source = source;
	(void)motor_profile_seq_external_capture_disable(g_motor_params);
	if (source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
		if (ret < 0) {
			shell_warn(sh, "External trigger timing unavailable until capture device is ready (err %d)",
				   ret);
		}
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence trigger source set to %s",
		    motor_profile_seq_trigger_source_to_string(source));
	return 0;
}

/* motor profile seq trigger edge <rising|falling|both> */
int cmd_motor_profile_seq_trigger_edge(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq trigger edge <rising|falling|both>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->profile_sequence_running) {
		shell_error(sh, "Stop sequence before changing trigger edge.");
		return -EBUSY;
	}

	uint8_t edge;
	if (strcmp(argv[1], "rising") == 0) {
		edge = PROFILE_SEQUENCE_TRIGGER_EDGE_RISING;
	} else if (strcmp(argv[1], "falling") == 0) {
		edge = PROFILE_SEQUENCE_TRIGGER_EDGE_FALLING;
	} else if (strcmp(argv[1], "both") == 0) {
		edge = PROFILE_SEQUENCE_TRIGGER_EDGE_BOTH;
	} else {
		shell_error(sh, "Edge must be 'rising', 'falling', or 'both'");
		return -EINVAL;
	}

	g_motor_params->profile_sequence_trigger_edge = edge;
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence trigger edge set to %s",
		    motor_profile_seq_trigger_edge_to_string(edge));
	return 0;
}

/* motor profile seq trigger channel <index> */
int cmd_motor_profile_seq_trigger_channel(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq trigger channel <0..3>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->profile_sequence_running) {
		shell_error(sh, "Stop sequence before changing trigger channel.");
		return -EBUSY;
	}

	char *endp = NULL;
	unsigned long channel_ul = strtoul(argv[1], &endp, 10);
	if (endp == argv[1] || *endp != '\0' || channel_ul > 3UL) {
		shell_error(sh, "channel must be 0..3");
		return -EINVAL;
	}

	g_motor_params->profile_sequence_trigger_channel = (uint8_t)channel_ul;
	g_motor_params->profile_sequence_ext_last_capture_valid = false;

	int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
	if (ret < 0 &&
	    g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_warn(sh, "Channel timing unavailable until capture device is ready (err %d)", ret);
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence trigger channel set to %u",
		    g_motor_params->profile_sequence_trigger_channel);
	return 0;
}

/* motor profile seq trigger min_interval_us <us> */
int cmd_motor_profile_seq_trigger_min_interval(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq trigger min_interval_us <us>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	char *endp = NULL;
	unsigned long interval_ul = strtoul(argv[1], &endp, 10);
	if (endp == argv[1] || *endp != '\0' || interval_ul > UINT32_MAX) {
		shell_error(sh, "min_interval_us must be an integer in [0, 4294967295]");
		return -EINVAL;
	}

	g_motor_params->profile_sequence_ext_min_interval_us = (uint32_t)interval_ul;
	g_motor_params->profile_sequence_ext_last_capture_valid = false;

	int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
	if (ret < 0 &&
	    g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_warn(sh, "Capture timing conversion unavailable (err %d)", ret);
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence trigger min interval set to %u us (%u cycles)",
		    g_motor_params->profile_sequence_ext_min_interval_us,
		    g_motor_params->profile_sequence_ext_min_interval_cycles);
	return 0;
}

/* motor profile seq trigger fire */
int cmd_motor_profile_seq_trigger_fire(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!g_motor_params->profile_sequence_running) {
		shell_error(sh, "Sequence is not running.");
		return -EACCES;
	}

	if (g_motor_params->profile_sequence_trigger_source != PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_error(sh, "Trigger fire is only valid when source is 'external'.");
		return -EACCES;
	}

	int ret = motor_profile_seq_post_tick_event(g_motor_params, true);
	if (ret != 0) {
		shell_error(sh, "Failed to inject trigger (queue full)");
		return ret;
	}

	shell_print(sh, "Injected external sequence trigger");
	return 0;
}

/* motor profile seq trigger status */
int cmd_motor_profile_seq_trigger_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Sequence Trigger:");
	shell_print(sh, "  Source:       %s",
		    motor_profile_seq_trigger_source_to_string(
			    g_motor_params->profile_sequence_trigger_source));
	shell_print(sh, "  Edge:         %s",
		    motor_profile_seq_trigger_edge_to_string(
			    g_motor_params->profile_sequence_trigger_edge));
	shell_print(sh, "  Channel:      %u", g_motor_params->profile_sequence_trigger_channel);
	shell_print(sh, "  Capture:      %s",
		    g_motor_params->profile_sequence_ext_capture_enabled ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Min interval: %u us (%u cycles)",
		    g_motor_params->profile_sequence_ext_min_interval_us,
		    g_motor_params->profile_sequence_ext_min_interval_cycles);
	shell_print(sh, "  Accepted:     %u", g_motor_params->profile_sequence_ext_trigger_count);
	shell_print(sh, "  Rejected:     %u", g_motor_params->profile_sequence_ext_reject_count);
	return 0;
}

/* motor profile seq start */
int cmd_motor_profile_seq_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		shell_error(sh, "Sequence start requires ONLINE_POSITION mode.");
		return -EACCES;
	}

	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before sequence start.");
		return -EACCES;
	}

	if (g_motor_params->profile_sequence_count == 0U) {
		shell_error(sh, "Sequence is empty. Add points with 'motor profile seq add <deg>'.");
		return -EINVAL;
	}

#if !PROFILE_SEQ_CAPTURE_AVAILABLE
	if (g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_error(sh, "External trigger source unavailable (capture driver not enabled).");
		return -ENOTSUP;
	}
#else
	if (g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		if (!device_is_ready(profile_seq_capture_dev)) {
			shell_error(sh, "Capture device not ready");
			return -ENODEV;
		}
		if (g_motor_params->chopper_cal_active &&
		    g_motor_params->profile_sequence_trigger_channel == CHOPPER_CAL_CAPTURE_CHANNEL) {
			shell_error(sh, "Chopper calibration is using capture channel %u",
				    CHOPPER_CAL_CAPTURE_CHANNEL);
			return -EBUSY;
		}
	}
#endif

	g_motor_params->profile_sequence_running = true;
	g_motor_params->profile_sequence_next_idx = 0U;
	g_motor_params->profile_sequence_tick_counter = 0U;
	g_motor_params->profile_sequence_event_drop_count = 0U;
	g_motor_params->profile_sequence_ext_trigger_count = 0U;
	g_motor_params->profile_sequence_ext_reject_count = 0U;
	g_motor_params->profile_sequence_ext_last_capture_valid = false;
	(void)motor_profile_seq_external_capture_disable(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);

	if (g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
		if (ret < 0) {
			g_motor_params->profile_sequence_running = false;
			shell_error(sh, "Failed to derive external trigger timing (err %d)", ret);
			return ret;
		}

		ret = motor_profile_seq_external_capture_enable(g_motor_params);
		if (ret < 0) {
			g_motor_params->profile_sequence_running = false;
			shell_error(sh, "Failed to enable external trigger capture (err %d)", ret);
			return ret;
		}

		shell_print(sh,
			    "Profile sequence started (%u points), source=EXTERNAL edge=%s ch=%u",
			    g_motor_params->profile_sequence_count,
			    motor_profile_seq_trigger_edge_to_string(
				    g_motor_params->profile_sequence_trigger_edge),
			    g_motor_params->profile_sequence_trigger_channel);
	} else {
		int ret = motor_profile_seq_post_tick_event(g_motor_params, false);
		if (ret != 0) {
			shell_warn(sh, "Sequence started, initial tick dropped (queue full)");
		}

		if (g_motor_params->command_timeout_ms > 0U &&
		    g_motor_params->profile_sequence_period_ms >= g_motor_params->command_timeout_ms) {
			shell_warn(sh, "period_ms >= command_timeout_ms; increase timeout to avoid disarm");
		}

		shell_print(sh, "Profile sequence started (%u points), source=TIMER",
			    g_motor_params->profile_sequence_count);
	}
	return 0;
}

/* motor profile seq stop */
int cmd_motor_profile_seq_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->profile_sequence_running = false;
	g_motor_params->profile_sequence_tick_counter = 0U;
	(void)motor_profile_seq_external_capture_disable(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Profile sequence stopped");
	return 0;
}

/* motor profile seq status */
int cmd_motor_profile_seq_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!g_motor_params->profile_sequence_running &&
	    g_motor_params->profile_sequence_ext_capture_enabled) {
		(void)motor_profile_seq_external_capture_disable(g_motor_params);
	}

	shell_print(sh, "Profile Sequence:");
	shell_print(sh, "  Running:      %s", g_motor_params->profile_sequence_running ? "YES" : "NO");
	shell_print(sh, "  Mode:         %s",
		    motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION) ?
			    "ONLINE_POSITION" :
			    motor_state_to_string(motor_api_get_state()));
	shell_print(sh, "  Armed:        %s", motor_control_is_armed(g_motor_params) ? "YES" : "NO");
	shell_print(sh, "  Loop:         %s", g_motor_params->profile_sequence_loop ? "YES" : "NO");
	shell_print(sh, "  Count:        %u / %u", g_motor_params->profile_sequence_count,
		    MOTOR_PROFILE_SEQUENCE_MAX_POINTS);
	shell_print(sh, "  Next index:   %u", g_motor_params->profile_sequence_next_idx);
	shell_print(sh, "  Trigger src:  %s",
		    motor_profile_seq_trigger_source_to_string(
			    g_motor_params->profile_sequence_trigger_source));
	shell_print(sh, "  Trigger edge: %s",
		    motor_profile_seq_trigger_edge_to_string(
			    g_motor_params->profile_sequence_trigger_edge));
	shell_print(sh, "  Trigger ch:   %u", g_motor_params->profile_sequence_trigger_channel);
	shell_print(sh, "  Capture:      %s",
		    g_motor_params->profile_sequence_ext_capture_enabled ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Min trig dt:  %u us (%u cycles)",
		    g_motor_params->profile_sequence_ext_min_interval_us,
		    g_motor_params->profile_sequence_ext_min_interval_cycles);
	shell_print(sh, "  Period:       %u ms (%u ticks)", g_motor_params->profile_sequence_period_ms,
		    g_motor_params->profile_sequence_period_ticks);
	shell_print(sh, "  Move:         %.1f ms",
		    (double)(g_motor_params->profile_sequence_move_duration_s * 1000.0f));
	shell_print(sh, "  End vel:      %.2f Hz",
		    (double)(g_motor_params->profile_sequence_end_velocity_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Dropped ticks:%u", g_motor_params->profile_sequence_event_drop_count);
	shell_print(sh, "  Ext accepted: %u", g_motor_params->profile_sequence_ext_trigger_count);
	shell_print(sh, "  Ext rejected: %u", g_motor_params->profile_sequence_ext_reject_count);

	if (g_motor_params->profile_sequence_count > 0U) {
		uint16_t idx = g_motor_params->profile_sequence_next_idx;
		if (idx >= g_motor_params->profile_sequence_count) {
			idx = 0U;
		}
		shell_print(sh, "  Next target:  %.2f deg",
			    (double)(g_motor_params->profile_sequence_points_rad[idx] *
				     180.0f / PI_F32));
	}

	return 0;
}

/* motor profile seq list */
int cmd_motor_profile_seq_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (g_motor_params->profile_sequence_count == 0U) {
		shell_print(sh, "Sequence is empty");
		return 0;
	}

	shell_print(sh, "Sequence Points (%u):", g_motor_params->profile_sequence_count);
	for (uint16_t i = 0; i < g_motor_params->profile_sequence_count; i++) {
		shell_print(sh, "  [%u] %.2f deg", (unsigned int)i,
			    (double)(g_motor_params->profile_sequence_points_rad[i] *
				     180.0f / PI_F32));
	}

	return 0;
}
