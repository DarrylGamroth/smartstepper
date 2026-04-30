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
#include "motor/math/angle_wrap.h"
#include "shell_parse.h"

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

static inline bool motor_profile_seq_mode_active(const struct motor_parameters *params)
{
	return params != NULL &&
	       (motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION) ||
		motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_PROFILE_OPEN));
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

static bool motor_profile_seq_parse_trigger_source(const char *text, uint8_t *source_out)
{
	if (text == NULL || source_out == NULL) {
		return false;
	}

	if (strcmp(text, "timer") == 0 || strcmp(text, "internal") == 0) {
		*source_out = PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL;
		return true;
	}
	if (strcmp(text, "external") == 0) {
		*source_out = PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL;
		return true;
	}

	return false;
}

static int motor_profile_seq_set_period_ms(struct motor_parameters *params, uint32_t period_ms)
{
	if (!params) {
		return -ENODEV;
	}
	if (period_ms == 0U) {
		return -EINVAL;
	}

	params->profile_seq.period_ms = period_ms;
	params->profile_seq.period_ticks = motor_profile_period_ms_to_ticks(period_ms);
	params->profile_seq.tick_counter = 0U;
	return 0;
}

static int motor_profile_seq_set_move_ms(struct motor_parameters *params, uint32_t move_ms)
{
	if (!params) {
		return -ENODEV;
	}
	if (move_ms == 0U) {
		return -EINVAL;
	}

	params->profile_seq.move_duration_s = (float32_t)move_ms * 0.001f;
	return 0;
}

static int motor_profile_seq_set_end_vel_hz(struct motor_parameters *params, float end_vel_hz)
{
	if (!params || !isfinite(end_vel_hz)) {
		return -EINVAL;
	}

	params->profile_seq.end_velocity_rad_s = end_vel_hz * 2.0f * PI_F32;
	return 0;
}

static int motor_profile_seq_set_loop(struct motor_parameters *params, bool loop_enabled)
{
	if (!params) {
		return -ENODEV;
	}

	params->profile_seq.loop = loop_enabled;
	return 0;
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
			params->profile_seq.event_drop_count++;
		}
		return ret;
	}

	if (params && external_trigger) {
		params->profile_seq.ext_trigger_count++;
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
		params->profile_seq.ext_min_interval_cycles = 0U;
		return -ENODEV;
	}

	uint64_t cycles_per_sec = 0U;
	int ret = timer_ic_get_cycles_per_sec(profile_seq_capture_dev,
					      params->profile_seq.trigger_channel,
					      &cycles_per_sec);
	if (ret < 0) {
		params->profile_seq.ext_min_interval_cycles = 0U;
		return ret;
	}

	uint64_t min_cycles = (cycles_per_sec * (uint64_t)params->profile_seq.ext_min_interval_us +
			       999999ULL) /
			      1000000ULL;
	if (min_cycles > UINT32_MAX) {
		min_cycles = UINT32_MAX;
	}
	params->profile_seq.ext_min_interval_cycles = (uint32_t)min_cycles;
	return 0;
#else
	params->profile_seq.ext_min_interval_cycles = 0U;
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
	if (params->profile_seq.ext_capture_enabled && device_is_ready(profile_seq_capture_dev)) {
		ret = timer_ic_disable_capture(profile_seq_capture_dev,
					       params->profile_seq.trigger_channel);
	}
	params->profile_seq.ext_capture_enabled = false;
	params->profile_seq.ext_last_capture_valid = false;
	return ret;
#else
	params->profile_seq.ext_capture_enabled = false;
	params->profile_seq.ext_last_capture_valid = false;
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
		params->profile_seq.trigger_edge) |
			       TIMER_IC_CAPTURE_MODE_CONTINUOUS;
	int ret = timer_ic_configure_capture(profile_seq_capture_dev,
					     params->profile_seq.trigger_channel,
					     flags,
					     motor_profile_seq_external_capture_callback,
					     NULL);
	if (ret < 0) {
		params->profile_seq.ext_capture_enabled = false;
		params->profile_seq.ext_last_capture_valid = false;
		return ret;
	}

	ret = timer_ic_enable_capture(profile_seq_capture_dev,
				      params->profile_seq.trigger_channel);
	if (ret < 0) {
		params->profile_seq.ext_capture_enabled = false;
		params->profile_seq.ext_last_capture_valid = false;
		return ret;
	}

	params->profile_seq.ext_capture_enabled = true;
	params->profile_seq.ext_last_capture_valid = false;
	return 0;
#else
	params->profile_seq.ext_capture_enabled = false;
	params->profile_seq.ext_last_capture_valid = false;
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
	if (!params || !params->profile_seq.ext_capture_enabled ||
	    params->profile_seq.trigger_source != PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		return;
	}

	if (status < 0 || channel != params->profile_seq.trigger_channel) {
		params->profile_seq.ext_reject_count++;
		return;
	}

	if (!params->profile_seq.running ||
	    atomic_get(&params->control_armed) == 0 ||
	    !motor_profile_seq_mode_active(params)) {
		return;
	}

	if (params->profile_seq.ext_min_interval_cycles > 0U &&
	    params->profile_seq.ext_last_capture_valid) {
		uint32_t delta_cycles = cycles - params->profile_seq.ext_last_capture_cycles;
		if (delta_cycles < params->profile_seq.ext_min_interval_cycles) {
			params->profile_seq.ext_reject_count++;
			return;
		}
	}

	params->profile_seq.ext_last_capture_valid = true;
	params->profile_seq.ext_last_capture_cycles = cycles;
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

	g_motor_params->profile_seq.running = false;
	(void)motor_profile_seq_external_capture_disable(g_motor_params);
	g_motor_params->profile_seq.count = 0U;
	g_motor_params->profile_seq.next_idx = 0U;
	g_motor_params->profile_seq.tick_counter = 0U;
	g_motor_params->profile_seq.event_drop_count = 0U;
	g_motor_params->profile_seq.ext_trigger_count = 0U;
	g_motor_params->profile_seq.ext_reject_count = 0U;
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

	if (g_motor_params->profile_seq.count >= MOTOR_PROFILE_SEQUENCE_MAX_POINTS) {
		shell_error(sh, "Sequence full (%u points max)", MOTOR_PROFILE_SEQUENCE_MAX_POINTS);
		return -ENOMEM;
	}

	float target_deg = 0.0f;
	if (!shell_parse_finite_float(argv[1], &target_deg)) {
		shell_error(sh, "target_deg must be a finite number");
		return -EINVAL;
	}

	float target_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	uint16_t idx = g_motor_params->profile_seq.count;
	g_motor_params->profile_seq.points_rad[idx] = target_rad;
	g_motor_params->profile_seq.count++;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Added seq[%u] = %.2f deg", (unsigned int)idx,
		    (double)(target_rad * 180.0f / PI_F32));
	return 0;
}

/* motor profile seq period_ms <ms> */
int cmd_motor_profile_seq_period_ms(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq period_ms <ms>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t period_ms = 0U;
	if (!shell_parse_u32(argv[1], &period_ms) || period_ms == 0U) {
		shell_error(sh, "period_ms must be a positive integer");
		return -EINVAL;
	}

	int ret = motor_profile_seq_set_period_ms(g_motor_params, period_ms);
	if (ret != 0) {
		shell_error(sh, "Failed to set period_ms (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence period set to %u ms (%u ticks)",
		    g_motor_params->profile_seq.period_ms,
		    g_motor_params->profile_seq.period_ticks);
	return 0;
}

/* motor profile seq move_ms <ms> */
int cmd_motor_profile_seq_move_ms(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq move_ms <ms>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t move_ms = 0U;
	if (!shell_parse_u32(argv[1], &move_ms) || move_ms == 0U) {
		shell_error(sh, "move_ms must be a positive integer");
		return -EINVAL;
	}

	int ret = motor_profile_seq_set_move_ms(g_motor_params, move_ms);
	if (ret != 0) {
		shell_error(sh, "Failed to set move_ms (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence move duration set to %.1f ms",
		    (double)(g_motor_params->profile_seq.move_duration_s * 1000.0f));
	return 0;
}

/* motor profile seq end_vel_hz <hz> */
int cmd_motor_profile_seq_end_vel_hz(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq end_vel_hz <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float end_vel_hz = 0.0f;
	if (!shell_parse_finite_float(argv[1], &end_vel_hz)) {
		shell_error(sh, "end_vel_hz must be a finite number");
		return -EINVAL;
	}

	int ret = motor_profile_seq_set_end_vel_hz(g_motor_params, end_vel_hz);
	if (ret != 0) {
		shell_error(sh, "Failed to set end_vel_hz (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence end velocity set to %.2f Hz",
		    (double)(g_motor_params->profile_seq.end_velocity_rad_s / (2.0f * PI_F32)));
	return 0;
}

/* motor profile seq loop <0|1> */
int cmd_motor_profile_seq_loop(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor profile seq loop <0|1>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	bool loop_enabled = false;
	if (!shell_parse_bool01(argv[1], &loop_enabled)) {
		shell_error(sh, "loop must be 0 or 1");
		return -EINVAL;
	}

	int ret = motor_profile_seq_set_loop(g_motor_params, loop_enabled);
	if (ret != 0) {
		shell_error(sh, "Failed to set loop (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence loop set to %s", g_motor_params->profile_seq.loop ? "YES" : "NO");
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

	uint32_t period_ms = 0U;
	if (!shell_parse_u32(argv[1], &period_ms) || period_ms == 0U) {
		shell_error(sh, "period_ms must be a positive integer");
		return -EINVAL;
	}

	uint32_t move_ms = 0U;
	if (!shell_parse_u32(argv[2], &move_ms) || move_ms == 0U) {
		shell_error(sh, "move_ms must be a positive integer");
		return -EINVAL;
	}

	float end_vel_hz = 0.0f;
	if (!shell_parse_finite_float(argv[3], &end_vel_hz)) {
		shell_error(sh, "end_vel_hz must be a finite number");
		return -EINVAL;
	}

	bool loop_enabled = false;
	if (!shell_parse_bool01(argv[4], &loop_enabled)) {
		shell_error(sh, "loop must be 0 or 1");
		return -EINVAL;
	}

	int ret = motor_profile_seq_set_period_ms(g_motor_params, period_ms);
	if (ret != 0) {
		shell_error(sh, "Failed to apply period_ms (err %d)", ret);
		return ret;
	}
	ret = motor_profile_seq_set_move_ms(g_motor_params, move_ms);
	if (ret != 0) {
		shell_error(sh, "Failed to apply move_ms (err %d)", ret);
		return ret;
	}
	ret = motor_profile_seq_set_end_vel_hz(g_motor_params, end_vel_hz);
	if (ret != 0) {
		shell_error(sh, "Failed to apply end_vel_hz (err %d)", ret);
		return ret;
	}
	ret = motor_profile_seq_set_loop(g_motor_params, loop_enabled);
	if (ret != 0) {
		shell_error(sh, "Failed to apply loop (err %d)", ret);
		return ret;
	}

	shell_warn(sh, "Legacy command: prefer period_ms/move_ms/end_vel_hz/loop subcommands.");
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Sequence config: period=%u ms (%u ticks), move=%u ms, vend=%.2f Hz, loop=%s",
		    period_ms, g_motor_params->profile_seq.period_ticks, move_ms,
		    (double)end_vel_hz, g_motor_params->profile_seq.loop ? "YES" : "NO");
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

	if (g_motor_params->profile_seq.running) {
		shell_error(sh, "Stop sequence before changing trigger source.");
		return -EBUSY;
	}

	uint8_t source = PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL;
	if (!motor_profile_seq_parse_trigger_source(argv[1], &source)) {
		shell_error(sh, "Source must be 'timer' or 'external' (alias: 'internal').");
		return -EINVAL;
	}

	g_motor_params->profile_seq.trigger_source = source;
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

	if (g_motor_params->profile_seq.running) {
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

	g_motor_params->profile_seq.trigger_edge = edge;
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

	if (g_motor_params->profile_seq.running) {
		shell_error(sh, "Stop sequence before changing trigger channel.");
		return -EBUSY;
	}

	uint32_t channel = 0U;
	if (!shell_parse_u32(argv[1], &channel) || channel > 3U) {
		shell_error(sh, "channel must be 0..3");
		return -EINVAL;
	}

	g_motor_params->profile_seq.trigger_channel = (uint8_t)channel;
	g_motor_params->profile_seq.ext_last_capture_valid = false;

	int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
	if (ret < 0 &&
	    g_motor_params->profile_seq.trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_warn(sh, "Channel timing unavailable until capture device is ready (err %d)", ret);
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence trigger channel set to %u",
		    g_motor_params->profile_seq.trigger_channel);
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

	uint32_t interval = 0U;
	if (!shell_parse_u32(argv[1], &interval)) {
		shell_error(sh, "min_interval_us must be an integer in [0, 4294967295]");
		return -EINVAL;
	}

	g_motor_params->profile_seq.ext_min_interval_us = interval;
	g_motor_params->profile_seq.ext_last_capture_valid = false;

	int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
	if (ret < 0 &&
	    g_motor_params->profile_seq.trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_warn(sh, "Capture timing conversion unavailable (err %d)", ret);
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Sequence trigger min interval set to %u us (%u cycles)",
		    g_motor_params->profile_seq.ext_min_interval_us,
		    g_motor_params->profile_seq.ext_min_interval_cycles);
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

	if (!g_motor_params->profile_seq.running) {
		shell_error(sh, "Sequence is not running.");
		return -EACCES;
	}

	if (g_motor_params->profile_seq.trigger_source != PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
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
			    g_motor_params->profile_seq.trigger_source));
	shell_print(sh, "  Edge:         %s",
		    motor_profile_seq_trigger_edge_to_string(
			    g_motor_params->profile_seq.trigger_edge));
	shell_print(sh, "  Channel:      %u", g_motor_params->profile_seq.trigger_channel);
	shell_print(sh, "  Capture:      %s",
		    g_motor_params->profile_seq.ext_capture_enabled ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Min interval: %u us (%u cycles)",
		    g_motor_params->profile_seq.ext_min_interval_us,
		    g_motor_params->profile_seq.ext_min_interval_cycles);
	shell_print(sh, "  Accepted:     %u", g_motor_params->profile_seq.ext_trigger_count);
	shell_print(sh, "  Rejected:     %u", g_motor_params->profile_seq.ext_reject_count);
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

	if (!motor_profile_seq_mode_active(g_motor_params)) {
		shell_error(sh, "Sequence start requires ONLINE_POSITION or ONLINE_PROFILE_OPEN mode.");
		return -EACCES;
	}

	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before sequence start.");
		return -EACCES;
	}

	if (g_motor_params->profile_seq.count == 0U) {
		shell_error(sh, "Sequence is empty. Add points with 'motor profile seq add <deg>'.");
		return -EINVAL;
	}

#if !PROFILE_SEQ_CAPTURE_AVAILABLE
	if (g_motor_params->profile_seq.trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_error(sh, "External trigger source unavailable (capture driver not enabled).");
		return -ENOTSUP;
	}
#else
	if (g_motor_params->profile_seq.trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		if (!device_is_ready(profile_seq_capture_dev)) {
			shell_error(sh, "Capture device not ready");
			return -ENODEV;
		}
		if (g_motor_params->chopper_cal.active &&
		    g_motor_params->profile_seq.trigger_channel == CHOPPER_CAL_CAPTURE_CHANNEL) {
			shell_error(sh, "Chopper calibration is using capture channel %u",
				    CHOPPER_CAL_CAPTURE_CHANNEL);
			return -EBUSY;
		}
	}
#endif

	g_motor_params->profile_seq.running = true;
	g_motor_params->profile_seq.next_idx = 0U;
	g_motor_params->profile_seq.tick_counter = 0U;
	g_motor_params->profile_seq.event_drop_count = 0U;
	g_motor_params->profile_seq.ext_trigger_count = 0U;
	g_motor_params->profile_seq.ext_reject_count = 0U;
	g_motor_params->profile_seq.ext_last_capture_valid = false;
	(void)motor_profile_seq_external_capture_disable(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);

	if (g_motor_params->profile_seq.trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		int ret = motor_profile_seq_update_ext_min_interval_cycles(g_motor_params);
		if (ret < 0) {
			g_motor_params->profile_seq.running = false;
			shell_error(sh, "Failed to derive external trigger timing (err %d)", ret);
			return ret;
		}

		ret = motor_profile_seq_external_capture_enable(g_motor_params);
		if (ret < 0) {
			g_motor_params->profile_seq.running = false;
			shell_error(sh, "Failed to enable external trigger capture (err %d)", ret);
			return ret;
		}

		shell_print(sh,
			    "Profile sequence started (%u points), source=EXTERNAL edge=%s ch=%u",
			    g_motor_params->profile_seq.count,
			    motor_profile_seq_trigger_edge_to_string(
				    g_motor_params->profile_seq.trigger_edge),
			    g_motor_params->profile_seq.trigger_channel);
	} else {
		int ret = motor_profile_seq_post_tick_event(g_motor_params, false);
		if (ret != 0) {
			shell_warn(sh, "Sequence started, initial tick dropped (queue full)");
		}

		if (g_motor_params->command_timeout_ms > 0U &&
		    g_motor_params->profile_seq.period_ms >= g_motor_params->command_timeout_ms) {
			shell_warn(sh, "period_ms >= command_timeout_ms; increase timeout to avoid disarm");
		}

		shell_print(sh, "Profile sequence started (%u points), source=TIMER",
			    g_motor_params->profile_seq.count);
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

	g_motor_params->profile_seq.running = false;
	g_motor_params->profile_seq.tick_counter = 0U;
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

	if (!g_motor_params->profile_seq.running &&
	    g_motor_params->profile_seq.ext_capture_enabled) {
		(void)motor_profile_seq_external_capture_disable(g_motor_params);
	}

	shell_print(sh, "Profile Sequence:");
	shell_print(sh, "  Running:      %s", g_motor_params->profile_seq.running ? "YES" : "NO");
	shell_print(sh, "  Mode:         %s", motor_state_to_string(motor_api_get_state()));
	shell_print(sh, "  Armed:        %s", motor_control_is_armed(g_motor_params) ? "YES" : "NO");
	shell_print(sh, "  Loop:         %s", g_motor_params->profile_seq.loop ? "YES" : "NO");
	shell_print(sh, "  Count:        %u / %u", g_motor_params->profile_seq.count,
		    MOTOR_PROFILE_SEQUENCE_MAX_POINTS);
	shell_print(sh, "  Next index:   %u", g_motor_params->profile_seq.next_idx);
	shell_print(sh, "  Trigger src:  %s",
		    motor_profile_seq_trigger_source_to_string(
			    g_motor_params->profile_seq.trigger_source));
	shell_print(sh, "  Trigger edge: %s",
		    motor_profile_seq_trigger_edge_to_string(
			    g_motor_params->profile_seq.trigger_edge));
	shell_print(sh, "  Trigger ch:   %u", g_motor_params->profile_seq.trigger_channel);
	shell_print(sh, "  Capture:      %s",
		    g_motor_params->profile_seq.ext_capture_enabled ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Min trig dt:  %u us (%u cycles)",
		    g_motor_params->profile_seq.ext_min_interval_us,
		    g_motor_params->profile_seq.ext_min_interval_cycles);
	shell_print(sh, "  Period:       %u ms (%u ticks)", g_motor_params->profile_seq.period_ms,
		    g_motor_params->profile_seq.period_ticks);
	shell_print(sh, "  Move:         %.1f ms",
		    (double)(g_motor_params->profile_seq.move_duration_s * 1000.0f));
	shell_print(sh, "  End vel:      %.2f Hz",
		    (double)(g_motor_params->profile_seq.end_velocity_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Dropped ticks:%u", g_motor_params->profile_seq.event_drop_count);
	shell_print(sh, "  Ext accepted: %u", g_motor_params->profile_seq.ext_trigger_count);
	shell_print(sh, "  Ext rejected: %u", g_motor_params->profile_seq.ext_reject_count);

	if (g_motor_params->profile_seq.count > 0U) {
		uint16_t idx = g_motor_params->profile_seq.next_idx;
		if (idx >= g_motor_params->profile_seq.count) {
			idx = 0U;
		}
		shell_print(sh, "  Next target:  %.2f deg",
			    (double)(g_motor_params->profile_seq.points_rad[idx] *
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

	if (g_motor_params->profile_seq.count == 0U) {
		shell_print(sh, "Sequence is empty");
		return 0;
	}

	shell_print(sh, "Sequence Points (%u):", g_motor_params->profile_seq.count);
	for (uint16_t i = 0; i < g_motor_params->profile_seq.count; i++) {
		shell_print(sh, "  [%u] %.2f deg", (unsigned int)i,
			    (double)(g_motor_params->profile_seq.points_rad[i] *
				     180.0f / PI_F32));
	}

	return 0;
}
