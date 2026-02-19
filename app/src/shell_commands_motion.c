/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>
#include <drivers/timer_ic.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "shell_commands.h"
#include "shell_commands_motion.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"
#include "config.h"
#include "angle_wrap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

#if defined(CONFIG_TIMER_IC) && DT_HAS_COMPAT_STATUS_OKAY(st_stm32_timer_ic)
#define CHOPPER_CAL_CAPTURE_AVAILABLE 1
static const struct device *const chopper_capture_dev = DEVICE_DT_GET_ANY(st_stm32_timer_ic);
#else
#define CHOPPER_CAL_CAPTURE_AVAILABLE 0
#endif

#define CHOPPER_CAL_CAPTURE_CHANNEL 1U
#define CHOPPER_CAL_MIN_STEP_DEG_DEFAULT 0.5f
#define PROFILE_SEQ_EXT_CAPTURE_CHANNEL_DEFAULT 0U

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

#if CHOPPER_CAL_CAPTURE_AVAILABLE
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

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (!device_is_ready(chopper_capture_dev)) {
		params->profile_sequence_ext_min_interval_cycles = 0U;
		return -ENODEV;
	}

	uint64_t cycles_per_sec = 0U;
	int ret = timer_ic_get_cycles_per_sec(chopper_capture_dev,
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

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	int ret = 0;
	if (params->profile_sequence_ext_capture_enabled && device_is_ready(chopper_capture_dev)) {
		ret = timer_ic_disable_capture(chopper_capture_dev,
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

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (!device_is_ready(chopper_capture_dev)) {
		return -ENODEV;
	}

	timer_ic_flags_t flags = motor_profile_seq_ext_edge_to_capture_flags(
		params->profile_sequence_trigger_edge) |
			       TIMER_IC_CAPTURE_MODE_CONTINUOUS;
	int ret = timer_ic_configure_capture(chopper_capture_dev,
					     params->profile_sequence_trigger_channel,
					     flags,
					     motor_profile_seq_external_capture_callback,
					     NULL);
	if (ret < 0) {
		params->profile_sequence_ext_capture_enabled = false;
		params->profile_sequence_ext_last_capture_valid = false;
		return ret;
	}

	ret = timer_ic_enable_capture(chopper_capture_dev,
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

#if CHOPPER_CAL_CAPTURE_AVAILABLE
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

	float max_hz = strtof(argv[1], NULL);
	float max_accel_hz_s = strtof(argv[2], NULL);
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

	float target_deg = strtof(argv[1], NULL);
	float end_vel_hz = strtof(argv[2], NULL);
	float duration_ms = strtof(argv[3], NULL);
	if (duration_ms <= 0.0f) {
		shell_error(sh, "duration_ms must be > 0");
		return -EINVAL;
	}

	float start_pos_rad = g_motor_params->position_rad;
	float start_vel_rad_s = g_motor_params->velocity_rad_s;
	float target_wrapped_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	float delta_rad = wrap_rad_pi(target_wrapped_rad - start_pos_rad);
	float end_pos_rad = start_pos_rad + delta_rad;
	float end_vel_rad_s = end_vel_hz * 2.0f * PI_F32;
	float duration_s = duration_ms * 0.001f;
	g_motor_params->profile_sequence_running = false;
	g_motor_params->profile_sequence_tick_counter = 0U;

	int ret = motion_profile_quintic_plan(&g_motor_params->position_profile,
					      start_pos_rad, start_vel_rad_s, 0.0f,
					      end_pos_rad, end_vel_rad_s, 0.0f, duration_s);
	if (ret != 0) {
		shell_error(sh, "Failed to plan profile (err %d)", ret);
		return ret;
	}

	float peak_vel = 0.0f;
	float peak_acc = 0.0f;
	ret = motion_profile_quintic_check_limits(&g_motor_params->position_profile,
						  g_motor_params->profile_max_velocity_rad_s,
						  g_motor_params->profile_max_accel_rad_s2, 64U,
						  &peak_vel, &peak_acc);
	if (ret != 0) {
		motion_profile_quintic_cancel(&g_motor_params->position_profile, start_pos_rad);
		shell_error(sh,
			    "Profile violates limits (peak %.2f Hz, %.2f Hz/s). Increase duration.",
			    (double)(peak_vel / (2.0f * PI_F32)),
			    (double)(peak_acc / (2.0f * PI_F32)));
		return -ERANGE;
	}

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
	if (!g_motor_params->profile_sequence_running &&
	    g_motor_params->profile_sequence_ext_capture_enabled &&
	    g_motor_params->profile_sequence_trigger_channel == CHOPPER_CAL_CAPTURE_CHANNEL) {
		(void)motor_profile_seq_external_capture_disable(g_motor_params);
	}
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

	char *endp = NULL;
	unsigned long slots_ul = strtoul(argv[1], &endp, 10);
	if (endp == argv[1] || *endp != '\0' || slots_ul == 0UL ||
	    slots_ul > CHOPPER_CAL_MAX_SLOTS) {
		shell_error(sh, "slots must be 1..%u", CHOPPER_CAL_MAX_SLOTS);
		return -EINVAL;
	}

	endp = NULL;
	unsigned long revs_ul = strtoul(argv[2], &endp, 10);
	if (endp == argv[2] || *endp != '\0' || revs_ul == 0UL || revs_ul > 10000UL) {
		shell_error(sh, "revs must be 1..10000");
		return -EINVAL;
	}

	float speed_hz = strtof(argv[3], &endp);
	if (endp == argv[3] || *endp != '\0' || !isfinite(speed_hz) || speed_hz <= 0.0f) {
		shell_error(sh, "speed_hz must be a positive finite value");
		return -EINVAL;
	}

	uint16_t slots = (uint16_t)slots_ul;
	uint16_t revs = (uint16_t)revs_ul;
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

#if !CHOPPER_CAL_CAPTURE_AVAILABLE
	if (g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		shell_error(sh, "External trigger source unavailable (capture driver not enabled).");
		return -ENOTSUP;
	}
#else
	if (g_motor_params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL) {
		if (!device_is_ready(chopper_capture_dev)) {
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

