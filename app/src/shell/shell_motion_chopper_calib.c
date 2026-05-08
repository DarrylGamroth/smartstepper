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

	params->chopper_cal.complete = false;
	params->chopper_cal.valid = false;
	params->chopper_cal.midpoint_count = 0U;
	params->chopper_cal.total_edges_target = 0U;
	params->chopper_cal.total_edges_captured = 0U;
	params->chopper_cal.discarded_edges = 0U;
	params->chopper_cal.spacing_min_rad = 0.0f;
	params->chopper_cal.spacing_max_rad = 0.0f;
	params->chopper_cal.spacing_mean_rad = 0.0f;
	params->chopper_cal.spacing_max_error_rad = 0.0f;
	params->chopper_cal.last_wrapped_rad = 0.0f;
	params->chopper_cal.last_unwrapped_rad = 0.0f;
	params->chopper_cal.start_unwrapped_rad = 0.0f;

	for (uint32_t i = 0U; i < CHOPPER_CAL_MAX_EDGES; i++) {
		params->chopper_cal.edge_sum_rad[i] = 0.0f;
		params->chopper_cal.edge_count[i] = 0U;
		params->chopper_cal.edge_status[i] = 0U;
	}
	for (uint32_t i = 0U; i < CHOPPER_CAL_MAX_SLOTS; i++) {
		params->chopper_cal.blade_midpoints_rad[i] = 0.0f;
		params->chopper_cal.midpoint_kind[i] = CHOPPER_REGION_KIND_UNKNOWN;
	}
}

static inline void motor_chopper_cal_restore_timeout(struct motor_parameters *params)
{
	if (!params) {
		return;
	}
	params->command_timeout_ms = params->chopper_cal.saved_timeout_ms;
}

static uint16_t motor_chopper_cal_edge_bins(const struct motor_chopper_cal_ctx *cal)
{
	if (cal == NULL || cal->slots == 0U || cal->teeth == 0U) {
		return 0U;
	}
	uint32_t bins = (uint32_t)cal->slots + (uint32_t)cal->teeth;

	return bins <= CHOPPER_CAL_MAX_SLOTS ? (uint16_t)bins : 0U;
}

static bool motor_chopper_cal_geometry_valid(uint16_t slots, uint16_t teeth)
{
	uint32_t centers = (uint32_t)slots + (uint32_t)teeth;

	return slots > 0U && teeth > 0U && centers <= CHOPPER_CAL_MAX_SLOTS;
}

struct chopper_edge_mean {
	float32_t angle_rad;
	uint8_t status;
};

static void motor_chopper_sort_edges(struct chopper_edge_mean *edges, uint16_t count)
{
	for (uint16_t i = 1U; i < count; i++) {
		struct chopper_edge_mean key = edges[i];
		uint16_t j = i;

		while (j > 0U && edges[j - 1U].angle_rad > key.angle_rad) {
			edges[j] = edges[j - 1U];
			j--;
		}
		edges[j] = key;
	}
}

static uint8_t motor_chopper_region_kind_from_edge_status(uint8_t status, bool reverse_motion)
{
	if ((status & TIMER_IC_STATUS_EDGE_RISING) != 0U) {
		if (reverse_motion) {
			return CHOPPER_REGION_KIND_TOOTH;
		}
		return CHOPPER_REGION_KIND_SLOT;
	}
	if ((status & TIMER_IC_STATUS_EDGE_FALLING) != 0U) {
		if (reverse_motion) {
			return CHOPPER_REGION_KIND_SLOT;
		}
		return CHOPPER_REGION_KIND_TOOTH;
	}
	return CHOPPER_REGION_KIND_UNKNOWN;
}

static void motor_chopper_assign_alternating_kinds(const struct chopper_edge_mean *edges,
						   uint16_t edge_bins,
						   bool reverse_motion,
						   uint8_t *midpoint_kind)
{
	uint16_t parity_slot_votes[2] = {0U, 0U};
	uint16_t parity_tooth_votes[2] = {0U, 0U};

	for (uint16_t i = 0U; i < edge_bins; i++) {
		uint8_t kind =
			motor_chopper_region_kind_from_edge_status(edges[i].status,
								   reverse_motion);
		uint16_t parity = (uint16_t)(i & 1U);

		if (kind == CHOPPER_REGION_KIND_SLOT) {
			parity_slot_votes[parity]++;
		} else if (kind == CHOPPER_REGION_KIND_TOOTH) {
			parity_tooth_votes[parity]++;
		}
	}

	/*
	 * Slot and tooth regions must alternate around a normal chopper wheel.
	 * Per-edge GPIO level sampling can be noisy because it is inferred after
	 * the timer capture event, so use it only to choose the starting parity.
	 */
	uint8_t even_kind = CHOPPER_REGION_KIND_SLOT;
	int even_slot_score = (int)parity_slot_votes[0] - (int)parity_tooth_votes[0];
	int odd_slot_score = (int)parity_slot_votes[1] - (int)parity_tooth_votes[1];

	if (odd_slot_score > even_slot_score) {
		even_kind = CHOPPER_REGION_KIND_TOOTH;
	}

	for (uint16_t i = 0U; i < edge_bins; i++) {
		bool even = ((i & 1U) == 0U);
		if (even) {
			midpoint_kind[i] = even_kind;
		} else {
			midpoint_kind[i] = (even_kind == CHOPPER_REGION_KIND_SLOT) ?
						   CHOPPER_REGION_KIND_TOOTH :
						   CHOPPER_REGION_KIND_SLOT;
		}
	}
}

static const char *motor_chopper_region_kind_name(uint8_t kind)
{
	switch (kind) {
	case CHOPPER_REGION_KIND_SLOT:
		return "slot";
	case CHOPPER_REGION_KIND_TOOTH:
		return "tooth";
	default:
		return "unknown";
	}
}

static bool motor_chopper_cal_sample_encoder_angle(const struct motor_parameters *params,
						   float32_t *angle_rad)
{
	if (!params || !angle_rad ||
	    params->live.encoder_sample_fresh == 0U ||
	    params->live.encoder_sample_error != 0U) {
		return false;
	}

	float32_t sign = (params->encoder_direction_sign >= 0) ? 1.0f : -1.0f;
	*angle_rad = wrap_rad_2pi(params->live.encoder_raw_deg * sign * (PI_F32 / 180.0f));
	return true;
}

static int motor_chopper_cal_compute_midpoints(struct motor_parameters *params)
{
	if (!params) {
		return -ENODEV;
	}
	uint16_t edge_bins = motor_chopper_cal_edge_bins(&params->chopper_cal);
	if (edge_bins == 0U) {
		return -EINVAL;
	}

	for (uint16_t i = 0U; i < edge_bins; i++) {
		if (params->chopper_cal.edge_count[i] == 0U) {
			params->chopper_cal.valid = false;
			params->chopper_cal.midpoint_count = 0U;
			return -ENODATA;
		}
	}

	struct chopper_edge_mean edges[CHOPPER_CAL_MAX_SLOTS];
	for (uint16_t i = 0U; i < edge_bins; i++) {
		float32_t mean = params->chopper_cal.edge_sum_rad[i] /
				 (float32_t)params->chopper_cal.edge_count[i];
		edges[i].angle_rad = wrap_rad_2pi(mean);
		edges[i].status = params->chopper_cal.edge_status[i];
	}
	motor_chopper_sort_edges(edges, edge_bins);
	uint8_t midpoint_kind[CHOPPER_CAL_MAX_SLOTS];
	if (params->chopper_cal.slots == params->chopper_cal.teeth) {
		motor_chopper_assign_alternating_kinds(
			edges, edge_bins,
			params->chopper_cal.speed_target_rad_s < 0.0f,
			midpoint_kind);
	} else {
		for (uint16_t i = 0U; i < edge_bins; i++) {
			midpoint_kind[i] =
				motor_chopper_region_kind_from_edge_status(
					edges[i].status,
					params->chopper_cal.speed_target_rad_s < 0.0f);
		}
	}

	float32_t ideal_spacing = 2.0f * PI_F32 / (float32_t)edge_bins;
	float32_t spacing_sum = 0.0f;
	float32_t spacing_min = 2.0f * PI_F32;
	float32_t spacing_max = 0.0f;
	float32_t spacing_max_error = 0.0f;
	for (uint16_t i = 0U; i < edge_bins; i++) {
		float32_t a = edges[i].angle_rad;
		float32_t b = edges[(uint16_t)((i + 1U) % edge_bins)].angle_rad;
		if ((i + 1U) == edge_bins) {
			b += 2.0f * PI_F32;
		}
		float32_t spacing = b - a;
		float32_t midpoint = a + (0.5f * spacing);

		params->chopper_cal.blade_midpoints_rad[i] = wrap_rad_2pi(midpoint);
		params->chopper_cal.midpoint_kind[i] = midpoint_kind[i];
		spacing_sum += spacing;
		spacing_min = fminf(spacing_min, spacing);
		spacing_max = fmaxf(spacing_max, spacing);
		spacing_max_error = fmaxf(spacing_max_error, fabsf(spacing - ideal_spacing));
	}

	params->chopper_cal.midpoint_count = edge_bins;
	params->chopper_cal.spacing_min_rad = spacing_min;
	params->chopper_cal.spacing_max_rad = spacing_max;
	params->chopper_cal.spacing_mean_rad = spacing_sum / (float32_t)edge_bins;
	params->chopper_cal.spacing_max_error_rad = spacing_max_error;
	params->chopper_cal.valid = true;
	return 0;
}

static void motor_chopper_release_sequence_capture_channel(struct motor_parameters *params)
{
	if (!params || params->profile_seq.running ||
	    !params->profile_seq.ext_capture_enabled ||
	    params->profile_seq.trigger_channel != CHOPPER_CAL_CAPTURE_CHANNEL) {
		return;
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (device_is_ready(chopper_capture_dev)) {
		(void)timer_ic_disable_capture(chopper_capture_dev,
					       params->profile_seq.trigger_channel);
	}
#endif
	params->profile_seq.ext_capture_enabled = false;
	params->profile_seq.ext_last_capture_valid = false;
}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
static void motor_chopper_capture_callback(const struct device *dev, uint32_t channel,
					   uint32_t cycles, int status, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cycles);
	ARG_UNUSED(user_data);

	struct motor_parameters *params = g_motor_params;
	if (!params || !params->chopper_cal.active) {
		return;
	}
	if ((!motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) &&
	     !motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_ENCODER)) ||
	    atomic_get(&params->control_armed) == 0) {
		params->chopper_cal.active = false;
		params->chopper_cal.complete = false;
		params->chopper_cal.valid = false;
		motor_chopper_cal_restore_timeout(params);
		return;
	}

	if (status < 0 || channel != CHOPPER_CAL_CAPTURE_CHANNEL) {
		params->chopper_cal.discarded_edges++;
		return;
	}

	uint16_t edge_bins = motor_chopper_cal_edge_bins(&params->chopper_cal);
	if (edge_bins == 0U || edge_bins > CHOPPER_CAL_MAX_EDGES) {
		params->chopper_cal.discarded_edges++;
		return;
	}

	uint32_t captured = params->chopper_cal.total_edges_captured;
	float32_t wrapped_rad;
	if (!motor_chopper_cal_sample_encoder_angle(params, &wrapped_rad)) {
		params->chopper_cal.discarded_edges++;
		return;
	}
	float32_t unwrapped_rad = wrapped_rad;

	if (captured == 0U) {
		params->chopper_cal.start_unwrapped_rad = wrapped_rad;
		params->chopper_cal.last_wrapped_rad = wrapped_rad;
		params->chopper_cal.last_unwrapped_rad = wrapped_rad;
	} else {
		float32_t delta_rad = wrap_rad_pi(wrapped_rad - params->chopper_cal.last_wrapped_rad);
		if (fabsf(delta_rad) < params->chopper_cal.edge_min_step_rad) {
			params->chopper_cal.discarded_edges++;
			return;
		}
		unwrapped_rad = params->chopper_cal.last_unwrapped_rad + delta_rad;
		params->chopper_cal.last_wrapped_rad = wrapped_rad;
		params->chopper_cal.last_unwrapped_rad = unwrapped_rad;
	}

	uint16_t bin = (uint16_t)(captured % edge_bins);
	params->chopper_cal.edge_sum_rad[bin] += unwrapped_rad;
	params->chopper_cal.edge_count[bin]++;
	if ((status & (TIMER_IC_STATUS_EDGE_RISING | TIMER_IC_STATUS_EDGE_FALLING)) != 0) {
		params->chopper_cal.edge_status[bin] =
			(uint8_t)(status & (TIMER_IC_STATUS_EDGE_RISING |
					    TIMER_IC_STATUS_EDGE_FALLING));
	}
	params->chopper_cal.total_edges_captured = captured + 1U;

	if (params->chopper_cal.total_edges_captured >= params->chopper_cal.total_edges_target) {
		params->chopper_cal.active = false;
		params->chopper_cal.complete = true;
		params->chopper_cal.valid = false;
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

	g_motor_params->chopper_cal.active = false;
	g_motor_params->chopper_cal.revs_target = 0U;
	g_motor_params->chopper_cal.samples_per_edge = 0U;
	g_motor_params->chopper_cal.speed_target_rad_s = 0.0f;
	motor_chopper_cal_restore_timeout(g_motor_params);
	motor_chopper_cal_reset_buffers(g_motor_params);

	int ret = motor_hardware_set_photo_interruptor_enable(false);
	if (ret < 0) {
		shell_warn(sh, "Failed to disable photo interrupter output (%d)", ret);
	}

	shell_print(sh, "Chopper calibration buffers cleared");
	return 0;
}

/* motor chopper geometry [slots [teeth]] */
int cmd_motor_chopper_geometry(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (argc == 1U) {
		uint16_t centers = motor_chopper_cal_edge_bins(&g_motor_params->chopper_cal);
		shell_print(sh, "Chopper Geometry:");
		shell_print(sh, "  Slots:        %u", g_motor_params->chopper_cal.slots);
		shell_print(sh, "  Teeth:        %u", g_motor_params->chopper_cal.teeth);
		shell_print(sh, "  Centerpoints: %u", centers);
		shell_print(sh, "  Map valid:    %s", g_motor_params->chopper_cal.valid ? "YES" : "NO");
		return 0;
	}
	if (argc != 2U && argc != 3U) {
		shell_error(sh, "Usage: motor chopper geometry [slots [teeth]]");
		return -EINVAL;
	}
	if (g_motor_params->chopper_cal.active || g_motor_params->profile_seq.running) {
		shell_error(sh, "Stop chopper calibration/sequence before changing geometry");
		return -EBUSY;
	}

	uint32_t slots_u32 = 0U;
	uint32_t teeth_u32 = 0U;
	if (!shell_parse_u32(argv[1], &slots_u32)) {
		shell_error(sh, "slots must be positive");
		return -EINVAL;
	}
	teeth_u32 = (argc == 3U) ? 0U : slots_u32;
	if (argc == 3U && !shell_parse_u32(argv[2], &teeth_u32)) {
		shell_error(sh, "teeth must be positive");
		return -EINVAL;
	}
	if (slots_u32 == 0U || teeth_u32 == 0U ||
	    slots_u32 > UINT16_MAX || teeth_u32 > UINT16_MAX ||
	    !motor_chopper_cal_geometry_valid((uint16_t)slots_u32, (uint16_t)teeth_u32)) {
		shell_error(sh, "slots and teeth must be positive and total <= %u",
			    CHOPPER_CAL_MAX_SLOTS);
		return -EINVAL;
	}

	g_motor_params->chopper_cal.slots = (uint16_t)slots_u32;
	g_motor_params->chopper_cal.teeth = (uint16_t)teeth_u32;
	motor_chopper_cal_reset_buffers(g_motor_params);
	shell_print(sh, "Chopper geometry set: slots=%u teeth=%u centers=%u",
		    g_motor_params->chopper_cal.slots,
		    g_motor_params->chopper_cal.teeth,
		    motor_chopper_cal_edge_bins(&g_motor_params->chopper_cal));
	return 0;
}

/* motor chopper sensor [0|1] */
int cmd_motor_chopper_sensor(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1U) {
		bool enabled = false;
		int ret = motor_hardware_get_photo_interruptor_enable(&enabled);
		if (ret < 0) {
			shell_error(sh, "Failed to read photo-interrupter enable GPIO (%d)", ret);
			return ret;
		}
		shell_print(sh, "Photo-interrupter emitter: %s", enabled ? "enabled" : "disabled");
		return 0;
	}
	if (argc != 2U) {
		shell_error(sh, "Usage: motor chopper sensor [0|1]");
		return -EINVAL;
	}

	bool enable = false;
	if (!shell_parse_bool01(argv[1], &enable)) {
		shell_error(sh, "sensor value must be 0/1, true/false, on/off");
		return -EINVAL;
	}
	int ret = motor_hardware_set_photo_interruptor_enable(enable);
	if (ret < 0) {
		shell_error(sh, "Failed to set photo-interrupter enable GPIO (%d)", ret);
		return ret;
	}
	shell_print(sh, "Photo-interrupter emitter %s", enable ? "enabled" : "disabled");
	return 0;
}

/* motor chopper calib start <revs> <velocity_hz> */
int cmd_motor_chopper_calib_start(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor chopper calib start <revs> <velocity_hz>");
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
	if (g_motor_params->profile_seq.ext_capture_enabled &&
	    g_motor_params->profile_seq.trigger_channel == CHOPPER_CAL_CAPTURE_CHANNEL) {
		shell_error(sh, "Sequence external trigger is using capture channel %u",
			    CHOPPER_CAL_CAPTURE_CHANNEL);
		return -EBUSY;
	}
#endif

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr,
				     MOTOR_STATE_ONLINE_VELOCITY_GENERATED) &&
	    !motor_state_ptr_is_mode(g_motor_params->state_for_isr,
				     MOTOR_STATE_ONLINE_VELOCITY_ENCODER)) {
		shell_error(sh, "Calibration requires velocity_generated or velocity_encoder mode.");
		return -EACCES;
	}

	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' first.");
		return -EACCES;
	}

	uint16_t edge_bins = motor_chopper_cal_edge_bins(&g_motor_params->chopper_cal);
	if (edge_bins == 0U) {
		shell_error(sh, "Invalid geometry; run 'motor chopper geometry <slots> [teeth]'");
		return -EINVAL;
	}

	uint32_t revs_u32 = 0U;
	if (!shell_parse_u32(argv[1], &revs_u32) || revs_u32 == 0U || revs_u32 > 10000U) {
		shell_error(sh, "revs must be 1..10000");
		return -EINVAL;
	}

	float velocity_hz = 0.0f;
	if (!shell_parse_finite_float(argv[2], &velocity_hz) || fabsf(velocity_hz) <= 1e-5f) {
		shell_error(sh, "velocity_hz must be a non-zero finite value");
		return -EINVAL;
	}

	uint16_t revs = (uint16_t)revs_u32;
	uint64_t edges_target_u64 = (uint64_t)edge_bins * (uint64_t)revs;
	if (edges_target_u64 > UINT32_MAX) {
		shell_error(sh, "Edge target too large");
		return -ERANGE;
	}

	float32_t speed_target_rad_s = velocity_hz * 2.0f * PI_F32;
	speed_target_rad_s = clampf(speed_target_rad_s, -g_motor_params->profile_max_velocity_rad_s,
				    g_motor_params->profile_max_velocity_rad_s);
	if (fabsf(speed_target_rad_s) < 1e-4f) {
		shell_error(sh, "velocity_hz too small after clamping");
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

	if (g_motor_params->chopper_cal.active) {
		motor_chopper_cal_restore_timeout(g_motor_params);
	}

	g_motor_params->chopper_cal.active = false;
	g_motor_params->chopper_cal.revs_target = revs;
	g_motor_params->chopper_cal.samples_per_edge = revs;
	g_motor_params->chopper_cal.speed_target_rad_s = speed_target_rad_s;
	g_motor_params->chopper_cal.edge_min_step_rad = CHOPPER_CAL_MIN_STEP_DEG_DEFAULT * PI_F32 / 180.0f;
	g_motor_params->chopper_cal.saved_timeout_ms = g_motor_params->command_timeout_ms;
	g_motor_params->command_timeout_ms = 0U; /* Prevent disarm while calibration runs. */
	motor_chopper_cal_reset_buffers(g_motor_params);
	g_motor_params->chopper_cal.total_edges_target = (uint32_t)edges_target_u64;
	g_motor_params->chopper_cal.active = true;

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	ret = timer_ic_configure_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL,
					 TIMER_IC_CAPTURE_EDGE_BOTH |
						 TIMER_IC_CAPTURE_MODE_CONTINUOUS,
					 motor_chopper_capture_callback, NULL);
	if (ret < 0) {
		g_motor_params->chopper_cal.active = false;
		motor_chopper_cal_restore_timeout(g_motor_params);
		shell_error(sh, "Failed to configure capture channel %u (err %d)",
			    CHOPPER_CAL_CAPTURE_CHANNEL, ret);
		return ret;
	}

	ret = timer_ic_enable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
	if (ret < 0) {
		g_motor_params->chopper_cal.active = false;
		motor_chopper_cal_restore_timeout(g_motor_params);
		shell_error(sh, "Failed to enable capture channel %u (err %d)",
			    CHOPPER_CAL_CAPTURE_CHANNEL, ret);
		return ret;
	}
#endif

	traj_set_target_value(&g_motor_params->traj_velocity, speed_target_rad_s);
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh,
		    "Chopper calibration started: slots=%u teeth=%u revs=%u edges=%u velocity=%.3f Hz (channel=%u)",
		    g_motor_params->chopper_cal.slots, g_motor_params->chopper_cal.teeth,
		    revs, g_motor_params->chopper_cal.total_edges_target,
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

	g_motor_params->chopper_cal.active = false;
	motor_chopper_cal_restore_timeout(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);
	int ret = motor_hardware_set_photo_interruptor_enable(false);
	if (ret < 0) {
		shell_warn(sh, "Failed to disable photo interrupter output (%d)", ret);
	}

	if (g_motor_params->chopper_cal.complete && !g_motor_params->chopper_cal.valid) {
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

	if (g_motor_params->chopper_cal.complete && !g_motor_params->chopper_cal.valid) {
		int ret = motor_chopper_cal_compute_midpoints(g_motor_params);
		if (ret < 0) {
			shell_warn(sh, "Midpoint computation incomplete (err %d)", ret);
		}
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (!g_motor_params->chopper_cal.active && g_motor_params->chopper_cal.complete) {
		(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
	}
#endif
	if (!g_motor_params->chopper_cal.active && g_motor_params->chopper_cal.complete) {
		int ret = motor_hardware_set_photo_interruptor_enable(false);
		if (ret < 0) {
			shell_warn(sh, "Failed to disable photo interrupter output (%d)", ret);
		}
	}

	shell_print(sh, "Chopper Calibration:");
	shell_print(sh, "  Active:         %s", g_motor_params->chopper_cal.active ? "YES" : "NO");
	shell_print(sh, "  Complete:       %s", g_motor_params->chopper_cal.complete ? "YES" : "NO");
	shell_print(sh, "  Valid:          %s", g_motor_params->chopper_cal.valid ? "YES" : "NO");
	shell_print(sh, "  Slots:          %u", g_motor_params->chopper_cal.slots);
	shell_print(sh, "  Teeth:          %u", g_motor_params->chopper_cal.teeth);
	shell_print(sh, "  Revolutions:    %u", g_motor_params->chopper_cal.revs_target);
	shell_print(sh, "  Samples/edge:   %u", g_motor_params->chopper_cal.samples_per_edge);
	shell_print(sh, "  Speed target:   %.3f Hz",
		    (double)(g_motor_params->chopper_cal.speed_target_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Edges:          %u / %u",
		    g_motor_params->chopper_cal.total_edges_captured,
		    g_motor_params->chopper_cal.total_edges_target);
	shell_print(sh, "  Discarded:      %u", g_motor_params->chopper_cal.discarded_edges);
	shell_print(sh, "  Midpoints:      %u", g_motor_params->chopper_cal.midpoint_count);
	if (g_motor_params->chopper_cal.valid) {
		shell_print(sh, "  Spacing min/max/mean/error: %.3f / %.3f / %.3f / %.3f deg",
			    (double)(g_motor_params->chopper_cal.spacing_min_rad * 180.0f / PI_F32),
			    (double)(g_motor_params->chopper_cal.spacing_max_rad * 180.0f / PI_F32),
			    (double)(g_motor_params->chopper_cal.spacing_mean_rad * 180.0f / PI_F32),
			    (double)(g_motor_params->chopper_cal.spacing_max_error_rad * 180.0f / PI_F32));
		shell_print(sh, "  Labels:         slot=logical-high region after rising edge; tooth=logical-low region after falling edge");
	}

	if (g_motor_params->chopper_cal.valid && g_motor_params->chopper_cal.midpoint_count > 0U) {
		for (uint16_t i = 0U; i < g_motor_params->chopper_cal.midpoint_count; i++) {
			const char *kind = motor_chopper_region_kind_name(
				g_motor_params->chopper_cal.midpoint_kind[i]);
			shell_print(sh, "    [%u] %-5s %.3f deg", i, kind,
				    (double)(g_motor_params->chopper_cal.blade_midpoints_rad[i] *
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

	if (g_motor_params->chopper_cal.complete && !g_motor_params->chopper_cal.valid) {
		int ret = motor_chopper_cal_compute_midpoints(g_motor_params);
		if (ret < 0) {
			shell_error(sh, "Calibration data incomplete (err %d)", ret);
			return ret;
		}
	}

#if CHOPPER_CAL_CAPTURE_AVAILABLE
	if (!g_motor_params->chopper_cal.active && g_motor_params->chopper_cal.complete) {
		(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
	}
#endif

	if (!g_motor_params->chopper_cal.valid || g_motor_params->chopper_cal.midpoint_count == 0U) {
		shell_error(sh, "No valid midpoint table. Run 'motor chopper calib start ...' first.");
		return -EINVAL;
	}

	g_motor_params->profile_seq.running = false;
	g_motor_params->profile_seq.tick_counter = 0U;
	g_motor_params->profile_seq.count = g_motor_params->chopper_cal.midpoint_count;
	g_motor_params->profile_seq.next_idx = 0U;
	for (uint16_t i = 0U; i < g_motor_params->profile_seq.count; i++) {
		g_motor_params->profile_seq.points_rad[i] = g_motor_params->chopper_cal.blade_midpoints_rad[i];
	}
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Applied %u midpoint targets into profile sequence",
		    g_motor_params->profile_seq.count);
	return 0;
}
