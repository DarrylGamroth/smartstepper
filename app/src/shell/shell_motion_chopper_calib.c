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
#include <stdio.h>
#include <stdlib.h>

#include "shell_commands_motion.h"
#include "shell_commands_motion_common.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"
#include "motor_chopper_map.h"
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
	params->chopper_cal.edge_map_count = 0U;
	params->chopper_cal.midpoint_count = 0U;
	params->chopper_cal.blade_state_edge_idx = 0U;
	params->chopper_cal.blade_state_edge_idx_valid = false;
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
		params->chopper_cal.blade_edges_rad[i] = 0.0f;
		params->chopper_cal.edge_region_after[i] = CHOPPER_REGION_KIND_UNKNOWN;
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

static bool motor_chopper_encoder_mapping_ready(const struct motor_parameters *params)
{
	return params != NULL && params->calibration.encoder_mapping_complete;
}

struct chopper_edge_mean {
	float32_t angle_rad;
	uint8_t status;
};

struct chopper_cal_map_snapshot {
	uint16_t count;
	float32_t edges_rad[CHOPPER_CAL_MAX_SLOTS];
	uint8_t edge_region_after[CHOPPER_CAL_MAX_SLOTS];
	float32_t spacing_min_rad;
	float32_t spacing_max_rad;
	float32_t spacing_mean_rad;
	float32_t spacing_max_error_rad;
	uint32_t captured_edges;
	uint32_t discarded_edges;
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
						   uint8_t *edge_region_after)
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
			edge_region_after[i] = even_kind;
		} else {
			edge_region_after[i] = (even_kind == CHOPPER_REGION_KIND_SLOT) ?
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
	uint8_t edge_region_after[CHOPPER_CAL_MAX_SLOTS];
	if (params->chopper_cal.slots == params->chopper_cal.teeth) {
		motor_chopper_assign_alternating_kinds(
			edges, edge_bins,
			params->chopper_cal.speed_target_rad_s < 0.0f,
			edge_region_after);
	} else {
		for (uint16_t i = 0U; i < edge_bins; i++) {
			edge_region_after[i] =
				motor_chopper_region_kind_from_edge_status(
					edges[i].status,
					params->chopper_cal.speed_target_rad_s < 0.0f);
		}
	}

	for (uint16_t i = 0U; i < edge_bins; i++) {
		params->chopper_cal.blade_edges_rad[i] = edges[i].angle_rad;
		params->chopper_cal.edge_region_after[i] = edge_region_after[i];
	}

	params->chopper_cal.edge_map_count = edge_bins;
	return motor_chopper_map_derive_centers(&params->chopper_cal);
}

static void motor_chopper_cal_snapshot(const struct motor_parameters *params,
				       struct chopper_cal_map_snapshot *snapshot)
{
	snapshot->count = params->chopper_cal.edge_map_count;
	snapshot->spacing_min_rad = params->chopper_cal.spacing_min_rad;
	snapshot->spacing_max_rad = params->chopper_cal.spacing_max_rad;
	snapshot->spacing_mean_rad = params->chopper_cal.spacing_mean_rad;
	snapshot->spacing_max_error_rad = params->chopper_cal.spacing_max_error_rad;
	snapshot->captured_edges = params->chopper_cal.total_edges_captured;
	snapshot->discarded_edges = params->chopper_cal.discarded_edges;

	for (uint16_t i = 0U; i < snapshot->count; i++) {
		snapshot->edges_rad[i] = params->chopper_cal.blade_edges_rad[i];
		snapshot->edge_region_after[i] = params->chopper_cal.edge_region_after[i];
	}
}

static float32_t motor_chopper_cal_average_angle(float32_t a_rad, float32_t b_rad)
{
	float32_t s = sinf(a_rad) + sinf(b_rad);
	float32_t c = cosf(a_rad) + cosf(b_rad);

	return wrap_rad_2pi(atan2f(s, c));
}

static int motor_chopper_cal_stage_average(struct motor_parameters *params,
					   const struct chopper_cal_map_snapshot *forward,
					   const struct chopper_cal_map_snapshot *reverse,
					   float32_t max_delta_rad,
					   float32_t *max_delta_out_rad,
					   float32_t *mean_delta_out_rad)
{
	if (!params || !forward || !reverse ||
	    forward->count == 0U ||
	    forward->count != reverse->count ||
	    forward->count > CHOPPER_CAL_MAX_SLOTS) {
		return -EINVAL;
	}

	float32_t max_delta = 0.0f;
	float32_t delta_sum = 0.0f;

	for (uint16_t i = 0U; i < forward->count; i++) {
		if (forward->edge_region_after[i] != reverse->edge_region_after[i]) {
			return -EINVAL;
		}
		float32_t delta = fabsf(wrap_rad_pi(reverse->edges_rad[i] -
						    forward->edges_rad[i]));
		max_delta = fmaxf(max_delta, delta);
		delta_sum += delta;
	}

	*max_delta_out_rad = max_delta;
	*mean_delta_out_rad = delta_sum / (float32_t)forward->count;
	if (max_delta > max_delta_rad) {
		return -ERANGE;
	}

	params->chopper_cal.edge_map_count = forward->count;
	for (uint16_t i = 0U; i < forward->count; i++) {
		params->chopper_cal.blade_edges_rad[i] =
			motor_chopper_cal_average_angle(forward->edges_rad[i],
							reverse->edges_rad[i]);
		params->chopper_cal.edge_region_after[i] = forward->edge_region_after[i];
	}

	int ret = motor_chopper_map_derive_centers(&params->chopper_cal);
	if (ret != 0) {
		return ret;
	}
	params->chopper_cal.complete = true;
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
		shell_print(sh, "  Edges:        %u", g_motor_params->chopper_cal.edge_map_count);
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
		bool slot = false;
		ret = motor_hardware_get_chopper_blade_state(&slot);
		if (ret < 0) {
			shell_error(sh, "Failed to read chopper blade-state GPIO (%d)", ret);
			return ret;
		}
		shell_print(sh, "Photo-interrupter emitter: %s", enabled ? "enabled" : "disabled");
		shell_print(sh, "Blade-state output PB4:    %s", slot ? "slot/high" : "tooth/low");
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

	if (!motor_chopper_encoder_mapping_ready(g_motor_params)) {
		shell_error(sh,
			    "Chopper edge capture requires applied encoder alignment; run/load encoder commissioning first.");
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

static int motor_chopper_cal_wait_complete(struct motor_parameters *params, uint32_t timeout_ms)
{
	int64_t deadline_ms = k_uptime_get() + (int64_t)timeout_ms;

	while (params->chopper_cal.active && !params->chopper_cal.complete) {
		if (k_uptime_get() > deadline_ms) {
#if CHOPPER_CAL_CAPTURE_AVAILABLE
			(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
#endif
			params->chopper_cal.active = false;
			params->chopper_cal.complete = false;
			params->chopper_cal.valid = false;
			motor_chopper_cal_restore_timeout(params);
			(void)motor_hardware_set_photo_interruptor_enable(false);
			return -ETIMEDOUT;
		}
		motor_command_feed_watchdog(params);
		k_sleep(K_MSEC(50));
	}

	if (!params->chopper_cal.complete) {
		return -EIO;
	}

	int ret = motor_chopper_cal_compute_midpoints(params);
#if CHOPPER_CAL_CAPTURE_AVAILABLE
	(void)timer_ic_disable_capture(chopper_capture_dev, CHOPPER_CAL_CAPTURE_CHANNEL);
#endif
	(void)motor_hardware_set_photo_interruptor_enable(false);
	return ret;
}

static int motor_chopper_cal_run_blocking(const struct shell *sh,
					  struct motor_parameters *params,
					  uint16_t revs,
					  float32_t velocity_hz,
					  struct chopper_cal_map_snapshot *snapshot)
{
	char revs_buf[12];
	char velocity_buf[24];
	char *start_argv[] = {
		"start",
		revs_buf,
		velocity_buf,
	};

	snprintf(revs_buf, sizeof(revs_buf), "%u", revs);
	snprintf(velocity_buf, sizeof(velocity_buf), "%.6f", (double)velocity_hz);

	int ret = cmd_motor_chopper_calib_start(sh, 3U, start_argv);
	if (ret < 0) {
		return ret;
	}

	uint32_t motion_time_ms =
		(uint32_t)ceilf((float32_t)revs / fabsf(velocity_hz) * 1000.0f);
	uint32_t timeout_ms = motion_time_ms + 10000U;

	ret = motor_chopper_cal_wait_complete(params, timeout_ms);
	if (ret < 0) {
		return ret;
	}
	if (!params->chopper_cal.valid) {
		return -EIO;
	}

	motor_chopper_cal_snapshot(params, snapshot);
	return 0;
}

/* motor chopper calib bidir <revs> <velocity_hz> [max_delta_deg] */
int cmd_motor_chopper_calib_bidir(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3U && argc != 4U) {
		shell_error(sh,
			    "Usage: motor chopper calib bidir <revs> <velocity_hz> [max_delta_deg]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t revs_u32 = 0U;
	if (!shell_parse_u32(argv[1], &revs_u32) || revs_u32 == 0U || revs_u32 > UINT16_MAX) {
		shell_error(sh, "revs must be 1..%u", UINT16_MAX);
		return -EINVAL;
	}

	float32_t velocity_hz = 0.0f;
	if (!shell_parse_finite_float(argv[2], &velocity_hz) || fabsf(velocity_hz) <= 1e-5f) {
		shell_error(sh, "velocity_hz must be a non-zero finite value");
		return -EINVAL;
	}
	velocity_hz = fabsf(velocity_hz);

	float32_t max_delta_deg = 2.0f;
	if (argc == 4U &&
	    (!shell_parse_finite_float(argv[3], &max_delta_deg) || max_delta_deg <= 0.0f)) {
		shell_error(sh, "max_delta_deg must be positive");
		return -EINVAL;
	}

	struct chopper_cal_map_snapshot forward = {0};
	struct chopper_cal_map_snapshot reverse = {0};
	struct motor_parameters *params = g_motor_params;
	uint16_t revs = (uint16_t)revs_u32;

	shell_print(sh, "Bidirectional chopper capture: forward %.3f Hz", (double)velocity_hz);
	int ret = motor_chopper_cal_run_blocking(sh, params, revs, velocity_hz, &forward);
	if (ret < 0) {
		shell_error(sh, "Forward capture failed (%d)", ret);
		return ret;
	}

	shell_print(sh, "Bidirectional chopper capture: reverse %.3f Hz", (double)-velocity_hz);
	ret = motor_chopper_cal_run_blocking(sh, params, revs, -velocity_hz, &reverse);
	if (ret < 0) {
		shell_error(sh, "Reverse capture failed (%d)", ret);
		return ret;
	}

	float32_t max_delta_rad = 0.0f;
	float32_t mean_delta_rad = 0.0f;
	ret = motor_chopper_cal_stage_average(params, &forward, &reverse,
					      max_delta_deg * PI_F32 / 180.0f,
					      &max_delta_rad, &mean_delta_rad);
	if (ret < 0) {
		shell_error(sh,
			    "Forward/reverse mismatch: max=%.3f deg mean=%.3f deg limit=%.3f deg",
			    (double)(max_delta_rad * 180.0f / PI_F32),
			    (double)(mean_delta_rad * 180.0f / PI_F32),
			    (double)max_delta_deg);
		return ret;
	}

	shell_print(sh, "Bidirectional chopper map staged:");
	shell_print(sh, "  Forward edges/discarded: %u / %u",
		    forward.captured_edges, forward.discarded_edges);
	shell_print(sh, "  Reverse edges/discarded: %u / %u",
		    reverse.captured_edges, reverse.discarded_edges);
	shell_print(sh, "  F/R delta max/mean: %.3f / %.3f deg",
		    (double)(max_delta_rad * 180.0f / PI_F32),
		    (double)(mean_delta_rad * 180.0f / PI_F32));
	shell_print(sh, "  Averaged spacing min/max/mean/error: %.3f / %.3f / %.3f / %.3f deg",
		    (double)(params->chopper_cal.spacing_min_rad * 180.0f / PI_F32),
		    (double)(params->chopper_cal.spacing_max_rad * 180.0f / PI_F32),
		    (double)(params->chopper_cal.spacing_mean_rad * 180.0f / PI_F32),
		    (double)(params->chopper_cal.spacing_max_error_rad * 180.0f / PI_F32));
	shell_print(sh, "Run 'motor chopper calib apply' then 'motor settings save chopper' to use it.");
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
	shell_print(sh, "  Edge map:       %u", g_motor_params->chopper_cal.edge_map_count);
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
			shell_print(sh, "    center[%u] %-5s %.3f deg", i, kind,
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
