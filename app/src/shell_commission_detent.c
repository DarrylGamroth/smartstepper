/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "shell_commands_commission.h"
#include "shell_commands_motion.h"
#include "shell_commission_internal.h"
#include "motor_control_api.h"
#include "motor_torque.h"
#include "shell_parse.h"
#include "motor/math/math_constants.h"
#include "motor/control/dob.h"
#include "motor/control/velocity_regulator.h"
#include "motor/compensation/detent_map.h"
#include "motor/motion/traj.h"

#define MOTOR_COMMISSION_DETENT_MAX_SPEED_HZ 1.0f
#define MOTOR_COMMISSION_DETENT_RECOMMENDED_SPEED_HZ 0.10f
#define MOTOR_COMMISSION_DETENT_RECOMMENDED_CYCLES 10.0f
#define MOTOR_COMMISSION_DETENT_LOW_SPEED_WARN_HZ 0.08f
#define MOTOR_COMMISSION_DETENT_SAFE_IQ_LIMIT_A 0.12f
#define MOTOR_COMMISSION_DETENT_SAFE_GAIN_SPEED_HZ 1.0f
#define MOTOR_COMMISSION_DETENT_DEFAULT_DECIMATION 1U
#define MOTOR_COMMISSION_DETENT_MIN_DECIMATION 1U
#define MOTOR_COMMISSION_DETENT_MAX_DECIMATION 128U
#define MOTOR_COMMISSION_DETENT_MIN_APPLY_BINS 250U
#define MOTOR_COMMISSION_DETENT_MIN_RAW_BINS 192U
#define MOTOR_COMMISSION_DETENT_MAX_FILL_GAP_BINS 4U
#define MOTOR_COMMISSION_DETENT_MIN_SAMPLES_PER_BIN 4U
#define MOTOR_COMMISSION_DETENT_MAX_RUN_MS 60000U
#define MOTOR_COMMISSION_DETENT_VELOCITY_BAND_RATIO 0.40f
#define MOTOR_COMMISSION_DETENT_MIN_VELOCITY_BAND_HZ 0.03f
#define MOTOR_COMMISSION_DETENT_ACCEL_LIMIT_FACTOR 3.0f
#define MOTOR_COMMISSION_DETENT_MAX_REJECT_RATIO 0.95f
#define MOTOR_COMMISSION_DETENT_MAX_ADJ_STEP_A 0.08f
#define MOTOR_COMMISSION_DETENT_VALIDATE_MIN_SAMPLES 10U
#define MOTOR_COMMISSION_DETENT_VALIDATE_MATCH_RATIO 1.02f
#define MOTOR_COMMISSION_DETENT_VALIDATE_PEAK_MATCH_RATIO 1.02f

enum motor_commission_detent_recommendation {
	MOTOR_COMMISSION_DETENT_RECOMMEND_INCONCLUSIVE = 0,
	MOTOR_COMMISSION_DETENT_RECOMMEND_DO_NOT_APPLY,
	MOTOR_COMMISSION_DETENT_RECOMMEND_APPLY,
};

struct motor_commission_detent_result {
	bool valid;
	bool raw_valid;
	bool fill_valid;
	bool apply_valid;
	uint32_t sample_count;
	uint32_t rejected_samples;
	uint32_t rejected_quality;
	uint32_t rejected_velocity;
	uint32_t rejected_accel;
	uint32_t rejected_saturation;
	uint32_t accepted_forward;
	uint32_t accepted_reverse;
	uint16_t populated_bins;
	uint16_t raw_populated_bins;
	uint16_t filled_bins;
	uint16_t forward_bins;
	uint16_t reverse_bins;
	uint16_t both_direction_bins;
	uint16_t min_bin_count;
	uint16_t max_bin_count;
	uint32_t run_ms;
	uint32_t decimation;
	float32_t speed_hz;
	float32_t cycles;
	float32_t confidence;
	float32_t reject_ratio;
	float32_t max_adjacent_step_a;
	float32_t forward_reverse_rms_a;
	float32_t min_iq_ff_a;
	float32_t max_iq_ff_a;
	float32_t mean_abs_iq_ff_a;
	float32_t rms_iq_ff_a;
	float32_t recommended_limit_a;
	bool validation_complete;
	enum motor_commission_detent_recommendation validation_recommendation;
	float32_t validation_off_rms_hz;
	float32_t validation_on_rms_hz;
	float32_t validation_off_peak_hz;
	float32_t validation_on_peak_hz;
	float32_t validation_improvement_pct;
	uint32_t validation_encoder_errors;
	float32_t table_iq_a[MOTOR_DETENT_MAP_BINS];
	uint16_t bin_counts[MOTOR_DETENT_MAP_BINS];
	uint8_t bin_flags[MOTOR_DETENT_MAP_BINS];
};

static struct motor_commission_detent_result detent_result;

static const char *motor_commission_detent_recommendation_str(
	enum motor_commission_detent_recommendation recommendation)
{
	switch (recommendation) {
	case MOTOR_COMMISSION_DETENT_RECOMMEND_APPLY:
		return "RECOMMEND_APPLY";
	case MOTOR_COMMISSION_DETENT_RECOMMEND_DO_NOT_APPLY:
		return "DO_NOT_APPLY";
	case MOTOR_COMMISSION_DETENT_RECOMMEND_INCONCLUSIVE:
	default:
		return "INCONCLUSIVE";
	}
}

enum motor_commission_detent_bin_flag {
	MOTOR_COMMISSION_DETENT_BIN_RAW = BIT(0),
	MOTOR_COMMISSION_DETENT_BIN_FILLED = BIT(1),
	MOTOR_COMMISSION_DETENT_BIN_BOTH_DIR = BIT(2),
};

struct motor_commission_velocity_gain_restore {
	float32_t kp_a_per_rad_s;
	float32_t ki_a_per_rad;
	float32_t iq_limit_a;
	float32_t i_term_a;
	bool valid;
};
static int motor_commission_apply_detent_velocity_gains(
	float32_t speed_hz,
	float32_t iq_limit_a,
	struct motor_commission_velocity_gain_restore *restore)
{
	if (g_motor_params == NULL || restore == NULL) {
		return -EINVAL;
	}

	iq_limit_a = clampf(iq_limit_a, 0.02f, MOTOR_MAX_CURRENT_A);
	float32_t gain_speed_rad_s =
		2.0f * PI_F32 * fmaxf(speed_hz, MOTOR_COMMISSION_DETENT_SAFE_GAIN_SPEED_HZ);
	float32_t kp = iq_limit_a / gain_speed_rad_s;
	float32_t ki = 2.0f * kp;

	restore->kp_a_per_rad_s = g_motor_params->velocity_cl_kp_A_per_rad_s;
	restore->ki_a_per_rad = g_motor_params->velocity_cl_ki_A_per_rad;
	restore->iq_limit_a = g_motor_params->velocity_cl_iq_limit_A;
	restore->i_term_a = g_motor_params->velocity_cl_i_term_A;
	restore->valid = true;

	int ret = motor_api_set_param("velocity_cl_kp_A_per_rad_s", kp);
	if (ret != 0) {
		return ret;
	}
	ret = motor_api_set_param("velocity_cl_ki_A_per_rad", ki);
	if (ret != 0) {
		return ret;
	}
	ret = motor_api_set_param("velocity_cl_iq_limit_A", iq_limit_a);
	if (ret != 0) {
		return ret;
	}
	g_motor_params->velocity_cl_i_term_A = 0.0f;
	motor_velocity_regulator_reset(&g_motor_params->velocity_reg_state, 0.0f);

	return 0;
}

static void motor_commission_restore_velocity_gains(
	const struct motor_commission_velocity_gain_restore *restore)
{
	if (g_motor_params == NULL || restore == NULL || !restore->valid) {
		return;
	}

	(void)motor_api_set_param("velocity_cl_kp_A_per_rad_s", restore->kp_a_per_rad_s);
	(void)motor_api_set_param("velocity_cl_ki_A_per_rad", restore->ki_a_per_rad);
	(void)motor_api_set_param("velocity_cl_iq_limit_A", restore->iq_limit_a);
	g_motor_params->velocity_cl_i_term_A = restore->i_term_a;
	motor_velocity_regulator_reset(&g_motor_params->velocity_reg_state,
				       restore->i_term_a);
}

static void motor_commission_detent_clear_staged(void)
{
	memset(&detent_result, 0, sizeof(detent_result));
}

static void motor_commission_detent_capture_reset(float32_t kt_nm_per_a, uint32_t decimation)
{
	struct motor_detent_capture_ctx *cap = &g_motor_params->detent_capture;

	cap->active = false;
	memset(cap->sum_iq_a, 0, sizeof(cap->sum_iq_a));
	memset(cap->sum_iq_forward_a, 0, sizeof(cap->sum_iq_forward_a));
	memset(cap->sum_iq_reverse_a, 0, sizeof(cap->sum_iq_reverse_a));
	memset(cap->bin_counts, 0, sizeof(cap->bin_counts));
	memset(cap->bin_counts_forward, 0, sizeof(cap->bin_counts_forward));
	memset(cap->bin_counts_reverse, 0, sizeof(cap->bin_counts_reverse));
	cap->decimation = decimation;
	cap->decimation_counter = 0U;
	cap->sample_count = 0U;
	cap->rejected_samples = 0U;
	cap->rejected_quality = 0U;
	cap->rejected_velocity = 0U;
	cap->rejected_accel = 0U;
	cap->rejected_saturation = 0U;
	cap->accepted_forward = 0U;
	cap->accepted_reverse = 0U;
	cap->kt_nm_per_a = kt_nm_per_a;
	cap->inertia_kgm2 = g_motor_params->inertia_kgm2_active;
	cap->viscous_friction_nm_per_rad_s =
		g_motor_params->viscous_friction_nm_per_rad_s_active;
	cap->coulomb_friction_nm = g_motor_params->coulomb_friction_nm_active;
	cap->target_speed_rad_s = 0.0f;
	cap->velocity_band_rad_s = 0.0f;
	cap->accel_limit_rad_s2 = 0.0f;
	cap->iq_saturation_limit_a = 0.0f;
}

static void motor_commission_detent_capture_enable(float32_t target_hz)
{
	struct motor_detent_capture_ctx *cap = &g_motor_params->detent_capture;

	cap->decimation_counter = 0U;
	cap->target_speed_rad_s = target_hz * 2.0f * PI_F32;
	float32_t band_hz =
		fmaxf(fabsf(target_hz) * MOTOR_COMMISSION_DETENT_VELOCITY_BAND_RATIO,
		      MOTOR_COMMISSION_DETENT_MIN_VELOCITY_BAND_HZ);
	cap->velocity_band_rad_s = band_hz * 2.0f * PI_F32;
	cap->accel_limit_rad_s2 =
		MOTOR_COMMISSION_DETENT_ACCEL_LIMIT_FACTOR *
		fmaxf(g_motor_params->profile_max_accel_rad_s2, 1.0f);
	cap->iq_saturation_limit_a =
		fmaxf(0.0f, 0.98f * g_motor_params->velocity_cl_iq_limit_A);
	cap->active = true;
}

static void motor_commission_detent_capture_disable(void)
{
	g_motor_params->detent_capture.active = false;
}

static int motor_commission_detent_collect_pass(float32_t target_hz,
						uint32_t settle_ms,
						uint32_t collect_ms)
{
	motor_commission_set_velocity_target_hz(target_hz);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_commission_wait_ms_or_fault(settle_ms);
	if (ret != 0) {
		return ret;
	}

	motor_commission_detent_capture_enable(target_hz);
	ret = motor_commission_wait_ms_or_fault(collect_ms);
	motor_commission_detent_capture_disable();

	return ret;
}

static uint16_t motor_commission_detent_prev_raw(uint16_t start)
{
	for (uint16_t step = 1U; step < MOTOR_DETENT_MAP_BINS; step++) {
		uint16_t idx = (uint16_t)((start + MOTOR_DETENT_MAP_BINS - step) %
					  MOTOR_DETENT_MAP_BINS);
		if ((detent_result.bin_flags[idx] & MOTOR_COMMISSION_DETENT_BIN_RAW) != 0U) {
			return idx;
		}
	}

	return UINT16_MAX;
}

static uint16_t motor_commission_detent_next_raw(uint16_t start)
{
	for (uint16_t step = 1U; step < MOTOR_DETENT_MAP_BINS; step++) {
		uint16_t idx = (uint16_t)((start + step) % MOTOR_DETENT_MAP_BINS);
		if ((detent_result.bin_flags[idx] & MOTOR_COMMISSION_DETENT_BIN_RAW) != 0U) {
			return idx;
		}
	}

	return UINT16_MAX;
}

static void motor_commission_detent_fill_short_holes(void)
{
	for (uint16_t i = 0U; i < MOTOR_DETENT_MAP_BINS; i++) {
		if ((detent_result.bin_flags[i] & MOTOR_COMMISSION_DETENT_BIN_RAW) != 0U) {
			continue;
		}

		uint16_t prev = motor_commission_detent_prev_raw(i);
		uint16_t next = motor_commission_detent_next_raw(i);
		if (prev == UINT16_MAX || next == UINT16_MAX) {
			continue;
		}

		uint16_t gap = (uint16_t)((next + MOTOR_DETENT_MAP_BINS - prev) %
					  MOTOR_DETENT_MAP_BINS);
		if (gap == 0U || gap > (MOTOR_COMMISSION_DETENT_MAX_FILL_GAP_BINS + 1U)) {
			continue;
		}

		uint16_t from_prev = (uint16_t)((i + MOTOR_DETENT_MAP_BINS - prev) %
						MOTOR_DETENT_MAP_BINS);
		float32_t frac = (float32_t)from_prev / (float32_t)gap;
		float32_t y0 = detent_result.table_iq_a[prev];
		float32_t y1 = detent_result.table_iq_a[next];
		detent_result.table_iq_a[i] = y0 + frac * (y1 - y0);
		detent_result.bin_flags[i] |= MOTOR_COMMISSION_DETENT_BIN_FILLED;
		detent_result.filled_bins++;
	}
}

static void motor_commission_detent_smooth_table(void)
{
	float32_t smoothed[MOTOR_DETENT_MAP_BINS];

	for (uint16_t i = 0U; i < MOTOR_DETENT_MAP_BINS; i++) {
		uint16_t prev = (uint16_t)((i + MOTOR_DETENT_MAP_BINS - 1U) %
					   MOTOR_DETENT_MAP_BINS);
		uint16_t next = (uint16_t)((i + 1U) % MOTOR_DETENT_MAP_BINS);

		if (detent_result.bin_flags[i] == 0U ||
		    detent_result.bin_flags[prev] == 0U ||
		    detent_result.bin_flags[next] == 0U) {
			smoothed[i] = detent_result.table_iq_a[i];
			continue;
		}

		smoothed[i] = (0.25f * detent_result.table_iq_a[prev]) +
			      (0.50f * detent_result.table_iq_a[i]) +
			      (0.25f * detent_result.table_iq_a[next]);
	}

	memcpy(detent_result.table_iq_a, smoothed, sizeof(smoothed));
}

static int motor_commission_detent_finalize(void)
{
	struct motor_detent_capture_ctx *cap = &g_motor_params->detent_capture;

	detent_result.sample_count = cap->sample_count;
	detent_result.rejected_samples = cap->rejected_samples;
	detent_result.rejected_quality = cap->rejected_quality;
	detent_result.rejected_velocity = cap->rejected_velocity;
	detent_result.rejected_accel = cap->rejected_accel;
	detent_result.rejected_saturation = cap->rejected_saturation;
	detent_result.accepted_forward = cap->accepted_forward;
	detent_result.accepted_reverse = cap->accepted_reverse;

	float32_t sum_abs = 0.0f;
	float32_t sum_sq = 0.0f;
	float32_t max_abs = 0.0f;
	float32_t sum_fr_sq = 0.0f;
	uint32_t populated = 0U;
	uint32_t raw_populated = 0U;
	uint32_t forward_bins = 0U;
	uint32_t reverse_bins = 0U;
	uint32_t both_bins = 0U;
	uint16_t min_count = UINT16_MAX;
	uint16_t max_count = 0U;

	for (uint16_t i = 0U; i < MOTOR_DETENT_MAP_BINS; i++) {
		uint16_t fwd_count = cap->bin_counts_forward[i];
		uint16_t rev_count = cap->bin_counts_reverse[i];
		bool has_fwd = fwd_count >= MOTOR_COMMISSION_DETENT_MIN_SAMPLES_PER_BIN;
		bool has_rev = rev_count >= MOTOR_COMMISSION_DETENT_MIN_SAMPLES_PER_BIN;
		float32_t iq = 0.0f;
		uint16_t total_count = (uint16_t)MIN((uint32_t)fwd_count + (uint32_t)rev_count,
						     UINT16_MAX);

		if (has_fwd) {
			forward_bins++;
		}
		if (has_rev) {
			reverse_bins++;
		}
		if (has_fwd && has_rev) {
			float32_t iq_fwd = cap->sum_iq_forward_a[i] / (float32_t)fwd_count;
			float32_t iq_rev = cap->sum_iq_reverse_a[i] / (float32_t)rev_count;
			iq = 0.5f * (iq_fwd + iq_rev);
			float32_t diff = iq_fwd - iq_rev;
			sum_fr_sq += diff * diff;
			both_bins++;
		} else if (has_fwd) {
			iq = cap->sum_iq_forward_a[i] / (float32_t)fwd_count;
		} else if (has_rev) {
			iq = cap->sum_iq_reverse_a[i] / (float32_t)rev_count;
		} else {
			detent_result.table_iq_a[i] = 0.0f;
			detent_result.bin_counts[i] = total_count;
			detent_result.bin_flags[i] = 0U;
			continue;
		}

		detent_result.table_iq_a[i] = iq;
		detent_result.bin_counts[i] = total_count;
		detent_result.bin_flags[i] = MOTOR_COMMISSION_DETENT_BIN_RAW;
		if (has_fwd && has_rev) {
			detent_result.bin_flags[i] |= MOTOR_COMMISSION_DETENT_BIN_BOTH_DIR;
		}
		raw_populated++;
		min_count = MIN(min_count, total_count);
		max_count = MAX(max_count, total_count);
	}

	if (raw_populated == 0U) {
		detent_result.min_bin_count = 0U;
		detent_result.max_bin_count = 0U;
		return -ENODATA;
	}

	motor_commission_detent_fill_short_holes();
	motor_commission_detent_smooth_table();

	for (uint16_t i = 0U; i < MOTOR_DETENT_MAP_BINS; i++) {
		if (detent_result.bin_flags[i] == 0U) {
			continue;
		}
		float32_t iq = detent_result.table_iq_a[i];
		populated++;
		detent_result.min_iq_ff_a = (populated == 1U) ?
						    iq :
						    fminf(detent_result.min_iq_ff_a, iq);
		detent_result.max_iq_ff_a = (populated == 1U) ?
						    iq :
						    fmaxf(detent_result.max_iq_ff_a, iq);
		sum_abs += fabsf(iq);
		sum_sq += iq * iq;
		max_abs = fmaxf(max_abs, fabsf(iq));

		uint16_t next = (uint16_t)((i + 1U) % MOTOR_DETENT_MAP_BINS);
		if (detent_result.bin_flags[next] != 0U) {
			detent_result.max_adjacent_step_a =
				fmaxf(detent_result.max_adjacent_step_a,
				      fabsf(detent_result.table_iq_a[i] -
					    detent_result.table_iq_a[next]));
		}
	}

	detent_result.populated_bins = (uint16_t)populated;
	detent_result.raw_populated_bins = (uint16_t)raw_populated;
	detent_result.forward_bins = (uint16_t)forward_bins;
	detent_result.reverse_bins = (uint16_t)reverse_bins;
	detent_result.both_direction_bins = (uint16_t)both_bins;
	detent_result.min_bin_count = min_count;
	detent_result.max_bin_count = max_count;
	detent_result.mean_abs_iq_ff_a = sum_abs / (float32_t)populated;
	detent_result.rms_iq_ff_a = sqrtf(sum_sq / (float32_t)populated);
	detent_result.forward_reverse_rms_a =
		(both_bins > 0U) ? sqrtf(sum_fr_sq / (float32_t)both_bins) : 0.0f;
	uint32_t total_candidate_samples =
		detent_result.sample_count + detent_result.rejected_samples;
	detent_result.reject_ratio =
		(total_candidate_samples > 0U) ?
			((float32_t)detent_result.rejected_samples /
			 (float32_t)total_candidate_samples) :
			0.0f;
	detent_result.recommended_limit_a =
		fminf(fmaxf(1.25f * max_abs, 0.0f), g_motor_params->velocity_cl_iq_limit_A);
	float32_t coverage = (float32_t)detent_result.populated_bins /
			     (float32_t)MOTOR_DETENT_MAP_BINS;
	float32_t both_dir_score = (float32_t)detent_result.both_direction_bins /
				   (float32_t)MOTOR_DETENT_MAP_BINS;
	float32_t reject_penalty = clampf(detent_result.reject_ratio /
					  MOTOR_COMMISSION_DETENT_MAX_REJECT_RATIO,
					  0.0f, 1.0f);
	detent_result.confidence =
		clampf((0.70f * coverage) + (0.30f * both_dir_score) -
		       (0.20f * reject_penalty),
		       0.0f, 1.0f);
	detent_result.raw_valid =
		detent_result.raw_populated_bins >= MOTOR_COMMISSION_DETENT_MIN_RAW_BINS &&
		detent_result.sample_count >= raw_populated &&
		detent_result.recommended_limit_a > 0.0f;
	detent_result.fill_valid =
		detent_result.raw_valid &&
		detent_result.populated_bins >= MOTOR_COMMISSION_DETENT_MIN_APPLY_BINS;
	detent_result.apply_valid =
		detent_result.fill_valid &&
		detent_result.reject_ratio <= MOTOR_COMMISSION_DETENT_MAX_REJECT_RATIO &&
		detent_result.max_adjacent_step_a <= MOTOR_COMMISSION_DETENT_MAX_ADJ_STEP_A;
	detent_result.valid = detent_result.apply_valid;

	return detent_result.valid ? 0 : -ERANGE;
}

int cmd_motor_commission_detent_run(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3 || argc > 5) {
		shell_error(sh,
			    "Usage: motor commission detent run <mech_hz> <cycles> [decimation] [iq_limit_a]");
		shell_error(sh, "Recommended start: motor commission detent run %.2f %.0f 1 %.2f",
			    (double)MOTOR_COMMISSION_DETENT_RECOMMENDED_SPEED_HZ,
			    (double)MOTOR_COMMISSION_DETENT_RECOMMENDED_CYCLES,
			    (double)MOTOR_COMMISSION_DETENT_SAFE_IQ_LIMIT_A);
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Calibration is not complete; run calibration first");
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' first");
		return -EACCES;
	}
	if (!g_motor_params->calibration.encoder_mapping_complete) {
		shell_error(sh,
			    "Encoder mapping has not been applied; run 'motor commission encoder run ...' and 'motor commission encoder apply' first");
		return -EACCES;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}
	float32_t speed_hz = 0.0f;
	float32_t cycles = 0.0f;
	uint32_t decimation = MOTOR_COMMISSION_DETENT_DEFAULT_DECIMATION;
	float32_t capture_iq_limit_a = MOTOR_COMMISSION_DETENT_SAFE_IQ_LIMIT_A;
	if (!shell_parse_finite_float(argv[1], &speed_hz) ||
	    !shell_parse_finite_float(argv[2], &cycles) ||
	    (argc >= 4 && !shell_parse_u32(argv[3], &decimation)) ||
	    (argc >= 5 && !shell_parse_finite_float(argv[4], &capture_iq_limit_a))) {
		shell_error(sh, "Invalid argument");
		return -EINVAL;
	}
	if (speed_hz <= 0.0f || cycles <= 0.0f) {
		shell_error(sh, "mech_hz and cycles must be positive");
		return -EINVAL;
	}
	if (speed_hz > MOTOR_COMMISSION_DETENT_MAX_SPEED_HZ) {
		shell_error(sh, "detent capture speed is limited to %.3f Hz",
			    (double)MOTOR_COMMISSION_DETENT_MAX_SPEED_HZ);
		return -ERANGE;
	}
	if (speed_hz < MOTOR_COMMISSION_DETENT_LOW_SPEED_WARN_HZ) {
		shell_warn(sh,
			   "Capture speed %.3f Hz is low; friction/stiction can dominate. Recommended start is %.2f Hz.",
			   (double)speed_hz,
			   (double)MOTOR_COMMISSION_DETENT_RECOMMENDED_SPEED_HZ);
	}
	if (cycles < MOTOR_COMMISSION_DETENT_RECOMMENDED_CYCLES) {
		shell_warn(sh,
			   "Only %.1f cycles requested; recommended start is %.0f cycles for forward/reverse bin agreement.",
			   (double)cycles,
			   (double)MOTOR_COMMISSION_DETENT_RECOMMENDED_CYCLES);
	}
	if (capture_iq_limit_a <= 0.0f || capture_iq_limit_a > MOTOR_MAX_CURRENT_A) {
		shell_error(sh, "iq_limit_a must be within (0, %.3f] A",
			    (double)MOTOR_MAX_CURRENT_A);
		return -ERANGE;
	}
	float32_t max_hz = g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	if (speed_hz > max_hz) {
		shell_error(sh, "mech_hz exceeds profile max %.3f Hz", (double)max_hz);
		return -ERANGE;
	}
	decimation = CLAMP(decimation,
			    MOTOR_COMMISSION_DETENT_MIN_DECIMATION,
			    MOTOR_COMMISSION_DETENT_MAX_DECIMATION);

	float32_t collect_ms_f = (cycles / speed_hz) * 1000.0f;
	if (!isfinite(collect_ms_f) || collect_ms_f < 1.0f ||
	    collect_ms_f > (float32_t)MOTOR_COMMISSION_DETENT_MAX_RUN_MS) {
		shell_error(sh, "Capture duration per direction invalid or above %u ms",
			    MOTOR_COMMISSION_DETENT_MAX_RUN_MS);
		return -ERANGE;
	}
	uint32_t collect_ms = (uint32_t)(collect_ms_f + 0.5f);
	float32_t accel_hz_s = g_motor_params->profile_max_accel_rad_s2 / (2.0f * PI_F32);
	uint32_t settle_ms = 500U;
	if (isfinite(accel_hz_s) && accel_hz_s > 0.0f) {
		settle_ms += (uint32_t)(((speed_hz / accel_hz_s) * 1000.0f) + 0.5f);
	}
	settle_ms = CLAMP(settle_ms, 500U, 10000U);

	float32_t kt = motor_torque_gain_resolve_active(g_motor_params);
	if (!isfinite(kt) || kt <= 0.0f) {
		shell_error(sh, "Active torque gain is invalid; run commissioning first");
		return -ERANGE;
	}

	bool saved_detent_enable = g_motor_params->detent_map_cfg.enabled;
	bool saved_dob_enable = g_motor_params->velocity_dob_cfg.enabled;
	uint8_t saved_outer_loop = g_motor_params->outer_loop_mode;
	struct motor_commission_velocity_gain_restore velocity_restore = {0};

	motor_commission_detent_clear_staged();
	motor_commission_detent_capture_reset(kt, decimation);
	int ret = motor_commission_apply_detent_velocity_gains(speed_hz, capture_iq_limit_a,
							       &velocity_restore);
	if (ret != 0) {
		motor_commission_restore_velocity_gains(&velocity_restore);
		shell_error(sh, "Failed to apply detent velocity gains (err %d)", ret);
		return ret;
	}

	g_motor_params->detent_map_cfg.enabled = false;
	g_motor_params->velocity_dob_cfg.enabled = false;
	g_motor_params->outer_loop_mode = MOTOR_OUTER_LOOP_MODE_PI;
	motor_detent_map_reset(&g_motor_params->detent_map_state);
	motor_dob_reset(&g_motor_params->velocity_dob_state,
			g_motor_params->live.velocity_rad_s);

	ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		goto restore_runtime;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		goto restore_runtime;
	}

	shell_print(sh,
		    "Detent capture: speed=%.3f Hz cycles=%.2f decimation=%u settle=%u ms safe_vel(kp=%.5f ki=%.5f iq=%.3f)",
		    (double)speed_hz, (double)cycles, decimation, settle_ms,
		    (double)g_motor_params->velocity_cl_kp_A_per_rad_s,
		    (double)g_motor_params->velocity_cl_ki_A_per_rad,
		    (double)g_motor_params->velocity_cl_iq_limit_A);

	ret = motor_commission_detent_collect_pass(speed_hz, settle_ms, collect_ms);
	if (ret == 0) {
		ret = motor_commission_detent_collect_pass(-speed_hz, settle_ms, collect_ms);
	}

	motor_commission_set_velocity_target_hz(0.0f);
	(void)motor_commission_wait_ms_or_fault(settle_ms);

	if (ret == 0) {
		detent_result.speed_hz = speed_hz;
		detent_result.cycles = cycles;
		detent_result.decimation = decimation;
		detent_result.run_ms = 2U * collect_ms;
		ret = motor_commission_detent_finalize();
	}

restore_runtime:
	motor_commission_detent_capture_disable();
	motor_commission_set_velocity_target_hz(0.0f);
	g_motor_params->detent_map_cfg.enabled = saved_detent_enable;
	g_motor_params->velocity_dob_cfg.enabled = saved_dob_enable;
	g_motor_params->outer_loop_mode = saved_outer_loop;
	motor_commission_restore_velocity_gains(&velocity_restore);
	motor_detent_map_reset(&g_motor_params->detent_map_state);
	motor_dob_reset(&g_motor_params->velocity_dob_state,
			g_motor_params->live.velocity_rad_s);
	motor_command_feed_watchdog(g_motor_params);

	if (ret != 0) {
		detent_result.sample_count = g_motor_params->detent_capture.sample_count;
		detent_result.rejected_samples = g_motor_params->detent_capture.rejected_samples;
		shell_error(sh,
			    "Detent capture failed (err %d): samples=%u rejected=%u bins=%u/%u raw=%u/%u conf=%.2f",
			    ret, detent_result.sample_count, detent_result.rejected_samples,
			    detent_result.populated_bins, MOTOR_DETENT_MAP_BINS,
			    detent_result.raw_populated_bins, MOTOR_DETENT_MAP_BINS,
			    (double)detent_result.confidence);
		shell_error(sh,
			    "  quality: raw=%s fill=%s apply=%s reject_ratio=%.2f max_step=%.5f A",
			    detent_result.raw_valid ? "PASS" : "FAIL",
			    detent_result.fill_valid ? "PASS" : "FAIL",
			    detent_result.apply_valid ? "PASS" : "FAIL",
			    (double)detent_result.reject_ratio,
			    (double)detent_result.max_adjacent_step_a);
		return ret;
	}

	shell_print(sh,
		    "Detent capture staged: samples=%u rejected=%u bins=%u/%u raw=%u filled=%u minN=%u maxN=%u",
		    detent_result.sample_count, detent_result.rejected_samples,
		    detent_result.populated_bins, MOTOR_DETENT_MAP_BINS,
		    detent_result.raw_populated_bins, detent_result.filled_bins,
		    detent_result.min_bin_count, detent_result.max_bin_count);
	shell_print(sh,
		    "  iq_ff: min=%.5f max=%.5f mean_abs=%.5f rms=%.5f rec_limit=%.5f A",
		    (double)detent_result.min_iq_ff_a,
		    (double)detent_result.max_iq_ff_a,
		    (double)detent_result.mean_abs_iq_ff_a,
		    (double)detent_result.rms_iq_ff_a,
		    (double)detent_result.recommended_limit_a);
	shell_print(sh,
		    "  quality: raw=%s fill=%s apply=%s conf=%.2f reject_ratio=%.2f fr_rms=%.5f max_step=%.5f A",
		    detent_result.raw_valid ? "PASS" : "FAIL",
		    detent_result.fill_valid ? "PASS" : "FAIL",
		    detent_result.apply_valid ? "PASS" : "FAIL",
		    (double)detent_result.confidence,
		    (double)detent_result.reject_ratio,
		    (double)detent_result.forward_reverse_rms_a,
		    (double)detent_result.max_adjacent_step_a);
	shell_print(sh,
		    "Run 'motor commission detent validate <hz> <ms>' before applying; apply only if validation improves or matches baseline.");
	return 0;
}

int cmd_motor_commission_detent_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Detent Feedforward:");
	shell_print(sh, "  Runtime:   en=%s gain=%.3f limit=%.5f A live=%.5f A idx=%u",
		    g_motor_params->detent_map_cfg.enabled ? "YES" : "NO",
		    (double)g_motor_params->detent_map_cfg.gain,
		    (double)g_motor_params->detent_map_cfg.iq_ff_limit_a,
		    (double)g_motor_params->live.detent_iq_ff_a,
		    g_motor_params->detent_map_state.last_index);
	uint32_t candidate_samples = detent_result.sample_count + detent_result.rejected_samples;
	float32_t accepted_ratio =
		(candidate_samples > 0U) ?
			((float32_t)detent_result.sample_count / (float32_t)candidate_samples) :
			0.0f;
	float32_t fwd_coverage =
		(float32_t)detent_result.forward_bins / (float32_t)MOTOR_DETENT_MAP_BINS;
	float32_t rev_coverage =
		(float32_t)detent_result.reverse_bins / (float32_t)MOTOR_DETENT_MAP_BINS;
	float32_t both_coverage =
		(float32_t)detent_result.both_direction_bins / (float32_t)MOTOR_DETENT_MAP_BINS;
	shell_print(sh, "  Staged:    %s", detent_result.valid ? "YES" : "NO");
	shell_print(sh, "  Quality:   raw=%s fill=%s apply=%s conf=%.2f reject_ratio=%.2f max_step=%.5f A",
		    detent_result.raw_valid ? "PASS" : "FAIL",
		    detent_result.fill_valid ? "PASS" : "FAIL",
		    detent_result.apply_valid ? "PASS" : "FAIL",
		    (double)detent_result.confidence,
		    (double)detent_result.reject_ratio,
		    (double)detent_result.max_adjacent_step_a);
	shell_print(sh, "  Capture:   speed=%.3f Hz cycles=%.2f run=%u ms decimation=%u",
		    (double)detent_result.speed_hz,
		    (double)detent_result.cycles,
		    detent_result.run_ms,
		    detent_result.decimation);
	shell_print(sh, "  Samples:   accepted=%u rejected=%u bins=%u/%u raw=%u filled=%u minN=%u maxN=%u",
		    detent_result.sample_count,
		    detent_result.rejected_samples,
		    detent_result.populated_bins,
		    MOTOR_DETENT_MAP_BINS,
		    detent_result.raw_populated_bins,
		    detent_result.filled_bins,
		    detent_result.min_bin_count,
		    detent_result.max_bin_count);
	shell_print(sh, "  Samples:   accepted_ratio=%.1f%% candidate=%u",
		    (double)(accepted_ratio * 100.0f), candidate_samples);
	shell_print(sh, "  Direction: fwd=%u rev=%u both_bins=%u fr_rms=%.5f A",
		    detent_result.accepted_forward,
		    detent_result.accepted_reverse,
		    detent_result.both_direction_bins,
		    (double)detent_result.forward_reverse_rms_a);
	shell_print(sh, "  Coverage:  fwd=%.1f%% rev=%.1f%% both=%.1f%%",
		    (double)(fwd_coverage * 100.0f),
		    (double)(rev_coverage * 100.0f),
		    (double)(both_coverage * 100.0f));
	shell_print(sh, "  Rejects:   quality=%u velocity=%u accel=%u saturation=%u",
		    detent_result.rejected_quality,
		    detent_result.rejected_velocity,
		    detent_result.rejected_accel,
		    detent_result.rejected_saturation);
	shell_print(sh, "  Iq FF:     min=%.5f max=%.5f mean_abs=%.5f rms=%.5f rec_limit=%.5f A",
		    (double)detent_result.min_iq_ff_a,
		    (double)detent_result.max_iq_ff_a,
		    (double)detent_result.mean_abs_iq_ff_a,
		    (double)detent_result.rms_iq_ff_a,
		    (double)detent_result.recommended_limit_a);
	shell_print(sh,
		    "  Validate:  %s off_rms=%.4f Hz on_rms=%.4f Hz off_peak=%.4f Hz on_peak=%.4f Hz improve=%.1f%% enc_err=%u",
		    motor_commission_detent_recommendation_str(
			    detent_result.validation_recommendation),
		    (double)detent_result.validation_off_rms_hz,
		    (double)detent_result.validation_on_rms_hz,
		    (double)detent_result.validation_off_peak_hz,
		    (double)detent_result.validation_on_peak_hz,
		    (double)detent_result.validation_improvement_pct,
		    detent_result.validation_encoder_errors);
	return 0;
}

int cmd_motor_commission_detent_apply(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 4) {
		shell_error(sh, "Usage: motor commission detent apply [enable] [gain] [limit_a]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!detent_result.valid) {
		shell_error(sh, "No valid staged detent table");
		return -ENOENT;
	}

	bool enable = false;
	float32_t gain = 1.0f;
	float32_t limit_a = detent_result.recommended_limit_a;
	if (argc >= 2 && !shell_parse_bool01(argv[1], &enable)) {
		shell_error(sh, "enable must be 0 or 1");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_finite_float(argv[2], &gain)) {
		shell_error(sh, "gain must be finite");
		return -EINVAL;
	}
	if (argc >= 4 && !shell_parse_finite_float(argv[3], &limit_a)) {
		shell_error(sh, "limit_a must be finite");
		return -EINVAL;
	}
	if (gain < 0.0f || limit_a < 0.0f || limit_a > g_motor_params->velocity_cl_iq_limit_A) {
		shell_error(sh, "gain must be >=0 and limit_a must be within velocity Iq limit");
		return -ERANGE;
	}
	if (enable && detent_result.validation_recommendation !=
		      MOTOR_COMMISSION_DETENT_RECOMMEND_APPLY) {
		shell_error(sh,
			    "Detent table has not been validated for enabled apply: recommendation=%s",
			    motor_commission_detent_recommendation_str(
				    detent_result.validation_recommendation));
		shell_error(sh,
			    "Run 'motor commission detent validate <hz> <ms>' first, or apply disabled with 'motor commission detent apply 0'.");
		return -EACCES;
	}

	memcpy(g_motor_params->detent_map_iq_table_a,
	       detent_result.table_iq_a,
	       sizeof(g_motor_params->detent_map_iq_table_a));
	g_motor_params->detent_map_cfg.table_iq_a = g_motor_params->detent_map_iq_table_a;
	g_motor_params->detent_map_cfg.table_len = MOTOR_DETENT_MAP_BINS;
	g_motor_params->detent_map_cfg.gain = gain;
	g_motor_params->detent_map_cfg.iq_ff_limit_a = limit_a;
	g_motor_params->detent_map_cfg.enabled = enable;
	motor_detent_map_reset(&g_motor_params->detent_map_state);
	g_motor_params->live.detent_iq_ff_a = 0.0f;

	shell_print(sh, "Detent table applied: enable=%s gain=%.3f limit=%.5f A",
		    enable ? "YES" : "NO", (double)gain, (double)limit_a);
	return 0;
}

struct motor_commission_detent_validate_metrics {
	uint32_t samples;
	float32_t sum_abs_vel_err_hz;
	float32_t sum_sq_vel_err_hz;
	float32_t peak_abs_vel_err_hz;
	float32_t sum_sq_iq_a;
	float32_t peak_abs_iq_a;
	uint32_t encoder_errors_start;
	uint32_t encoder_errors_end;
};

static void motor_commission_detent_validate_sample(
	struct motor_commission_detent_validate_metrics *m,
	float32_t target_hz)
{
	float32_t measured_hz = g_motor_params->live.velocity_rad_s / (2.0f * PI_F32);
	float32_t err_hz = target_hz - measured_hz;
	float32_t iq_ref_a = g_motor_params->live.Iq_ref_A;

	if (!isfinite(err_hz) || !isfinite(iq_ref_a)) {
		return;
	}

	m->samples++;
	m->sum_abs_vel_err_hz += fabsf(err_hz);
	m->sum_sq_vel_err_hz += err_hz * err_hz;
	m->peak_abs_vel_err_hz = fmaxf(m->peak_abs_vel_err_hz, fabsf(err_hz));
	m->sum_sq_iq_a += iq_ref_a * iq_ref_a;
	m->peak_abs_iq_a = fmaxf(m->peak_abs_iq_a, fabsf(iq_ref_a));
}

static uint32_t motor_commission_detent_encoder_error_total(void)
{
	return g_motor_params->encoder_error_count +
	       g_motor_params->live.position_glitch_count;
}

static int motor_commission_detent_validate_pass(float32_t target_hz,
						 uint32_t duration_ms,
						 struct motor_commission_detent_validate_metrics *m)
{
	m->encoder_errors_start = motor_commission_detent_encoder_error_total();
	motor_commission_set_velocity_target_hz(target_hz);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_commission_wait_ms_or_fault(500U);
	if (ret != 0) {
		return ret;
	}

	uint32_t elapsed_ms = 0U;
	while (elapsed_ms < duration_ms) {
		ret = motor_commission_wait_ms_or_fault(20U);
		if (ret != 0) {
			return ret;
		}
		elapsed_ms += 20U;
		motor_commission_detent_validate_sample(m, target_hz);
	}
	m->encoder_errors_end = motor_commission_detent_encoder_error_total();

	return 0;
}

static void motor_commission_detent_validate_print(
	const struct shell *sh,
	const char *label,
	const struct motor_commission_detent_validate_metrics *m)
{
	float32_t n = (m->samples == 0U) ? 1.0f : (float32_t)m->samples;
	float32_t mean_abs_err = m->sum_abs_vel_err_hz / n;
	float32_t rms_err = sqrtf(m->sum_sq_vel_err_hz / n);
	float32_t rms_iq = sqrtf(m->sum_sq_iq_a / n);
	uint32_t encoder_delta =
		(m->encoder_errors_end >= m->encoder_errors_start) ?
			(m->encoder_errors_end - m->encoder_errors_start) : 0U;

	shell_print(sh,
		    "  %s: N=%u mean_abs_err=%.4f Hz rms_err=%.4f Hz peak_err=%.4f Hz rms_iq=%.5f A peak_iq=%.5f A enc_err_delta=%u",
		    label,
		    m->samples,
		    (double)mean_abs_err,
		    (double)rms_err,
		    (double)m->peak_abs_vel_err_hz,
		    (double)rms_iq,
		    (double)m->peak_abs_iq_a,
		    encoder_delta);
}

static float32_t motor_commission_detent_validate_rms_hz(
	const struct motor_commission_detent_validate_metrics *m)
{
	if (m == NULL || m->samples == 0U) {
		return 0.0f;
	}

	return sqrtf(m->sum_sq_vel_err_hz / (float32_t)m->samples);
}

static uint32_t motor_commission_detent_validate_encoder_delta(
	const struct motor_commission_detent_validate_metrics *m)
{
	if (m == NULL || m->encoder_errors_end < m->encoder_errors_start) {
		return 0U;
	}

	return m->encoder_errors_end - m->encoder_errors_start;
}

static enum motor_commission_detent_recommendation
motor_commission_detent_validate_recommend(
	const struct motor_commission_detent_validate_metrics *off,
	const struct motor_commission_detent_validate_metrics *on,
	float32_t *off_rms_hz,
	float32_t *on_rms_hz,
	float32_t *improvement_pct,
	uint32_t *encoder_errors)
{
	float32_t off_rms = motor_commission_detent_validate_rms_hz(off);
	float32_t on_rms = motor_commission_detent_validate_rms_hz(on);
	uint32_t enc_delta = motor_commission_detent_validate_encoder_delta(off) +
			     motor_commission_detent_validate_encoder_delta(on);
	float32_t improvement = 0.0f;

	if (off_rms > 0.0f) {
		improvement = 100.0f * (off_rms - on_rms) / off_rms;
	}

	if (off_rms_hz != NULL) {
		*off_rms_hz = off_rms;
	}
	if (on_rms_hz != NULL) {
		*on_rms_hz = on_rms;
	}
	if (improvement_pct != NULL) {
		*improvement_pct = improvement;
	}
	if (encoder_errors != NULL) {
		*encoder_errors = enc_delta;
	}

	if (off == NULL || on == NULL ||
	    off->samples < MOTOR_COMMISSION_DETENT_VALIDATE_MIN_SAMPLES ||
	    on->samples < MOTOR_COMMISSION_DETENT_VALIDATE_MIN_SAMPLES ||
	    off_rms <= 0.0f ||
	    enc_delta != 0U) {
		return MOTOR_COMMISSION_DETENT_RECOMMEND_INCONCLUSIVE;
	}

	if (on->peak_abs_vel_err_hz >
	    (off->peak_abs_vel_err_hz * MOTOR_COMMISSION_DETENT_VALIDATE_PEAK_MATCH_RATIO)) {
		return MOTOR_COMMISSION_DETENT_RECOMMEND_DO_NOT_APPLY;
	}

	return (on_rms <= (off_rms * MOTOR_COMMISSION_DETENT_VALIDATE_MATCH_RATIO)) ?
		       MOTOR_COMMISSION_DETENT_RECOMMEND_APPLY :
		       MOTOR_COMMISSION_DETENT_RECOMMEND_DO_NOT_APPLY;
}

int cmd_motor_commission_detent_validate(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor commission detent validate <mech_hz> <duration_ms>");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' first");
		return -EACCES;
	}
	if (!g_motor_params->calibration.encoder_mapping_complete) {
		shell_error(sh, "Encoder mapping is not complete");
		return -EACCES;
	}

	float32_t speed_hz = 0.0f;
	uint32_t duration_ms = 0U;
	if (!shell_parse_finite_float(argv[1], &speed_hz) ||
	    !shell_parse_u32(argv[2], &duration_ms) ||
	    speed_hz <= 0.0f ||
	    duration_ms < 500U ||
	    duration_ms > 30000U) {
		shell_error(sh, "mech_hz must be positive and duration_ms must be 500..30000");
		return -EINVAL;
	}

	float32_t saved_table[MOTOR_DETENT_MAP_BINS];
	struct motor_detent_map_config saved_cfg = g_motor_params->detent_map_cfg;
	bool saved_dob_enable = g_motor_params->velocity_dob_cfg.enabled;
	uint8_t saved_outer_loop = g_motor_params->outer_loop_mode;

	memcpy(saved_table, g_motor_params->detent_map_iq_table_a, sizeof(saved_table));
	g_motor_params->velocity_dob_cfg.enabled = false;
	g_motor_params->outer_loop_mode = MOTOR_OUTER_LOOP_MODE_PI;
	g_motor_params->detent_map_cfg.enabled = false;
	motor_detent_map_reset(&g_motor_params->detent_map_state);
	motor_dob_reset(&g_motor_params->velocity_dob_state,
			g_motor_params->live.velocity_rad_s);

	int ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret == 0) {
		ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
						     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	}
	if (ret != 0) {
		goto restore_runtime;
	}

	struct motor_commission_detent_validate_metrics off = {0};
	struct motor_commission_detent_validate_metrics on = {0};

	shell_print(sh, "Detent validation: speed=+/-%.3f Hz duration=%u ms/pass",
		    (double)speed_hz, duration_ms);
	ret = motor_commission_detent_validate_pass(speed_hz, duration_ms, &off);
	if (ret == 0) {
		ret = motor_commission_detent_validate_pass(-speed_hz, duration_ms, &off);
	}
	if (ret != 0) {
		goto stop_restore;
	}

	if (detent_result.valid) {
		memcpy(g_motor_params->detent_map_iq_table_a,
		       detent_result.table_iq_a,
		       sizeof(g_motor_params->detent_map_iq_table_a));
		g_motor_params->detent_map_cfg.table_iq_a = g_motor_params->detent_map_iq_table_a;
		g_motor_params->detent_map_cfg.table_len = MOTOR_DETENT_MAP_BINS;
		g_motor_params->detent_map_cfg.gain = 1.0f;
		g_motor_params->detent_map_cfg.iq_ff_limit_a = detent_result.recommended_limit_a;
	}
	if (detent_result.valid || saved_cfg.iq_ff_limit_a > 0.0f) {
		g_motor_params->detent_map_cfg.enabled = true;
	}
	motor_detent_map_reset(&g_motor_params->detent_map_state);

	ret = motor_commission_detent_validate_pass(speed_hz, duration_ms, &on);
	if (ret == 0) {
		ret = motor_commission_detent_validate_pass(-speed_hz, duration_ms, &on);
	}

stop_restore:
	motor_commission_set_velocity_target_hz(0.0f);
	(void)motor_commission_wait_ms_or_fault(500U);
	motor_commission_detent_validate_print(sh, "detent_off", &off);
	motor_commission_detent_validate_print(sh, "detent_on ", &on);
	detent_result.validation_off_peak_hz = off.peak_abs_vel_err_hz;
	detent_result.validation_on_peak_hz = on.peak_abs_vel_err_hz;
	detent_result.validation_recommendation =
		motor_commission_detent_validate_recommend(&off, &on,
							   &detent_result.validation_off_rms_hz,
							   &detent_result.validation_on_rms_hz,
							   &detent_result.validation_improvement_pct,
							   &detent_result.validation_encoder_errors);
	detent_result.validation_complete = true;
	shell_print(sh,
		    "  recommendation: %s off_rms=%.4f Hz on_rms=%.4f Hz off_peak=%.4f Hz on_peak=%.4f Hz improvement=%.1f%% enc_err=%u",
		    motor_commission_detent_recommendation_str(
			    detent_result.validation_recommendation),
		    (double)detent_result.validation_off_rms_hz,
		    (double)detent_result.validation_on_rms_hz,
		    (double)detent_result.validation_off_peak_hz,
		    (double)detent_result.validation_on_peak_hz,
		    (double)detent_result.validation_improvement_pct,
		    detent_result.validation_encoder_errors);

restore_runtime:
	memcpy(g_motor_params->detent_map_iq_table_a, saved_table, sizeof(saved_table));
	g_motor_params->detent_map_cfg = saved_cfg;
	g_motor_params->velocity_dob_cfg.enabled = saved_dob_enable;
	g_motor_params->outer_loop_mode = saved_outer_loop;
	motor_detent_map_reset(&g_motor_params->detent_map_state);
	motor_dob_reset(&g_motor_params->velocity_dob_state,
			g_motor_params->live.velocity_rad_s);
	motor_command_feed_watchdog(g_motor_params);

	return ret;
}

int cmd_motor_commission_detent_dump(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t start = 0U;
	uint32_t count = 32U;

	if (argc > 3) {
		shell_error(sh, "Usage: motor commission detent dump [start_bin] [count]");
		return -EINVAL;
	}
	if (argc >= 2 && !shell_parse_u32(argv[1], &start)) {
		shell_error(sh, "start_bin must be an integer");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &count)) {
		shell_error(sh, "count must be an integer");
		return -EINVAL;
	}
	if (start >= MOTOR_DETENT_MAP_BINS || count == 0U) {
		shell_error(sh, "start_bin must be < %u and count must be positive",
			    MOTOR_DETENT_MAP_BINS);
		return -EINVAL;
	}
	count = MIN(count, MOTOR_DETENT_MAP_BINS);

	shell_print(sh, "Detent bins: start=%u count=%u", start, count);
	for (uint32_t n = 0U; n < count; n++) {
		uint32_t idx = (start + n) % MOTOR_DETENT_MAP_BINS;
		shell_print(sh, "  %3u: iq=% .6f count=%u flags=0x%02x",
			    idx,
			    (double)detent_result.table_iq_a[idx],
			    detent_result.bin_counts[idx],
			    detent_result.bin_flags[idx]);
	}

	return 0;
}

int cmd_motor_commission_detent_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	motor_commission_detent_clear_staged();
	motor_commission_detent_capture_reset(0.0f, MOTOR_COMMISSION_DETENT_DEFAULT_DECIMATION);
	g_motor_params->detent_map_cfg.enabled = false;
	g_motor_params->detent_map_cfg.gain = 1.0f;
	g_motor_params->detent_map_cfg.iq_ff_limit_a = 0.0f;
	g_motor_params->detent_map_cfg.phase_advance_bins = 0;
	motor_detent_map_clear(&g_motor_params->detent_map_cfg);
	motor_detent_map_reset(&g_motor_params->detent_map_state);
	g_motor_params->live.detent_iq_ff_a = 0.0f;

	shell_print(sh, "Detent feedforward cleared");
	return 0;
}
