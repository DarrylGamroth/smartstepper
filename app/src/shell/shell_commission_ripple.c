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
#include "motor/compensation/electrical_ripple_ff.h"

#define MOTOR_COMMISSION_RIPPLE_MAX_VELOCITY_HZ 1.0f
#define MOTOR_COMMISSION_RIPPLE_RECOMMENDED_VELOCITY_HZ 0.20f
#define MOTOR_COMMISSION_RIPPLE_RECOMMENDED_DURATION_MS 5000U
#define MOTOR_COMMISSION_RIPPLE_SAFE_IQ_LIMIT_A 0.12f
#define MOTOR_COMMISSION_RIPPLE_SAFE_GAIN_VELOCITY_HZ 1.0f
#define MOTOR_COMMISSION_RIPPLE_DEFAULT_DECIMATION 1U
#define MOTOR_COMMISSION_RIPPLE_MIN_DECIMATION 1U
#define MOTOR_COMMISSION_RIPPLE_MAX_DECIMATION 128U
#define MOTOR_COMMISSION_RIPPLE_MIN_SAMPLES_PER_BIN 4U
#define MOTOR_COMMISSION_RIPPLE_MAX_FILL_GAP_BINS 2U
#define MOTOR_COMMISSION_RIPPLE_MAX_RUN_MS 60000U
#define MOTOR_COMMISSION_RIPPLE_VELOCITY_BAND_RATIO 0.40f
#define MOTOR_COMMISSION_RIPPLE_MIN_VELOCITY_BAND_HZ 0.03f
#define MOTOR_COMMISSION_RIPPLE_ACCEL_LIMIT_FACTOR 3.0f
#define MOTOR_COMMISSION_RIPPLE_MAX_REJECT_RATIO 0.95f
#define MOTOR_COMMISSION_RIPPLE_MAX_ADJ_STEP_A 0.08f
#define MOTOR_COMMISSION_RIPPLE_VALIDATE_MIN_SAMPLES 10U
#define MOTOR_COMMISSION_RIPPLE_VALIDATE_MATCH_RATIO 1.02f

enum motor_commission_ripple_recommendation {
	MOTOR_COMMISSION_RIPPLE_RECOMMEND_INCONCLUSIVE = 0,
	MOTOR_COMMISSION_RIPPLE_RECOMMEND_DO_NOT_APPLY,
	MOTOR_COMMISSION_RIPPLE_RECOMMEND_APPLY,
};

enum motor_commission_ripple_bin_flag {
	MOTOR_COMMISSION_RIPPLE_BIN_RAW = BIT(0),
	MOTOR_COMMISSION_RIPPLE_BIN_FILLED = BIT(1),
	MOTOR_COMMISSION_RIPPLE_BIN_BOTH_DIR = BIT(2),
};

struct motor_commission_ripple_result {
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
	float32_t velocity_hz;
	uint32_t duration_ms;
	float32_t confidence;
	float32_t reject_ratio;
	float32_t max_adjacent_step_a;
	float32_t forward_reverse_rms_a;
	float32_t min_iq_ff_a;
	float32_t max_iq_ff_a;
	float32_t dc_bias_iq_a;
	float32_t mean_abs_iq_ff_a;
	float32_t rms_iq_ff_a;
	float32_t recommended_limit_a;
	bool validation_complete;
	enum motor_commission_ripple_recommendation validation_recommendation;
	float32_t validation_off_rms_hz;
	float32_t validation_on_rms_hz;
	float32_t validation_improvement_pct;
	uint32_t validation_encoder_errors;
	float32_t table_iq_a[MOTOR_ELECTRICAL_RIPPLE_FF_BINS];
	uint16_t bin_counts[MOTOR_ELECTRICAL_RIPPLE_FF_BINS];
	uint8_t bin_flags[MOTOR_ELECTRICAL_RIPPLE_FF_BINS];
};

struct motor_commission_ripple_velocity_restore {
	float32_t kp_a_per_rad_s;
	float32_t ki_a_per_rad;
	float32_t iq_limit_a;
	float32_t i_term_a;
	bool valid;
};

static struct motor_commission_ripple_result ripple_result;

static const char *motor_commission_ripple_recommendation_str(
	enum motor_commission_ripple_recommendation recommendation)
{
	switch (recommendation) {
	case MOTOR_COMMISSION_RIPPLE_RECOMMEND_APPLY:
		return "RECOMMEND_APPLY";
	case MOTOR_COMMISSION_RIPPLE_RECOMMEND_DO_NOT_APPLY:
		return "DO_NOT_APPLY";
	case MOTOR_COMMISSION_RIPPLE_RECOMMEND_INCONCLUSIVE:
	default:
		return "INCONCLUSIVE";
	}
}

static int motor_commission_ripple_apply_velocity_gains(
	float32_t velocity_hz,
	float32_t iq_limit_a,
	struct motor_commission_ripple_velocity_restore *restore)
{
	if (g_motor_params == NULL || restore == NULL) {
		return -EINVAL;
	}

	iq_limit_a = clampf(iq_limit_a, 0.02f, MOTOR_MAX_CURRENT_A);
	float32_t gain_speed_rad_s =
		2.0f * PI_F32 * fmaxf(velocity_hz, MOTOR_COMMISSION_RIPPLE_SAFE_GAIN_VELOCITY_HZ);
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

static void motor_commission_ripple_restore_velocity_gains(
	const struct motor_commission_ripple_velocity_restore *restore)
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

static void motor_commission_ripple_clear_staged(void)
{
	memset(&ripple_result, 0, sizeof(ripple_result));
}

static void motor_commission_ripple_capture_reset(float32_t kt_nm_per_a, uint32_t decimation)
{
	struct motor_electrical_ripple_capture_ctx *cap =
		&g_motor_params->electrical_ripple_capture;

	memset(cap, 0, sizeof(*cap));
	cap->decimation = decimation;
	cap->kt_nm_per_a = kt_nm_per_a;
	cap->inertia_kgm2 = g_motor_params->inertia_kgm2_active;
	cap->viscous_friction_nm_per_rad_s =
		g_motor_params->viscous_friction_nm_per_rad_s_active;
	cap->coulomb_friction_nm = g_motor_params->coulomb_friction_nm_active;
}

static void motor_commission_ripple_capture_enable(float32_t target_hz)
{
	struct motor_electrical_ripple_capture_ctx *cap =
		&g_motor_params->electrical_ripple_capture;

	cap->decimation_counter = 0U;
	cap->target_speed_rad_s = target_hz * 2.0f * PI_F32;
	float32_t band_hz =
		fmaxf(fabsf(target_hz) * MOTOR_COMMISSION_RIPPLE_VELOCITY_BAND_RATIO,
		      MOTOR_COMMISSION_RIPPLE_MIN_VELOCITY_BAND_HZ);
	cap->velocity_band_rad_s = band_hz * 2.0f * PI_F32;
	cap->accel_limit_rad_s2 =
		MOTOR_COMMISSION_RIPPLE_ACCEL_LIMIT_FACTOR *
		fmaxf(g_motor_params->profile_max_accel_rad_s2, 1.0f);
	cap->iq_saturation_limit_a =
		fmaxf(0.0f, 0.98f * g_motor_params->velocity_cl_iq_limit_A);
	cap->active = true;
}

static void motor_commission_ripple_capture_disable(void)
{
	g_motor_params->electrical_ripple_capture.active = false;
}

static int motor_commission_ripple_collect_pass(float32_t target_hz,
						uint32_t settle_ms,
						uint32_t collect_ms)
{
	motor_commission_set_velocity_target_hz(target_hz);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_commission_wait_ms_or_fault(settle_ms);
	if (ret != 0) {
		return ret;
	}

	motor_commission_ripple_capture_enable(target_hz);
	ret = motor_commission_wait_ms_or_fault(collect_ms);
	motor_commission_ripple_capture_disable();

	return ret;
}

static uint16_t motor_commission_ripple_prev_raw(uint16_t start)
{
	for (uint16_t step = 1U; step < MOTOR_ELECTRICAL_RIPPLE_FF_BINS; step++) {
		uint16_t idx = (uint16_t)((start + MOTOR_ELECTRICAL_RIPPLE_FF_BINS - step) %
					  MOTOR_ELECTRICAL_RIPPLE_FF_BINS);
		if ((ripple_result.bin_flags[idx] & MOTOR_COMMISSION_RIPPLE_BIN_RAW) != 0U) {
			return idx;
		}
	}

	return UINT16_MAX;
}

static uint16_t motor_commission_ripple_next_raw(uint16_t start)
{
	for (uint16_t step = 1U; step < MOTOR_ELECTRICAL_RIPPLE_FF_BINS; step++) {
		uint16_t idx = (uint16_t)((start + step) % MOTOR_ELECTRICAL_RIPPLE_FF_BINS);
		if ((ripple_result.bin_flags[idx] & MOTOR_COMMISSION_RIPPLE_BIN_RAW) != 0U) {
			return idx;
		}
	}

	return UINT16_MAX;
}

static void motor_commission_ripple_fill_short_holes(void)
{
	for (uint16_t i = 0U; i < MOTOR_ELECTRICAL_RIPPLE_FF_BINS; i++) {
		if ((ripple_result.bin_flags[i] & MOTOR_COMMISSION_RIPPLE_BIN_RAW) != 0U) {
			continue;
		}

		uint16_t prev = motor_commission_ripple_prev_raw(i);
		uint16_t next = motor_commission_ripple_next_raw(i);
		if (prev == UINT16_MAX || next == UINT16_MAX) {
			continue;
		}

		uint16_t gap = (uint16_t)((next + MOTOR_ELECTRICAL_RIPPLE_FF_BINS - prev) %
					  MOTOR_ELECTRICAL_RIPPLE_FF_BINS);
		if (gap == 0U || gap > (MOTOR_COMMISSION_RIPPLE_MAX_FILL_GAP_BINS + 1U)) {
			continue;
		}

		uint16_t from_prev = (uint16_t)((i + MOTOR_ELECTRICAL_RIPPLE_FF_BINS - prev) %
						MOTOR_ELECTRICAL_RIPPLE_FF_BINS);
		float32_t frac = (float32_t)from_prev / (float32_t)gap;
		float32_t y0 = ripple_result.table_iq_a[prev];
		float32_t y1 = ripple_result.table_iq_a[next];
		ripple_result.table_iq_a[i] = y0 + frac * (y1 - y0);
		ripple_result.bin_flags[i] |= MOTOR_COMMISSION_RIPPLE_BIN_FILLED;
		ripple_result.filled_bins++;
	}
}

static bool motor_commission_ripple_all_bins_present(void)
{
	for (uint16_t i = 0U; i < MOTOR_ELECTRICAL_RIPPLE_FF_BINS; i++) {
		if (ripple_result.bin_flags[i] == 0U) {
			return false;
		}
	}

	return true;
}

static int motor_commission_ripple_finalize(void)
{
	struct motor_electrical_ripple_capture_ctx *cap =
		&g_motor_params->electrical_ripple_capture;

	ripple_result.sample_count = cap->sample_count;
	ripple_result.rejected_samples = cap->rejected_samples;
	ripple_result.rejected_quality = cap->rejected_quality;
	ripple_result.rejected_velocity = cap->rejected_velocity;
	ripple_result.rejected_accel = cap->rejected_accel;
	ripple_result.rejected_saturation = cap->rejected_saturation;
	ripple_result.accepted_forward = cap->accepted_forward;
	ripple_result.accepted_reverse = cap->accepted_reverse;

	uint32_t raw_populated = 0U;
	uint32_t forward_bins = 0U;
	uint32_t reverse_bins = 0U;
	uint32_t both_bins = 0U;
	uint16_t min_count = UINT16_MAX;
	uint16_t max_count = 0U;
	float32_t sum_fr_sq = 0.0f;

	for (uint16_t i = 0U; i < MOTOR_ELECTRICAL_RIPPLE_FF_BINS; i++) {
		uint16_t fwd_count = cap->bin_counts_forward[i];
		uint16_t rev_count = cap->bin_counts_reverse[i];
		bool has_fwd = fwd_count >= MOTOR_COMMISSION_RIPPLE_MIN_SAMPLES_PER_BIN;
		bool has_rev = rev_count >= MOTOR_COMMISSION_RIPPLE_MIN_SAMPLES_PER_BIN;
		uint16_t total_count = (uint16_t)MIN((uint32_t)fwd_count + (uint32_t)rev_count,
						     UINT16_MAX);
		float32_t iq = 0.0f;

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
			ripple_result.table_iq_a[i] = 0.0f;
			ripple_result.bin_counts[i] = total_count;
			ripple_result.bin_flags[i] = 0U;
			continue;
		}

		ripple_result.table_iq_a[i] = iq;
		ripple_result.bin_counts[i] = total_count;
		ripple_result.bin_flags[i] = MOTOR_COMMISSION_RIPPLE_BIN_RAW;
		if (has_fwd && has_rev) {
			ripple_result.bin_flags[i] |= MOTOR_COMMISSION_RIPPLE_BIN_BOTH_DIR;
		}
		raw_populated++;
		min_count = MIN(min_count, total_count);
		max_count = MAX(max_count, total_count);
	}

	if (raw_populated == 0U) {
		ripple_result.min_bin_count = 0U;
		ripple_result.max_bin_count = 0U;
		return -ENODATA;
	}

	motor_commission_ripple_fill_short_holes();
	if (motor_commission_ripple_all_bins_present()) {
		struct motor_electrical_ripple_ff_config staged_cfg = {
			.enabled = true,
			.table_iq_a = ripple_result.table_iq_a,
			.table_len = MOTOR_ELECTRICAL_RIPPLE_FF_BINS,
			.phase_advance_bins = 0,
			.gain = 1.0f,
			.iq_ff_limit_a = MOTOR_MAX_CURRENT_A,
		};

		(void)motor_electrical_ripple_ff_remove_mean(&staged_cfg,
							     &ripple_result.dc_bias_iq_a);
	}

	float32_t sum_abs = 0.0f;
	float32_t sum_sq = 0.0f;
	float32_t max_abs = 0.0f;
	uint32_t populated = 0U;
	for (uint16_t i = 0U; i < MOTOR_ELECTRICAL_RIPPLE_FF_BINS; i++) {
		if (ripple_result.bin_flags[i] == 0U) {
			continue;
		}

		float32_t iq = ripple_result.table_iq_a[i];
		populated++;
		ripple_result.min_iq_ff_a =
			(populated == 1U) ? iq : fminf(ripple_result.min_iq_ff_a, iq);
		ripple_result.max_iq_ff_a =
			(populated == 1U) ? iq : fmaxf(ripple_result.max_iq_ff_a, iq);
		sum_abs += fabsf(iq);
		sum_sq += iq * iq;
		max_abs = fmaxf(max_abs, fabsf(iq));

		uint16_t next = (uint16_t)((i + 1U) % MOTOR_ELECTRICAL_RIPPLE_FF_BINS);
		if (ripple_result.bin_flags[next] != 0U) {
			ripple_result.max_adjacent_step_a =
				fmaxf(ripple_result.max_adjacent_step_a,
				      fabsf(ripple_result.table_iq_a[i] -
					    ripple_result.table_iq_a[next]));
		}
	}

	ripple_result.populated_bins = (uint16_t)populated;
	ripple_result.raw_populated_bins = (uint16_t)raw_populated;
	ripple_result.forward_bins = (uint16_t)forward_bins;
	ripple_result.reverse_bins = (uint16_t)reverse_bins;
	ripple_result.both_direction_bins = (uint16_t)both_bins;
	ripple_result.min_bin_count = (min_count == UINT16_MAX) ? 0U : min_count;
	ripple_result.max_bin_count = max_count;
	ripple_result.mean_abs_iq_ff_a = sum_abs / (float32_t)populated;
	ripple_result.rms_iq_ff_a = sqrtf(sum_sq / (float32_t)populated);
	ripple_result.forward_reverse_rms_a =
		(both_bins > 0U) ? sqrtf(sum_fr_sq / (float32_t)both_bins) : 0.0f;

	uint32_t total_candidate_samples =
		ripple_result.sample_count + ripple_result.rejected_samples;
	ripple_result.reject_ratio =
		(total_candidate_samples > 0U) ?
			((float32_t)ripple_result.rejected_samples /
			 (float32_t)total_candidate_samples) :
			0.0f;
	ripple_result.recommended_limit_a =
		fminf(fmaxf(1.25f * max_abs, 0.0f), g_motor_params->velocity_cl_iq_limit_A);
	float32_t coverage = (float32_t)ripple_result.populated_bins /
			     (float32_t)MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
	float32_t both_dir_score = (float32_t)ripple_result.both_direction_bins /
				   (float32_t)MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
	float32_t reject_penalty = clampf(ripple_result.reject_ratio /
					  MOTOR_COMMISSION_RIPPLE_MAX_REJECT_RATIO,
					  0.0f, 1.0f);
	ripple_result.confidence =
		clampf((0.70f * coverage) + (0.30f * both_dir_score) -
		       (0.20f * reject_penalty),
		       0.0f, 1.0f);
	ripple_result.raw_valid =
		ripple_result.raw_populated_bins == MOTOR_ELECTRICAL_RIPPLE_FF_BINS &&
		ripple_result.sample_count >= ripple_result.raw_populated_bins &&
		ripple_result.recommended_limit_a > 0.0f;
	ripple_result.fill_valid =
		ripple_result.raw_valid &&
		ripple_result.populated_bins == MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
	ripple_result.apply_valid =
		ripple_result.fill_valid &&
		ripple_result.reject_ratio <= MOTOR_COMMISSION_RIPPLE_MAX_REJECT_RATIO &&
		ripple_result.max_adjacent_step_a <= MOTOR_COMMISSION_RIPPLE_MAX_ADJ_STEP_A;
	ripple_result.valid = ripple_result.apply_valid;

	return ripple_result.valid ? 0 : -ERANGE;
}

int cmd_motor_commission_ripple_run(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3 || argc > 5) {
		shell_error(sh,
			    "Usage: motor commission ripple run <velocity_hz> <duration_ms> [decimation] [iq_limit_a]");
		shell_error(sh, "Recommended start: motor commission ripple run %.2f %.0f 1 %.2f",
			    (double)MOTOR_COMMISSION_RIPPLE_RECOMMENDED_VELOCITY_HZ,
			    (double)MOTOR_COMMISSION_RIPPLE_RECOMMENDED_DURATION_MS,
			    (double)MOTOR_COMMISSION_RIPPLE_SAFE_IQ_LIMIT_A);
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		return -ENODEV;
	}
	if (!g_motor_params->calibration.complete ||
	    !g_motor_params->calibration.encoder_mapping_complete) {
		shell_error(sh,
			    "Run boot/generated-sweep commissioning first: cal_complete=%u enc_mapped=%u",
			    g_motor_params->calibration.complete ? 1U : 0U,
			    g_motor_params->calibration.encoder_mapping_complete ? 1U : 0U);
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' first");
		return -EACCES;
	}

	float32_t velocity_hz;
	uint32_t duration_ms;
	uint32_t decimation = MOTOR_COMMISSION_RIPPLE_DEFAULT_DECIMATION;
	float32_t capture_iq_limit_a = MOTOR_COMMISSION_RIPPLE_SAFE_IQ_LIMIT_A;
	if (!shell_parse_finite_float(argv[1], &velocity_hz) ||
	    !shell_parse_u32(argv[2], &duration_ms) ||
	    (argc >= 4 && !shell_parse_u32(argv[3], &decimation)) ||
	    (argc >= 5 && !shell_parse_finite_float(argv[4], &capture_iq_limit_a))) {
		shell_error(sh, "Invalid numeric argument");
		return -EINVAL;
	}
	if (velocity_hz <= 0.0f || duration_ms == 0U) {
		shell_error(sh, "velocity_hz and duration_ms must be positive");
		return -EINVAL;
	}
	if (velocity_hz > MOTOR_COMMISSION_RIPPLE_MAX_VELOCITY_HZ) {
		shell_error(sh, "ripple capture velocity is limited to %.3f Hz",
			    (double)MOTOR_COMMISSION_RIPPLE_MAX_VELOCITY_HZ);
		return -ERANGE;
	}
	if (capture_iq_limit_a <= 0.0f || capture_iq_limit_a > MOTOR_MAX_CURRENT_A) {
		shell_error(sh, "iq_limit_a must be within (0, %.3f] A",
			    (double)MOTOR_MAX_CURRENT_A);
		return -ERANGE;
	}
	float32_t max_hz = g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	if (velocity_hz > max_hz) {
		shell_error(sh, "velocity_hz exceeds profile max %.3f Hz", (double)max_hz);
		return -ERANGE;
	}
	decimation = CLAMP(decimation,
			    MOTOR_COMMISSION_RIPPLE_MIN_DECIMATION,
			    MOTOR_COMMISSION_RIPPLE_MAX_DECIMATION);

	if (duration_ms > MOTOR_COMMISSION_RIPPLE_MAX_RUN_MS) {
		shell_error(sh, "Capture duration per direction invalid or above %u ms",
			    MOTOR_COMMISSION_RIPPLE_MAX_RUN_MS);
		return -ERANGE;
	}
	uint32_t collect_ms = duration_ms;
	float32_t accel_hz_s = g_motor_params->profile_max_accel_rad_s2 / (2.0f * PI_F32);
	uint32_t settle_ms = 500U;
	if (isfinite(accel_hz_s) && accel_hz_s > 0.0f) {
		settle_ms += (uint32_t)(((velocity_hz / accel_hz_s) * 1000.0f) + 0.5f);
	}
	settle_ms = CLAMP(settle_ms, 500U, 10000U);

	float32_t kt = motor_torque_gain_resolve_active(g_motor_params);
	if (!isfinite(kt) || kt <= 0.0f) {
		shell_error(sh, "Active torque gain is invalid; run commissioning first");
		return -ERANGE;
	}

	bool saved_ripple_enable = g_motor_params->electrical_ripple_ff_cfg.enabled;
	bool saved_detent_enable = g_motor_params->detent_map_cfg.enabled;
	bool saved_dob_enable = g_motor_params->velocity_dob_cfg.enabled;
	uint8_t saved_outer_loop = g_motor_params->outer_loop_mode;
	struct motor_commission_ripple_velocity_restore velocity_restore = {0};

	motor_commission_ripple_clear_staged();
	motor_commission_ripple_capture_reset(kt, decimation);
	int ret = motor_commission_ripple_apply_velocity_gains(velocity_hz, capture_iq_limit_a,
							       &velocity_restore);
	if (ret != 0) {
		motor_commission_ripple_restore_velocity_gains(&velocity_restore);
		shell_error(sh, "Failed to apply ripple velocity gains (err %d)", ret);
		return ret;
	}

	g_motor_params->electrical_ripple_ff_cfg.enabled = false;
	g_motor_params->detent_map_cfg.enabled = false;
	g_motor_params->velocity_dob_cfg.enabled = false;
	g_motor_params->outer_loop_mode = MOTOR_OUTER_LOOP_MODE_PI;
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);
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
		    "Electrical ripple capture: velocity=%.3f Hz duration=%u ms/dir decimation=%u settle=%u ms",
		    (double)velocity_hz, duration_ms, decimation, settle_ms);

	ret = motor_commission_ripple_collect_pass(velocity_hz, settle_ms, collect_ms);
	if (ret == 0) {
		ret = motor_commission_ripple_collect_pass(-velocity_hz, settle_ms, collect_ms);
	}

	motor_commission_set_velocity_target_hz(0.0f);
	(void)motor_commission_wait_ms_or_fault(settle_ms);

	if (ret == 0) {
		ripple_result.velocity_hz = velocity_hz;
		ripple_result.duration_ms = duration_ms;
		ripple_result.decimation = decimation;
		ripple_result.run_ms = 2U * collect_ms;
		ret = motor_commission_ripple_finalize();
	}

restore_runtime:
	motor_commission_ripple_capture_disable();
	motor_commission_set_velocity_target_hz(0.0f);
	g_motor_params->electrical_ripple_ff_cfg.enabled = saved_ripple_enable;
	g_motor_params->detent_map_cfg.enabled = saved_detent_enable;
	g_motor_params->velocity_dob_cfg.enabled = saved_dob_enable;
	g_motor_params->outer_loop_mode = saved_outer_loop;
	motor_commission_ripple_restore_velocity_gains(&velocity_restore);
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);
	motor_dob_reset(&g_motor_params->velocity_dob_state,
			g_motor_params->live.velocity_rad_s);
	motor_command_feed_watchdog(g_motor_params);

	if (ret != 0) {
		ripple_result.sample_count =
			g_motor_params->electrical_ripple_capture.sample_count;
		ripple_result.rejected_samples =
			g_motor_params->electrical_ripple_capture.rejected_samples;
		ripple_result.rejected_quality =
			g_motor_params->electrical_ripple_capture.rejected_quality;
		ripple_result.rejected_velocity =
			g_motor_params->electrical_ripple_capture.rejected_velocity;
		ripple_result.rejected_accel =
			g_motor_params->electrical_ripple_capture.rejected_accel;
		ripple_result.rejected_saturation =
			g_motor_params->electrical_ripple_capture.rejected_saturation;
		shell_error(sh,
			    "Electrical ripple capture failed (err %d): samples=%u rejected=%u bins=%u/%u raw=%u/%u conf=%.2f",
			    ret, ripple_result.sample_count, ripple_result.rejected_samples,
			    ripple_result.populated_bins, MOTOR_ELECTRICAL_RIPPLE_FF_BINS,
			    ripple_result.raw_populated_bins, MOTOR_ELECTRICAL_RIPPLE_FF_BINS,
			    (double)ripple_result.confidence);
		shell_error(sh, "  rejects: quality=%u velocity=%u accel=%u saturation=%u",
			    ripple_result.rejected_quality,
			    ripple_result.rejected_velocity,
			    ripple_result.rejected_accel,
			    ripple_result.rejected_saturation);
		return ret;
	}

	shell_print(sh,
		    "Electrical ripple staged: samples=%u rejected=%u bins=%u/%u raw=%u filled=%u minN=%u maxN=%u",
		    ripple_result.sample_count, ripple_result.rejected_samples,
		    ripple_result.populated_bins, MOTOR_ELECTRICAL_RIPPLE_FF_BINS,
		    ripple_result.raw_populated_bins, ripple_result.filled_bins,
		    ripple_result.min_bin_count, ripple_result.max_bin_count);
	shell_print(sh,
		    "  iq_ff: min=%.5f max=%.5f bias_removed=%.5f mean_abs=%.5f rms=%.5f rec_limit=%.5f A",
		    (double)ripple_result.min_iq_ff_a,
		    (double)ripple_result.max_iq_ff_a,
		    (double)ripple_result.dc_bias_iq_a,
		    (double)ripple_result.mean_abs_iq_ff_a,
		    (double)ripple_result.rms_iq_ff_a,
		    (double)ripple_result.recommended_limit_a);
	shell_print(sh,
		    "  quality: raw=%s fill=%s apply=%s conf=%.2f reject_ratio=%.2f fr_rms=%.5f max_step=%.5f A",
		    ripple_result.raw_valid ? "PASS" : "FAIL",
		    ripple_result.fill_valid ? "PASS" : "FAIL",
		    ripple_result.apply_valid ? "PASS" : "FAIL",
		    (double)ripple_result.confidence,
		    (double)ripple_result.reject_ratio,
		    (double)ripple_result.forward_reverse_rms_a,
		    (double)ripple_result.max_adjacent_step_a);
	shell_print(sh,
		    "Run 'motor commission ripple validate <velocity_hz> <duration_ms>' before applying; apply only if validation improves or matches baseline.");

	return 0;
}

int cmd_motor_commission_ripple_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (g_motor_params == NULL) {
		return -ENODEV;
	}

	shell_print(sh, "Electrical ripple FF runtime:");
	shell_print(sh,
		    "  enabled=%u gain=%.4f limit=%.5f A phase=%d bins=%u live=%.5f A",
		    g_motor_params->electrical_ripple_ff_cfg.enabled ? 1U : 0U,
		    (double)g_motor_params->electrical_ripple_ff_cfg.gain,
		    (double)g_motor_params->electrical_ripple_ff_cfg.iq_ff_limit_a,
		    g_motor_params->electrical_ripple_ff_cfg.phase_advance_bins,
		    g_motor_params->electrical_ripple_ff_cfg.table_len,
		    (double)g_motor_params->live.electrical_ripple_iq_ff_a);
	shell_print(sh, "Staged electrical ripple table:");
	shell_print(sh,
		    "  valid=%s raw=%s fill=%s apply=%s conf=%.2f samples=%u rejected=%u reject_ratio=%.2f",
		    ripple_result.valid ? "yes" : "no",
		    ripple_result.raw_valid ? "PASS" : "FAIL",
		    ripple_result.fill_valid ? "PASS" : "FAIL",
		    ripple_result.apply_valid ? "PASS" : "FAIL",
		    (double)ripple_result.confidence,
		    ripple_result.sample_count,
		    ripple_result.rejected_samples,
		    (double)ripple_result.reject_ratio);
	shell_print(sh,
		    "  bins=%u/%u raw=%u fwd=%u rev=%u both=%u minN=%u maxN=%u",
		    ripple_result.populated_bins, MOTOR_ELECTRICAL_RIPPLE_FF_BINS,
		    ripple_result.raw_populated_bins,
		    ripple_result.forward_bins,
		    ripple_result.reverse_bins,
		    ripple_result.both_direction_bins,
		    ripple_result.min_bin_count,
		    ripple_result.max_bin_count);
	shell_print(sh,
		    "  rejects: quality=%u velocity=%u accel=%u saturation=%u",
		    ripple_result.rejected_quality,
		    ripple_result.rejected_velocity,
		    ripple_result.rejected_accel,
		    ripple_result.rejected_saturation);
	shell_print(sh,
		    "  iq_ff min=%.5f max=%.5f mean_abs=%.5f rms=%.5f rec_limit=%.5f A",
		    (double)ripple_result.min_iq_ff_a,
		    (double)ripple_result.max_iq_ff_a,
		    (double)ripple_result.mean_abs_iq_ff_a,
		    (double)ripple_result.rms_iq_ff_a,
		    (double)ripple_result.recommended_limit_a);
	shell_print(sh,
		    "  validation=%s off_rms=%.5f Hz on_rms=%.5f Hz improvement=%.1f%% errors=%u",
		    ripple_result.validation_complete ?
			    motor_commission_ripple_recommendation_str(
				    ripple_result.validation_recommendation) :
			    "not-run",
		    (double)ripple_result.validation_off_rms_hz,
		    (double)ripple_result.validation_on_rms_hz,
		    (double)ripple_result.validation_improvement_pct,
		    ripple_result.validation_encoder_errors);

	return 0;
}

int cmd_motor_commission_ripple_apply(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 4) {
		shell_error(sh, "Usage: motor commission ripple apply [enable] [gain] [limit_a]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		return -ENODEV;
	}

	bool enable = ripple_result.valid;
	float32_t gain = 1.0f;
	float32_t limit = ripple_result.recommended_limit_a;
	if (argc >= 2) {
		if (!shell_parse_bool01(argv[1], &enable)) {
			shell_error(sh, "enable must be 0 or 1");
			return -EINVAL;
		}
	}
	if (argc >= 3 && !shell_parse_finite_float(argv[2], &gain)) {
		shell_error(sh, "Invalid gain");
		return -EINVAL;
	}
	if (argc >= 4 && !shell_parse_finite_float(argv[3], &limit)) {
		shell_error(sh, "Invalid limit_a");
		return -EINVAL;
	}
	if (gain < 0.0f || limit < 0.0f || limit > MOTOR_MAX_CURRENT_A) {
		shell_error(sh, "gain must be >= 0 and limit within [0, %.3f] A",
			    (double)MOTOR_MAX_CURRENT_A);
		return -ERANGE;
	}
	if (enable && !ripple_result.valid) {
		shell_error(sh, "No valid staged ripple table; run/status first");
		return -EINVAL;
	}
	if (enable && ripple_result.validation_complete &&
	    ripple_result.validation_recommendation != MOTOR_COMMISSION_RIPPLE_RECOMMEND_APPLY) {
		shell_error(sh,
			    "Validation recommendation is %s; apply disabled with 'motor commission ripple apply 0' or recapture",
			    motor_commission_ripple_recommendation_str(
				    ripple_result.validation_recommendation));
		return -ERANGE;
	}

	g_motor_params->electrical_ripple_ff_cfg.table_iq_a =
		g_motor_params->electrical_ripple_iq_table_a;
	g_motor_params->electrical_ripple_ff_cfg.table_len = MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
	g_motor_params->electrical_ripple_ff_cfg.phase_advance_bins = 0;
	g_motor_params->electrical_ripple_ff_cfg.gain = gain;
	g_motor_params->electrical_ripple_ff_cfg.iq_ff_limit_a = limit;
	if (enable) {
		memcpy(g_motor_params->electrical_ripple_iq_table_a,
		       ripple_result.table_iq_a,
		       sizeof(g_motor_params->electrical_ripple_iq_table_a));
	}
	g_motor_params->electrical_ripple_ff_cfg.enabled = enable;
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);

	shell_print(sh, "Electrical ripple FF %s: gain=%.4f limit=%.5f A",
		    enable ? "enabled" : "disabled",
		    (double)gain,
		    (double)limit);

	return 0;
}

struct motor_commission_ripple_validation_metrics {
	float32_t rms_hz;
	uint32_t samples;
	uint32_t encoder_errors;
};

static int motor_commission_ripple_measure_velocity_error(float32_t target_hz,
							  uint32_t duration_ms,
							  struct motor_commission_ripple_validation_metrics *out)
{
	memset(out, 0, sizeof(*out));
	motor_commission_set_velocity_target_hz(target_hz);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_commission_wait_ms_or_fault(500U);
	if (ret != 0) {
		return ret;
	}

	uint32_t start = k_uptime_get_32();
	float32_t sum_sq = 0.0f;
	while ((uint32_t)(k_uptime_get_32() - start) < duration_ms) {
		ret = motor_commission_wait_ms_or_fault(20U);
		if (ret != 0) {
			return ret;
		}
		float32_t measured_hz = g_motor_params->live.velocity_filtered_rad_s /
					(2.0f * PI_F32);
		float32_t err_hz = target_hz - measured_hz;
		sum_sq += err_hz * err_hz;
		out->samples++;
		out->encoder_errors = g_motor_params->encoder_error_count;
	}

	if (out->samples < MOTOR_COMMISSION_RIPPLE_VALIDATE_MIN_SAMPLES) {
		return -ENODATA;
	}
	out->rms_hz = sqrtf(sum_sq / (float32_t)out->samples);
	return 0;
}

int cmd_motor_commission_ripple_validate(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor commission ripple validate <velocity_hz> <duration_ms>");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		return -ENODEV;
	}
	if (!ripple_result.valid) {
		shell_error(sh, "No valid staged ripple table to validate");
		return -EINVAL;
	}
	if (!g_motor_params->calibration.complete ||
	    !g_motor_params->calibration.encoder_mapping_complete) {
		shell_error(sh,
			    "Run boot/generated-sweep commissioning first: cal_complete=%u enc_mapped=%u",
			    g_motor_params->calibration.complete ? 1U : 0U,
			    g_motor_params->calibration.encoder_mapping_complete ? 1U : 0U);
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' first");
		return -EACCES;
	}

	float32_t velocity_hz;
	uint32_t duration_ms;
	if (!shell_parse_finite_float(argv[1], &velocity_hz) ||
	    !shell_parse_u32(argv[2], &duration_ms)) {
		shell_error(sh, "Invalid numeric argument");
		return -EINVAL;
	}
	if (velocity_hz <= 0.0f || duration_ms < 500U || duration_ms > 10000U) {
		shell_error(sh, "velocity must be positive and duration must be 500..10000 ms");
		return -ERANGE;
	}

	bool saved_ripple_enable = g_motor_params->electrical_ripple_ff_cfg.enabled;
	float32_t saved_gain = g_motor_params->electrical_ripple_ff_cfg.gain;
	float32_t saved_limit = g_motor_params->electrical_ripple_ff_cfg.iq_ff_limit_a;
	int16_t saved_phase = g_motor_params->electrical_ripple_ff_cfg.phase_advance_bins;
	float32_t saved_table[MOTOR_ELECTRICAL_RIPPLE_FF_BINS];
	memcpy(saved_table, g_motor_params->electrical_ripple_iq_table_a, sizeof(saved_table));

	bool saved_detent_enable = g_motor_params->detent_map_cfg.enabled;
	bool saved_dob_enable = g_motor_params->velocity_dob_cfg.enabled;
	uint8_t saved_outer_loop = g_motor_params->outer_loop_mode;

	g_motor_params->detent_map_cfg.enabled = false;
	g_motor_params->velocity_dob_cfg.enabled = false;
	g_motor_params->outer_loop_mode = MOTOR_OUTER_LOOP_MODE_PI;

	int ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret == 0) {
		ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
						     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	}
	if (ret != 0) {
		goto restore_runtime;
	}

	g_motor_params->electrical_ripple_ff_cfg.enabled = false;
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);
	struct motor_commission_ripple_validation_metrics off = {0};
	ret = motor_commission_ripple_measure_velocity_error(velocity_hz, duration_ms, &off);
	if (ret != 0) {
		goto restore_runtime;
	}

	memcpy(g_motor_params->electrical_ripple_iq_table_a,
	       ripple_result.table_iq_a,
	       sizeof(g_motor_params->electrical_ripple_iq_table_a));
	g_motor_params->electrical_ripple_ff_cfg.table_iq_a =
		g_motor_params->electrical_ripple_iq_table_a;
	g_motor_params->electrical_ripple_ff_cfg.table_len = MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
	g_motor_params->electrical_ripple_ff_cfg.phase_advance_bins = 0;
	g_motor_params->electrical_ripple_ff_cfg.gain = 1.0f;
	g_motor_params->electrical_ripple_ff_cfg.iq_ff_limit_a =
		ripple_result.recommended_limit_a;
	g_motor_params->electrical_ripple_ff_cfg.enabled = true;
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);
	struct motor_commission_ripple_validation_metrics on = {0};
	ret = motor_commission_ripple_measure_velocity_error(velocity_hz, duration_ms, &on);
	if (ret != 0) {
		goto restore_runtime;
	}

	ripple_result.validation_complete = true;
	ripple_result.validation_off_rms_hz = off.rms_hz;
	ripple_result.validation_on_rms_hz = on.rms_hz;
	ripple_result.validation_encoder_errors = on.encoder_errors - off.encoder_errors;
	if (off.rms_hz > 0.0f) {
		ripple_result.validation_improvement_pct =
			100.0f * (off.rms_hz - on.rms_hz) / off.rms_hz;
	}
	ripple_result.validation_recommendation =
		(on.rms_hz <= off.rms_hz * MOTOR_COMMISSION_RIPPLE_VALIDATE_MATCH_RATIO) ?
			MOTOR_COMMISSION_RIPPLE_RECOMMEND_APPLY :
			MOTOR_COMMISSION_RIPPLE_RECOMMEND_DO_NOT_APPLY;

	shell_print(sh,
		    "Electrical ripple validation: off_rms=%.5f Hz on_rms=%.5f Hz improvement=%.1f%% recommendation=%s",
		    (double)off.rms_hz,
		    (double)on.rms_hz,
		    (double)ripple_result.validation_improvement_pct,
		    motor_commission_ripple_recommendation_str(
			    ripple_result.validation_recommendation));

restore_runtime:
	motor_commission_set_velocity_target_hz(0.0f);
	(void)motor_commission_wait_ms_or_fault(500U);
	memcpy(g_motor_params->electrical_ripple_iq_table_a, saved_table, sizeof(saved_table));
	g_motor_params->electrical_ripple_ff_cfg.table_iq_a =
		g_motor_params->electrical_ripple_iq_table_a;
	g_motor_params->electrical_ripple_ff_cfg.table_len = MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
	g_motor_params->electrical_ripple_ff_cfg.phase_advance_bins = saved_phase;
	g_motor_params->electrical_ripple_ff_cfg.gain = saved_gain;
	g_motor_params->electrical_ripple_ff_cfg.iq_ff_limit_a = saved_limit;
	g_motor_params->electrical_ripple_ff_cfg.enabled = saved_ripple_enable;
	g_motor_params->detent_map_cfg.enabled = saved_detent_enable;
	g_motor_params->velocity_dob_cfg.enabled = saved_dob_enable;
	g_motor_params->outer_loop_mode = saved_outer_loop;
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);
	motor_dob_reset(&g_motor_params->velocity_dob_state,
			g_motor_params->live.velocity_rad_s);
	motor_command_feed_watchdog(g_motor_params);

	return ret;
}

int cmd_motor_commission_ripple_dump(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t start = 0U;
	uint32_t count = MOTOR_ELECTRICAL_RIPPLE_FF_BINS;

	if (argc > 3) {
		shell_error(sh, "Usage: motor commission ripple dump [start_bin] [count]");
		return -EINVAL;
	}
	if (argc >= 2 && !shell_parse_u32(argv[1], &start)) {
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &count)) {
		return -EINVAL;
	}
	if (start >= MOTOR_ELECTRICAL_RIPPLE_FF_BINS || count == 0U) {
		shell_error(sh, "start must be < %u and count must be nonzero",
			    MOTOR_ELECTRICAL_RIPPLE_FF_BINS);
		return -ERANGE;
	}
	count = MIN(count, MOTOR_ELECTRICAL_RIPPLE_FF_BINS);

	shell_print(sh, "bin,iq_ff_a,count,flags");
	for (uint32_t n = 0U; n < count; n++) {
		uint32_t idx = (start + n) % MOTOR_ELECTRICAL_RIPPLE_FF_BINS;
		shell_print(sh, "%u,%.7f,%u,0x%02x",
			    idx,
			    (double)ripple_result.table_iq_a[idx],
			    ripple_result.bin_counts[idx],
			    ripple_result.bin_flags[idx]);
	}

	return 0;
}

int cmd_motor_commission_ripple_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (g_motor_params == NULL) {
		return -ENODEV;
	}

	motor_commission_ripple_clear_staged();
	motor_commission_ripple_capture_reset(0.0f,
					      MOTOR_COMMISSION_RIPPLE_DEFAULT_DECIMATION);
	motor_electrical_ripple_ff_clear(&g_motor_params->electrical_ripple_ff_cfg);
	motor_electrical_ripple_ff_reset(&g_motor_params->electrical_ripple_ff_state);
	g_motor_params->electrical_ripple_ff_cfg.enabled = false;
	g_motor_params->live.electrical_ripple_iq_ff_a = 0.0f;
	shell_print(sh, "Electrical ripple feedforward cleared");

	return 0;
}
