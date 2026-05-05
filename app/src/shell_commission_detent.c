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
#define MOTOR_COMMISSION_DETENT_SAFE_IQ_LIMIT_A 0.15f
#define MOTOR_COMMISSION_DETENT_SAFE_GAIN_SPEED_HZ 1.0f
#define MOTOR_COMMISSION_DETENT_DEFAULT_DECIMATION 1U
#define MOTOR_COMMISSION_DETENT_MIN_DECIMATION 1U
#define MOTOR_COMMISSION_DETENT_MAX_DECIMATION 128U
#define MOTOR_COMMISSION_DETENT_MIN_BIN_COVERAGE_MPU 650U
#define MOTOR_COMMISSION_DETENT_MIN_SAMPLES_PER_BIN 2U
#define MOTOR_COMMISSION_DETENT_MAX_RUN_MS 60000U
struct motor_commission_detent_result {
	bool valid;
	uint32_t sample_count;
	uint32_t rejected_samples;
	uint16_t populated_bins;
	uint16_t min_bin_count;
	uint16_t max_bin_count;
	uint32_t run_ms;
	uint32_t decimation;
	float32_t speed_hz;
	float32_t cycles;
	float32_t min_iq_ff_a;
	float32_t max_iq_ff_a;
	float32_t mean_abs_iq_ff_a;
	float32_t rms_iq_ff_a;
	float32_t recommended_limit_a;
	float32_t table_iq_a[MOTOR_DETENT_MAP_BINS];
	uint16_t bin_counts[MOTOR_DETENT_MAP_BINS];
};

static struct motor_commission_detent_result detent_result;

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
	memset(cap->bin_counts, 0, sizeof(cap->bin_counts));
	cap->decimation = decimation;
	cap->decimation_counter = 0U;
	cap->sample_count = 0U;
	cap->rejected_samples = 0U;
	cap->kt_nm_per_a = kt_nm_per_a;
	cap->inertia_kgm2 = g_motor_params->inertia_kgm2_active;
	cap->viscous_friction_nm_per_rad_s =
		g_motor_params->viscous_friction_nm_per_rad_s_active;
	cap->coulomb_friction_nm = g_motor_params->coulomb_friction_nm_active;
}

static void motor_commission_detent_capture_enable(void)
{
	struct motor_detent_capture_ctx *cap = &g_motor_params->detent_capture;

	cap->decimation_counter = 0U;
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

	motor_commission_detent_capture_enable();
	ret = motor_commission_wait_ms_or_fault(collect_ms);
	motor_commission_detent_capture_disable();

	return ret;
}

static int motor_commission_detent_finalize(const float32_t *sum_iq,
					    const uint16_t *count)
{
	if (sum_iq == NULL || count == NULL) {
		return -EINVAL;
	}

	detent_result.sample_count = g_motor_params->detent_capture.sample_count;
	detent_result.rejected_samples = g_motor_params->detent_capture.rejected_samples;

	float32_t sum_abs = 0.0f;
	float32_t sum_sq = 0.0f;
	float32_t max_abs = 0.0f;
	uint32_t populated = 0U;
	uint16_t min_count = UINT16_MAX;
	uint16_t max_count = 0U;

	for (uint16_t i = 0U; i < MOTOR_DETENT_MAP_BINS; i++) {
		if (count[i] >= MOTOR_COMMISSION_DETENT_MIN_SAMPLES_PER_BIN) {
			float32_t iq = sum_iq[i] / (float32_t)count[i];

			detent_result.table_iq_a[i] = iq;
			detent_result.bin_counts[i] = count[i];
			populated++;
			min_count = MIN(min_count, count[i]);
			max_count = MAX(max_count, count[i]);
			detent_result.min_iq_ff_a = (populated == 1U) ?
							    iq :
							    fminf(detent_result.min_iq_ff_a, iq);
			detent_result.max_iq_ff_a = (populated == 1U) ?
							    iq :
							    fmaxf(detent_result.max_iq_ff_a, iq);
			sum_abs += fabsf(iq);
			sum_sq += iq * iq;
			max_abs = fmaxf(max_abs, fabsf(iq));
		} else {
			detent_result.table_iq_a[i] = 0.0f;
			detent_result.bin_counts[i] = count[i];
		}
	}

	if (populated == 0U) {
		detent_result.min_bin_count = 0U;
		detent_result.max_bin_count = 0U;
		return -ENODATA;
	}

	float32_t coverage_mpu =
		1000.0f * (float32_t)populated / (float32_t)MOTOR_DETENT_MAP_BINS;
	detent_result.populated_bins = (uint16_t)populated;
	detent_result.min_bin_count = min_count;
	detent_result.max_bin_count = max_count;
	detent_result.mean_abs_iq_ff_a = sum_abs / (float32_t)populated;
	detent_result.rms_iq_ff_a = sqrtf(sum_sq / (float32_t)populated);
	detent_result.recommended_limit_a =
		fminf(fmaxf(1.25f * max_abs, 0.0f), g_motor_params->velocity_cl_iq_limit_A);
	detent_result.valid =
		detent_result.sample_count >= populated &&
		coverage_mpu >= (float32_t)MOTOR_COMMISSION_DETENT_MIN_BIN_COVERAGE_MPU &&
		detent_result.recommended_limit_a > 0.0f;

	return detent_result.valid ? 0 : -ERANGE;
}

int cmd_motor_commission_detent_run(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3 || argc > 5) {
		shell_error(sh,
			    "Usage: motor commission detent run <mech_hz> <cycles> [decimation] [iq_limit_a]");
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

	ret = motor_api_request_online();
	if (ret != 0) {
		goto restore_runtime;
	}
	ret = motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
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
		ret = motor_commission_detent_finalize(g_motor_params->detent_capture.sum_iq_a,
						       g_motor_params->detent_capture.bin_counts);
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
			    "Detent capture failed (err %d): samples=%u rejected=%u bins=%u/%u",
			    ret, detent_result.sample_count, detent_result.rejected_samples,
			    detent_result.populated_bins, MOTOR_DETENT_MAP_BINS);
		return ret;
	}

	shell_print(sh,
		    "Detent capture staged: samples=%u rejected=%u bins=%u/%u minN=%u maxN=%u",
		    detent_result.sample_count, detent_result.rejected_samples,
		    detent_result.populated_bins, MOTOR_DETENT_MAP_BINS,
		    detent_result.min_bin_count, detent_result.max_bin_count);
	shell_print(sh,
		    "  iq_ff: min=%.5f max=%.5f mean_abs=%.5f rms=%.5f rec_limit=%.5f A",
		    (double)detent_result.min_iq_ff_a,
		    (double)detent_result.max_iq_ff_a,
		    (double)detent_result.mean_abs_iq_ff_a,
		    (double)detent_result.rms_iq_ff_a,
		    (double)detent_result.recommended_limit_a);
	shell_print(sh, "Run 'motor commission detent apply 1' to apply and enable.");
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
	shell_print(sh, "  Staged:    %s", detent_result.valid ? "YES" : "NO");
	shell_print(sh, "  Capture:   speed=%.3f Hz cycles=%.2f run=%u ms decimation=%u",
		    (double)detent_result.speed_hz,
		    (double)detent_result.cycles,
		    detent_result.run_ms,
		    detent_result.decimation);
	shell_print(sh, "  Samples:   accepted=%u rejected=%u bins=%u/%u minN=%u maxN=%u",
		    detent_result.sample_count,
		    detent_result.rejected_samples,
		    detent_result.populated_bins,
		    MOTOR_DETENT_MAP_BINS,
		    detent_result.min_bin_count,
		    detent_result.max_bin_count);
	shell_print(sh, "  Iq FF:     min=%.5f max=%.5f mean_abs=%.5f rms=%.5f rec_limit=%.5f A",
		    (double)detent_result.min_iq_ff_a,
		    (double)detent_result.max_iq_ff_a,
		    (double)detent_result.mean_abs_iq_ff_a,
		    (double)detent_result.rms_iq_ff_a,
		    (double)detent_result.recommended_limit_a);
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
