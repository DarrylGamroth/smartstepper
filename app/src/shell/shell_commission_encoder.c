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
#include "shell_commands_state.h"
#include "shell_commission_internal.h"
#include "motor_control_api.h"
#include "motor_current_slew.h"
#include "motor_encoder_acquisition.h"
#include "motor_state_utils.h"
#include "shell_parse.h"
#include "motor/math/math_constants.h"
#include "motor/math/angle_wrap.h"
#include "motor/calibration/encoder_map_detect.h"
#include "motor/observers/angle_observer.h"

#define MOTOR_COMMISSION_AUTO_POLL_MS 10U
#define MOTOR_COMMISSION_ENCODER_MAX_SAMPLES 512U
#define MOTOR_COMMISSION_ENCODER_MIN_SAMPLE_MS 5U
#define MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES 4U
#define MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_ENCODER_MAP_CAL_TIMEOUT_MS 8000U
#define MOTOR_COMMISSION_ENCODER_MAP_DEFAULT_CURRENT_A 0.150f
#define MOTOR_COMMISSION_ENCODER_MAP_DEFAULT_MECH_HZ 0.100f
#define MOTOR_COMMISSION_ENCODER_MAP_DEFAULT_CYCLES 1.0f
#define MOTOR_COMMISSION_ENCODER_PRE_SWEEP_SETTLE_MS 500U
static struct motor_encoder_map_detect_sample encoder_detect_samples[
	MOTOR_COMMISSION_ENCODER_MAX_SAMPLES];
static struct motor_encoder_map_detect_result encoder_detect_result;
static bool encoder_detect_result_valid;
static uint32_t encoder_detect_duration_ms;
static uint32_t encoder_detect_sample_period_ms;
static float32_t encoder_detect_current_a;



struct motor_commission_encoder_sweep_config {
	float32_t current_a;
	float32_t mech_hz;
	float32_t cycles;
};

int motor_post_mode_change(enum motor_state target_mode)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_MODE_CHANGE,
		.target_mode = target_mode,
	};
	int ret = motor_api_post_event(&evt);

	return ret;
}

enum motor_state motor_commission_set_requested_online_mode(enum motor_state mode)
{
	enum motor_state previous = MOTOR_STATE_ONLINE_VELOCITY_GENERATED;

	if (g_motor_params != NULL) {
		previous = (enum motor_state)g_motor_params->calibration.requested_online_mode;
		g_motor_params->calibration.requested_online_mode = (uint8_t)mode;
	}

	return previous;
}

void motor_commission_restore_requested_online_mode(enum motor_state saved_mode)
{
	if (g_motor_params != NULL) {
		g_motor_params->calibration.requested_online_mode = (uint8_t)saved_mode;
	}
}

int motor_commission_request_online_mode(enum motor_state mode)
{
	if (g_motor_params == NULL) {
		return -ENODEV;
	}

	g_motor_params->calibration.requested_online_mode = (uint8_t)mode;

	int state = motor_api_get_state();
	if (state == MOTOR_STATE_ONLINE || motor_state_is_online_submode((enum motor_state)state)) {
		return motor_post_mode_change(mode);
	}

	return motor_api_request_online();
}

static int motor_commission_wait_for_offset_calibration(uint32_t timeout_ms)
{
	uint32_t start_ms = k_uptime_get_32();
	bool observed_calibration = false;

	while ((k_uptime_get_32() - start_ms) < timeout_ms) {
		int state = motor_api_get_state();
		if (state == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		if (g_motor_params != NULL &&
		    (g_motor_params->calibration.running ||
		     !g_motor_params->calibration.complete ||
		     state == MOTOR_STATE_OFFSET_MEAS)) {
			observed_calibration = true;
		}
		if (g_motor_params != NULL &&
		    observed_calibration &&
		    g_motor_params->calibration.complete &&
		    !g_motor_params->calibration.running &&
		    state != MOTOR_STATE_PREPARE_ONLINE &&
		    state != MOTOR_STATE_OFFSET_MEAS) {
			return 0;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return -ETIMEDOUT;
}

static void motor_commission_set_direct_current(float32_t id_a, float32_t iq_a)
{
	motor_current_slew_params_set_target(g_motor_params, id_a, iq_a);
}


void motor_commission_encoder_clear_result(void)
{
	memset(encoder_detect_samples, 0, sizeof(encoder_detect_samples));
	memset(&encoder_detect_result, 0, sizeof(encoder_detect_result));
	encoder_detect_result_valid = false;
	encoder_detect_duration_ms = 0U;
	encoder_detect_sample_period_ms = 0U;
	encoder_detect_current_a = 0.0f;
}

static void motor_commission_encoder_stop_generated(void)
{
	if (g_motor_params == NULL) {
		return;
	}

	motor_commission_set_velocity_target_hz(0.0f);
	motor_commission_set_direct_current(0.0f, 0.0f);
	motor_command_feed_watchdog(g_motor_params);
}

static int motor_commission_encoder_validate_sweep(
	const struct shell *sh,
	const struct motor_commission_encoder_sweep_config *sweep)
{
	if (sweep == NULL ||
	    !isfinite(sweep->current_a) ||
	    !isfinite(sweep->mech_hz) ||
	    !isfinite(sweep->cycles) ||
	    fabsf(sweep->current_a) < 1.0e-6f ||
	    fabsf(sweep->mech_hz) < 1.0e-6f ||
	    sweep->cycles <= 0.0f) {
		shell_error(sh, "current_a and mech_hz must be non-zero; cycles must be > 0");
		return -EINVAL;
	}
	if (fabsf(sweep->current_a) > MOTOR_MAX_CURRENT_A) {
		shell_error(sh, "current_a exceeds motor current limit %.3f A",
			    (double)MOTOR_MAX_CURRENT_A);
		return -ERANGE;
	}

	return 0;
}

static int motor_commission_encoder_parse_required_sweep(
	const struct shell *sh,
	char **argv,
	struct motor_commission_encoder_sweep_config *sweep)
{
	if (sweep == NULL ||
	    !shell_parse_finite_float(argv[1], &sweep->current_a) ||
	    !shell_parse_finite_float(argv[2], &sweep->mech_hz) ||
	    !shell_parse_finite_float(argv[3], &sweep->cycles)) {
		shell_error(sh, "Invalid encoder mapping arguments");
		return -EINVAL;
	}

	return motor_commission_encoder_validate_sweep(sh, sweep);
}

static int motor_commission_encoder_prepare_generated_mode(const struct shell *sh)
{
	int ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED);
	if (ret != 0) {
		shell_error(sh, "Failed to request ONLINE state (err %d)", ret);
		return ret;
	}
	if (motor_api_get_state() != MOTOR_STATE_ONLINE_VELOCITY_GENERATED) {
		ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
						     MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS);
		if (ret != 0) {
			shell_error(sh, "Timed out waiting for velocity_generated mode");
			return ret;
		}
	}

	ret = motor_commission_wait_for_control_loop(MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Control ISR is not advancing in velocity_generated mode (err %d)",
			    ret);
		return ret;
	}

	return 0;
}

static int motor_commission_encoder_run_generated_sweep(
	const struct shell *sh,
	const struct motor_commission_encoder_sweep_config *sweep,
	bool print_apply_hint)
{
	int ret = motor_commission_encoder_validate_sweep(sh, sweep);
	if (ret != 0) {
		return ret;
	}

	motor_commission_encoder_clear_result();

	ret = motor_commission_encoder_prepare_generated_mode(sh);
	if (ret != 0) {
		return ret;
	}

	float32_t duration_ms_f = (sweep->cycles / fabsf(sweep->mech_hz)) * 1000.0f;
	uint32_t duration_ms = (uint32_t)ceilf(duration_ms_f);
	duration_ms = MAX(duration_ms, MOTOR_COMMISSION_ENCODER_MIN_SAMPLE_MS);
	uint32_t sample_period_ms =
		MAX(MOTOR_COMMISSION_ENCODER_MIN_SAMPLE_MS,
		    (uint32_t)ceilf((float32_t)duration_ms /
				   (float32_t)MOTOR_COMMISSION_ENCODER_MAX_SAMPLES));
	uint32_t target_samples = duration_ms / sample_period_ms;
	target_samples = CLAMP(target_samples, 4U, MOTOR_COMMISSION_ENCODER_MAX_SAMPLES);

	encoder_detect_duration_ms = duration_ms;
	encoder_detect_sample_period_ms = sample_period_ms;
	encoder_detect_current_a = sweep->current_a;

	uint32_t accepted = 0U;
	uint32_t last_trace_loop = 0U;
	struct motor_commission_encoder_trace_guard trace_guard;
	motor_commission_encoder_trace_force_on(&trace_guard);
	ret = motor_commission_wait_for_control_loop(MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS);
	if (ret != 0) {
		motor_commission_encoder_trace_restore(&trace_guard);
		shell_error(sh, "Control ISR did not advance after enabling encoder trace (err %d)",
			    ret);
		return ret;
	}

	/* D-axis excitation means the generated reference angle is the commanded
	 * rotor flux axis. The detected offset can therefore be applied directly
	 * as the encoder FOC commutation offset.
	 */
	motor_commission_set_velocity_target_hz(0.0f);
	motor_commission_set_direct_current(sweep->current_a, 0.0f);
	motor_command_feed_watchdog(g_motor_params);
	ret = motor_commission_wait_ms_or_fault(MOTOR_COMMISSION_ENCODER_PRE_SWEEP_SETTLE_MS);
	if (ret != 0) {
		motor_commission_encoder_stop_generated();
		motor_commission_encoder_trace_restore(&trace_guard);
		motor_encoder_acquisition_abort();
		shell_error(sh, "Encoder mapping pre-sweep hold failed (err %d)", ret);
		return ret;
	}

	last_trace_loop = g_motor_params->rt_fast.control_loop_count;
	motor_commission_set_velocity_target_hz(sweep->mech_hz);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh,
		    "Encoder mapping detect: Id=%.3f A velocity=%.3f Hz cycles=%.2f duration=%u ms sample=%u ms N=%u prehold=%u ms",
		    (double)sweep->current_a, (double)sweep->mech_hz,
		    (double)sweep->cycles, duration_ms, sample_period_ms,
		    target_samples, MOTOR_COMMISSION_ENCODER_PRE_SWEEP_SETTLE_MS);
	shell_print(sh,
		    "  Expect slow visible rotation: %.2f rev at %.3f Hz (%.1f s)",
		    (double)sweep->cycles, (double)sweep->mech_hz,
		    (double)duration_ms / 1000.0);

	for (uint32_t i = 0U; i < target_samples; i++) {
		k_msleep(sample_period_ms);
		motor_command_feed_watchdog(g_motor_params);

		struct motor_encoder_map_detect_sample *sample =
			&encoder_detect_samples[accepted++];
		sample->flags = 0U;

		struct motor_encoder_raw_trace_sample raw_trace = {0};
		bool have_raw_trace =
			motor_commission_encoder_latest_raw_trace_after(last_trace_loop,
									&raw_trace);
		if (have_raw_trace) {
			last_trace_loop = raw_trace.control_loop_count;
			sample->generated_mech_rad = wrap_rad_2pi(raw_trace.generated_mech_rad);
			sample->generated_elec_rad = wrap_rad_2pi(raw_trace.generated_elec_rad);
			sample->encoder_mech_rad = wrap_rad_2pi(raw_trace.raw_angle_rad);
		} else {
			sample->generated_mech_rad = 0.0f;
			sample->generated_elec_rad = 0.0f;
			sample->encoder_mech_rad = 0.0f;
			sample->flags |= MOTOR_ENCODER_MAP_SAMPLE_ERROR;
		}

		if (have_raw_trace && raw_trace.sample_warning != 0U) {
			sample->flags |= MOTOR_ENCODER_MAP_SAMPLE_WARNING;
		}
		if ((have_raw_trace &&
		     (raw_trace.sample_error != 0U ||
		      raw_trace.sample_io_fault != 0U ||
		      raw_trace.sample_fresh == 0U)) ||
		    !isfinite(sample->generated_mech_rad) ||
		    !isfinite(sample->generated_elec_rad) ||
		    !isfinite(sample->encoder_mech_rad)) {
			sample->flags |= MOTOR_ENCODER_MAP_SAMPLE_ERROR;
		}
	}

	motor_commission_encoder_stop_generated();
	motor_commission_encoder_trace_restore(&trace_guard);
	motor_encoder_acquisition_abort();

	struct motor_encoder_map_detect_config cfg = {
		.pole_pairs = (float32_t)MOTOR_POLE_PAIRS,
		.min_mech_motion_rad = 0.02f,
		/* Hybrid steppers can show substantial electrical phase ripple during
		 * generated-angle sweeps because detent torque and open-loop load angle
		 * modulate the measured rotor position. Keep the residual visible in
		 * the report, but do not reject an otherwise clean full-revolution
		 * mapping unless the RMS phase spread is clearly excessive.
		 */
		.max_offset_residual_rad = 0.80f,
		.max_direction_residual_rad = 0.50f,
		.min_direction_correlation = 0.70f,
		.max_error_samples = MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES,
		.estimate_ratio = false,
	};
	ret = motor_encoder_map_detect_compute(&cfg, encoder_detect_samples, accepted,
					       &encoder_detect_result);
	encoder_detect_result_valid = encoder_detect_result.valid;

	shell_print(sh,
		    "Encoder mapping result: valid=%s dir=%d corr=%.4f off_mech=%.3f deg off_elec=%.3f deg",
		    encoder_detect_result.valid ? "YES" : "NO",
		    encoder_detect_result.direction_sign,
		    (double)encoder_detect_result.direction_corr,
		    (double)(encoder_detect_result.offset_mech_rad * 180.0f / PI_F32),
		    (double)(encoder_detect_result.offset_elec_rad * 180.0f / PI_F32));
	shell_print(sh,
		    "  residuals: offset=%.4f rad direction=%.4f rad motion=%.3f deg samples=%u rejected=%u warn=%u err=%u ret=%d",
		    (double)encoder_detect_result.offset_residual_rad,
		    (double)encoder_detect_result.direction_residual_rad,
		    (double)(encoder_detect_result.mech_motion_rad * 180.0f / PI_F32),
		    encoder_detect_result.sample_count,
		    encoder_detect_result.rejected_samples,
		    encoder_detect_result.encoder_warning_count,
		    encoder_detect_result.encoder_error_count,
		    ret);
	if (accepted > 0U) {
		const struct motor_encoder_map_detect_sample *first = &encoder_detect_samples[0];
		const struct motor_encoder_map_detect_sample *last =
			&encoder_detect_samples[accepted - 1U];
		shell_print(sh,
			    "  angles: generated %.3f -> %.3f deg, encoder %.3f -> %.3f deg",
			    (double)(first->generated_mech_rad * 180.0f / PI_F32),
			    (double)(last->generated_mech_rad * 180.0f / PI_F32),
			    (double)(first->encoder_mech_rad * 180.0f / PI_F32),
			    (double)(last->encoder_mech_rad * 180.0f / PI_F32));
	}
	if (encoder_detect_result.valid) {
		motor_encoder_acquisition_reset_stats();
		if (print_apply_hint) {
			shell_print(sh, "Run 'motor commission encoder apply' to apply staged mapping.");
		}
	}

	return ret;
}

static void motor_commission_encoder_result_combine_bidirectional(
	const struct motor_encoder_map_detect_result *forward,
	const struct motor_encoder_map_detect_result *reverse,
	struct motor_encoder_map_detect_result *out)
{
	*out = *forward;

	float32_t sin_sum = sinf(forward->offset_elec_rad) + sinf(reverse->offset_elec_rad);
	float32_t cos_sum = cosf(forward->offset_elec_rad) + cosf(reverse->offset_elec_rad);
	if (fabsf(sin_sum) > 1.0e-6f || fabsf(cos_sum) > 1.0e-6f) {
		out->offset_elec_rad = wrap_rad_pi(atan2f(sin_sum, cos_sum));
		out->offset_mech_rad = wrap_rad_pi(out->offset_elec_rad / (float32_t)MOTOR_POLE_PAIRS);
	}

	out->direction_corr = 0.5f * (forward->direction_corr + reverse->direction_corr);
	out->direction_residual_rad = fmaxf(forward->direction_residual_rad,
					    reverse->direction_residual_rad);
	out->offset_residual_rad = fmaxf(forward->offset_residual_rad,
					 reverse->offset_residual_rad);
	out->mech_motion_rad = forward->mech_motion_rad + reverse->mech_motion_rad;
	out->sample_count = forward->sample_count + reverse->sample_count;
	out->rejected_samples = forward->rejected_samples + reverse->rejected_samples;
	out->encoder_error_count = forward->encoder_error_count + reverse->encoder_error_count;
	out->encoder_warning_count =
		forward->encoder_warning_count + reverse->encoder_warning_count;

	float32_t offset_delta =
		fabsf(wrap_rad_pi(forward->offset_elec_rad - reverse->offset_elec_rad));
	bool compatible = forward->valid && reverse->valid &&
			  forward->direction_sign == reverse->direction_sign &&
			  offset_delta <= 0.50f;

	out->direction_valid = compatible && forward->direction_valid && reverse->direction_valid;
	out->offset_valid = compatible && forward->offset_valid && reverse->offset_valid;
	out->ratio_valid = forward->ratio_valid && reverse->ratio_valid;
	out->valid = compatible;
}

static int motor_commission_encoder_run_robust_sweep(
	const struct shell *sh,
	const struct motor_commission_encoder_sweep_config *sweep,
	bool bidirectional,
	bool print_apply_hint)
{
	struct motor_encoder_map_detect_result forward = {0};
	struct motor_encoder_map_detect_result reverse = {0};

	int ret = motor_commission_encoder_run_generated_sweep(sh, sweep, false);
	forward = encoder_detect_result;
	if (ret != 0 || !forward.valid || !bidirectional) {
		if (print_apply_hint && encoder_detect_result.valid) {
			shell_print(sh, "Run 'motor commission encoder apply' to apply staged mapping.");
		}
		return ret;
	}

	struct motor_commission_encoder_sweep_config reverse_sweep = *sweep;
	reverse_sweep.mech_hz = -reverse_sweep.mech_hz;
	k_msleep(MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS);
	ret = motor_commission_encoder_run_generated_sweep(sh, &reverse_sweep, false);
	reverse = encoder_detect_result;
	if (ret != 0 || !reverse.valid) {
		encoder_detect_result = forward;
		encoder_detect_result_valid = forward.valid;
		shell_error(sh, "Reverse encoder mapping sweep failed (err %d)", ret);
		return (ret != 0) ? ret : -ERANGE;
	}

	motor_commission_encoder_result_combine_bidirectional(&forward, &reverse,
							      &encoder_detect_result);
	encoder_detect_result_valid = encoder_detect_result.valid;
	shell_print(sh,
		    "Robust encoder mapping combined: valid=%s dir=%d corr=%.4f off_mech=%.3f deg off_elec=%.3f deg",
		    encoder_detect_result.valid ? "YES" : "NO",
		    encoder_detect_result.direction_sign,
		    (double)encoder_detect_result.direction_corr,
		    (double)(encoder_detect_result.offset_mech_rad * 180.0f / PI_F32),
		    (double)(encoder_detect_result.offset_elec_rad * 180.0f / PI_F32));
	shell_print(sh,
		    "  combined: offset_res=%.4f rad dir_res=%.4f rad motion=%.3f deg samples=%u rejected=%u warn=%u err=%u",
		    (double)encoder_detect_result.offset_residual_rad,
		    (double)encoder_detect_result.direction_residual_rad,
		    (double)(encoder_detect_result.mech_motion_rad * 180.0f / PI_F32),
		    encoder_detect_result.sample_count,
		    encoder_detect_result.rejected_samples,
		    encoder_detect_result.encoder_warning_count,
		    encoder_detect_result.encoder_error_count);

	if (!encoder_detect_result.valid) {
		return -ERANGE;
	}
	if (print_apply_hint) {
		shell_print(sh, "Run 'motor commission encoder apply' to apply staged mapping.");
	}
	return 0;
}

static float32_t motor_commission_encoder_commutation_offset_mech_rad(void)
{
	/* Generated-sweep encoder mapping uses Id d-axis excitation, so the
	 * generated-reference offset is already the mechanical d-axis FOC
	 * commutation offset. Do not apply the +/-90 electrical q-axis correction
	 * used by the previous Iq-excited mapping experiment.
	 */
	return wrap_rad_pi(encoder_detect_result.offset_mech_rad);
}

int cmd_motor_commission_encoder_run(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "Usage: motor commission encoder run <current_a> <mech_hz> <cycles>");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Calibration is not complete; run calibration before encoder detect");
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before encoder detect");
		return -EACCES;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	struct motor_commission_encoder_sweep_config sweep = {0};
	int ret = motor_commission_encoder_parse_required_sweep(sh, argv, &sweep);
	if (ret != 0) {
		return ret;
	}

	return motor_commission_encoder_run_generated_sweep(sh, &sweep, true);
}

int cmd_motor_commission_encoder_robust(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 4 || argc > 5) {
		shell_error(sh,
			    "Usage: motor commission encoder robust <current_a> <mech_hz> <cycles> [bidirectional]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Calibration is not complete; run calibration before encoder detect");
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before encoder detect");
		return -EACCES;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	struct motor_commission_encoder_sweep_config sweep = {0};
	int ret = motor_commission_encoder_parse_required_sweep(sh, argv, &sweep);
	if (ret != 0) {
		return ret;
	}

	bool bidirectional = argc == 5 && strcmp(argv[4], "bidirectional") == 0;
	if (argc == 5 && !bidirectional) {
		shell_error(sh, "Optional fourth argument must be 'bidirectional'");
		return -EINVAL;
	}

	shell_print(sh,
		    "Encoder commutation mapping: generated Id sweep, no motor parameter ID, no gain tuning");
	shell_print(sh,
		    "Prerequisite: current offsets complete and control armed; next: 'motor commission encoder apply'");
	return motor_commission_encoder_run_robust_sweep(sh, &sweep, bidirectional, true);
}

int cmd_motor_commission_encoder_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Encoder Mapping Detect:");
	shell_print(sh, "  Staged valid:   %s", encoder_detect_result_valid ? "YES" : "NO");
	shell_print(sh, "  Duration/sample:%u ms / %u ms",
		    encoder_detect_duration_ms, encoder_detect_sample_period_ms);
	shell_print(sh, "  Excitation:     Id=%.3f A, Iq=0.000 A", (double)encoder_detect_current_a);
	shell_print(sh, "  Valid:          %s", encoder_detect_result.valid ? "YES" : "NO");
	shell_print(sh, "  Direction:      sign=%d valid=%s corr=%.4f residual=%.4f rad",
		    encoder_detect_result.direction_sign,
		    encoder_detect_result.direction_valid ? "YES" : "NO",
		    (double)encoder_detect_result.direction_corr,
		    (double)encoder_detect_result.direction_residual_rad);
	shell_print(sh, "  Offset:         valid=%s mech=%.4f deg elec=%.3f deg residual=%.4f rad",
		    encoder_detect_result.offset_valid ? "YES" : "NO",
		    (double)(encoder_detect_result.offset_mech_rad * 180.0f / PI_F32),
		    (double)(encoder_detect_result.offset_elec_rad * 180.0f / PI_F32),
		    (double)encoder_detect_result.offset_residual_rad);
	if (encoder_detect_result_valid && encoder_detect_result.valid) {
		float32_t commutation_offset_rad =
			motor_commission_encoder_commutation_offset_mech_rad();
		shell_print(sh,
			    "  FOC offset:      mech=%.4f deg (direct Id-axis d-axis offset)",
			    (double)(commutation_offset_rad * 180.0f / PI_F32));
	}
	shell_print(sh, "  Ratio:          %.4f valid=%s",
		    (double)encoder_detect_result.ratio,
		    encoder_detect_result.ratio_valid ? "YES" : "NO");
	shell_print(sh, "  Samples:        accepted=%u rejected=%u warn=%u err=%u motion=%.3f deg",
		    encoder_detect_result.sample_count,
		    encoder_detect_result.rejected_samples,
		    encoder_detect_result.encoder_warning_count,
		    encoder_detect_result.encoder_error_count,
		    (double)(encoder_detect_result.mech_motion_rad * 180.0f / PI_F32));
	return 0;
}

static int motor_commission_encoder_apply_staged(const struct shell *sh)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!encoder_detect_result_valid || !encoder_detect_result.valid) {
		shell_error(sh, "No valid staged encoder mapping result");
		return -ENOENT;
	}

	g_motor_params->encoder_direction_sign =
		(encoder_detect_result.direction_sign >= 0) ? 1 : -1;
	g_motor_params->observer_alignment_offset_rad =
		motor_commission_encoder_commutation_offset_mech_rad();
	g_motor_params->observer_elec_trim_rad = 0.0f;
	angle_observer_set_offset(&g_motor_params->observer,
				  g_motor_params->observer_alignment_offset_rad);
	g_motor_params->calibration.encoder_mapping_complete = true;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh,
		    "Encoder mapping applied: sign=%d commutation_offset=%.4f deg mechanical",
		    g_motor_params->encoder_direction_sign,
		    (double)(g_motor_params->observer_alignment_offset_rad * 180.0f / PI_F32));
	shell_print(sh,
		    "Next: run 'motor commission validate current', then velocity/position validation.");
	return 0;
}

int cmd_motor_commission_encoder_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return motor_commission_encoder_apply_staged(sh);
}

static int motor_commission_encoder_map_apply(
	const struct shell *sh,
	const struct motor_commission_encoder_sweep_config *sweep,
	bool run_current_offsets,
	bool leave_online)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	int ret = motor_commission_encoder_validate_sweep(sh, sweep);
	if (ret != 0) {
		return ret;
	}

	(void)motor_api_set_param("outer_loop_mode", (float32_t)MOTOR_OUTER_LOOP_MODE_PI);
	(void)motor_api_set_param("velocity_dob_enable", 0.0f);
	(void)cmd_motor_commission_detent_clear(sh, 0, NULL);
	motor_commission_encoder_stop_generated();
	motor_commission_motion_stop_current();

	shell_print(sh, "Encoder commutation mapping: Id-axis generated sweep + apply");
	shell_print(sh, "Bring-up defaults: outer=PI, DOB=disabled, detent FF=disabled");
	shell_print(sh, "Completion: %s", leave_online ? "velocity_generated/armed" : "idle/disarmed");

	ret = motor_commission_prepare_idle_zero_current(MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to prepare idle zero-current state (err %d)", ret);
		return ret;
	}

	if (run_current_offsets) {
		shell_print(sh, "[map 1/3] Current offset calibration");
		shell_print(sh, "  No rotor rotation expected during current offset calibration.");
		ret = motor_api_request_calibrate();
		if (ret != 0) {
			shell_error(sh, "Failed to request current offset calibration (err %d)", ret);
			return ret;
		}
		ret = motor_commission_wait_for_offset_calibration(
			MOTOR_COMMISSION_ENCODER_MAP_CAL_TIMEOUT_MS);
		if (ret != 0) {
			shell_error(sh, "Current offset calibration failed/timed out (err %d)",
				    ret);
			return ret;
		}
		shell_print(sh, "  offsets: Ia=%.4f Ib=%.4f",
			    (double)g_motor_params->Ia_offset,
			    (double)g_motor_params->Ib_offset);
	} else if (!g_motor_params->calibration.complete) {
		shell_error(sh,
			    "Current offsets are not complete; run the standard commissioning workflow");
		return -EACCES;
	}

	shell_print(sh, "[map %u/3] Id-axis generated-sweep encoder mapping",
		    run_current_offsets ? 2U : 1U);
	shell_print(sh,
		    "  Expect %.2f mechanical rev at %.3f Hz; this can look very slow.",
		    (double)sweep->cycles, (double)sweep->mech_hz);
	ret = cmd_motor_arm(sh, 0, NULL);
	if (ret != 0) {
		shell_error(sh, "Failed to arm control output (err %d)", ret);
		return ret;
	}
	ret = motor_commission_encoder_run_robust_sweep(sh, sweep, false, false);
	if (ret != 0 && motor_api_get_state() != MOTOR_STATE_ERROR) {
		shell_warn(sh, "Encoder mapping sweep failed once (err %d), retrying", ret);
		motor_commission_encoder_stop_generated();
		motor_commission_motion_stop_current();
		motor_encoder_acquisition_reset_stats();
		k_msleep(MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS);
		ret = motor_commission_encoder_run_robust_sweep(sh, sweep, false, false);
	}
	if (ret != 0) {
		shell_error(sh, "Encoder mapping sweep failed (err %d)", ret);
		return ret;
	}
	if (!encoder_detect_result_valid || !encoder_detect_result.valid) {
		shell_error(sh, "Encoder mapping quality rejected");
		return -ERANGE;
	}

	shell_print(sh, "[map %u/3] Apply encoder mapping", run_current_offsets ? 3U : 2U);
	ret = motor_commission_encoder_apply_staged(sh);
	if (ret != 0) {
		return ret;
	}
	motor_commission_encoder_stop_generated();
	motor_command_feed_watchdog(g_motor_params);

	motor_encoder_acquisition_reset_stats();
	motor_commission_motion_stop_current();
	motor_commission_encoder_stop_generated();
	(void)motor_api_set_param("outer_loop_mode", (float32_t)MOTOR_OUTER_LOOP_MODE_PI);
	(void)motor_api_set_param("velocity_dob_enable", 0.0f);

	ret = leave_online ?
		motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED) :
		motor_commission_prepare_idle_zero_current(MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS);
	if (ret == 0 && leave_online) {
		ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
						     MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS);
	}
	if (ret != 0) {
		shell_warn(sh,
			   "Encoder mapping succeeded but failed to enter completion state (err %d)",
			   ret);
	}

	shell_print(sh,
		    "Encoder mapping complete: sign=%d commutation_offset=%.4f deg mechanical outer=PI state=%s",
		    g_motor_params->encoder_direction_sign,
		    (double)(g_motor_params->observer_alignment_offset_rad * 180.0f / PI_F32),
		    leave_online ? "velocity_generated" : "idle");
	return 0;
}

int motor_commission_encoder_default_map_apply(const struct shell *sh,
					       bool run_current_offsets,
					       bool leave_online)
{
	const struct motor_commission_encoder_sweep_config sweep = {
		.current_a = MOTOR_COMMISSION_ENCODER_MAP_DEFAULT_CURRENT_A,
		.mech_hz = MOTOR_COMMISSION_ENCODER_MAP_DEFAULT_MECH_HZ,
		.cycles = MOTOR_COMMISSION_ENCODER_MAP_DEFAULT_CYCLES,
	};

	return motor_commission_encoder_map_apply(sh, &sweep, run_current_offsets, leave_online);
}

int cmd_motor_commission_encoder_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_commission_encoder_clear_result();
	shell_print(sh, "Encoder mapping detect result cleared");
	return 0;
}
