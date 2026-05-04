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
#include "motor/runtime/commission_runtime.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "shell_parse.h"
#include "config.h"
#include "motor_commission_adapter.h"
#include "motor/motion/traj.h"
#include "motor/math/math_constants.h"
#include "motor/math/angle_wrap.h"
#include "motor/calibration/encoder_map_detect.h"
#include "motor/observers/angle_observer.h"
#include "motor_torque.h"
#include "motor_encoder_pipeline.h"

#define MOTOR_COMMISSION_AUTO_POLL_MS 10U
#define MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS 8000U
#define MOTOR_COMMISSION_AUTO_POST_WAIT_MS 2500U
#define MOTOR_COMMISSION_ENCODER_MAX_SAMPLES 512U
#define MOTOR_COMMISSION_ENCODER_MIN_SAMPLE_MS 5U
#define MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES 4U
#define MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_MOTION_SAMPLE_MS 5U
#define MOTOR_COMMISSION_MOTION_MIN_SAMPLES 4U
#define MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS 80U
#define MOTOR_COMMISSION_AUTO_MECH_RUNS 3U
#define MOTOR_COMMISSION_AUTO_MECH_MAX_ATTEMPTS 5U
#define MOTOR_COMMISSION_AUTO_MECH_CYCLES_PER_CAPTURE 1U
#define MOTOR_COMMISSION_AUTO_MECH_ACCEL_MARGIN 1.25f
#define MOTOR_COMMISSION_AUTO_MECH_VALIDATE_RMS_NM 0.005f
#define MOTOR_COMMISSION_AUTO_NORMAL_FLUX_MAX_HZ 3.0f
#define MOTOR_COMMISSION_AUTO_NORMAL_FLUX_MIN_HZ 0.5f
#define MOTOR_COMMISSION_AUTO_NORMAL_MECH_MAX_HZ 3.0f
#define MOTOR_COMMISSION_AUTO_NORMAL_MECH_BASE_HZ 1.5f
#define MOTOR_COMMISSION_AUTO_NORMAL_MECH_DITHER_HZ 0.5f
#define MOTOR_COMMISSION_AUTO_SLOW_FLUX_MAX_HZ 0.75f
#define MOTOR_COMMISSION_AUTO_SLOW_FLUX_MIN_HZ 0.25f
#define MOTOR_COMMISSION_AUTO_SLOW_MECH_MAX_HZ 1.0f
#define MOTOR_COMMISSION_AUTO_SLOW_MECH_BASE_HZ 0.5f
#define MOTOR_COMMISSION_AUTO_SLOW_MECH_DITHER_HZ 0.15f
#define MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_MAX_HZ 5.0f
#define MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HZ_CAP 20.0f
#define MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_HOLD_MS 2000U
#define MOTOR_COMMISSION_AUTO_VALIDATE_MIN_HOLD_MS 500U
#define MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS 10000U
#define MOTOR_COMMISSION_DETENT_MAX_SPEED_HZ 1.0f
#define MOTOR_COMMISSION_DETENT_SAFE_IQ_LIMIT_A 0.15f
#define MOTOR_COMMISSION_DETENT_SAFE_GAIN_SPEED_HZ 1.0f
#define MOTOR_COMMISSION_DETENT_DEFAULT_DECIMATION 1U
#define MOTOR_COMMISSION_DETENT_MIN_DECIMATION 1U
#define MOTOR_COMMISSION_DETENT_MAX_DECIMATION 128U
#define MOTOR_COMMISSION_DETENT_MIN_BIN_COVERAGE_MPU 650U
#define MOTOR_COMMISSION_DETENT_MIN_SAMPLES_PER_BIN 2U
#define MOTOR_COMMISSION_DETENT_MAX_RUN_MS 60000U

static const float32_t motor_commission_mech_step_pattern[] = {
	-1.0f, 0.0f, 1.0f, -0.5f,
};

static const float32_t motor_commission_validate_step_scale[] = {
	0.2f, 0.6f, 1.0f, -0.2f, -0.6f, -1.0f, 0.0f,
};

static struct motor_encoder_map_detect_sample encoder_detect_samples[
	MOTOR_COMMISSION_ENCODER_MAX_SAMPLES];
static struct motor_encoder_map_detect_result encoder_detect_result;
static bool encoder_detect_result_valid;
static uint32_t encoder_detect_duration_ms;
static uint32_t encoder_detect_sample_period_ms;

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

static int motor_post_mode_change(enum motor_state target_mode)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_MODE_CHANGE,
		.target_mode = target_mode,
	};
	int ret = motor_api_post_event(&evt);

	return ret;
}

static const char *motor_commission_expected_mode_to_string(uint8_t expected_mode)
{
	switch (expected_mode) {
	case MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED:
		return "ONLINE_VELOCITY_ENCODER";
	case MOTOR_COMMISSION_EXPECT_TORQUE:
		return "ONLINE_CURRENT_ENCODER";
	case MOTOR_COMMISSION_EXPECT_ANY:
	default:
		return "ANY";
	}
}

static const char *motor_commission_tune_error_to_string(int err)
{
	switch (err) {
	case 0:
		return "none";
	case -ERANGE:
		return "quality_rejected";
	case -ENOENT:
		return "not_staged";
	case -ETIMEDOUT:
		return "timeout";
	case -EFAULT:
		return "state_error";
	case -ECANCELED:
		return "aborted";
	default:
		return "error";
	}
}

static void motor_commission_print_tune_reject_flags(const struct shell *sh, uint32_t flags)
{
	if (flags == MOTOR_COMMISSION_TUNE_REJECT_NONE) {
		shell_print(sh, "  Tune reject:    none");
		return;
	}

	shell_print(sh, "  Tune reject:    0x%08X", flags);
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_INVALID) != 0U) {
		shell_print(sh, "    - psi_f estimate invalid");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_SAMPLES) != 0U) {
		shell_print(sh, "    - psi_f sample count too low");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_R2) != 0U) {
		shell_print(sh, "    - psi_f R2 below threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_RMS) != 0U) {
		shell_print(sh, "    - psi_f residual RMS above threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_SIGN) != 0U) {
		shell_print(sh, "    - psi_f sign/finite check failed");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_INVALID) != 0U) {
		shell_print(sh, "    - mechanical estimate invalid");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_SAMPLES) != 0U) {
		shell_print(sh, "    - mechanical sample count too low");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_R2) != 0U) {
		shell_print(sh, "    - mechanical R2 below threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_RMS) != 0U) {
		shell_print(sh, "    - mechanical residual RMS above threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_INERTIA_SIGN) != 0U) {
		shell_print(sh, "    - inertia sign/finite check failed");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_VISCOUS_SIGN) != 0U) {
		shell_print(sh, "    - viscous friction sign/finite check failed");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_KT_INVALID) != 0U) {
		shell_print(sh, "    - torque constant check failed");
	}
}

static int motor_commission_wait_for_mode(enum motor_state mode, uint32_t timeout_ms)
{
	uint32_t start_ms = k_uptime_get_32();

	while ((k_uptime_get_32() - start_ms) < timeout_ms) {
		int state = motor_api_get_state();
		if (state == (int)mode) {
			return 0;
		}
		if (state == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return -ETIMEDOUT;
}

static int motor_commission_wait_for_capture_stop(uint32_t timeout_ms)
{
	uint32_t start_ms = k_uptime_get_32();

	while ((k_uptime_get_32() - start_ms) < timeout_ms) {
		const struct motor_commission_ctx *ctx = &g_motor_params->commission;
		if (!ctx->active) {
			if (ctx->stage == MOTOR_COMMISSION_STAGE_ABORTED) {
				return -ECANCELED;
			}
			if (ctx->stage == MOTOR_COMMISSION_STAGE_COMPLETED) {
				return 0;
			}
			return -EIO;
		}
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return -ETIMEDOUT;
}

static void motor_commission_set_velocity_target_hz(float32_t target_hz)
{
	float32_t target_rad_s = target_hz * 2.0f * PI_F32;
	float32_t limited = clampf(target_rad_s,
				   -g_motor_params->profile_max_velocity_rad_s,
				   g_motor_params->profile_max_velocity_rad_s);

	traj_set_target_value(&g_motor_params->traj_velocity, limited);
}

static void motor_commission_set_direct_current(float32_t id_a, float32_t iq_a)
{
	g_motor_params->Id_setpoint_A = id_a;
	g_motor_params->Iq_setpoint_A = iq_a;
	g_motor_params->live.Id_ref_A = id_a;
	g_motor_params->live.Iq_ref_A = iq_a;
}

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

static int motor_commission_wait_ms_or_fault(uint32_t hold_ms)
{
	uint32_t start_ms = k_uptime_get_32();

	while ((k_uptime_get_32() - start_ms) < hold_ms) {
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return 0;
}

static void motor_commission_print_velocity_validation_sample(const struct shell *sh,
							     float32_t target_hz)
{
	float32_t ref_hz = g_motor_params->live.velocity_ref_rad_s / (2.0f * PI_F32);
	float32_t meas_hz = g_motor_params->live.velocity_filtered_rad_s / (2.0f * PI_F32);
	float32_t err_hz = target_hz - meas_hz;

	shell_print(sh,
		    "  target=%7.3f Hz ref=%7.3f Hz meas=%7.3f Hz err=%7.3f Hz Iq=%.4f A Id=%.4f A warn=%u err=%u",
		    (double)target_hz,
		    (double)ref_hz,
		    (double)meas_hz,
		    (double)err_hz,
		    (double)g_motor_params->live.Iq_A,
		    (double)g_motor_params->live.Id_A,
		    g_motor_params->live.encoder_sample_warning,
		    g_motor_params->live.encoder_sample_error);
}

static void motor_commission_encoder_clear_result(void)
{
	memset(encoder_detect_samples, 0, sizeof(encoder_detect_samples));
	memset(&encoder_detect_result, 0, sizeof(encoder_detect_result));
	encoder_detect_result_valid = false;
	encoder_detect_duration_ms = 0U;
	encoder_detect_sample_period_ms = 0U;
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

struct motor_commission_encoder_trace_guard {
	bool raw_trace_enabled;
	uint16_t raw_trace_decimation;
	uint16_t raw_trace_phase;
};

static void motor_commission_encoder_trace_force_on(
	struct motor_commission_encoder_trace_guard *guard)
{
	guard->raw_trace_enabled = g_motor_params->encoder_raw_trace.enabled;
	guard->raw_trace_decimation = g_motor_params->encoder_raw_trace.decimation;
	guard->raw_trace_phase = g_motor_params->encoder_raw_trace.phase;

	/* Generated/open-loop modes do not normally request encoder samples.
	 * Raw-trace enable is the existing ISR-safe telemetry gate that asks the
	 * encoder pipeline to sample without changing the commutation policy.
	 */
	g_motor_params->encoder_raw_trace.enabled = true;
	g_motor_params->encoder_raw_trace.decimation = 1U;
	g_motor_params->encoder_raw_trace.phase = 0U;
}

static void motor_commission_encoder_trace_restore(
	const struct motor_commission_encoder_trace_guard *guard)
{
	g_motor_params->encoder_raw_trace.enabled = guard->raw_trace_enabled;
	g_motor_params->encoder_raw_trace.decimation = guard->raw_trace_decimation;
	g_motor_params->encoder_raw_trace.phase = guard->raw_trace_phase;
}

static bool motor_commission_encoder_latest_raw_trace_after(
	uint32_t min_loop,
	struct motor_encoder_raw_trace_sample *out)
{
	if (out == NULL || g_motor_params == NULL ||
	    g_motor_params->encoder_raw_trace.count == 0U) {
		return false;
	}

	uint16_t write_idx = g_motor_params->encoder_raw_trace.write_idx;
	uint16_t idx = (write_idx == 0U) ?
			       (uint16_t)(MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES - 1U) :
			       (uint16_t)(write_idx - 1U);
	struct motor_encoder_raw_trace_sample sample =
		g_motor_params->encoder_raw_trace.samples[idx];
	if (sample.control_loop_count <= min_loop) {
		return false;
	}

	*out = sample;
	return true;
}

struct motor_commission_motion_measurement {
	float32_t iq_a;
	float32_t net_motion_rad;
	float32_t abs_motion_rad;
	uint16_t sample_count;
	uint16_t warning_count;
	uint16_t error_count;
	bool valid;
};

struct motor_commission_motion_threshold_result {
	float32_t threshold_a;
	float32_t best_net_motion_rad;
	float32_t best_abs_motion_rad;
	uint16_t sample_count;
	uint16_t warning_count;
	uint16_t error_count;
	bool valid;
};

static bool motor_commission_motion_sample_clean(
	const struct motor_encoder_raw_trace_sample *sample)
{
	return sample != NULL &&
	       sample->sample_fresh != 0U &&
	       sample->sample_error == 0U &&
	       sample->sample_io_fault == 0U &&
	       isfinite(sample->raw_angle_rad);
}

static void motor_commission_motion_stop_current(void)
{
	if (g_motor_params == NULL) {
		return;
	}

	(void)motor_api_set_param("Id_setpoint_A", 0.0f);
	(void)motor_api_set_param("Iq_setpoint_A", 0.0f);
	g_motor_params->Id_setpoint_A = 0.0f;
	g_motor_params->Iq_setpoint_A = 0.0f;
	motor_command_feed_watchdog(g_motor_params);
}

static int motor_commission_motion_measure_current(
	float32_t signed_iq_a,
	uint32_t hold_ms,
	float32_t min_motion_rad,
	struct motor_commission_motion_measurement *out)
{
	if (g_motor_params == NULL || out == NULL ||
	    !isfinite(signed_iq_a) || !isfinite(min_motion_rad) ||
	    hold_ms < MOTOR_COMMISSION_MOTION_SAMPLE_MS) {
		return -EINVAL;
	}

	*out = (struct motor_commission_motion_measurement){
		.iq_a = signed_iq_a,
	};

	(void)motor_api_set_param("Id_setpoint_A", 0.0f);
	(void)motor_api_set_param("Iq_setpoint_A", signed_iq_a);
	g_motor_params->Id_setpoint_A = 0.0f;
	g_motor_params->Iq_setpoint_A = signed_iq_a;
	motor_command_feed_watchdog(g_motor_params);

	bool have_prev = false;
	float32_t prev_angle_rad = 0.0f;
	uint32_t last_trace_loop = g_motor_params->rt_fast.control_loop_count;
	uint32_t start_ms = k_uptime_get_32();

	while ((k_uptime_get_32() - start_ms) < hold_ms) {
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}

		k_msleep(MOTOR_COMMISSION_MOTION_SAMPLE_MS);
		motor_command_feed_watchdog(g_motor_params);

		struct motor_encoder_raw_trace_sample raw_trace = {0};
		if (!motor_commission_encoder_latest_raw_trace_after(last_trace_loop,
								     &raw_trace)) {
			continue;
		}
		last_trace_loop = raw_trace.control_loop_count;
		out->warning_count += raw_trace.sample_warning ? 1U : 0U;
		if (!motor_commission_motion_sample_clean(&raw_trace)) {
			out->error_count++;
			continue;
		}

		out->sample_count++;
		if (!have_prev) {
			prev_angle_rad = raw_trace.raw_angle_rad;
			have_prev = true;
			continue;
		}

		float32_t delta_rad = wrap_rad_pi(raw_trace.raw_angle_rad - prev_angle_rad);
		if (!isfinite(delta_rad)) {
			out->error_count++;
			continue;
		}
		out->net_motion_rad += delta_rad;
		out->abs_motion_rad += fabsf(delta_rad);
		prev_angle_rad = raw_trace.raw_angle_rad;
	}

	out->valid = out->sample_count >= MOTOR_COMMISSION_MOTION_MIN_SAMPLES &&
		     out->error_count == 0U &&
		     fabsf(out->net_motion_rad) >= min_motion_rad &&
		     out->abs_motion_rad >= min_motion_rad;

	return 0;
}

static int motor_commission_motion_sweep_direction(
	const struct shell *sh,
	float32_t sign,
	float32_t start_a,
	float32_t stop_a,
	float32_t step_a,
	uint32_t hold_ms,
	float32_t min_motion_rad,
	struct motor_commission_motion_threshold_result *out)
{
	if (out == NULL || !isfinite(sign) || sign == 0.0f ||
	    !isfinite(start_a) || !isfinite(stop_a) || !isfinite(step_a) ||
	    start_a <= 0.0f || stop_a < start_a || step_a <= 0.0f) {
		return -EINVAL;
	}

	*out = (struct motor_commission_motion_threshold_result){0};

	for (float32_t amp_a = start_a; amp_a <= (stop_a + 0.5f * step_a); amp_a += step_a) {
		float32_t limited_amp_a = MIN(amp_a, stop_a);
		struct motor_commission_motion_measurement meas = {0};
		int ret = motor_commission_motion_measure_current(sign * limited_amp_a,
								  hold_ms,
								  min_motion_rad,
								  &meas);
		motor_commission_motion_stop_current();
		k_msleep(MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS);
		motor_command_feed_watchdog(g_motor_params);
		if (ret != 0) {
			return ret;
		}

		out->warning_count += meas.warning_count;
		out->error_count += meas.error_count;
		out->sample_count += meas.sample_count;
		if (fabsf(meas.abs_motion_rad) > fabsf(out->best_abs_motion_rad)) {
			out->best_abs_motion_rad = meas.abs_motion_rad;
			out->best_net_motion_rad = meas.net_motion_rad;
		}

		shell_print(sh,
			    "  %s Iq=%.3f A: net=%.3f deg abs=%.3f deg samples=%u warn=%u err=%u %s",
			    (sign > 0.0f) ? "pos" : "neg",
			    (double)limited_amp_a,
			    (double)(meas.net_motion_rad * 180.0f / PI_F32),
			    (double)(meas.abs_motion_rad * 180.0f / PI_F32),
			    meas.sample_count,
			    meas.warning_count,
			    meas.error_count,
			    meas.valid ? "PASS" : "wait");

		if (meas.valid) {
			out->threshold_a = limited_amp_a;
			out->best_abs_motion_rad = meas.abs_motion_rad;
			out->best_net_motion_rad = meas.net_motion_rad;
			out->valid = true;
			return 0;
		}

		if (limited_amp_a >= stop_a) {
			break;
		}
	}

	return -ENODATA;
}

static int motor_commission_run_motion_threshold(
	const struct shell *sh,
	float32_t start_a,
	float32_t stop_a,
	float32_t step_a,
	uint32_t hold_ms,
	float32_t min_motion_rad)
{
	int ret = motor_api_request_online();
	if (ret != 0) {
		return ret;
	}

	ret = motor_post_mode_change(MOTOR_STATE_ONLINE_CURRENT_ENCODER);
	if (ret != 0) {
		return ret;
	}

	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_CURRENT_ENCODER,
					     MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS);
	if (ret != 0) {
		return ret;
	}

	struct motor_commission_encoder_trace_guard trace_guard;
	motor_commission_encoder_trace_force_on(&trace_guard);

	struct motor_commission_motion_threshold_result pos = {0};
	struct motor_commission_motion_threshold_result neg = {0};
	ret = motor_commission_motion_sweep_direction(sh, 1.0f, start_a, stop_a, step_a,
						      hold_ms, min_motion_rad, &pos);
	int neg_ret = 0;
	if (ret == 0 || ret == -ENODATA) {
		neg_ret = motor_commission_motion_sweep_direction(sh, -1.0f, start_a, stop_a,
								  step_a, hold_ms,
								  min_motion_rad, &neg);
	}

	motor_commission_motion_stop_current();
	motor_commission_encoder_trace_restore(&trace_guard);

	struct motor_commission_results *res = &g_motor_params->commission.results;
	res->iq_move_pos_valid = pos.valid;
	res->iq_move_neg_valid = neg.valid;
	res->iq_move_min_pos_a = pos.valid ? pos.threshold_a : 0.0f;
	res->iq_move_min_neg_a = neg.valid ? neg.threshold_a : 0.0f;
	res->iq_move_recommended_a = (pos.valid && neg.valid) ?
					     fmaxf(pos.threshold_a, neg.threshold_a) :
					     0.0f;
	res->iq_move_valid = pos.valid && neg.valid;
	if (res->iq_move_valid &&
	    pos.best_net_motion_rad > 0.0f && neg.best_net_motion_rad < 0.0f) {
		res->iq_to_mech_sign = 1;
	} else if (res->iq_move_valid &&
		   pos.best_net_motion_rad < 0.0f && neg.best_net_motion_rad > 0.0f) {
		res->iq_to_mech_sign = -1;
	} else {
		res->iq_to_mech_sign = 0;
	}
	res->iq_move_pos_sample_count = pos.sample_count;
	res->iq_move_neg_sample_count = neg.sample_count;
	res->iq_move_warning_count = pos.warning_count + neg.warning_count;
	res->iq_move_error_count = pos.error_count + neg.error_count;

	if (ret != 0 || neg_ret != 0 || !res->iq_move_valid) {
		return (ret != 0) ? ret : ((neg_ret != 0) ? neg_ret : -ENODATA);
	}

	return 0;
}

static inline void motor_commission_ctx_from_global(struct motor_commission_runtime_ctx *ctx)
{
	motor_commission_runtime_ctx_init(ctx, g_motor_params);
}

static int motor_commission_auto_run_flux(const struct shell *sh,
					  const struct motor_commission_flux_config *cfg)
{
	int ret = motor_api_request_online();
	if (ret != 0) {
		return ret;
	}

	ret = motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		return ret;
	}

	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		return ret;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	ret = motor_commission_start_flux(&commission_ctx, cfg);
	if (ret != 0) {
		return ret;
	}

	(void)motor_api_set_param("velocity_cl_iq_limit_A", cfg->iq_limit_a);

	float32_t delta_hz = (cfg->steps > 1U) ?
				    ((cfg->max_speed_hz - cfg->min_speed_hz) /
				     (float32_t)(cfg->steps - 1U)) :
				    0.0f;
	uint32_t dwell_ms = cfg->settle_ms + cfg->sample_ms;

	for (uint32_t pass = 0U; pass < 2U; pass++) {
		float32_t sign = (pass == 0U) ? 1.0f : -1.0f;
		for (uint32_t i = 0U; i < cfg->steps; i++) {
			float32_t speed_hz = cfg->min_speed_hz + (delta_hz * (float32_t)i);
			float32_t target_hz = sign * speed_hz;
			motor_commission_set_velocity_target_hz(target_hz);
			motor_command_feed_watchdog(g_motor_params);

			shell_print(sh, "  Flux sweep step %u/%u: target=%.3f Hz",
				    (unsigned int)(pass * cfg->steps + i + 1U),
				    (unsigned int)(2U * cfg->steps),
				    (double)target_hz);

			uint32_t start_ms = k_uptime_get_32();
			while ((k_uptime_get_32() - start_ms) < dwell_ms) {
				if (g_motor_params->commission.stage == MOTOR_COMMISSION_STAGE_ABORTED) {
					return -ECANCELED;
				}
				if (motor_api_get_state() == MOTOR_STATE_ERROR) {
					return -EFAULT;
				}
				motor_command_feed_watchdog(g_motor_params);
				k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
			}
		}
	}

	ret = motor_commission_wait_for_capture_stop(MOTOR_COMMISSION_AUTO_POST_WAIT_MS);
	if (ret != 0) {
		return ret;
	}
	if (!g_motor_params->commission.results.psi_f_valid) {
		return -ERANGE;
	}

	return 0;
}

static int motor_commission_auto_run_mech(const struct shell *sh,
					  const struct motor_commission_mech_config *cfg,
					  bool require_fit)
{
	int ret = motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		return ret;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		return ret;
	}

	motor_commission_set_velocity_target_hz(cfg->base_speed_hz);

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	ret = motor_commission_start_mech(&commission_ctx, cfg);
	if (ret != 0) {
		return ret;
	}

	const uint32_t pattern_len = ARRAY_SIZE(motor_commission_mech_step_pattern);
	const uint32_t one_direction_steps = pattern_len;
	const uint32_t full_cycle_steps = 2U * one_direction_steps;
	uint32_t run_start_ms = k_uptime_get_32();
	uint32_t last_step = UINT32_MAX;
	while (g_motor_params->commission.active) {
		uint32_t now_ms = k_uptime_get_32();
		uint32_t elapsed_ms = now_ms - run_start_ms;
		uint32_t step = elapsed_ms / cfg->dither_period_ms;
		if (step != last_step) {
			uint32_t cycle_step = step % full_cycle_steps;
			float32_t direction = (cycle_step < one_direction_steps) ? 1.0f : -1.0f;
			uint32_t pattern_index = cycle_step % pattern_len;
			float32_t dither =
				motor_commission_mech_step_pattern[pattern_index] *
				cfg->dither_speed_hz;
			float32_t target_hz = direction * (cfg->base_speed_hz + dither);
			motor_commission_set_velocity_target_hz(target_hz);
			last_step = step;
		}

		if (g_motor_params->commission.stage == MOTOR_COMMISSION_STAGE_ABORTED) {
			ret = -ECANCELED;
			goto stop_current;
		}
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			ret = -EFAULT;
			goto stop_current;
		}

		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

stop_current:
	g_motor_params->Id_setpoint_A = 0.0f;
	g_motor_params->Iq_setpoint_A = 0.0f;
	motor_commission_set_velocity_target_hz(0.0f);
	if (ret != 0) {
		return ret;
	}

	ret = motor_commission_wait_for_capture_stop(MOTOR_COMMISSION_AUTO_POST_WAIT_MS);
	if (ret != 0) {
		return ret;
	}
	if (require_fit && !g_motor_params->commission.results.mech_valid) {
		return -ERANGE;
	}

	shell_print(sh, "  Mech excitation complete");
	return 0;
}

struct motor_commission_mech_aggregate {
	uint8_t count;
	uint32_t sample_count;
	float32_t sum_j;
	float32_t sum_j2;
	float32_t sum_b;
	float32_t sum_b2;
	float32_t sum_tc;
	float32_t sum_tc2;
	float32_t sum_t0;
	float32_t sum_rms;
	float32_t sum_r2;
	float32_t min_r2;
	float32_t max_rms;
};

static float32_t motor_commission_stddev(uint8_t count, float32_t sum, float32_t sum2)
{
	if (count < 2U) {
		return 0.0f;
	}

	float32_t n = (float32_t)count;
	float32_t mean = sum / n;
	float32_t variance = (sum2 / n) - (mean * mean);
	return sqrtf(fmaxf(variance, 0.0f));
}

static void motor_commission_mech_aggregate_add(
	struct motor_commission_mech_aggregate *agg,
	const struct motor_commission_results *res)
{
	if (agg == NULL || res == NULL || !res->mech_valid) {
		return;
	}

	if (agg->count == 0U) {
		agg->min_r2 = res->mech_r2;
		agg->max_rms = res->mech_residual_rms_nm;
	} else {
		agg->min_r2 = fminf(agg->min_r2, res->mech_r2);
		agg->max_rms = fmaxf(agg->max_rms, res->mech_residual_rms_nm);
	}

	agg->count++;
	agg->sample_count += res->mech_sample_count;
	agg->sum_j += res->inertia_kgm2;
	agg->sum_j2 += res->inertia_kgm2 * res->inertia_kgm2;
	agg->sum_b += res->viscous_friction_nm_per_rad_s;
	agg->sum_b2 += res->viscous_friction_nm_per_rad_s *
		       res->viscous_friction_nm_per_rad_s;
	agg->sum_tc += res->coulomb_friction_nm;
	agg->sum_tc2 += res->coulomb_friction_nm * res->coulomb_friction_nm;
	agg->sum_t0 += res->offset_friction_nm;
	agg->sum_rms += res->mech_residual_rms_nm;
	agg->sum_r2 += res->mech_r2;
}

static int motor_commission_mech_aggregate_finalize(
	const struct motor_commission_mech_aggregate *agg,
	struct motor_commission_results *res)
{
	if (agg == NULL || res == NULL || agg->count == 0U) {
		return -ENODATA;
	}

	float32_t n = (float32_t)agg->count;
	res->mech_capture_count = agg->count;
	res->inertia_kgm2 = agg->sum_j / n;
	res->viscous_friction_nm_per_rad_s = agg->sum_b / n;
	res->coulomb_friction_nm = agg->sum_tc / n;
	res->offset_friction_nm = agg->sum_t0 / n;
	res->mech_residual_rms_nm = agg->sum_rms / n;
	res->mech_r2 = agg->sum_r2 / n;
	res->mech_sample_count = (uint16_t)MIN(agg->sample_count / agg->count, UINT16_MAX);
	res->inertia_stddev_kgm2 =
		motor_commission_stddev(agg->count, agg->sum_j, agg->sum_j2);
	res->viscous_friction_stddev_nm_per_rad_s =
		motor_commission_stddev(agg->count, agg->sum_b, agg->sum_b2);
	res->coulomb_friction_stddev_nm =
		motor_commission_stddev(agg->count, agg->sum_tc, agg->sum_tc2);

	float32_t j_cv = res->inertia_stddev_kgm2 /
			 fmaxf(fabsf(res->inertia_kgm2), 1.0e-9f);
	float32_t b_cv = res->viscous_friction_stddev_nm_per_rad_s /
			 fmaxf(fabsf(res->viscous_friction_nm_per_rad_s), 1.0e-9f);
	float32_t tc_cv = res->coulomb_friction_stddev_nm /
			  fmaxf(fabsf(res->coulomb_friction_nm), 1.0e-9f);
	float32_t spread_penalty = clampf((0.50f * j_cv) + (0.25f * b_cv) +
					  (0.25f * tc_cv),
					  0.0f, 0.75f);
	res->mech_confidence = clampf(res->mech_r2, 0.0f, 1.0f) * (1.0f - spread_penalty);
	res->mech_valid = isfinite(res->inertia_kgm2) && res->inertia_kgm2 > 0.0f &&
			  isfinite(res->viscous_friction_nm_per_rad_s) &&
			  res->viscous_friction_nm_per_rad_s >= 0.0f &&
			  isfinite(res->coulomb_friction_nm) &&
			  res->coulomb_friction_nm >= 0.0f &&
			  isfinite(res->mech_r2) && res->mech_r2 >= 0.20f;
	return res->mech_valid ? 0 : -ERANGE;
}

static int motor_commission_mech_validate_fit(
	const struct motor_commission_results *fit,
	float32_t *rms_nm,
	uint16_t *sample_count)
{
	if (fit == NULL || rms_nm == NULL || sample_count == NULL ||
	    !fit->psi_f_valid || !fit->mech_valid) {
		return -EINVAL;
	}

	float32_t kt = motor_torque_gain_from_flux_pole_pairs(fit->psi_f_wb,
							      MOTOR_POLE_PAIRS);
	if (!isfinite(kt) || kt <= 0.0f) {
		return -EINVAL;
	}

	float32_t sum_sq = 0.0f;
	uint32_t count = 0U;
	for (uint32_t i = 0U; i < g_motor_params->commission.sample_count; i++) {
		const struct motor_commission_sample *s = &g_motor_params->commission.samples[i];
		if (!isfinite(s->mech_speed_rad_s) || !isfinite(s->mech_accel_rad_s2) ||
		    !isfinite(s->iq_a) || fabsf(s->mech_speed_rad_s) < 0.5f) {
			continue;
		}

		float32_t sign_term = (s->mech_speed_rad_s >= 0.0f) ? 1.0f : -1.0f;
		float32_t predicted_nm =
			(fit->inertia_kgm2 * s->mech_accel_rad_s2) +
			(fit->viscous_friction_nm_per_rad_s * s->mech_speed_rad_s) +
			(fit->coulomb_friction_nm * sign_term) +
			fit->offset_friction_nm;
		float32_t measured_nm = kt * s->iq_a;
		float32_t err = measured_nm - predicted_nm;
		sum_sq += err * err;
		count++;
	}

	if (count < 32U) {
		return -ENODATA;
	}

	*rms_nm = sqrtf(sum_sq / (float32_t)count);
	*sample_count = (uint16_t)MIN(count, UINT16_MAX);
	return isfinite(*rms_nm) ? 0 : -ERANGE;
}

int cmd_motor_commission_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct motor_commission_ctx *ctx = &g_motor_params->commission;
	uint32_t loops_total = 0U;
	uint32_t loops_done = 0U;

	if (ctx->stop_loop_count > ctx->start_loop_count) {
		loops_total = ctx->stop_loop_count - ctx->start_loop_count;
	}
	if (g_motor_params->control_loop_count > ctx->start_loop_count) {
		loops_done = g_motor_params->control_loop_count - ctx->start_loop_count;
	}
	if (loops_total == 0U) {
		loops_done = 0U;
	} else if (loops_done > loops_total) {
		 loops_done = loops_total;
	}

	shell_print(sh, "Commission Status:");
	shell_print(sh, "  Active:         %s", ctx->active ? "YES" : "NO");
	shell_print(sh, "  Mode:           %s", motor_commission_mode_to_string(ctx->mode));
	shell_print(sh, "  Stage:          %s", motor_commission_stage_to_string(ctx->stage));
	shell_print(sh, "  Expected mode:  %s",
		    motor_commission_expected_mode_to_string(ctx->expected_mode));
	shell_print(sh, "  Progress:       %u / %u loops", loops_done, loops_total);
	shell_print(sh, "  Samples:        accepted=%u rejected=%u stored=%u/%u",
		    ctx->accepted_samples, ctx->rejected_samples, ctx->sample_count,
		    MOTOR_COMMISSION_MAX_SAMPLES);
	shell_print(sh, "  Reject reasons: mode=%u disarmed=%u encoder=%u fault=%u sat=%u invalid=%u",
		    ctx->reject_mode_mismatch, ctx->reject_disarmed, ctx->reject_encoder,
		    ctx->reject_fault, ctx->reject_saturation, ctx->reject_data_invalid);
	shell_print(sh, "  Decimation:     %u", ctx->sample_decimation);
	shell_print(sh, "  Last abort:     %s", ctx->last_abort_reason);
	shell_print(sh, "  Motion thresh:  valid=%s pos=%.3f A neg=%.3f A rec=%.3f A",
		    ctx->results.iq_move_valid ? "YES" : "NO",
		    (double)ctx->results.iq_move_min_pos_a,
		    (double)ctx->results.iq_move_min_neg_a,
		    (double)ctx->results.iq_move_recommended_a);
	shell_print(sh, "  Motion counts:  posN=%u negN=%u warn=%u err=%u",
		    ctx->results.iq_move_pos_sample_count,
		    ctx->results.iq_move_neg_sample_count,
		    ctx->results.iq_move_warning_count,
		    ctx->results.iq_move_error_count);
	shell_print(sh, "  Iq->mech sign: %d", ctx->results.iq_to_mech_sign);
	shell_print(sh, "  Estimates:      psi_f=%s mech=%s",
		    ctx->results.psi_f_valid ? "VALID" : "INVALID",
		    ctx->results.mech_valid ? "VALID" : "INVALID");
	shell_print(sh, "  Flux fit:       psi_f=%.8f Wb bias=%.4f V rms=%.4f V R2=%.4f N=%u",
		    (double)ctx->results.psi_f_wb, (double)ctx->results.psi_f_bias_v,
		    (double)ctx->results.psi_f_residual_rms_v, (double)ctx->results.psi_f_r2,
		    ctx->results.psi_f_sample_count);
	shell_print(sh, "  Mech fit:       J=%.8f kgm2 B=%.8f Nm/(rad/s) Tc=%.8f Nm T0=%.8f Nm",
		    (double)ctx->results.inertia_kgm2,
		    (double)ctx->results.viscous_friction_nm_per_rad_s,
		    (double)ctx->results.coulomb_friction_nm,
		    (double)ctx->results.offset_friction_nm);
	shell_print(sh, "  Mech quality:   rms=%.6f Nm R2=%.4f N=%u",
		    (double)ctx->results.mech_residual_rms_nm,
		    (double)ctx->results.mech_r2,
		    ctx->results.mech_sample_count);
	shell_print(sh, "  Mech repeat:    runs=%u conf=%.2f Jstd=%.8f Bstd=%.8f Tcstd=%.8f",
		    ctx->results.mech_capture_count,
		    (double)ctx->results.mech_confidence,
		    (double)ctx->results.inertia_stddev_kgm2,
		    (double)ctx->results.viscous_friction_stddev_nm_per_rad_s,
		    (double)ctx->results.coulomb_friction_stddev_nm);
	shell_print(sh, "  Mech validate:  %s rms=%.6f Nm -> %s",
		    ctx->results.mech_validation_valid ? "YES" : "NO",
		    (double)ctx->results.mech_validation_residual_rms_nm,
		    ctx->results.mech_validation_pass ? "PASS" : "FAIL");
	shell_print(sh, "  Mapping:        valid=%s pass=%s confidence=%.2f",
		    ctx->results.mapping_valid ? "YES" : "NO",
		    ctx->results.mapping_pass ? "YES" : "NO",
		    (double)ctx->results.mapping_confidence);
	if (ctx->results.mapping_direction_valid) {
		shell_print(sh, "  Direction chk:  corr=%.4f -> %s",
			    (double)ctx->results.mapping_direction_corr,
			    ctx->results.mapping_direction_pass ? "PASS" : "FAIL");
	} else {
		shell_print(sh, "  Direction chk:  unavailable (run mechanical commissioning)");
	}
	if (ctx->results.mapping_offset_valid) {
		shell_print(sh, "  Offset chk:     |Id|/|Iq|=%.4f -> %s",
			    (double)ctx->results.mapping_offset_ratio,
			    ctx->results.mapping_offset_pass ? "PASS" : "FAIL");
	} else {
		shell_print(sh, "  Offset chk:     unavailable (run flux commissioning)");
	}
	if (ctx->results.mapping_pole_pairs_valid) {
		shell_print(sh, "  Pole-pair chk:  est=%.4f cfg=%u -> %s",
			    (double)ctx->results.mapping_pole_pairs_est,
			    MOTOR_POLE_PAIRS,
			    ctx->results.mapping_pole_pairs_pass ? "PASS" : "FAIL");
	} else {
		shell_print(sh, "  Pole-pair chk:  unavailable (run flux commissioning)");
	}
	shell_print(sh, "  Auto-tune:      staged=%s applied=%s last_err=%d (%s)",
		    ctx->auto_tune_valid ? "YES" : "NO",
		    ctx->auto_tune_applied ? "YES" : "NO",
		    ctx->auto_tune_last_error,
		    motor_commission_tune_error_to_string(ctx->auto_tune_last_error));
	shell_print(sh, "  Auto PI:        vel(kp=%.5f ki=%.5f iq=%.3f) pos(kp=%.5f ki=%.5f)",
		    (double)ctx->auto_tune_staged.velocity_kp_a_per_rad_s,
		    (double)ctx->auto_tune_staged.velocity_ki_a_per_rad,
		    (double)ctx->auto_tune_staged.velocity_iq_limit_a,
		    (double)ctx->auto_tune_staged.position_kp_rad_s_per_rad,
		    (double)ctx->auto_tune_staged.position_ki_rad_s2_per_rad);
	shell_print(sh, "  Auto MPR:       v(q=%.4f r=%.4f h=%u dIq=%.4f) p(q=%.4f qv=%.4f r=%.4f h=%u dVel=%.4f)",
		    (double)ctx->auto_tune_staged.velocity_mpr_q_speed,
		    (double)ctx->auto_tune_staged.velocity_mpr_r_delta_iq,
		    ctx->auto_tune_staged.velocity_mpr_horizon,
		    (double)ctx->auto_tune_staged.velocity_mpr_max_delta_iq_a,
		    (double)ctx->auto_tune_staged.position_mpr_q_position,
		    (double)ctx->auto_tune_staged.position_mpr_q_velocity_ff,
		    (double)ctx->auto_tune_staged.position_mpr_r_delta_velocity,
		    ctx->auto_tune_staged.position_mpr_horizon,
		    (double)ctx->auto_tune_staged.position_mpr_max_delta_velocity_rad_s);
	shell_print(sh, "  Auto DOB:       en=%s gain=%.5f tq_lim=%.5f iq_ff_lim=%.5f",
		    ctx->auto_tune_staged.velocity_dob_enable ? "YES" : "NO",
		    (double)ctx->auto_tune_staged.velocity_dob_observer_gain_nm_per_rad_s,
		    (double)ctx->auto_tune_staged.velocity_dob_torque_limit_nm,
		    (double)ctx->auto_tune_staged.velocity_dob_iq_ff_limit_a);
	motor_commission_print_tune_reject_flags(sh, ctx->auto_tune_staged.reject_flags);
	shell_print(sh, "  Active params:  psi_f=%.8f Wb Kt=%.8f Nm/A J=%.8f kgm2 B=%.8f Nm/(rad/s) Tc=%.8f Nm",
		    (double)g_motor_params->flux_linkage_wb_active,
		    (double)motor_torque_gain_resolve_active(g_motor_params),
		    (double)g_motor_params->inertia_kgm2_active,
		    (double)g_motor_params->viscous_friction_nm_per_rad_s_active,
		    (double)g_motor_params->coulomb_friction_nm_active);

	return 0;
}

int cmd_motor_commission_motion_threshold(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 5 && argc != 6) {
		shell_error(sh,
			    "Usage: motor commission motion threshold <start_a> <stop_a> <step_a> <hold_ms> [min_motion_deg]");
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
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	float32_t start_a = 0.0f;
	float32_t stop_a = 0.0f;
	float32_t step_a = 0.0f;
	float32_t min_motion_deg = 2.0f;
	uint32_t hold_ms = 0U;
	if (!shell_parse_finite_float(argv[1], &start_a) ||
	    !shell_parse_finite_float(argv[2], &stop_a) ||
	    !shell_parse_finite_float(argv[3], &step_a) ||
	    !shell_parse_u32(argv[4], &hold_ms) ||
	    (argc == 6 && !shell_parse_finite_float(argv[5], &min_motion_deg))) {
		shell_error(sh, "Invalid threshold arguments");
		return -EINVAL;
	}
	if (start_a <= 0.0f || stop_a < start_a || step_a <= 0.0f ||
	    stop_a > MOTOR_MAX_CURRENT_A || hold_ms < MOTOR_COMMISSION_MOTION_SAMPLE_MS ||
	    min_motion_deg <= 0.0f) {
		shell_error(sh,
			    "Expected 0 < start_a <= stop_a <= %.3f, step_a > 0, hold_ms >= %u, min_motion_deg > 0",
			    (double)MOTOR_MAX_CURRENT_A, MOTOR_COMMISSION_MOTION_SAMPLE_MS);
		return -ERANGE;
	}

	float32_t min_motion_rad = min_motion_deg * (PI_F32 / 180.0f);
	shell_print(sh,
		    "Motion threshold sweep: start=%.3f A stop=%.3f A step=%.3f A hold=%u ms min=%.3f deg",
		    (double)start_a, (double)stop_a, (double)step_a, hold_ms,
		    (double)min_motion_deg);

	int ret = motor_commission_run_motion_threshold(sh, start_a, stop_a, step_a,
							hold_ms, min_motion_rad);
	const struct motor_commission_results *res = &g_motor_params->commission.results;
	if (ret != 0 || !res->iq_move_valid) {
		shell_error(sh,
			    "Motion threshold failed (err %d): pos=%s neg=%s warn=%u err=%u",
			    ret,
			    res->iq_move_pos_valid ? "PASS" : "FAIL",
			    res->iq_move_neg_valid ? "PASS" : "FAIL",
			    res->iq_move_warning_count,
			    res->iq_move_error_count);
		return (ret != 0) ? ret : -ENODATA;
	}

	shell_print(sh,
		    "Motion threshold result: pos=%.3f A neg=%.3f A recommended=%.3f A iq_to_mech_sign=%d warn=%u err=%u",
		    (double)res->iq_move_min_pos_a,
		    (double)res->iq_move_min_neg_a,
		    (double)res->iq_move_recommended_a,
		    res->iq_to_mech_sign,
		    res->iq_move_warning_count,
		    res->iq_move_error_count);
	return 0;
}

int cmd_motor_commission_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	motor_commission_reset(&commission_ctx);
	motor_commission_encoder_clear_result();
	shell_print(sh, "Commission context cleared");
	return 0;
}

int cmd_motor_commission_abort(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	motor_commission_abort(&commission_ctx, "aborted by user");
	shell_print(sh, "Commissioning aborted");
	return 0;
}

int cmd_motor_commission_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_apply_results(&commission_ctx);
	if (ret == -ENOENT) {
		shell_error(sh, "No valid commissioning estimates to apply yet");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "Failed to apply commissioning results (err %d)", ret);
		return ret;
	}

	if (g_motor_params->commission.results.mapping_valid &&
	    !g_motor_params->commission.results.mapping_pass) {
		shell_warn(sh,
			   "Commissioning apply succeeded but mapping checks FAILED. Review 'motor commission status'.");
	}

	shell_print(sh, "Commissioning results applied to active runtime parameters");
	return 0;
}

int cmd_motor_commission_flux_run(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 7) {
		shell_error(sh,
			    "Usage: motor commission flux run <min_hz> <max_hz> <steps> <settle_ms> <sample_ms> <iq_limit_a>");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_flux_config cfg = {0};
	if (!shell_parse_finite_float(argv[1], &cfg.min_speed_hz) ||
	    !shell_parse_finite_float(argv[2], &cfg.max_speed_hz)) {
		shell_error(sh, "Invalid min_hz/max_hz");
		return -EINVAL;
	}
	uint32_t steps_u32 = 0U;
	if (!shell_parse_u32(argv[3], &steps_u32) || steps_u32 == 0U || steps_u32 > UINT16_MAX) {
		shell_error(sh, "steps must be 1..65535");
		return -EINVAL;
	}
	cfg.steps = (uint16_t)steps_u32;
	if (!shell_parse_u32(argv[4], &cfg.settle_ms) ||
	    !shell_parse_u32(argv[5], &cfg.sample_ms) ||
	    !shell_parse_finite_float(argv[6], &cfg.iq_limit_a)) {
		shell_error(sh, "Invalid settle_ms/sample_ms/iq_limit_a");
		return -EINVAL;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_start_flux(&commission_ctx, &cfg);
	if (ret < 0) {
		shell_error(sh, "Failed to start flux commissioning (err %d)", ret);
		return ret;
	}

	/* Prepare expected control mode and limits; capture gating handles transitions. */
	(void)motor_api_set_param("velocity_cl_iq_limit_A", cfg.iq_limit_a);
	(void)motor_api_request_online();
	(void)motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);

	shell_print(sh,
		    "Flux commissioning started: %.3f..%.3f Hz, steps=%u, settle=%u ms, sample=%u ms, iq_limit=%.3f A",
		    (double)cfg.min_speed_hz, (double)cfg.max_speed_hz, cfg.steps, cfg.settle_ms,
		    cfg.sample_ms, (double)cfg.iq_limit_a);
	shell_print(sh,
		    "Ensure control is armed and velocity commands are applied; capture expects ONLINE_VELOCITY_ENCODER.");
	return 0;
}

int cmd_motor_commission_mech_run(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 5) {
		shell_error(sh,
			    "Usage: motor commission mech run <base_hz> <dither_hz> <dither_period_ms> <duration_ms>");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_mech_config cfg = {0};
	if (!shell_parse_finite_float(argv[1], &cfg.base_speed_hz) ||
	    !shell_parse_finite_float(argv[2], &cfg.dither_speed_hz) ||
	    !shell_parse_u32(argv[3], &cfg.dither_period_ms) ||
	    !shell_parse_u32(argv[4], &cfg.duration_ms)) {
		shell_error(sh, "Invalid mechanical commissioning arguments");
		return -EINVAL;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_start_mech(&commission_ctx, &cfg);
	if (ret < 0) {
		shell_error(sh, "Failed to start mechanical commissioning (err %d)", ret);
		return ret;
	}

	(void)motor_api_request_online();
	(void)motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);

	shell_print(sh,
		    "Mechanical commissioning started: base=%.3f Hz, dither=%.3f Hz, dither_period=%u ms, duration=%u ms",
		    (double)cfg.base_speed_hz, (double)cfg.dither_speed_hz,
		    cfg.dither_period_ms, cfg.duration_ms);
	shell_print(sh,
		    "Capture expects ONLINE_VELOCITY_ENCODER; command velocity during the capture.");
	return 0;
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

	float32_t current_a = 0.0f;
	float32_t mech_hz = 0.0f;
	float32_t cycles_f = 0.0f;
	if (!shell_parse_finite_float(argv[1], &current_a) ||
	    !shell_parse_finite_float(argv[2], &mech_hz) ||
	    !shell_parse_finite_float(argv[3], &cycles_f) ||
	    fabsf(current_a) < 1.0e-6f ||
	    fabsf(mech_hz) < 1.0e-6f ||
	    cycles_f <= 0.0f) {
		shell_error(sh, "current_a and mech_hz must be non-zero; cycles must be > 0");
		return -EINVAL;
	}
	if (fabsf(current_a) > MOTOR_MAX_CURRENT_A) {
		shell_error(sh, "current_a exceeds motor current limit %.3f A",
			    (double)MOTOR_MAX_CURRENT_A);
		return -ERANGE;
	}

	motor_commission_encoder_clear_result();

	int ret = motor_api_request_online();
	if (ret != 0) {
		shell_error(sh, "Failed to request ONLINE state (err %d)", ret);
		return ret;
	}
	if (motor_api_get_state() != MOTOR_STATE_ONLINE_VELOCITY_GENERATED) {
		ret = motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_GENERATED);
		if (ret != 0) {
			shell_error(sh, "Failed to request velocity_generated mode (err %d)",
				    ret);
			return ret;
		}
		ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
						     MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS);
		if (ret != 0) {
			shell_error(sh, "Timed out waiting for velocity_generated mode");
			return ret;
		}
	}

	float32_t duration_ms_f = (cycles_f / fabsf(mech_hz)) * 1000.0f;
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

	struct motor_commission_encoder_trace_guard trace_guard;
	motor_commission_encoder_trace_force_on(&trace_guard);

	motor_commission_set_direct_current(0.0f, current_a);
	motor_commission_set_velocity_target_hz(mech_hz);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh,
		    "Encoder mapping detect: current=%.3f A velocity=%.3f Hz cycles=%.2f duration=%u ms sample=%u ms N=%u",
		    (double)current_a, (double)mech_hz, (double)cycles_f,
		    duration_ms, sample_period_ms, target_samples);

	uint32_t accepted = 0U;
	uint32_t last_trace_loop = g_motor_params->rt_fast.control_loop_count;
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
		if (encoder_detect_result.valid) {
			motor_encoder_pipeline_reset_stats();
			shell_print(sh, "Run 'motor commission encoder apply' to apply staged mapping.");
		}

	return ret;
}

int cmd_motor_commission_encoder_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Encoder Mapping Detect:");
	shell_print(sh, "  Staged valid:   %s", encoder_detect_result_valid ? "YES" : "NO");
	shell_print(sh, "  Duration/sample:%u ms / %u ms",
		    encoder_detect_duration_ms, encoder_detect_sample_period_ms);
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

int cmd_motor_commission_encoder_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

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
		wrap_rad_pi(encoder_detect_result.offset_mech_rad);
	g_motor_params->observer_elec_trim_rad = 0.0f;
	angle_observer_set_offset(&g_motor_params->observer,
				  g_motor_params->observer_alignment_offset_rad);
	g_motor_params->calibration.encoder_mapping_complete = true;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Encoder mapping applied: sign=%d offset=%.4f deg mechanical",
		    g_motor_params->encoder_direction_sign,
		    (double)(g_motor_params->observer_alignment_offset_rad * 180.0f / PI_F32));
	return 0;
}

int cmd_motor_commission_encoder_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_commission_encoder_clear_result();
	shell_print(sh, "Encoder mapping detect result cleared");
	return 0;
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

int cmd_motor_commission_auto_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct motor_commission_ctx *ctx = &g_motor_params->commission;
	shell_print(sh, "Commission Auto Status:");
	shell_print(sh, "  Staged:        %s", ctx->auto_tune_valid ? "YES" : "NO");
	shell_print(sh, "  Applied:       %s", ctx->auto_tune_applied ? "YES" : "NO");
	shell_print(sh, "  Last error:    %d (%s)",
		    ctx->auto_tune_last_error,
		    motor_commission_tune_error_to_string(ctx->auto_tune_last_error));
	shell_print(sh, "  Tuned BW:      vel=%.2f Hz pos=%.2f Hz",
		    (double)ctx->auto_tune_staged.velocity_bw_hz,
		    (double)ctx->auto_tune_staged.position_bw_hz);
	shell_print(sh, "  Tuned PI:      vel(kp=%.5f ki=%.5f iq=%.3f) pos(kp=%.5f ki=%.5f)",
		    (double)ctx->auto_tune_staged.velocity_kp_a_per_rad_s,
		    (double)ctx->auto_tune_staged.velocity_ki_a_per_rad,
		    (double)ctx->auto_tune_staged.velocity_iq_limit_a,
		    (double)ctx->auto_tune_staged.position_kp_rad_s_per_rad,
		    (double)ctx->auto_tune_staged.position_ki_rad_s2_per_rad);
	shell_print(sh, "  Tuned DOB:     en=%s gain=%.5f tq_lim=%.5f iq_ff_lim=%.5f",
		    ctx->auto_tune_staged.velocity_dob_enable ? "YES" : "NO",
		    (double)ctx->auto_tune_staged.velocity_dob_observer_gain_nm_per_rad_s,
		    (double)ctx->auto_tune_staged.velocity_dob_torque_limit_nm,
		    (double)ctx->auto_tune_staged.velocity_dob_iq_ff_limit_a);
	motor_commission_print_tune_reject_flags(sh, ctx->auto_tune_staged.reject_flags);
	return 0;
}

int cmd_motor_commission_auto_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_apply_staged_auto_tune(&commission_ctx);
	if (ret == -ENOENT) {
		shell_error(sh, "No staged auto-tune result to apply");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "Failed to apply staged auto-tune (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Auto-tuned parameters applied to active runtime configuration");
	return 0;
}

int cmd_motor_commission_auto_validate(const struct shell *sh, size_t argc, char **argv)
{
	float32_t max_hz = MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_MAX_HZ;
	uint32_t hold_ms = MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_HOLD_MS;

	if (argc > 3) {
		shell_error(sh, "Usage: motor commission auto validate [max_hz] [hold_ms]");
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
		shell_error(sh, "Control is disarmed; run 'motor arm' before validation");
		return -EACCES;
	}
	if (argc >= 2 && !shell_parse_finite_float(argv[1], &max_hz)) {
		shell_error(sh, "max_hz must be a finite number");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &hold_ms)) {
		shell_error(sh, "hold_ms must be an integer");
		return -EINVAL;
	}
	if (!isfinite(max_hz) || max_hz <= 0.0f) {
		shell_error(sh, "max_hz must be positive");
		return -EINVAL;
	}
	hold_ms = CLAMP(hold_ms,
			MOTOR_COMMISSION_AUTO_VALIDATE_MIN_HOLD_MS,
			MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS);

	float32_t profile_max_hz =
		g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	float32_t limited_max_hz =
		clampf(max_hz, 0.1f, fminf(profile_max_hz,
					    MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HZ_CAP));

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_apply_staged_auto_tune(&commission_ctx);
	if (ret == -ENOENT) {
		shell_error(sh, "No staged auto-tune result to validate");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "Failed to apply staged auto-tune before validation (err %d)",
			    ret);
		return ret;
	}

	/* Validate the conservative PI path first. DOB/MPR can be enabled after this passes. */
	(void)motor_api_set_param("outer_loop_mode", (float32_t)MOTOR_OUTER_LOOP_MODE_PI);
	(void)motor_api_set_param("velocity_dob_enable", 0.0f);

	ret = motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		shell_error(sh, "Failed to request velocity_encoder mode (err %d)", ret);
		return ret;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to enter velocity_encoder mode (err %d)", ret);
		return ret;
	}

	shell_print(sh,
		    "Auto validation: applied staged tune, PI outer loop, DOB disabled, max=%.3f Hz hold=%u ms",
		    (double)limited_max_hz, hold_ms);
	shell_print(sh,
		    "  active: psi_f=%.8f Wb Kt=%.8f Nm/A J=%.8f kgm2 B=%.8f Tc=%.8f",
		    (double)g_motor_params->flux_linkage_wb_active,
		    (double)motor_torque_gain_resolve_active(g_motor_params),
		    (double)g_motor_params->inertia_kgm2_active,
		    (double)g_motor_params->viscous_friction_nm_per_rad_s_active,
		    (double)g_motor_params->coulomb_friction_nm_active);

	for (uint32_t i = 0U; i < ARRAY_SIZE(motor_commission_validate_step_scale); i++) {
		float32_t target_hz = limited_max_hz * motor_commission_validate_step_scale[i];
		motor_commission_set_velocity_target_hz(target_hz);
		motor_command_feed_watchdog(g_motor_params);
		ret = motor_commission_wait_ms_or_fault(hold_ms);
		motor_commission_print_velocity_validation_sample(sh, target_hz);
		if (ret != 0) {
			shell_error(sh, "Validation stopped by motor fault (err %d)", ret);
			goto stop_velocity;
		}
	}

stop_velocity:
	motor_commission_set_velocity_target_hz(0.0f);
	motor_command_feed_watchdog(g_motor_params);
	if (ret == 0) {
		shell_print(sh, "Auto validation complete; velocity target returned to 0 Hz");
	}
	return ret;
}

static void motor_commission_auto_print_usage(const struct shell *sh)
{
	shell_error(sh,
		    "Usage: motor commission auto run [slow|confirm] [apply]");
}

int cmd_motor_commission_auto_run(const struct shell *sh, size_t argc, char **argv)
{
	bool apply_on_success = false;
	bool execute_motion = false;
	bool slow_profile = false;

	if (argc > 3) {
		motor_commission_auto_print_usage(sh);
		return -EINVAL;
	}
	for (size_t i = 1U; i < argc; i++) {
		if (strcmp(argv[i], "slow") == 0) {
			execute_motion = true;
			slow_profile = true;
		} else if (strcmp(argv[i], "confirm") == 0) {
			execute_motion = true;
			slow_profile = false;
		} else if (strcmp(argv[i], "apply") == 0 || strcmp(argv[i], "1") == 0 ||
			   strcmp(argv[i], "true") == 0) {
			apply_on_success = true;
		} else {
			motor_commission_auto_print_usage(sh);
			return -EINVAL;
		}
	}
	if (apply_on_success && !execute_motion) {
		shell_error(sh, "Refusing implicit motion. Use 'run slow apply' or 'run confirm apply'.");
		return -EACCES;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	float32_t max_velocity_hz = g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	if (!isfinite(max_velocity_hz) || max_velocity_hz < 0.10f) {
		max_velocity_hz = MOTOR_MAX_SPEED_HZ;
	}
	float32_t flux_cap_hz = slow_profile ?
					MOTOR_COMMISSION_AUTO_SLOW_FLUX_MAX_HZ :
					MOTOR_COMMISSION_AUTO_NORMAL_FLUX_MAX_HZ;
	float32_t flux_min_req_hz = slow_profile ?
					    MOTOR_COMMISSION_AUTO_SLOW_FLUX_MIN_HZ :
					    MOTOR_COMMISSION_AUTO_NORMAL_FLUX_MIN_HZ;
	float32_t mech_cap_hz = slow_profile ?
					MOTOR_COMMISSION_AUTO_SLOW_MECH_MAX_HZ :
					MOTOR_COMMISSION_AUTO_NORMAL_MECH_MAX_HZ;
	float32_t mech_base_req_hz = slow_profile ?
					     MOTOR_COMMISSION_AUTO_SLOW_MECH_BASE_HZ :
					     MOTOR_COMMISSION_AUTO_NORMAL_MECH_BASE_HZ;
	float32_t mech_dither_req_hz = slow_profile ?
					       MOTOR_COMMISSION_AUTO_SLOW_MECH_DITHER_HZ :
					       MOTOR_COMMISSION_AUTO_NORMAL_MECH_DITHER_HZ;
	float32_t planned_flux_max_hz = fminf(max_velocity_hz, flux_cap_hz);
	float32_t planned_flux_min_hz =
		fminf(flux_min_req_hz, fmaxf(0.05f, 0.50f * planned_flux_max_hz));
	float32_t planned_mech_upper_hz = fminf(max_velocity_hz, mech_cap_hz);
	float32_t planned_mech_base_hz =
		fminf(mech_base_req_hz, fmaxf(0.05f, 0.75f * planned_mech_upper_hz));
	float32_t planned_mech_dither_hz =
		fminf(mech_dither_req_hz,
		       fmaxf(0.0f, planned_mech_upper_hz - planned_mech_base_hz));
	float32_t planned_iq_limit_a = clampf(0.60f * g_motor_params->velocity_cl_iq_limit_A,
					      0.10f, MOTOR_MAX_CURRENT_A);

	if (planned_flux_max_hz < 0.10f || planned_mech_upper_hz < 0.10f) {
		shell_error(sh, "Profile max velocity is too low for auto commissioning");
		return -ERANGE;
	}
	if (!execute_motion) {
		shell_print(sh, "Auto commission plan only; no motion started.");
		shell_print(sh, "  Slow profile:  flux %.3f..%.3f Hz, mech base %.3f Hz dither %.3f Hz",
			    (double)fminf(MOTOR_COMMISSION_AUTO_SLOW_FLUX_MIN_HZ,
					  0.50f * MOTOR_COMMISSION_AUTO_SLOW_FLUX_MAX_HZ),
			    (double)MOTOR_COMMISSION_AUTO_SLOW_FLUX_MAX_HZ,
			    (double)MOTOR_COMMISSION_AUTO_SLOW_MECH_BASE_HZ,
			    (double)MOTOR_COMMISSION_AUTO_SLOW_MECH_DITHER_HZ);
		shell_print(sh, "  Confirm profile: flux %.3f..%.3f Hz, mech base %.3f Hz dither %.3f Hz",
			    (double)MOTOR_COMMISSION_AUTO_NORMAL_FLUX_MIN_HZ,
			    (double)MOTOR_COMMISSION_AUTO_NORMAL_FLUX_MAX_HZ,
			    (double)MOTOR_COMMISSION_AUTO_NORMAL_MECH_BASE_HZ,
			    (double)MOTOR_COMMISSION_AUTO_NORMAL_MECH_DITHER_HZ);
		shell_print(sh,
			    "Run 'motor commission auto run slow' for bounded bring-up, or 'motor commission auto run confirm' for the higher-speed profile.");
		return 0;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	if (motor_commission_is_active(&commission_ctx)) {
		shell_error(sh, "Commission capture is already active");
		return -EBUSY;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Calibration is not complete; run calibration before auto commission");
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before auto commission");
		return -EACCES;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	motor_commission_reset(&commission_ctx);

	struct motor_commission_flux_config flux_cfg = {0};
	struct motor_commission_mech_config mech_cfg = {0};
	struct motor_commission_tune_config tune_cfg = {0};
	float32_t iq_limit_default = planned_iq_limit_a;
	float32_t threshold_start_a = clampf(0.10f * MOTOR_MAX_CURRENT_A,
					     0.02f,
					     iq_limit_default);
	float32_t threshold_stop_a = iq_limit_default;
	float32_t threshold_step_a =
		fmaxf(0.01f, (threshold_stop_a - threshold_start_a) / 8.0f);

	shell_print(sh,
		    "Auto commission threshold sweep: start=%.3f A stop=%.3f A step=%.3f A",
		    (double)threshold_start_a,
		    (double)threshold_stop_a,
		    (double)threshold_step_a);
	int ret = motor_commission_run_motion_threshold(sh, threshold_start_a,
							threshold_stop_a,
							threshold_step_a,
							250U,
							2.0f * (PI_F32 / 180.0f));
	if (ret != 0 || !g_motor_params->commission.results.iq_move_valid) {
		g_motor_params->commission.auto_tune_last_error = (ret != 0) ? ret : -ENODATA;
		shell_error(sh, "Auto commission failed during motion threshold stage (err %d)",
			    g_motor_params->commission.auto_tune_last_error);
		return g_motor_params->commission.auto_tune_last_error;
	}

	float32_t iq_move_recommended = g_motor_params->commission.results.iq_move_recommended_a;
	iq_limit_default = clampf(fmaxf(iq_limit_default, 1.50f * iq_move_recommended),
				  0.10f,
				  MOTOR_MAX_CURRENT_A);

	flux_cfg.max_speed_hz = planned_flux_max_hz;
	flux_cfg.min_speed_hz = planned_flux_min_hz;
	flux_cfg.steps = slow_profile ? 4U : 5U;
	flux_cfg.settle_ms = slow_profile ? 500U : 400U;
	flux_cfg.sample_ms = slow_profile ? 500U : 400U;
	flux_cfg.iq_limit_a = iq_limit_default;

	float32_t mech_speed_upper_hz = planned_mech_upper_hz;
	mech_cfg.base_speed_hz = planned_mech_base_hz;
	mech_cfg.dither_speed_hz = planned_mech_dither_hz;

	float32_t max_accel_hz_s =
		fmaxf(g_motor_params->profile_max_accel_rad_s2 / (2.0f * PI_F32), 1.0f);
	float32_t half_cycle_ms =
		(MOTOR_COMMISSION_AUTO_MECH_ACCEL_MARGIN * 2.0f * mech_speed_upper_hz *
		 1000.0f) /
		max_accel_hz_s;
	mech_cfg.dither_period_ms =
		(uint32_t)ceilf(half_cycle_ms /
				(float32_t)ARRAY_SIZE(motor_commission_mech_step_pattern));
	mech_cfg.dither_period_ms =
		MAX(mech_cfg.dither_period_ms, slow_profile ? 1000U : 500U);
	mech_cfg.duration_ms = MOTOR_COMMISSION_AUTO_MECH_CYCLES_PER_CAPTURE *
			       2U * ARRAY_SIZE(motor_commission_mech_step_pattern) *
			       mech_cfg.dither_period_ms;

	(void)motor_commission_tune_config_default(&tune_cfg,
						   (float32_t)MOTOR_POLE_PAIRS,
						   1.0f / CONTROL_LOOP_FREQUENCY_HZ,
						   fmaxf(MOTOR_MAX_CURRENT_A, 0.1f),
						   fmaxf(g_motor_params->profile_max_velocity_rad_s,
							  1.0f),
						   fmaxf(g_motor_params->profile_max_accel_rad_s2,
							  1.0f));
	tune_cfg.iq_limit_a = flux_cfg.iq_limit_a;
	g_motor_params->commission.auto_tune_cfg = tune_cfg;

	shell_print(sh, "Auto commission start (%s profile):",
		    slow_profile ? "slow" : "confirmed");
	shell_print(sh, "  Motion threshold: pos=%.3f A neg=%.3f A rec=%.3f A",
		    (double)g_motor_params->commission.results.iq_move_min_pos_a,
		    (double)g_motor_params->commission.results.iq_move_min_neg_a,
		    (double)g_motor_params->commission.results.iq_move_recommended_a);
	shell_print(sh, "  Iq->mech sign: %d",
		    g_motor_params->commission.results.iq_to_mech_sign);
	shell_print(sh, "  Flux cfg: min=%.3f Hz max=%.3f Hz steps=%u settle=%u sample=%u iq=%.3f A",
		    (double)flux_cfg.min_speed_hz, (double)flux_cfg.max_speed_hz, flux_cfg.steps,
		    flux_cfg.settle_ms, flux_cfg.sample_ms, (double)flux_cfg.iq_limit_a);
	shell_print(sh,
		    "  Mech cfg: base=%.3f Hz dither=%.3f Hz accel=%.3f Hz/s dither_period=%u ms duration=%u ms",
		    (double)mech_cfg.base_speed_hz,
		    (double)mech_cfg.dither_speed_hz,
		    (double)max_accel_hz_s,
		    mech_cfg.dither_period_ms, mech_cfg.duration_ms);

	ret = motor_commission_auto_run_flux(sh, &flux_cfg);
	if (ret != 0) {
		g_motor_params->Id_setpoint_A = 0.0f;
		g_motor_params->Iq_setpoint_A = 0.0f;
		if (g_motor_params->commission.active) {
			struct motor_commission_runtime_ctx commission_ctx;
			motor_commission_ctx_from_global(&commission_ctx);
			motor_commission_abort(&commission_ctx, "auto flux failed");
		}
		g_motor_params->commission.auto_tune_last_error = ret;
		shell_error(sh, "Auto commission failed during flux stage (err %d)", ret);
		return ret;
	}

	shell_print(sh, "  Flux result: psi_f=%.8f Wb R2=%.4f rms=%.4fV N=%u",
		    (double)g_motor_params->commission.results.psi_f_wb,
		    (double)g_motor_params->commission.results.psi_f_r2,
		    (double)g_motor_params->commission.results.psi_f_residual_rms_v,
		    g_motor_params->commission.results.psi_f_sample_count);
	if (g_motor_params->commission.results.mapping_offset_valid ||
	    g_motor_params->commission.results.mapping_pole_pairs_valid) {
		shell_print(sh, "  Flux mapping: offset=%s pole_pairs=%s",
			    g_motor_params->commission.results.mapping_offset_pass ?
				    "PASS" :
				    "FAIL",
			    g_motor_params->commission.results.mapping_pole_pairs_pass ?
				    "PASS" :
				    "FAIL");
	}

	struct motor_commission_mech_aggregate mech_agg = {0};
	for (uint32_t attempt = 0U;
	     attempt < MOTOR_COMMISSION_AUTO_MECH_MAX_ATTEMPTS &&
	     mech_agg.count < MOTOR_COMMISSION_AUTO_MECH_RUNS;
	     attempt++) {
		ret = motor_commission_auto_run_mech(sh, &mech_cfg, true);
		if (ret != 0) {
			if (ret == -ERANGE) {
				shell_print(sh,
					    "  Mech attempt %u/%u rejected by fit quality",
					    (unsigned int)(attempt + 1U),
					    (unsigned int)MOTOR_COMMISSION_AUTO_MECH_MAX_ATTEMPTS);
				continue;
			}

			g_motor_params->Id_setpoint_A = 0.0f;
			g_motor_params->Iq_setpoint_A = 0.0f;
			if (g_motor_params->commission.active) {
				struct motor_commission_runtime_ctx commission_ctx;
				motor_commission_ctx_from_global(&commission_ctx);
				motor_commission_abort(&commission_ctx, "auto mech failed");
			}
			g_motor_params->commission.auto_tune_last_error = ret;
			shell_error(sh, "Auto commission failed during mechanical stage (err %d)",
				    ret);
			return ret;
		}

		const struct motor_commission_results *mech_res =
			&g_motor_params->commission.results;
		motor_commission_mech_aggregate_add(&mech_agg, mech_res);
		shell_print(sh,
			    "  Mech run %u/%u attempt %u: J=%.8f B=%.8f Tc=%.8f R2=%.4f rms=%.5f N=%u",
			    (unsigned int)mech_agg.count,
			    (unsigned int)MOTOR_COMMISSION_AUTO_MECH_RUNS,
			    (unsigned int)(attempt + 1U),
			    (double)mech_res->inertia_kgm2,
			    (double)mech_res->viscous_friction_nm_per_rad_s,
			    (double)mech_res->coulomb_friction_nm,
			    (double)mech_res->mech_r2,
			    (double)mech_res->mech_residual_rms_nm,
			    mech_res->mech_sample_count);
	}

	if (mech_agg.count < MOTOR_COMMISSION_AUTO_MECH_RUNS) {
		g_motor_params->commission.auto_tune_last_error = -ERANGE;
		shell_error(sh, "Auto commission only accepted %u/%u mechanical runs",
			    mech_agg.count,
			    MOTOR_COMMISSION_AUTO_MECH_RUNS);
		return -ERANGE;
	}

	ret = motor_commission_mech_aggregate_finalize(&mech_agg,
						       &g_motor_params->commission.results);
	if (ret != 0 || !g_motor_params->commission.results.mech_valid) {
		g_motor_params->commission.auto_tune_last_error = (ret != 0) ? ret : -ERANGE;
		shell_error(sh, "Auto commission failed to aggregate mechanical runs (err %d)",
			    g_motor_params->commission.auto_tune_last_error);
		return g_motor_params->commission.auto_tune_last_error;
	}

	struct motor_commission_results aggregate_results = g_motor_params->commission.results;
	ret = motor_commission_auto_run_mech(sh, &mech_cfg, false);
	if (ret != 0) {
		g_motor_params->commission.results = aggregate_results;
		g_motor_params->commission.auto_tune_last_error = ret;
		shell_error(sh, "Auto commission failed during mechanical validation (err %d)",
			    ret);
		return ret;
	}

	float32_t validation_rms_nm = 0.0f;
	uint16_t validation_samples = 0U;
	ret = motor_commission_mech_validate_fit(&aggregate_results,
						 &validation_rms_nm,
						 &validation_samples);
	aggregate_results.mech_validation_valid = (ret == 0);
	aggregate_results.mech_validation_residual_rms_nm =
		(ret == 0) ? validation_rms_nm : 0.0f;
	aggregate_results.mech_validation_pass =
		(ret == 0) &&
		(validation_rms_nm <= MOTOR_COMMISSION_AUTO_MECH_VALIDATE_RMS_NM);
	g_motor_params->commission.results = aggregate_results;
	if (!g_motor_params->commission.results.mech_validation_pass) {
		g_motor_params->commission.auto_tune_last_error = (ret != 0) ? ret : -ERANGE;
		shell_error(sh,
			    "Auto commission mechanical validation failed (err %d rms=%.6f Nm N=%u)",
			    g_motor_params->commission.auto_tune_last_error,
			    (double)validation_rms_nm,
			    validation_samples);
		return g_motor_params->commission.auto_tune_last_error;
	}

	shell_print(sh, "  Mech aggregate: runs=%u J=%.8f+/-%.8f B=%.8f+/-%.8f Tc=%.8f+/-%.8f R2=%.4f conf=%.2f",
		    g_motor_params->commission.results.mech_capture_count,
		    (double)g_motor_params->commission.results.inertia_kgm2,
		    (double)g_motor_params->commission.results.inertia_stddev_kgm2,
		    (double)g_motor_params->commission.results.viscous_friction_nm_per_rad_s,
		    (double)g_motor_params->commission.results.viscous_friction_stddev_nm_per_rad_s,
		    (double)g_motor_params->commission.results.coulomb_friction_nm,
		    (double)g_motor_params->commission.results.coulomb_friction_stddev_nm,
		    (double)g_motor_params->commission.results.mech_r2,
		    (double)g_motor_params->commission.results.mech_confidence);
	shell_print(sh, "  Mech validation: rms=%.6f Nm N=%u -> PASS",
		    (double)g_motor_params->commission.results.mech_validation_residual_rms_nm,
		    validation_samples);
	if (g_motor_params->commission.results.mapping_direction_valid) {
		shell_print(sh, "  Mech mapping: direction=%s corr=%.4f",
			    g_motor_params->commission.results.mapping_direction_pass ?
				    "PASS" :
				    "FAIL",
			    (double)g_motor_params->commission.results.mapping_direction_corr);
	}
	if (g_motor_params->commission.results.mapping_valid) {
		shell_print(sh, "  Mapping summary: PASS=%s confidence=%.2f",
			    g_motor_params->commission.results.mapping_pass ?
				    "YES" :
				    "NO",
			    (double)g_motor_params->commission.results.mapping_confidence);
	}

	ret = motor_commission_stage_auto_tune(&commission_ctx, &tune_cfg);
	if (ret != 0) {
		shell_error(sh, "Auto tune staging failed (err %d: %s)", ret,
			    motor_commission_tune_error_to_string(ret));
		motor_commission_print_tune_reject_flags(
			sh, g_motor_params->commission.auto_tune_staged.reject_flags);
		return ret;
	}

	shell_print(sh,
		    "  Tuned defaults staged: vel(kp=%.5f ki=%.5f iq=%.3f) pos(kp=%.5f ki=%.5f) dob(gain=%.5f tq=%.5f iqff=%.5f)",
		    (double)g_motor_params->commission.auto_tune_staged.velocity_kp_a_per_rad_s,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_ki_a_per_rad,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_iq_limit_a,
		    (double)g_motor_params->commission.auto_tune_staged.position_kp_rad_s_per_rad,
		    (double)g_motor_params->commission.auto_tune_staged.position_ki_rad_s2_per_rad,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_dob_observer_gain_nm_per_rad_s,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_dob_torque_limit_nm,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_dob_iq_ff_limit_a);

	if (apply_on_success) {
		ret = motor_commission_apply_staged_auto_tune(&commission_ctx);
		if (ret != 0) {
			shell_error(sh, "Auto commission completed but apply failed (err %d)", ret);
			return ret;
		}
		shell_print(sh, "Auto commission complete and applied");
	} else {
		shell_print(sh, "Auto commission complete. Run 'motor commission auto apply' to apply.");
	}

	return 0;
}
