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

#define MOTOR_COMMISSION_AUTO_POLL_MS 10U
#define MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS 8000U
#define MOTOR_COMMISSION_AUTO_POST_WAIT_MS 2500U
#define MOTOR_COMMISSION_ENCODER_MAX_SAMPLES 512U
#define MOTOR_COMMISSION_ENCODER_MIN_SAMPLE_MS 5U
#define MOTOR_COMMISSION_ENCODER_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_MOTION_SAMPLE_MS 5U
#define MOTOR_COMMISSION_MOTION_MIN_SAMPLES 4U
#define MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS 80U

static struct motor_encoder_map_detect_sample encoder_detect_samples[
	MOTOR_COMMISSION_ENCODER_MAX_SAMPLES];
static struct motor_encoder_map_detect_result encoder_detect_result;
static bool encoder_detect_result_valid;
static uint32_t encoder_detect_duration_ms;
static uint32_t encoder_detect_sample_period_ms;

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
	(void)motor_api_set_param("Id_setpoint_A", 0.0f);
	(void)motor_api_set_param("Iq_setpoint_A", 0.0f);
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
					     MAX(pos.threshold_a, neg.threshold_a) :
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

static uint32_t motor_commission_prbs_next(uint32_t state)
{
	state ^= (state << 13);
	state ^= (state >> 17);
	state ^= (state << 5);
	return state;
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
					  const struct motor_commission_mech_config *cfg)
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

	uint32_t prbs_state = 0x5A5AA5A5u;
	uint32_t next_tick_ms = k_uptime_get_32();
	uint32_t run_start_ms = next_tick_ms;
	while (g_motor_params->commission.active) {
		uint32_t now_ms = k_uptime_get_32();
		if ((int32_t)(now_ms - next_tick_ms) >= 0) {
			prbs_state = motor_commission_prbs_next(prbs_state);
			uint32_t elapsed_ms = now_ms - run_start_ms;
			float32_t direction =
				(elapsed_ms < (cfg->duration_ms / 2U)) ? 1.0f : -1.0f;
			float32_t dither = (prbs_state & 1U) ? cfg->dither_speed_hz :
							       -cfg->dither_speed_hz;
			float32_t target_hz = direction * (cfg->base_speed_hz + dither);
			motor_commission_set_velocity_target_hz(target_hz);
			next_tick_ms += cfg->dither_period_ms;
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
	if (!g_motor_params->commission.results.mech_valid) {
		return -ERANGE;
	}

	shell_print(sh, "  Mech excitation complete");
	return 0;
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

	(void)motor_api_set_param("Id_setpoint_A", 0.0f);
	(void)motor_api_set_param("Iq_setpoint_A", current_a);
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
		.max_offset_residual_rad = 0.35f,
		.max_direction_residual_rad = 0.50f,
		.min_direction_correlation = 0.70f,
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

int cmd_motor_commission_auto_run(const struct shell *sh, size_t argc, char **argv)
{
	bool apply_on_success = false;

	if (argc == 2) {
		if (strcmp(argv[1], "apply") == 0 || strcmp(argv[1], "1") == 0 ||
		    strcmp(argv[1], "true") == 0) {
			apply_on_success = true;
		} else {
			shell_error(sh, "Usage: motor commission auto run [apply]");
			return -EINVAL;
		}
	} else if (argc != 1) {
		shell_error(sh, "Usage: motor commission auto run [apply]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
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
	float32_t max_velocity_hz = g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	float32_t iq_limit_default = clampf(0.60f * g_motor_params->velocity_cl_iq_limit_A,
					    0.10f, MOTOR_MAX_CURRENT_A);
	float32_t threshold_start_a = clampf(0.10f * MOTOR_MAX_CURRENT_A,
					     0.02f,
					     iq_limit_default);
	float32_t threshold_stop_a = iq_limit_default;
	float32_t threshold_step_a = MAX(0.01f,
					 (threshold_stop_a - threshold_start_a) / 8.0f);

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
	iq_limit_default = clampf(MAX(iq_limit_default, 1.50f * iq_move_recommended),
				  0.10f,
				  MOTOR_MAX_CURRENT_A);

	flux_cfg.max_speed_hz = clampf(max_velocity_hz * 0.25f, 3.0f, 20.0f);
	flux_cfg.min_speed_hz = clampf(flux_cfg.max_speed_hz * 0.25f,
				       1.0f,
				       flux_cfg.max_speed_hz - 0.5f);
	flux_cfg.steps = 6U;
	flux_cfg.settle_ms = 250U;
	flux_cfg.sample_ms = 250U;
	flux_cfg.iq_limit_a = iq_limit_default;

	mech_cfg.base_speed_hz = clampf(flux_cfg.max_speed_hz * 0.45f, 2.0f, 8.0f);
	mech_cfg.dither_speed_hz = clampf(0.25f * mech_cfg.base_speed_hz,
					  0.5f,
					  0.50f * mech_cfg.base_speed_hz);
	mech_cfg.dither_period_ms = 300U;
	mech_cfg.duration_ms = 5000U;

	(void)motor_commission_tune_config_default(&tune_cfg,
						   (float32_t)MOTOR_POLE_PAIRS,
						   1.0f / CONTROL_LOOP_FREQUENCY_HZ,
						   MAX(MOTOR_MAX_CURRENT_A, 0.1f),
						   MAX(g_motor_params->profile_max_velocity_rad_s, 1.0f),
						   MAX(g_motor_params->profile_max_accel_rad_s2, 1.0f));
	tune_cfg.iq_limit_a = flux_cfg.iq_limit_a;
	g_motor_params->commission.auto_tune_cfg = tune_cfg;

	shell_print(sh, "Auto commission start:");
	shell_print(sh, "  Motion threshold: pos=%.3f A neg=%.3f A rec=%.3f A",
		    (double)g_motor_params->commission.results.iq_move_min_pos_a,
		    (double)g_motor_params->commission.results.iq_move_min_neg_a,
		    (double)g_motor_params->commission.results.iq_move_recommended_a);
	shell_print(sh, "  Iq->mech sign: %d",
		    g_motor_params->commission.results.iq_to_mech_sign);
	shell_print(sh, "  Flux cfg: min=%.3f Hz max=%.3f Hz steps=%u settle=%u sample=%u iq=%.3f A",
		    (double)flux_cfg.min_speed_hz, (double)flux_cfg.max_speed_hz, flux_cfg.steps,
		    flux_cfg.settle_ms, flux_cfg.sample_ms, (double)flux_cfg.iq_limit_a);
	shell_print(sh, "  Mech cfg: base=%.3f Hz dither=%.3f Hz dither_period=%u ms duration=%u ms",
		    (double)mech_cfg.base_speed_hz,
		    (double)mech_cfg.dither_speed_hz,
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

	ret = motor_commission_auto_run_mech(sh, &mech_cfg);
	if (ret != 0) {
		g_motor_params->Id_setpoint_A = 0.0f;
		g_motor_params->Iq_setpoint_A = 0.0f;
		if (g_motor_params->commission.active) {
			struct motor_commission_runtime_ctx commission_ctx;
			motor_commission_ctx_from_global(&commission_ctx);
			motor_commission_abort(&commission_ctx, "auto mech failed");
		}
		g_motor_params->commission.auto_tune_last_error = ret;
		shell_error(sh, "Auto commission failed during mechanical stage (err %d)", ret);
		return ret;
	}

	shell_print(sh, "  Mech result: J=%.8f B=%.8f Tc=%.8f R2=%.4f rms=%.5f N=%u",
		    (double)g_motor_params->commission.results.inertia_kgm2,
		    (double)g_motor_params->commission.results.viscous_friction_nm_per_rad_s,
		    (double)g_motor_params->commission.results.coulomb_friction_nm,
		    (double)g_motor_params->commission.results.mech_r2,
		    (double)g_motor_params->commission.results.mech_residual_rms_nm,
		    g_motor_params->commission.results.mech_sample_count);
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
