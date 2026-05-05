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
#include "shell_commission_internal.h"
#include "shell_commands_motion.h"
#include "shell_commands_state.h"
#include "motor/runtime/commission_runtime.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "shell_parse.h"
#include "config.h"
#include "motor_commission_adapter.h"
#include "motor/motion/traj.h"
#include "motor/motion/motion_planner.h"
#include "motor/motion/motion_profile.h"
#include "motor/math/math_constants.h"
#include "motor/math/angle_wrap.h"
#include "motor/calibration/encoder_map_detect.h"
#include "motor/observers/angle_observer.h"
#include "motor_torque.h"
#include "motor_encoder_acquisition.h"

#define MOTOR_COMMISSION_AUTO_POLL_MS 10U
#define MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS 8000U
#define MOTOR_COMMISSION_STANDARD_IDENT_TIMEOUT_MS 20000U

#define MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_MOTION_SAMPLE_MS 5U
#define MOTOR_COMMISSION_MOTION_MIN_SAMPLES 4U
#define MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS 80U




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

static const char *motor_commission_mech_reject_to_string(uint8_t reason)
{
	switch (reason) {
	case MOTOR_COMMISSION_MECH_REJECT_NONE:
		return "none";
	case MOTOR_COMMISSION_MECH_REJECT_KT_INVALID:
		return "kt_invalid";
	case MOTOR_COMMISSION_MECH_REJECT_SAMPLES:
		return "samples";
	case MOTOR_COMMISSION_MECH_REJECT_SOLVER:
		return "solver";
	case MOTOR_COMMISSION_MECH_REJECT_FINALIZE:
		return "finalize";
	case MOTOR_COMMISSION_MECH_REJECT_VISCOUS_NEGATIVE:
		return "viscous_negative";
	case MOTOR_COMMISSION_MECH_REJECT_FIT_INVALID:
		return "fit_invalid";
	default:
		return "unknown";
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



int motor_commission_wait_for_mode(enum motor_state mode, uint32_t timeout_ms)
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

static int motor_commission_wait_for_state_commission(uint32_t timeout_ms)
{
	uint32_t start_ms = k_uptime_get_32();
	bool observed_commission = false;

	while ((k_uptime_get_32() - start_ms) < timeout_ms) {
		int state = motor_api_get_state();
		if (state == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		if (g_motor_params != NULL &&
		    (g_motor_params->calibration.running ||
		     g_motor_params->calibration.mode == MOTOR_CALIBRATION_MODE_COMMISSIONING)) {
			observed_commission = true;
		}
		if (g_motor_params != NULL &&
		    observed_commission &&
		    g_motor_params->calibration.commissioning_complete &&
		    !g_motor_params->calibration.running &&
		    state == MOTOR_STATE_IDLE) {
			return 0;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return -ETIMEDOUT;
}

static void motor_commission_restore_timeout(uint32_t timeout_ms)
{
	if (g_motor_params == NULL) {
		return;
	}

	g_motor_params->command_timeout_ms = timeout_ms;
	if (timeout_ms == 0U) {
		g_motor_params->command_timeout_latched = false;
	}
	motor_command_feed_watchdog(g_motor_params);
}

static void motor_commission_apply_auto_iq_floor(void)
{
	if (g_motor_params == NULL ||
	    g_motor_params->velocity_cl_iq_limit_A >= COMMISSION_STANDARD_MIN_AUTO_IQ_A) {
		return;
	}

	g_motor_params->velocity_cl_iq_limit_A = COMMISSION_STANDARD_MIN_AUTO_IQ_A;
	g_motor_params->velocity_mpr_cfg.iq_limit_a = COMMISSION_STANDARD_MIN_AUTO_IQ_A;
	if (g_motor_params->velocity_dob_cfg.iq_ff_limit_a <= 0.0f ||
	    g_motor_params->velocity_dob_cfg.iq_ff_limit_a >
		    COMMISSION_STANDARD_MIN_AUTO_IQ_A) {
		g_motor_params->velocity_dob_cfg.iq_ff_limit_a =
			COMMISSION_STANDARD_MIN_AUTO_IQ_A;
	}
}

static void motor_commission_standard_cleanup(uint32_t saved_timeout_ms)
{
	if (g_motor_params != NULL) {
		motor_commission_motion_stop_current();
		motor_commission_set_velocity_target_hz(0.0f);
		g_motor_params->calibration.requested_online_mode =
			MOTOR_STATE_ONLINE_VELOCITY_GENERATED;
	}
	(void)motor_commission_request_idle_disarmed();
	motor_commission_restore_timeout(saved_timeout_ms);
}

static void motor_commission_run_usage(const struct shell *sh)
{
	shell_error(sh, "Usage: motor commission run [slow|confirm] [apply]");
}

int cmd_motor_commission_run(const struct shell *sh, size_t argc, char **argv)
{
	bool confirm_profile = false;
	bool apply_on_success = false;

	if (argc > 3U) {
		motor_commission_run_usage(sh);
		return -EINVAL;
	}
	for (size_t i = 1U; i < argc; i++) {
		if (strcmp(argv[i], "slow") == 0) {
			confirm_profile = false;
		} else if (strcmp(argv[i], "confirm") == 0) {
			confirm_profile = true;
		} else if (strcmp(argv[i], "apply") == 0 ||
			   strcmp(argv[i], "1") == 0 ||
			   strcmp(argv[i], "true") == 0) {
			apply_on_success = true;
		} else {
			motor_commission_run_usage(sh);
			return -EINVAL;
		}
	}

	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const uint32_t saved_timeout_ms = g_motor_params->command_timeout_ms;
	g_motor_params->command_timeout_ms = 0U;
	g_motor_params->command_timeout_latched = false;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Standard commissioning workflow started (%s%s)",
		    confirm_profile ? "confirm" : "slow",
		    apply_on_success ? ", apply" : "");
	shell_print(sh, "[0/4] Reset to safe idle and clear stale commissioning data");
	int ret = motor_commission_request_idle_disarmed();
	if (ret != 0) {
		shell_error(sh, "Failed to enter IDLE before commissioning (err %d)", ret);
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}
	ret = cmd_motor_commission_clear(sh, 0, NULL);
	if (ret != 0) {
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		ret = motor_api_clear_error();
		if (ret != 0) {
			shell_error(sh, "Failed to clear error state (err %d)", ret);
			motor_commission_standard_cleanup(saved_timeout_ms);
			return ret;
		}
	}

	shell_print(sh, "[1/4] Electrical identification: current offsets, R/L, Rs");
	ret = motor_api_request_commission();
	if (ret != 0) {
		shell_error(sh, "Failed to request state commissioning (err %d)", ret);
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}
	ret = motor_commission_wait_for_state_commission(
		MOTOR_COMMISSION_STANDARD_IDENT_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Electrical identification failed/timed out (err %d)", ret);
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}
	shell_print(sh, "  Rs=%.4f ohm L=%.6f H R/L=%.1f rad/s",
		    (double)g_motor_params->Rs_measured_ohm,
		    (double)g_motor_params->Ls_measured_H,
		    (double)g_motor_params->R_over_L_measured);

	shell_print(sh, "[2/4] Encoder commutation mapping and current smoke test");
	char *boot_argv[] = { "boot" };
	ret = cmd_motor_commission_boot(sh, ARRAY_SIZE(boot_argv), boot_argv);
	if (ret != 0) {
		shell_error(sh, "Encoder boot commissioning failed (err %d)", ret);
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}

	shell_print(sh, "[3/4] Flux and mechanical identification");
	motor_commission_apply_auto_iq_floor();
	ret = cmd_motor_arm(sh, 0, NULL);
	if (ret != 0) {
		shell_error(sh, "Failed to arm before auto commissioning (err %d)", ret);
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}
	char *auto_argv[3] = {
		"run",
		confirm_profile ? "confirm" : "slow",
		"apply",
	};
	size_t auto_argc = apply_on_success ? 3U : 2U;
	ret = cmd_motor_commission_auto_run(sh, auto_argc, auto_argv);
	if (ret != 0) {
		shell_error(sh, "Auto identify/tune failed (err %d)", ret);
		motor_commission_standard_cleanup(saved_timeout_ms);
		return ret;
	}

	shell_print(sh, "[4/4] Return to safe idle");
	motor_commission_standard_cleanup(saved_timeout_ms);
	shell_print(sh, "Standard commissioning workflow complete");
	return 0;
}




static bool motor_commission_motion_sample_clean(
	const struct motor_encoder_raw_trace_sample *sample)
{
	return sample != NULL &&
	       sample->sample_fresh != 0U &&
	       sample->sample_error == 0U &&
	       sample->sample_io_fault == 0U &&
	       isfinite(sample->raw_angle_rad) &&
	       isfinite(sample->control_angle_rad);
}

struct motor_commission_motion_threshold_result {
	float32_t threshold_a;
	float32_t best_net_motion_rad;
	float32_t best_abs_motion_rad;
	uint16_t sample_count;
	uint16_t warning_count;
	uint16_t error_count;
	bool valid;
};

void motor_commission_motion_stop_current(void)
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

int motor_commission_request_idle_disarmed(void)
{
	if (g_motor_params == NULL) {
		return -ENODEV;
	}

	motor_commission_motion_stop_current();
	motor_commission_set_velocity_target_hz(0.0f);
	atomic_set(&g_motor_params->control_armed, 0);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_api_request_idle();
	if (ret != 0) {
		return ret;
	}

	return motor_commission_wait_for_mode(MOTOR_STATE_IDLE,
					      MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS);
}

int motor_commission_motion_measure_current(
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
			prev_angle_rad = raw_trace.control_angle_rad;
			have_prev = true;
			continue;
		}

		float32_t delta_rad = wrap_rad_pi(raw_trace.control_angle_rad - prev_angle_rad);
		if (!isfinite(delta_rad)) {
			out->error_count++;
			continue;
		}
		out->net_motion_rad += delta_rad;
		out->abs_motion_rad += fabsf(delta_rad);
		prev_angle_rad = raw_trace.control_angle_rad;
	}

	out->valid = out->sample_count >= MOTOR_COMMISSION_MOTION_MIN_SAMPLES &&
		     out->error_count <= MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES &&
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

int motor_commission_run_motion_threshold(
	const struct shell *sh,
	float32_t start_a,
	float32_t stop_a,
	float32_t step_a,
	uint32_t hold_ms,
	float32_t min_motion_rad)
{
	int ret = motor_commission_request_idle_disarmed();
	if (ret != 0) {
		return ret;
	}

	ret = cmd_motor_arm(sh, 0, NULL);
	if (ret != 0) {
		return ret;
	}

	ret = cmd_motor_state_mode_current_encoder(sh, 0, NULL);
	if (ret != 0) {
		return ret;
	}
	ret = motor_api_request_online();
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

void motor_commission_ctx_from_global(struct motor_commission_runtime_ctx *ctx)
{
	motor_commission_runtime_ctx_init(ctx, g_motor_params);
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
	shell_print(sh, "  Mech reject:    reason=%s err=%d tq_sign=%d",
		    motor_commission_mech_reject_to_string(ctx->results.mech_reject_reason),
		    (int)ctx->results.mech_finalize_error,
		    ctx->results.mech_fit_torque_sign);
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
