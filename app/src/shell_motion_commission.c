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
#include "motor_torque.h"

#define MOTOR_COMMISSION_AUTO_POLL_MS 10U
#define MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS 8000U
#define MOTOR_COMMISSION_AUTO_POST_WAIT_MS 2500U
#define MOTOR_COMMISSION_AUTO_SPINUP_MS 1200U

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

	motor_commission_set_velocity_target_hz(cfg->coast_speed_hz);
	uint32_t spinup_start_ms = k_uptime_get_32();
	while ((k_uptime_get_32() - spinup_start_ms) < MOTOR_COMMISSION_AUTO_SPINUP_MS) {
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	ret = motor_post_mode_change(MOTOR_STATE_ONLINE_CURRENT_ENCODER);
	if (ret != 0) {
		return ret;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_CURRENT_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		return ret;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	ret = motor_commission_start_mech(&commission_ctx, cfg);
	if (ret != 0) {
		return ret;
	}

	uint32_t prbs_state = 0x5A5AA5A5u;
	uint32_t next_tick_ms = k_uptime_get_32();
	while (g_motor_params->commission.active) {
		uint32_t now_ms = k_uptime_get_32();
		if ((int32_t)(now_ms - next_tick_ms) >= 0) {
			prbs_state = motor_commission_prbs_next(prbs_state);
			float32_t sign = (prbs_state & 1U) ? 1.0f : -1.0f;
			g_motor_params->Id_setpoint_A = 0.0f;
			g_motor_params->Iq_setpoint_A = sign * cfg->prbs_amp_a;
			next_tick_ms += cfg->prbs_period_ms;
		}

		if (g_motor_params->commission.stage == MOTOR_COMMISSION_STAGE_ABORTED) {
			return -ECANCELED;
		}
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}

		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	g_motor_params->Id_setpoint_A = 0.0f;
	g_motor_params->Iq_setpoint_A = 0.0f;

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
	if (loops_total > 0U && loops_done > loops_total) {
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
			    "Usage: motor commission mech run <coast_hz> <prbs_amp_a> <prbs_period_ms> <duration_ms>");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_mech_config cfg = {0};
	if (!shell_parse_finite_float(argv[1], &cfg.coast_speed_hz) ||
	    !shell_parse_finite_float(argv[2], &cfg.prbs_amp_a) ||
	    !shell_parse_u32(argv[3], &cfg.prbs_period_ms) ||
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
	(void)motor_post_mode_change(MOTOR_STATE_ONLINE_CURRENT_ENCODER);

	shell_print(sh,
		    "Mechanical commissioning started: coast=%.3f Hz, prbs_amp=%.3f A, prbs_period=%u ms, duration=%u ms",
		    (double)cfg.coast_speed_hz, (double)cfg.prbs_amp_a, cfg.prbs_period_ms,
		    cfg.duration_ms);
	shell_print(sh,
		    "Ensure control is armed and current excitation is applied; capture expects ONLINE_CURRENT_ENCODER.");
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

	flux_cfg.max_speed_hz = clampf(max_velocity_hz * 0.25f, 3.0f, 20.0f);
	flux_cfg.min_speed_hz = clampf(flux_cfg.max_speed_hz * 0.25f,
				       1.0f,
				       flux_cfg.max_speed_hz - 0.5f);
	flux_cfg.steps = 6U;
	flux_cfg.settle_ms = 250U;
	flux_cfg.sample_ms = 250U;
	flux_cfg.iq_limit_a = iq_limit_default;

	mech_cfg.coast_speed_hz = clampf(flux_cfg.max_speed_hz * 0.5f, 2.0f, 10.0f);
	mech_cfg.prbs_amp_a = clampf(0.35f * flux_cfg.iq_limit_a, 0.05f,
				     flux_cfg.iq_limit_a);
	mech_cfg.prbs_period_ms = 20U;
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
	shell_print(sh, "  Flux cfg: min=%.3f Hz max=%.3f Hz steps=%u settle=%u sample=%u iq=%.3f A",
		    (double)flux_cfg.min_speed_hz, (double)flux_cfg.max_speed_hz, flux_cfg.steps,
		    flux_cfg.settle_ms, flux_cfg.sample_ms, (double)flux_cfg.iq_limit_a);
	shell_print(sh, "  Mech cfg: coast=%.3f Hz prbs_amp=%.3f A prbs_period=%u ms duration=%u ms",
		    (double)mech_cfg.coast_speed_hz, (double)mech_cfg.prbs_amp_a,
		    mech_cfg.prbs_period_ms, mech_cfg.duration_ms);

	int ret = motor_commission_auto_run_flux(sh, &flux_cfg);
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
