/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "shell_commands_commission.h"
#include "shell_commands_motion.h"
#include "motor_commission.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "shell_parse.h"

static int motor_post_mode_change(enum motor_state target_mode)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_MODE_CHANGE,
		.target_mode = target_mode,
	};
	extern struct k_msgq motor_event_queue;
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);

	return (ret == 0) ? 0 : -ENOMEM;
}

static const char *motor_commission_expected_mode_to_string(uint8_t expected_mode)
{
	switch (expected_mode) {
	case MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED:
		return "ONLINE_VELOCITY_CLOSED";
	case MOTOR_COMMISSION_EXPECT_TORQUE:
		return "ONLINE_TORQUE";
	case MOTOR_COMMISSION_EXPECT_ANY:
	default:
		return "ANY";
	}
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
	shell_print(sh, "  Active params:  psi_f=%.8f Wb J=%.8f kgm2 B=%.8f Nm/(rad/s) Tc=%.8f Nm",
		    (double)g_motor_params->flux_linkage_wb_active,
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

	motor_commission_reset(g_motor_params);
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

	motor_commission_abort(g_motor_params, "aborted by user");
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

	int ret = motor_commission_apply_results(g_motor_params);
	if (ret == -ENOENT) {
		shell_error(sh, "No valid commissioning estimates to apply yet");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "Failed to apply commissioning results (err %d)", ret);
		return ret;
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

	int ret = motor_commission_start_flux(g_motor_params, &cfg);
	if (ret < 0) {
		shell_error(sh, "Failed to start flux commissioning (err %d)", ret);
		return ret;
	}

	/* Prepare expected control mode and limits; capture gating handles transitions. */
	(void)motor_api_set_param("velocity_cl_iq_limit_A", cfg.iq_limit_a);
	(void)motor_api_request_online();
	(void)motor_post_mode_change(MOTOR_STATE_ONLINE_VELOCITY_CLOSED);

	shell_print(sh,
		    "Flux commissioning started: %.3f..%.3f Hz, steps=%u, settle=%u ms, sample=%u ms, iq_limit=%.3f A",
		    (double)cfg.min_speed_hz, (double)cfg.max_speed_hz, cfg.steps, cfg.settle_ms,
		    cfg.sample_ms, (double)cfg.iq_limit_a);
	shell_print(sh,
		    "Ensure control is armed and velocity commands are applied; capture expects ONLINE_VELOCITY_CLOSED.");
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

	int ret = motor_commission_start_mech(g_motor_params, &cfg);
	if (ret < 0) {
		shell_error(sh, "Failed to start mechanical commissioning (err %d)", ret);
		return ret;
	}

	(void)motor_api_request_online();
	(void)motor_post_mode_change(MOTOR_STATE_ONLINE_TORQUE);

	shell_print(sh,
		    "Mechanical commissioning started: coast=%.3f Hz, prbs_amp=%.3f A, prbs_period=%u ms, duration=%u ms",
		    (double)cfg.coast_speed_hz, (double)cfg.prbs_amp_a, cfg.prbs_period_ms,
		    cfg.duration_ms);
	shell_print(sh,
		    "Ensure control is armed and torque excitation is applied; capture expects ONLINE_TORQUE.");
	return 0;
}

