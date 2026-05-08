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
#include "shell_parse.h"
#include "motor_encoder_acquisition.h"
#include "motor/math/math_constants.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/motion_planner.h"
#include "motor/motion/motion_profile.h"
#include "motor/motion/traj.h"
#include "motor_torque.h"

static const float32_t motor_commission_validate_step_scale[] = {
	0.2f, 0.6f, 1.0f, -0.2f, -0.6f, -1.0f, 0.0f,
};

static int motor_commission_prepare_pi_encoder_validation(const struct shell *sh)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Current offsets are not complete; run 'motor commission run confirm apply' first");
		return -EACCES;
	}
	if (!g_motor_params->calibration.encoder_mapping_complete) {
		shell_error(sh, "Encoder mapping is not applied; run 'motor commission run confirm apply' first");
		return -EACCES;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	(void)motor_api_set_param("outer_loop_mode", (float32_t)MOTOR_OUTER_LOOP_MODE_PI);
	(void)motor_api_set_param("velocity_dob_enable", 0.0f);
	(void)cmd_motor_commission_detent_clear(sh, 0, NULL);
	motor_encoder_acquisition_reset_stats();
	motor_commission_motion_stop_current();
	motor_commission_set_velocity_target_hz(0.0f);
	motor_command_feed_watchdog(g_motor_params);
	return 0;
}

static int motor_commission_enter_mode_armed(const struct shell *sh,
					     enum motor_state mode,
					     uint32_t timeout_ms)
{
	int ret = motor_commission_request_idle_disarmed();
	if (ret != 0) {
		return ret;
	}

	uint32_t start_ms = k_uptime_get_32();
	while ((motor_encoder_acquisition_is_enabled() ||
		motor_encoder_acquisition_is_busy()) &&
	       (k_uptime_get_32() - start_ms) < MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS) {
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_MOTION_SAMPLE_MS);
	}
	if (motor_encoder_acquisition_is_enabled() ||
	    motor_encoder_acquisition_is_busy()) {
		return -EBUSY;
	}

	ret = motor_commission_request_online_mode(mode);
	if (ret != 0) {
		return ret;
	}
	ret = motor_commission_wait_for_mode(mode, timeout_ms);
	if (ret != 0) {
		return ret;
	}

	return cmd_motor_arm(sh, 0, NULL);
}

static bool motor_commission_current_model_cap(float32_t ramp_s,
					       float32_t max_velocity_rad_s,
					       float32_t *iq_cap_out)
{
	if (g_motor_params == NULL || iq_cap_out == NULL) {
		return false;
	}

	const struct motor_commission_results *res = &g_motor_params->commission.results;
	bool have_model = (res->mech_valid &&
			   res->mech_confidence >= COMMISSION_AUTO_MECH_MIN_CONFIDENCE &&
			   res->psi_f_valid) ||
			  (g_motor_params->commission.auto_tune_applied &&
			   g_motor_params->commission.auto_tune_staged.accepted);
	if (!have_model) {
		return false;
	}

	float32_t j = g_motor_params->inertia_kgm2_active;
	float32_t b = g_motor_params->viscous_friction_nm_per_rad_s_active;
	float32_t tc = g_motor_params->coulomb_friction_nm_active;
	float32_t kt = motor_torque_gain_resolve_active(g_motor_params);
	if (!isfinite(j) || j <= 0.0f || !isfinite(b) || b < 0.0f ||
	    !isfinite(tc) || tc < 0.0f || !isfinite(kt) || kt <= 0.0f ||
	    !isfinite(ramp_s) || ramp_s <= 0.0f ||
	    !isfinite(max_velocity_rad_s) || max_velocity_rad_s <= 0.0f) {
		return false;
	}

	float32_t alpha_limit_rad_s2 = max_velocity_rad_s / ramp_s;
	float32_t torque_nm = (j * alpha_limit_rad_s2) +
			      (b * max_velocity_rad_s) +
			      tc +
			      fmaxf(0.25f * tc, 1.0e-4f);
	float32_t iq_cap = torque_nm / kt;
	if (!isfinite(iq_cap) || iq_cap < 0.01f) {
		return false;
	}

	*iq_cap_out = iq_cap;
	return true;
}

int cmd_motor_commission_validate_current(const struct shell *sh, size_t argc, char **argv)
{
	float32_t iq_a = MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_IQ_A;
	uint32_t hold_ms = MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_HOLD_MS;
	uint32_t ramp_ms = MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_RAMP_MS;
	float32_t stop_motion_deg = MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_STOP_DEG;

	if (argc > 5) {
		shell_error(sh,
			    "Usage: motor commission validate current [iq_a] [hold_ms] [ramp_ms] [stop_deg]");
		return -EINVAL;
	}
	if (argc >= 2 && !shell_parse_finite_float(argv[1], &iq_a)) {
		shell_error(sh, "iq_a must be finite");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &hold_ms)) {
		shell_error(sh, "hold_ms must be an integer");
		return -EINVAL;
	}
	if (argc >= 4 && !shell_parse_u32(argv[3], &ramp_ms)) {
		shell_error(sh, "ramp_ms must be an integer");
		return -EINVAL;
	}
	if (argc >= 5 && !shell_parse_finite_float(argv[4], &stop_motion_deg)) {
		shell_error(sh, "stop_deg must be finite");
		return -EINVAL;
	}
	iq_a = clampf(fabsf(iq_a), 0.01f, MOTOR_COMMISSION_VALIDATE_CURRENT_MAX_IQ_A);
	hold_ms = CLAMP(hold_ms, MOTOR_COMMISSION_MOTION_SAMPLE_MS,
			MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS);
	ramp_ms = CLAMP(ramp_ms, MOTOR_COMMISSION_MOTION_SAMPLE_MS,
			MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS);
	stop_motion_deg = clampf(fabsf(stop_motion_deg), 0.1f, 45.0f);
	float32_t max_velocity_rad_s =
		MOTOR_COMMISSION_VALIDATE_CURRENT_MAX_SPEED_HZ * 2.0f * PI_F32;
	float32_t model_iq_cap = 0.0f;
	bool model_cap_valid =
		motor_commission_current_model_cap((float32_t)ramp_ms / 1000.0f,
						   max_velocity_rad_s,
						   &model_iq_cap);
	if (model_cap_valid) {
		iq_a = fminf(iq_a, fminf(model_iq_cap, MOTOR_MAX_CURRENT_A));
	}

	shell_print(sh,
		    "Validate current_encoder: requires standard commissioning; does not tune gains.");
	int ret = motor_commission_prepare_pi_encoder_validation(sh);
	if (ret != 0) {
		return ret;
	}
	ret = motor_commission_enter_mode_armed(sh, MOTOR_STATE_ONLINE_CURRENT_ENCODER,
					       MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to enter armed current_encoder mode (err %d)", ret);
		return ret;
	}

	struct motor_commission_encoder_trace_guard trace_guard;
	motor_commission_encoder_trace_force_on_decimated(&trace_guard, 4U);
	struct motor_commission_motion_measurement pos = {0};
	struct motor_commission_motion_measurement neg = {0};
	float32_t stop_motion_rad = stop_motion_deg * (PI_F32 / 180.0f);
	float32_t min_motion_rad = 0.5f * (PI_F32 / 180.0f);
	ret = motor_commission_motion_measure_current_bounded(iq_a, hold_ms, ramp_ms,
							     min_motion_rad,
							     stop_motion_rad,
							     max_velocity_rad_s,
							     &pos);
	motor_commission_motion_stop_current();
	k_msleep(MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS);
	if (ret == 0) {
		ret = motor_commission_motion_measure_current_bounded(-iq_a, hold_ms, ramp_ms,
								     min_motion_rad,
								     stop_motion_rad,
								     max_velocity_rad_s,
								     &neg);
	}
	motor_commission_motion_stop_current();
	motor_commission_encoder_trace_restore(&trace_guard);
	int idle_ret = motor_commission_request_idle_disarmed();

	shell_print(sh,
		    "Current encoder validation: Iq=+/-%.3f A hold=%u ms ramp=%u ms stop=%.2f deg speed_limit=%.2f Hz",
		    (double)iq_a, hold_ms, ramp_ms, (double)stop_motion_deg,
		    (double)MOTOR_COMMISSION_VALIDATE_CURRENT_MAX_SPEED_HZ);
	shell_print(sh, "  Current cap: %s limit=%.3f A",
		    model_cap_valid ? "model" : "configured",
		    (double)(model_cap_valid ? model_iq_cap :
					      MOTOR_COMMISSION_VALIDATE_CURRENT_MAX_IQ_A));
	shell_print(sh,
		    "  +Iq: net=%.3f deg abs=%.3f deg max_vel=%.2f Hz samples=%u warn=%u err=%u stop(motion=%s velocity=%s)",
		    (double)(pos.net_motion_rad * 180.0f / PI_F32),
		    (double)(pos.abs_motion_rad * 180.0f / PI_F32),
		    (double)(pos.max_abs_velocity_rad_s / (2.0f * PI_F32)),
		    pos.sample_count, pos.warning_count, pos.error_count,
		    pos.stopped_on_motion ? "YES" : "NO",
		    pos.stopped_on_velocity ? "YES" : "NO");
	if (pos.sample_count > 0U) {
		shell_print(sh, "       angle %.3f -> %.3f deg",
			    (double)(pos.start_angle_rad * 180.0f / PI_F32),
			    (double)(pos.end_angle_rad * 180.0f / PI_F32));
	}
	shell_print(sh,
		    "  -Iq: net=%.3f deg abs=%.3f deg max_vel=%.2f Hz samples=%u warn=%u err=%u stop(motion=%s velocity=%s)",
		    (double)(neg.net_motion_rad * 180.0f / PI_F32),
		    (double)(neg.abs_motion_rad * 180.0f / PI_F32),
		    (double)(neg.max_abs_velocity_rad_s / (2.0f * PI_F32)),
		    neg.sample_count, neg.warning_count, neg.error_count,
		    neg.stopped_on_motion ? "YES" : "NO",
		    neg.stopped_on_velocity ? "YES" : "NO");
	if (neg.sample_count > 0U) {
		shell_print(sh, "       angle %.3f -> %.3f deg",
			    (double)(neg.start_angle_rad * 180.0f / PI_F32),
			    (double)(neg.end_angle_rad * 180.0f / PI_F32));
	}
	if (ret != 0) {
		shell_error(sh, "Current encoder validation stopped by fault/error (err %d)", ret);
		return ret;
	}
	if (idle_ret != 0) {
		shell_error(sh, "Current encoder validation failed to return to IDLE (err %d)",
			    idle_ret);
		return idle_ret;
	}
	if (pos.sample_count < MOTOR_COMMISSION_MOTION_MIN_SAMPLES ||
	    neg.sample_count < MOTOR_COMMISSION_MOTION_MIN_SAMPLES ||
	    pos.error_count > MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES ||
	    neg.error_count > MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES) {
		shell_error(sh, "Current encoder validation had insufficient clean samples");
		return -ENODATA;
	}
	if (pos.stopped_on_velocity || neg.stopped_on_velocity) {
		shell_error(sh, "Current encoder validation exceeded velocity limit");
		return -ERANGE;
	}
	if (pos.net_motion_rad <= 0.0f || neg.net_motion_rad >= 0.0f) {
		shell_error(sh, "Current encoder validation direction check failed");
		return -ERANGE;
	}

	shell_print(sh, "Current encoder validation complete");
	shell_print(sh, "Next: 'motor commission validate velocity 0.50 1000'.");
	return 0;
}

int cmd_motor_commission_validate_velocity(const struct shell *sh, size_t argc, char **argv)
{
	float32_t max_hz = 0.10f;
	uint32_t hold_ms = 1000U;
	bool preserve_active_features = false;

	if (argc > 4) {
		shell_error(sh, "Usage: motor commission validate velocity [max_hz] [hold_ms] [active]");
		return -EINVAL;
	}
	if (argc >= 2 && !shell_parse_finite_float(argv[1], &max_hz)) {
		shell_error(sh, "max_hz must be finite");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &hold_ms)) {
		shell_error(sh, "hold_ms must be an integer");
		return -EINVAL;
	}
	if (argc >= 4) {
		if (strcmp(argv[3], "active") != 0) {
			shell_error(sh, "third optional argument must be 'active'");
			return -EINVAL;
		}
		preserve_active_features = true;
	}
	if (!isfinite(max_hz) || max_hz <= 0.0f) {
		shell_error(sh, "max_hz must be positive");
		return -EINVAL;
	}
	hold_ms = CLAMP(hold_ms, MOTOR_COMMISSION_AUTO_VALIDATE_MIN_HOLD_MS,
			MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS);

	int ret;
	if (preserve_active_features) {
		if (!g_motor_params) {
			shell_error(sh, "Motor not initialized");
			return -ENODEV;
		}
		if (!g_motor_params->calibration.complete ||
		    !g_motor_params->calibration.encoder_mapping_complete) {
			shell_error(sh, "Calibration/mapping incomplete; run commissioning first");
			return -EACCES;
		}
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			shell_error(sh, "Motor is in ERROR state; clear error first");
			return -EFAULT;
		}
		motor_encoder_acquisition_reset_stats();
		motor_commission_motion_stop_current();
		motor_commission_set_velocity_target_hz(0.0f);
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Validate velocity_encoder active features: outer=%s DOB=%s detent=%s.",
			    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ?
				    "MPR" : "PI",
			    g_motor_params->velocity_dob_cfg.enabled ? "on" : "off",
			    g_motor_params->detent_map_cfg.enabled ? "on" : "off");
	} else {
		shell_print(sh,
			    "Validate velocity_encoder PI: requires boot mapping; uses active velocity PI gains.");
		ret = motor_commission_prepare_pi_encoder_validation(sh);
		if (ret != 0) {
			return ret;
		}
	}
	ret = motor_commission_enter_mode_armed(sh, MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					       MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to enter armed velocity_encoder mode (err %d)", ret);
		return ret;
	}

	float32_t profile_max_hz =
		g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	float32_t limited_max_hz = clampf(max_hz, 0.01f, profile_max_hz);
	shell_print(sh,
		    "Velocity encoder validation: max=%.3f Hz hold=%u ms outer=%s DOB=%s detent=%s",
		    (double)limited_max_hz, hold_ms,
		    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ?
			    "MPR" : "PI",
		    g_motor_params->velocity_dob_cfg.enabled ? "on" : "off",
		    g_motor_params->detent_map_cfg.enabled ? "on" : "off");

	for (uint32_t i = 0U; i < ARRAY_SIZE(motor_commission_validate_step_scale); i++) {
		float32_t target_hz = limited_max_hz * motor_commission_validate_step_scale[i];
		motor_commission_set_velocity_target_hz(target_hz);
		motor_command_feed_watchdog(g_motor_params);
		ret = motor_commission_wait_ms_or_fault(hold_ms);
		motor_commission_print_velocity_validation_sample(sh, target_hz);
		if (ret != 0) {
			shell_error(sh, "Velocity validation stopped by motor fault (err %d)", ret);
			break;
		}
	}

	motor_commission_set_velocity_target_hz(0.0f);
	motor_command_feed_watchdog(g_motor_params);
	int idle_ret = motor_commission_request_idle_disarmed();
	if (ret == 0 && idle_ret != 0) {
		shell_error(sh, "Velocity validation failed to return to IDLE (err %d)",
			    idle_ret);
		ret = idle_ret;
	}
	if (ret == 0) {
		shell_print(sh, "Velocity encoder validation complete");
		shell_print(sh, "Next: 'motor commission validate position 5 2000'.");
	}
	return ret;
}

static float32_t motor_commission_position_duration_s(float32_t distance_rad)
{
	float32_t dist = fabsf(distance_rad);
	if (dist <= 1.0e-6f) {
		return MOTOR_COMMISSION_VALIDATE_POSITION_MIN_DURATION_S;
	}

	float32_t max_vel = fmaxf(g_motor_params->profile_max_velocity_rad_s, 0.1f);
	float32_t max_accel = fmaxf(g_motor_params->profile_max_accel_rad_s2, 0.1f);
	float32_t t_vel = 2.0f * dist / max_vel;
	float32_t t_accel = sqrtf(8.0f * dist / max_accel);

	return fmaxf(MOTOR_COMMISSION_VALIDATE_POSITION_MIN_DURATION_S, fmaxf(t_vel, t_accel));
}

static int motor_commission_plan_position_target(float32_t target_rad,
						 float32_t *duration_s_out)
{
	if (duration_s_out == NULL) {
		return -EINVAL;
	}

	float32_t start_pos_rad = g_motor_params->live.position_rad;
	float32_t start_vel_rad_s = g_motor_params->live.velocity_filtered_rad_s;
	if (!isfinite(start_pos_rad)) {
		start_pos_rad = g_motor_params->position_target_rad;
	}
	if (!isfinite(start_vel_rad_s)) {
		start_vel_rad_s = 0.0f;
	}

	float32_t delta_rad = wrap_rad_pi(target_rad - start_pos_rad);
	float32_t duration_s = motor_commission_position_duration_s(delta_rad);
	int ret = -ERANGE;
	for (uint8_t attempt = 0U; attempt < 5U; attempt++) {
		ret = motor_position_move_plan_sequence_segment(&g_motor_params->position_profile,
								start_pos_rad,
								start_vel_rad_s,
								wrap_rad_2pi(target_rad),
								0.0f,
								duration_s,
								g_motor_params->profile_max_velocity_rad_s,
								g_motor_params->profile_max_accel_rad_s2);
		if (ret == 0) {
			g_motor_params->position_target_rad = wrap_rad_2pi(start_pos_rad);
			*duration_s_out = duration_s;
			return 0;
		}
		duration_s *= 1.5f;
	}

	return ret;
}

int cmd_motor_commission_validate_position(const struct shell *sh, size_t argc, char **argv)
{
	float32_t delta_deg = MOTOR_COMMISSION_VALIDATE_POSITION_DEFAULT_DELTA_DEG;
	uint32_t hold_ms = MOTOR_COMMISSION_VALIDATE_POSITION_DEFAULT_HOLD_MS;

	if (argc > 3) {
		shell_error(sh, "Usage: motor commission validate position [delta_deg] [hold_ms]");
		return -EINVAL;
	}
	if (argc >= 2 && !shell_parse_finite_float(argv[1], &delta_deg)) {
		shell_error(sh, "delta_deg must be finite");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &hold_ms)) {
		shell_error(sh, "hold_ms must be an integer");
		return -EINVAL;
	}
	if (!isfinite(delta_deg) || fabsf(delta_deg) <= 0.0f || fabsf(delta_deg) > 45.0f) {
		shell_error(sh, "delta_deg must be in [-45,45] excluding 0");
		return -EINVAL;
	}
	hold_ms = CLAMP(hold_ms, MOTOR_COMMISSION_AUTO_VALIDATE_MIN_HOLD_MS,
			MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS);

	shell_print(sh,
		    "Validate position_encoder: requires boot mapping and stable velocity_encoder behavior.");
	int ret = motor_commission_prepare_pi_encoder_validation(sh);
	if (ret != 0) {
		return ret;
	}
	ret = motor_commission_enter_mode_armed(sh, MOTOR_STATE_ONLINE_POSITION_ENCODER,
					       MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to enter armed position_encoder mode (err %d)", ret);
		return ret;
	}

	float32_t start_rad = wrap_rad_2pi(g_motor_params->live.position_rad);
	float32_t delta_rad = delta_deg * (PI_F32 / 180.0f);
	float32_t duration_s = 0.0f;
	float32_t target_rad = wrap_rad_2pi(start_rad + delta_rad);

	ret = motor_commission_plan_position_target(target_rad, &duration_s);
	if (ret != 0) {
		shell_error(sh, "Failed to plan outbound position move (err %d)", ret);
		(void)motor_commission_request_idle_disarmed();
		return ret;
	}
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh,
		    "Position encoder validation: move %.3f deg over %.3f s, hold=%u ms",
		    (double)delta_deg, (double)duration_s, hold_ms);
	ret = motor_commission_wait_ms_or_fault((uint32_t)(duration_s * 1000.0f) + hold_ms);
	if (ret != 0) {
		shell_error(sh, "Outbound position validation stopped by fault (err %d)", ret);
		(void)motor_commission_request_idle_disarmed();
		return ret;
	}

	ret = motor_commission_plan_position_target(start_rad, &duration_s);
	if (ret != 0) {
		shell_error(sh, "Failed to plan return position move (err %d)", ret);
		(void)motor_commission_request_idle_disarmed();
		return ret;
	}
	motor_command_feed_watchdog(g_motor_params);
	ret = motor_commission_wait_ms_or_fault((uint32_t)(duration_s * 1000.0f) + hold_ms);
	if (ret != 0) {
		shell_error(sh, "Return position validation stopped by fault (err %d)", ret);
		(void)motor_commission_request_idle_disarmed();
		return ret;
	}

	ret = motor_commission_request_idle_disarmed();
	if (ret != 0) {
		shell_error(sh, "Position validation failed to return to IDLE (err %d)", ret);
		return ret;
	}

	shell_print(sh,
		    "Position encoder validation complete: pos=%.3f deg vel=%.3f Hz Iq=%.4f A",
		    (double)(g_motor_params->live.position_rad * 180.0f / PI_F32),
		    (double)(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32)),
		    (double)g_motor_params->live.Iq_ref_A);
	shell_print(sh, "Next: repeat validation or continue to full commissioning/tuning.");
	return 0;
}
