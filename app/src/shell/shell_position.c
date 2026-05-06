#include "shell_control_common.h"

/* Domain implementation split from shell_control.c. */

int cmd_motor_position_target(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor position target <deg>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		shell_error(sh,
			    "Not in position_encoder mode. Use 'motor state mode position_encoder' first.");
		return -EACCES;
	}

	float target_deg = 0.0f;
	if (!shell_parse_finite_float(argv[1], &target_deg)) {
		shell_error(sh, "target must be a finite number");
		return -EINVAL;
	}
	float target_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	g_motor_params->profile_seq.running = false;
	g_motor_params->profile_seq.tick_counter = 0U;

	float32_t duration_s = 0.0f;
	int ret = motor_position_target_plan_bounded(target_rad, &duration_s);
	if (ret != 0) {
		shell_error(sh, "Failed to plan bounded position target (err %d)", ret);
		return ret;
	}

	g_motor_params->position_target_rad =
		wrap_rad_2pi(g_motor_params->position_profile.start_position_rad);
	g_motor_params->position_cl_i_term_rad_s = 0.0f;
	motor_position_regulator_reset(&g_motor_params->position_reg_state, 0.0f);
	g_motor_params->velocity_cl_i_term_A = 0.0f;
	motor_velocity_regulator_reset(&g_motor_params->velocity_reg_state, 0.0f);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh,
		    "Position target planned to %.2f deg over %.3f s (bounded by profile limits)",
		    (double)(target_rad * 180.0f / PI_F32),
		    (double)duration_s);
	return 0;
}

/* motor position decimation <ticks> */
int cmd_motor_position_decimation(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor position decimation <ticks>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing position decimation.");
		return -EACCES;
	}

	uint32_t decimation = 0U;
	if (!shell_parse_u32(argv[1], &decimation)) {
		shell_error(sh, "ticks must be an integer");
		return -EINVAL;
	}
	if (decimation < OUTER_LOOP_DECIMATION_MIN || decimation > OUTER_LOOP_DECIMATION_MAX) {
		shell_error(sh, "ticks must be in [%u, %u]",
			    OUTER_LOOP_DECIMATION_MIN, OUTER_LOOP_DECIMATION_MAX);
		return -EINVAL;
	}

	int ret = motor_api_set_param("position_loop_decimation", (float)decimation);
	if (ret != 0) {
		shell_error(sh, "Failed to set position decimation (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Position decimation set to %u tick(s): %.3f ms, %.1f Hz update",
		    decimation,
		    (double)(1000.0f * (float32_t)decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)decimation));
	return 0;
}

/* motor position status */
int cmd_motor_position_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		shell_print(sh, "Position controller: INACTIVE");
		return 0;
	}

	float target_rad = g_motor_params->position_target_rad;
	float meas_rad = g_motor_params->live.position_rad;
	float err_rad = wrap_rad_pi(target_rad - meas_rad);
	uint32_t position_decimation =
		MAX(OUTER_LOOP_DECIMATION_MIN, g_motor_params->position_loop_decimation);

	shell_print(sh, "Position Controller Status:");
	shell_print(sh, "  Target:     %.2f deg", (double)(target_rad * 180.0f / PI_F32));
	shell_print(sh, "  Measured:   %.2f deg", (double)(meas_rad * 180.0f / PI_F32));
	shell_print(sh, "  Error:      %.2f deg", (double)(err_rad * 180.0f / PI_F32));
	shell_print(sh, "  Loop dt:    %.3f ms (%u tick, %.1f Hz)",
		    (double)(1000.0f * (float32_t)position_decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    position_decimation,
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)position_decimation));
	shell_print(sh, "  Profile:    %s",
		    motion_profile_quintic_is_active(&g_motor_params->position_profile) ?
			    "ACTIVE" :
			    (g_motor_params->position_profile.valid ? "COMPLETE" : "OFF"));
	shell_print(sh, "  Outer loop: %s",
		    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ?
			    "MPR" :
			    "PI");
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_print(sh, "  MPR q_pos:  %.4f", (double)g_motor_params->position_mpr_cfg.q_position);
		shell_print(sh, "  MPR q_vel:  %.4f", (double)g_motor_params->position_mpr_cfg.q_velocity_ff);
		shell_print(sh, "  MPR r:      %.4f",
			    (double)g_motor_params->position_mpr_cfg.r_delta_velocity);
		shell_print(sh, "  Horizon:    %u", g_motor_params->position_mpr_cfg.horizon);
		shell_print(sh, "  dVel max:   %.4f rad/s/sample",
			    (double)g_motor_params->position_mpr_cfg.max_delta_velocity_rad_s);
		shell_print(sh, "  Vel cmd:    %.4f rad/s",
			    (double)g_motor_params->position_mpr_state.velocity_cmd_rad_s);
	} else {
		shell_print(sh, "  Kp:         %.5f (rad/s)/rad",
			    (double)g_motor_params->position_cl_kp_rad_s_per_rad);
		shell_print(sh, "  Ki:         %.5f (rad/s^2)/rad",
			    (double)g_motor_params->position_cl_ki_rad_s2_per_rad);
		shell_print(sh, "  I term:     %.5f rad/s",
			    (double)g_motor_params->position_cl_i_term_rad_s);
	}
	return 0;
}

/* motor position pi status
 * motor position pi set <kp_rad_s_per_rad> <ki_rad_s2_per_rad>
 * motor position pi defaults <safe|nominal>
 * motor position pi bandwidth <hz> [zeta]
 */
int cmd_motor_position_pi(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || argc > 4) {
		shell_error(sh, "Usage: motor position pi status | "
			    "motor position pi set <kp> <ki> | "
			    "motor position pi defaults <safe|nominal> | "
			    "motor position pi bandwidth <hz> [zeta]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_warn(sh,
			   "outer mode is MPR; position PI settings are inactive until 'motor outer mode pi'");
	}

	if (strcmp(argv[1], "status") == 0) {
		if (argc != 2) {
			shell_error(sh, "Usage: motor position pi status");
			return -EINVAL;
		}
		shell_print(sh, "Position PI:");
		shell_print(sh, "  Active: %s",
			    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_PI ? "YES" : "NO");
		shell_print(sh, "  Kp:     %.6f (rad/s)/rad",
			    (double)g_motor_params->position_cl_kp_rad_s_per_rad);
		shell_print(sh, "  Ki:     %.6f (rad/s^2)/rad",
			    (double)g_motor_params->position_cl_ki_rad_s2_per_rad);
		shell_print(sh, "  I term: %.6f rad/s",
			    (double)g_motor_params->position_cl_i_term_rad_s);
		return 0;
	}

	if (strcmp(argv[1], "defaults") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor position pi defaults <safe|nominal>");
			return -EINVAL;
		}

		enum motor_gains_profile profile;
		if (motor_parse_gains_profile(argv[2], &profile) != 0) {
			shell_error(sh, "Profile must be 'safe' or 'nominal'");
			return -EINVAL;
		}

		float vel_kp_dummy = 0.0f;
		float vel_ki_dummy = 0.0f;
		float vel_iq_dummy = 0.0f;
		float kp = 0.0f;
		float ki = 0.0f;
		const char *source = "model";
		int ret = motor_compute_model_outer_gains(g_motor_params, profile,
							  &vel_kp_dummy, &vel_ki_dummy,
							  &vel_iq_dummy, &kp, &ki);
		if (ret != 0 && profile == MOTOR_GAINS_PROFILE_SAFE) {
			motor_compute_safe_outer_gains(g_motor_params, &vel_kp_dummy, &vel_ki_dummy,
						      &vel_iq_dummy, &kp, &ki);
			source = "empirical";
		} else if (ret != 0) {
			motor_compute_nominal_outer_gains(g_motor_params, &vel_kp_dummy, &vel_ki_dummy,
							 &vel_iq_dummy, &kp, &ki);
			source = "empirical";
		}

		ret = motor_apply_position_gains(kp, ki);
		if (ret != 0) {
			shell_error(sh, "Failed to apply position defaults (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Position %s defaults applied (source=%s): Kp=%.5f (rad/s)/rad, Ki=%.5f (rad/s^2)/rad",
			    (profile == MOTOR_GAINS_PROFILE_SAFE) ? "safe" : "nominal",
			    source,
			    (double)kp, (double)ki);
		return 0;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 4) {
			shell_error(sh, "Usage: motor position pi set <kp> <ki>");
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		if (!shell_parse_finite_float(argv[2], &kp) ||
		    !shell_parse_finite_float(argv[3], &ki)) {
			shell_error(sh, "kp/ki must be finite numbers");
			return -EINVAL;
		}

		int ret = motor_apply_position_gains(kp, ki);
		if (ret != 0) {
			shell_error(sh, "Failed to update position PI settings (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Position PI set: Kp=%.5f (rad/s)/rad, Ki=%.5f (rad/s^2)/rad",
			    (double)kp, (double)ki);
		return 0;
	}

	if (strcmp(argv[1], "bandwidth") == 0) {
		if (argc != 3 && argc != 4) {
			shell_error(sh, "Usage: motor position pi bandwidth <hz> [zeta]");
			return -EINVAL;
		}

		float bw_hz = 0.0f;
		float zeta = OUTER_LOOP_ZETA_DEFAULT;
			if (!shell_parse_finite_float(argv[2], &bw_hz) ||
			    (argc == 4 && !shell_parse_finite_float(argv[3], &zeta))) {
				shell_error(sh, "Bandwidth/zeta must be finite numbers");
				return -EINVAL;
			}
			if (bw_hz <= 0.0f || bw_hz > (CONTROL_LOOP_FREQUENCY_HZ * 0.25f) ||
			    zeta < OUTER_LOOP_ZETA_MIN || zeta > OUTER_LOOP_ZETA_MAX) {
				shell_error(sh, "Invalid bandwidth/zeta; bw in (0, %.1f] Hz, zeta in %.1f..%.1f",
					    (double)(CONTROL_LOOP_FREQUENCY_HZ * 0.25f),
					    (double)OUTER_LOOP_ZETA_MIN, (double)OUTER_LOOP_ZETA_MAX);
				return -EINVAL;
			}

		float vel_bw_hz = 0.0f;
		int ret_bw = motor_estimate_velocity_bandwidth_hz(g_motor_params, &vel_bw_hz);
		if (ret_bw == 0) {
			float max_pos_bw_hz = vel_bw_hz * POSITION_TO_VELOCITY_BW_RATIO_MAX;
			if (bw_hz > max_pos_bw_hz) {
				shell_warn(sh,
					   "Requested position BW %.2f Hz exceeds %.2f Hz (velocity BW/5); clamping",
					   (double)bw_hz, (double)max_pos_bw_hz);
				bw_hz = max_pos_bw_hz;
			}
		} else {
			shell_warn(sh, "Velocity BW estimate unavailable; skipping BW/5 cascade guard");
		}

		float omega = 2.0f * PI_F32 * bw_hz;
		float kp = 2.0f * zeta * omega;
		float ki = omega * omega;
		int ret = motor_apply_position_gains(kp, ki);
		if (ret != 0) {
			shell_error(sh, "Failed to apply position PI bandwidth settings (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Position bandwidth tuned: bw=%.2f Hz zeta=%.2f -> Kp=%.5f (rad/s)/rad, Ki=%.5f (rad/s^2)/rad",
			    (double)bw_hz, (double)zeta, (double)kp, (double)ki);
		return 0;
	}

	shell_error(sh, "Usage: motor position pi status | "
		    "motor position pi set <kp> <ki> | "
		    "motor position pi defaults <safe|nominal> | "
		    "motor position pi bandwidth <hz> [zeta]");
	return -EINVAL;
}

/* motor position mpr status
 * motor position mpr set <q_position> <q_velocity_ff> <r_delta_velocity> <horizon> [max_delta_velocity]
 * motor position mpr bandwidth <hz>
 */
int cmd_motor_position_mpr(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || argc > 7) {
		shell_error(sh, "Usage: motor position mpr status | "
			    "motor position mpr set <q_position> <q_velocity_ff> <r_delta_velocity> <horizon> [max_delta_velocity] | "
			    "motor position mpr bandwidth <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (strcmp(argv[1], "status") == 0) {
		if (argc != 2) {
			shell_error(sh, "Usage: motor position mpr status");
			return -EINVAL;
		}
		shell_print(sh, "Position MPR:");
		shell_print(sh, "  Active:           %s",
			    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ? "YES" : "NO");
		shell_print(sh, "  q_position:       %.6f",
			    (double)g_motor_params->position_mpr_cfg.q_position);
		shell_print(sh, "  q_velocity_ff:    %.6f",
			    (double)g_motor_params->position_mpr_cfg.q_velocity_ff);
		shell_print(sh, "  r_delta_velocity: %.6f",
			    (double)g_motor_params->position_mpr_cfg.r_delta_velocity);
		shell_print(sh, "  Horizon:          %u", g_motor_params->position_mpr_cfg.horizon);
		shell_print(sh, "  Velocity limit:   %.6f rad/s",
			    (double)g_motor_params->position_mpr_cfg.velocity_limit_rad_s);
		shell_print(sh, "  dVel max:         %.6f rad/s/sample",
			    (double)g_motor_params->position_mpr_cfg.max_delta_velocity_rad_s);
		shell_print(sh, "  dt:               %.6f s",
			    (double)g_motor_params->position_mpr_cfg.dt_s);
		shell_print(sh, "  Velocity cmd:     %.6f rad/s",
			    (double)g_motor_params->position_mpr_state.velocity_cmd_rad_s);
		if (g_motor_params->position_mpr_cfg.q_position > 0.0f) {
			const float bw_hz =
				g_motor_params->position_mpr_cfg.q_position /
				(4.0f * PI_F32);

			shell_print(sh, "  Bandwidth est:    %.3f Hz", (double)bw_hz);
		} else {
			shell_print(sh, "  Bandwidth est:    unavailable");
		}
		return 0;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 6 && argc != 7) {
			shell_error(sh, "Usage: motor position mpr set <q_position> <q_velocity_ff> <r_delta_velocity> <horizon> [max_delta_velocity]");
			return -EINVAL;
		}

		float q_position = 0.0f;
		float q_velocity_ff = 0.0f;
		float r_delta_velocity = 0.0f;
		uint32_t horizon = 0U;
		float max_delta_velocity =
			g_motor_params->position_mpr_cfg.max_delta_velocity_rad_s;
		if (!shell_parse_finite_float(argv[2], &q_position) ||
		    !shell_parse_finite_float(argv[3], &q_velocity_ff) ||
		    !shell_parse_finite_float(argv[4], &r_delta_velocity) ||
		    !shell_parse_u32(argv[5], &horizon) ||
		    (argc == 7 && !shell_parse_finite_float(argv[6], &max_delta_velocity))) {
			shell_error(sh, "MPR parameters must be finite numbers; horizon must be integer");
			return -EINVAL;
		}
		if (q_position < 0.0f || q_velocity_ff < 0.0f ||
		    (q_position == 0.0f && q_velocity_ff == 0.0f) ||
		    r_delta_velocity <= 0.0f || horizon == 0U ||
		    horizon > MOTOR_MPR_HORIZON_MAX || max_delta_velocity < 0.0f) {
			shell_error(sh, "Invalid position MPR limits; horizon must be 1..%u",
				    MOTOR_MPR_HORIZON_MAX);
			return -EINVAL;
		}

		int ret = motor_set_param_checked("position_mpr_q_position", q_position);
		ret |= motor_set_param_checked("position_mpr_q_velocity_ff", q_velocity_ff);
		ret |= motor_set_param_checked("position_mpr_r_delta_velocity",
					       r_delta_velocity);
		ret |= motor_set_param_checked("position_mpr_horizon", (float)horizon);
		ret |= motor_set_param_checked("position_mpr_max_delta_velocity_rad_s",
					       max_delta_velocity);
		if (ret != 0) {
			shell_error(sh, "Failed to update position MPR params (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Position MPR set: q_pos=%.6f q_vel=%.6f r=%.6f horizon=%u dVel=%.6f rad/s/sample",
			    (double)q_position, (double)q_velocity_ff,
			    (double)r_delta_velocity, horizon,
			    (double)max_delta_velocity);
		return 0;
	}

	if (strcmp(argv[1], "bandwidth") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor position mpr bandwidth <hz>");
			return -EINVAL;
		}

		float bw_hz = 0.0f;
		if (!shell_parse_finite_float(argv[2], &bw_hz) || bw_hz <= 0.0f) {
			shell_error(sh, "Bandwidth must be a positive finite number");
			return -EINVAL;
		}

		struct motor_mpr_position_config cfg = g_motor_params->position_mpr_cfg;
		struct motor_mpr_bandwidth_result result = {0};
		const struct motor_mpr_position_bandwidth_input input = {
			.bandwidth_hz = bw_hz,
			.velocity_limit_rad_s = g_motor_params->profile_max_velocity_rad_s,
			.accel_limit_rad_s2 = g_motor_params->profile_max_accel_rad_s2,
			.dt_s = g_motor_params->position_mpr_cfg.dt_s,
		};

		int ret = motor_mpr_position_config_from_bandwidth(&input, &cfg, &result);
		if (ret != 0) {
			shell_error(sh, "Failed to derive position MPR bandwidth params (err %d)",
				    ret);
			return ret;
		}

		ret = motor_set_param_checked("position_mpr_q_position", cfg.q_position);
		ret |= motor_set_param_checked("position_mpr_q_velocity_ff",
					       cfg.q_velocity_ff);
		ret |= motor_set_param_checked("position_mpr_r_delta_velocity",
					       cfg.r_delta_velocity);
		ret |= motor_set_param_checked("position_mpr_horizon",
					       (float)cfg.horizon);
		ret |= motor_set_param_checked("position_mpr_max_delta_velocity_rad_s",
					       cfg.max_delta_velocity_rad_s);
		if (ret != 0) {
			shell_error(sh, "Failed to apply position MPR bandwidth params (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Position MPR bandwidth tuned: bw=%.2f Hz -> q_pos=%.6f q_vel=%.6f r=%.6f horizon=%u dVel=%.6f rad/s/sample%s",
			    (double)result.applied_bandwidth_hz,
			    (double)cfg.q_position, (double)cfg.q_velocity_ff,
			    (double)cfg.r_delta_velocity, cfg.horizon,
			    (double)cfg.max_delta_velocity_rad_s,
			    result.clamped ? " clamped" : "");
		return 0;
	}

	shell_error(sh, "Usage: motor position mpr status | "
		    "motor position mpr set <q_position> <q_velocity_ff> <r_delta_velocity> <horizon> [max_delta_velocity] | "
		    "motor position mpr bandwidth <hz>");
	return -EINVAL;
}

/* motor control status
 * motor outer status
 * motor outer mode <pi|mpr>
 */

