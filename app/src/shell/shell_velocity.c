#include "shell_control_common.h"

/* Domain implementation split from shell_control.c. */

int cmd_motor_velocity_target(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor velocity target <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct smf_state *mode = g_motor_params->state_for_isr;
	if (!motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_ENCODER)) {
		shell_error(sh,
			    "Velocity target requires velocity_generated or velocity_encoder mode.");
		return -EACCES;
	}

	float target_hz = 0.0f;
	if (!shell_parse_finite_float(argv[1], &target_hz)) {
		shell_error(sh, "target must be a finite number");
		return -EINVAL;
	}
	if (fabsf(target_hz) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero velocity commands.");
		return -EACCES;
	}

	/* Convert Hz to mechanical rad/s for trajectory */
	float target_rad_s = target_hz * 2.0f * PI_F32;

	/* Respect profile and measured voltage-speed limits. */
	float32_t speed_limit_hz = motor_shell_velocity_command_limit_hz(g_motor_params);
	float32_t speed_limit_rad_s = speed_limit_hz * 2.0f * PI_F32;
	float32_t target_clamped = clampf(target_rad_s,
					  -speed_limit_rad_s,
					  speed_limit_rad_s);
	if (fabsf(target_clamped - target_rad_s) > 1.0e-6f) {
		shell_warn(sh,
			   "Velocity target limited to %.2f Hz by profile/voltage-speed limit",
			   (double)(target_clamped / (2.0f * PI_F32)));
	}
	float32_t prev_target = g_motor_params->live.velocity_target_rad_s;
	bool sign_change =
		(prev_target > 1e-6f && target_clamped < -1e-6f) ||
		(prev_target < -1e-6f && target_clamped > 1e-6f);
	bool large_step =
		fabsf(target_clamped - prev_target) > (0.25f * 2.0f * PI_F32);
	if (g_motor_params->velocity_dob_cfg.enabled &&
	    (sign_change || large_step || fabsf(target_clamped) <= 1e-6f)) {
		motor_dob_reset(&g_motor_params->velocity_dob_state,
				g_motor_params->live.velocity_rad_s);
		g_motor_params->live.velocity_dob_iq_ff_a = 0.0f;
		g_motor_params->live.velocity_dob_disturbance_nm = 0.0f;
		g_motor_params->live.velocity_dob_residual_rad_s = 0.0f;
	}

	/* Set trajectory target (thread-safe access) */
	traj_set_target_value(&g_motor_params->traj_velocity, target_clamped);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Velocity target set to %.2f Hz", (double)(target_clamped / (2.0f * PI_F32)));
	return 0;
}

/* motor velocity decimation <ticks> */
int cmd_motor_velocity_decimation(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor velocity decimation <ticks>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing velocity decimation.");
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

	int ret = motor_api_set_param("velocity_loop_decimation", (float)decimation);
	if (ret != 0) {
		shell_error(sh, "Failed to set velocity decimation (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Velocity decimation set to %u tick(s): %.3f ms, %.1f Hz update",
		    decimation,
		    (double)(1000.0f * (float32_t)decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)decimation));
	return 0;
}

/* motor velocity status */
int cmd_motor_velocity_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct smf_state *mode = g_motor_params->state_for_isr;
	if (!motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_POSITION_GENERATED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		shell_print(sh, "Velocity controller: INACTIVE");
		return 0;
	}

	/* Get values from trajectory/controller (in rad/s) and convert to Hz for display */
	float target_rad_s = traj_get_target_value(&g_motor_params->traj_velocity);
	float ref_rad_s = g_motor_params->live.velocity_ref_rad_s;
	float meas_rad_s = g_motor_params->live.velocity_rad_s;
	float target_hz = target_rad_s / (2.0f * PI_F32);
	float ref_hz = ref_rad_s / (2.0f * PI_F32);
	float meas_hz = meas_rad_s / (2.0f * PI_F32);
	float error_hz = ref_hz - meas_hz;
	bool traj_at_target = traj_is_at_target(&g_motor_params->traj_velocity);
	uint8_t quality_flags = g_motor_params->live.position_quality_flags;
	bool feedback_valid = motor_feedback_quality_is_usable(quality_flags);
	bool speed_tracking_ok = fabsf(error_hz) <= VELOCITY_STATUS_TRACK_TOL_HZ;
	bool at_target = traj_at_target && feedback_valid && speed_tracking_ok;
	uint32_t velocity_decimation =
		MAX(OUTER_LOOP_DECIMATION_MIN, g_motor_params->velocity_loop_decimation);

	/* Determine motion state */
	const char *motion_str;
	if (!feedback_valid) {
		motion_str = "HOLD_QUALITY";
	} else if (fabsf(meas_rad_s) < 0.1f && fabsf(ref_rad_s) < 0.1f) {
		motion_str = "STOPPED";
	} else if (at_target) {
		motion_str = "AT_SPEED";
	} else if (fabsf(ref_rad_s) > fabsf(meas_rad_s)) {
		motion_str = "ACCELERATING";
	} else {
		motion_str = "DECELERATING";
	}

	shell_print(sh, "Velocity Controller Status:");
	shell_print(sh, "  Mode:       %s", motor_state_to_string(motor_api_get_state()));
	shell_print(sh, "  Target:     %.2f Hz", (double)target_hz);
	shell_print(sh, "  Ref:        %.2f Hz", (double)ref_hz);
	shell_print(sh, "  Measured:   %.2f Hz", (double)meas_hz);
	shell_print(sh, "  Error:      %.2f Hz", (double)error_hz);
	shell_print(sh, "  Loop dt:    %.3f ms (%u tick, %.1f Hz)",
		    (double)(1000.0f * (float32_t)velocity_decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    velocity_decimation,
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)velocity_decimation));
	shell_print(sh, "  At Target:  %s", at_target ? "YES" : "NO");
	shell_print(sh, "  Feedback:   %s (flags=0x%02X)",
		    feedback_valid ? "VALID" : "DEGRADED",
		    quality_flags);
	shell_print(sh, "  Motion:     %s", motion_str);
	shell_print(sh, "  Outer loop: %s",
		    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ?
			    "MPR" :
			    "PI");
	shell_print(sh, "  Kt active:  %.6f Nm/A",
		    (double)motor_torque_gain_resolve_active(g_motor_params));
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_print(sh, "  MPR q/r:    %.4f / %.4f",
			    (double)g_motor_params->velocity_mpr_cfg.q_speed,
			    (double)g_motor_params->velocity_mpr_cfg.r_delta_iq);
		shell_print(sh, "  Horizon:    %u", g_motor_params->velocity_mpr_cfg.horizon);
		shell_print(sh, "  dIq max:    %.4f A/sample",
			    (double)g_motor_params->velocity_mpr_cfg.max_delta_iq_a);
		shell_print(sh, "  Dist KI:    %.4f Nm/(rad/s)",
			    (double)g_motor_params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s);
		shell_print(sh, "  Iq cmd:     %.5f A",
			    (double)g_motor_params->velocity_mpr_state.iq_cmd_a);
	} else {
		shell_print(sh, "  Kp:         %.5f A/(rad/s)",
			    (double)g_motor_params->velocity_cl_kp_A_per_rad_s);
		shell_print(sh, "  Ki:         %.5f A/rad",
			    (double)g_motor_params->velocity_cl_ki_A_per_rad);
		shell_print(sh, "  I term:     %.5f A",
			    (double)g_motor_params->velocity_cl_i_term_A);
	}
	shell_print(sh, "  DOB:        %s",
		    g_motor_params->velocity_dob_cfg.enabled ? "ENABLED" : "DISABLED");
	if (g_motor_params->velocity_dob_cfg.enabled) {
		shell_print(sh, "  DOB gain:   %.5f Nm/(rad/s)",
			    (double)g_motor_params->velocity_dob_cfg.observer_gain_nm_per_rad_s);
		shell_print(sh, "  DOB lim:    %.5f Nm, %.5f A",
			    (double)g_motor_params->velocity_dob_cfg.torque_limit_nm,
			    (double)g_motor_params->velocity_dob_cfg.iq_ff_limit_a);
		shell_print(sh, "  DOB est:    %.5f Nm, iq_ff=%.5f A, res=%.5f rad/s",
			    (double)g_motor_params->live.velocity_dob_disturbance_nm,
			    (double)g_motor_params->live.velocity_dob_iq_ff_a,
			    (double)g_motor_params->live.velocity_dob_residual_rad_s);
	}
	shell_print(sh, "  Detent FF:  %s gain=%.3f limit=%.4f A live=%.5f A",
		    g_motor_params->detent_map_cfg.enabled ? "ENABLED" : "DISABLED",
		    (double)g_motor_params->detent_map_cfg.gain,
		    (double)g_motor_params->detent_map_cfg.iq_ff_limit_a,
		    (double)g_motor_params->live.detent_iq_ff_a);
	shell_print(sh, "  Iq limit:   %.3f A", (double)g_motor_params->velocity_cl_iq_limit_A);
	struct motor_voltage_speed_limit_result voltage_limit = {0};
	if (motor_shell_voltage_speed_limit(g_motor_params,
					    g_motor_params->velocity_cl_iq_limit_A,
					    &voltage_limit) == 0 && voltage_limit.valid) {
		float command_limit_hz = motor_shell_velocity_command_limit_hz(g_motor_params);
		shell_print(sh, "  Speed limit: %.2f Hz cmd, %.2f Hz voltage (Vlim=%.2f V, bemf=%.2f V)",
			    (double)command_limit_hz,
			    (double)voltage_limit.max_mech_hz,
			    (double)voltage_limit.voltage_limit_v,
			    (double)voltage_limit.bemf_at_limit_v);
	}

	return 0;
}

/* motor velocity pi status
 * motor velocity pi set <kp_a_per_rad_s> <ki_a_per_rad> <iq_limit_a>
 * motor velocity pi defaults <safe|nominal>
 * motor velocity pi bandwidth <hz> [zeta] [iq_limit_a]
 */
int cmd_motor_velocity_pi(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || argc > 5) {
		shell_error(sh, "Usage: motor velocity pi status | "
			    "motor velocity pi set <kp> <ki> <iq_limit> | "
			    "motor velocity pi defaults <safe|nominal> | "
			    "motor velocity pi bandwidth <hz> [zeta] [iq_limit]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_warn(sh,
			   "outer mode is MPR; velocity PI settings are inactive until 'motor outer mode pi'");
	}

	if (strcmp(argv[1], "status") == 0) {
		if (argc != 2) {
			shell_error(sh, "Usage: motor velocity pi status");
			return -EINVAL;
		}
		shell_print(sh, "Velocity PI:");
		shell_print(sh, "  Active:   %s",
			    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_PI ? "YES" : "NO");
		shell_print(sh, "  Kp:       %.6f A/(rad/s)",
			    (double)g_motor_params->velocity_cl_kp_A_per_rad_s);
		shell_print(sh, "  Ki:       %.6f A/rad",
			    (double)g_motor_params->velocity_cl_ki_A_per_rad);
		shell_print(sh, "  I term:   %.6f A",
			    (double)g_motor_params->velocity_cl_i_term_A);
		shell_print(sh, "  Iq limit: %.6f A",
			    (double)g_motor_params->velocity_cl_iq_limit_A);
		return 0;
	}

	if (strcmp(argv[1], "defaults") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor velocity pi defaults <safe|nominal>");
			return -EINVAL;
		}

		enum motor_gains_profile profile;
		if (motor_parse_gains_profile(argv[2], &profile) != 0) {
			shell_error(sh, "Profile must be 'safe' or 'nominal'");
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		float iq_limit = 0.0f;
		float pos_kp_dummy = 0.0f;
		float pos_ki_dummy = 0.0f;
		const char *source = "model";
		int ret = motor_compute_model_outer_gains(g_motor_params, profile, &kp, &ki,
							  &iq_limit, &pos_kp_dummy,
							  &pos_ki_dummy);
		if (ret != 0 && profile == MOTOR_GAINS_PROFILE_SAFE) {
			motor_compute_safe_outer_gains(g_motor_params, &kp, &ki, &iq_limit,
						      &pos_kp_dummy, &pos_ki_dummy);
			source = "empirical";
		} else if (ret != 0) {
			motor_compute_nominal_outer_gains(g_motor_params, &kp, &ki, &iq_limit,
							 &pos_kp_dummy, &pos_ki_dummy);
			source = "empirical";
		}

		ret = motor_apply_velocity_gains(kp, ki, iq_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to apply velocity defaults (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity %s defaults applied (source=%s): Kp=%.5f A/(rad/s), Ki=%.5f A/rad, Iq limit=%.3f A",
			    (profile == MOTOR_GAINS_PROFILE_SAFE) ? "safe" : "nominal",
			    source,
			    (double)kp, (double)ki, (double)iq_limit);
		return 0;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 5) {
			shell_error(sh, "Usage: motor velocity pi set <kp> <ki> <iq_limit>");
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		float iq_limit = 0.0f;
		if (!shell_parse_finite_float(argv[2], &kp) ||
		    !shell_parse_finite_float(argv[3], &ki) ||
		    !shell_parse_finite_float(argv[4], &iq_limit)) {
			shell_error(sh, "kp/ki/iq_limit must be finite numbers");
			return -EINVAL;
		}

		int ret = motor_apply_velocity_gains(kp, ki, iq_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to update velocity PI settings (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Velocity PI set: Kp=%.5f A/(rad/s), Ki=%.5f A/rad, Iq limit=%.3f A",
			    (double)kp, (double)ki, (double)iq_limit);
		return 0;
	}

	if (strcmp(argv[1], "bandwidth") == 0) {
		if (argc < 3 || argc > 5) {
			shell_error(sh, "Usage: motor velocity pi bandwidth <hz> [zeta] [iq_limit]");
			return -EINVAL;
		}

		float bw_hz = 0.0f;
		float zeta = OUTER_LOOP_ZETA_DEFAULT;
		float iq_limit = g_motor_params->velocity_cl_iq_limit_A;
		if (!shell_parse_finite_float(argv[2], &bw_hz) ||
		    (argc >= 4 && !shell_parse_finite_float(argv[3], &zeta)) ||
		    (argc == 5 && !shell_parse_finite_float(argv[4], &iq_limit))) {
			shell_error(sh, "Bandwidth/zeta/iq_limit must be finite numbers");
			return -EINVAL;
		}
		if (bw_hz <= 0.0f || bw_hz > (CONTROL_LOOP_FREQUENCY_HZ * 0.25f)) {
			shell_error(sh, "Bandwidth must be in (0, %.1f] Hz",
				    (double)(CONTROL_LOOP_FREQUENCY_HZ * 0.25f));
			return -EINVAL;
		}
		if (iq_limit <= 0.0f || iq_limit > MOTOR_MAX_CURRENT_A) {
			shell_error(sh, "Iq limit must be in (0, %.3f] A",
				    (double)MOTOR_MAX_CURRENT_A);
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		float kt = 0.0f;
		int ret = motor_compute_velocity_bandwidth_gains(g_motor_params, bw_hz, zeta,
								 iq_limit, &kp, &ki, &kt);
		if (ret != 0) {
			if (ret == -ERANGE) {
				shell_error(sh,
					    "Need valid active commissioning params (J, torque_gain)");
			} else {
				shell_error(sh, "Invalid bandwidth/zeta; zeta range is %.1f..%.1f",
					    (double)OUTER_LOOP_ZETA_MIN, (double)OUTER_LOOP_ZETA_MAX);
			}
			return ret;
		}

		ret = motor_apply_velocity_gains(kp, ki, iq_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to apply velocity PI bandwidth settings (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity bandwidth tuned: bw=%.2f Hz zeta=%.2f iq_limit=%.3f A -> Kp=%.5f A/(rad/s), Ki=%.5f A/rad (Kt=%.6f Nm/A)",
			    (double)bw_hz, (double)zeta, (double)iq_limit,
			    (double)kp, (double)ki, (double)kt);
		return 0;
	}

	shell_error(sh, "Usage: motor velocity pi status | "
		    "motor velocity pi set <kp> <ki> <iq_limit> | "
		    "motor velocity pi defaults <safe|nominal> | "
		    "motor velocity pi bandwidth <hz> [zeta] [iq_limit]");
	return -EINVAL;
}

/* motor velocity mpr status
 * motor velocity mpr set <q_speed> <r_delta_iq> <horizon> <max_delta_iq> [disturbance_ki]
 * motor velocity mpr bandwidth <hz>
 */
int cmd_motor_velocity_mpr(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || argc > 7) {
		shell_error(sh, "Usage: motor velocity mpr status | "
			    "motor velocity mpr set <q_speed> <r_delta_iq> <horizon> <max_delta_iq> [disturbance_ki] | "
			    "motor velocity mpr bandwidth <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (strcmp(argv[1], "status") == 0) {
		if (argc != 2) {
			shell_error(sh, "Usage: motor velocity mpr status");
			return -EINVAL;
		}
		shell_print(sh, "Velocity MPR:");
		shell_print(sh, "  Active:        %s",
			    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ? "YES" : "NO");
		shell_print(sh, "  q_speed:       %.6f",
			    (double)g_motor_params->velocity_mpr_cfg.q_speed);
		shell_print(sh, "  r_delta_iq:    %.6f",
			    (double)g_motor_params->velocity_mpr_cfg.r_delta_iq);
		shell_print(sh, "  Horizon:       %u", g_motor_params->velocity_mpr_cfg.horizon);
		shell_print(sh, "  dIq max:       %.6f A/sample",
			    (double)g_motor_params->velocity_mpr_cfg.max_delta_iq_a);
		shell_print(sh, "  Disturbance KI %.6f Nm/(rad/s)",
			    (double)g_motor_params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s);
		shell_print(sh, "  dt:            %.6f s",
			    (double)g_motor_params->velocity_mpr_cfg.dt_s);
		shell_print(sh, "  Iq cmd:        %.6f A",
			    (double)g_motor_params->velocity_mpr_state.iq_cmd_a);
		shell_print(sh, "  Model omega:   %.6f rad/s",
			    (double)g_motor_params->velocity_mpr_state.omega_model_rad_s);
		shell_print(sh, "  Iq limit:      %.6f A",
			    (double)g_motor_params->velocity_mpr_cfg.iq_limit_a);
		const float j = g_motor_params->inertia_kgm2_active;
		const float kt = motor_torque_gain_resolve_active(g_motor_params);
		if (isfinite(j) && j > 0.0f && isfinite(kt) && kt > 0.0f) {
			const float bw_hz =
				(g_motor_params->velocity_mpr_cfg.q_speed * kt) /
				(2.0f * PI_F32 * j);

			shell_print(sh, "  Bandwidth est: %.3f Hz", (double)bw_hz);
		} else {
			shell_print(sh, "  Bandwidth est: unavailable (model invalid)");
		}
		return 0;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 6 && argc != 7) {
			shell_error(sh, "Usage: motor velocity mpr set <q_speed> <r_delta_iq> <horizon> <max_delta_iq> [disturbance_ki]");
			return -EINVAL;
		}

		float q_speed = 0.0f;
		float r_delta_iq = 0.0f;
		uint32_t horizon = 0U;
		float max_delta_iq = 0.0f;
		float disturbance_ki = g_motor_params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s;
		if (!shell_parse_finite_float(argv[2], &q_speed) ||
		    !shell_parse_finite_float(argv[3], &r_delta_iq) ||
		    !shell_parse_u32(argv[4], &horizon) ||
		    !shell_parse_finite_float(argv[5], &max_delta_iq) ||
		    (argc == 7 && !shell_parse_finite_float(argv[6], &disturbance_ki))) {
			shell_error(sh, "MPR parameters must be finite numbers; horizon must be integer");
			return -EINVAL;
		}
		if (q_speed <= 0.0f || r_delta_iq <= 0.0f || horizon == 0U ||
		    horizon > MOTOR_MPR_HORIZON_MAX || max_delta_iq < 0.0f || disturbance_ki < 0.0f) {
			shell_error(sh, "Invalid velocity MPR limits; horizon must be 1..%u",
				    MOTOR_MPR_HORIZON_MAX);
			return -EINVAL;
		}

		int ret = motor_set_param_checked("velocity_mpr_q_speed", q_speed);
		ret |= motor_set_param_checked("velocity_mpr_r_delta_iq", r_delta_iq);
		ret |= motor_set_param_checked("velocity_mpr_horizon", (float)horizon);
		ret |= motor_set_param_checked("velocity_mpr_max_delta_iq_a", max_delta_iq);
		ret |= motor_set_param_checked("velocity_mpr_disturbance_ki_nm_per_rad_s",
					       disturbance_ki);
		if (ret != 0) {
			shell_error(sh, "Failed to update velocity MPR params (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity MPR set: q=%.6f r=%.6f horizon=%u dIq=%.6f A/sample dist_ki=%.6f",
			    (double)q_speed, (double)r_delta_iq, horizon,
			    (double)max_delta_iq, (double)disturbance_ki);
		return 0;
	}

	if (strcmp(argv[1], "bandwidth") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor velocity mpr bandwidth <hz>");
			return -EINVAL;
		}

		float bw_hz = 0.0f;
		if (!shell_parse_finite_float(argv[2], &bw_hz) || bw_hz <= 0.0f) {
			shell_error(sh, "Bandwidth must be a positive finite number");
			return -EINVAL;
		}

		struct motor_mpr_velocity_config cfg = g_motor_params->velocity_mpr_cfg;
		struct motor_mpr_bandwidth_result result = {0};
		const struct motor_mpr_velocity_bandwidth_input input = {
			.bandwidth_hz = bw_hz,
			.inertia_kgm2 = g_motor_params->inertia_kgm2_active,
			.torque_constant_nm_per_a = motor_torque_gain_resolve_active(g_motor_params),
			.iq_limit_a = g_motor_params->velocity_cl_iq_limit_A,
			.dt_s = g_motor_params->velocity_mpr_cfg.dt_s,
		};

		int ret = motor_mpr_velocity_config_from_bandwidth(&input, &cfg, &result);
		if (ret != 0) {
			shell_error(sh, "Failed to derive velocity MPR bandwidth params (err %d)",
				    ret);
			return ret;
		}

		ret = motor_set_param_checked("velocity_mpr_q_speed", cfg.q_speed);
		ret |= motor_set_param_checked("velocity_mpr_r_delta_iq", cfg.r_delta_iq);
		ret |= motor_set_param_checked("velocity_mpr_horizon",
					       (float)cfg.horizon);
		ret |= motor_set_param_checked("velocity_mpr_max_delta_iq_a",
					       cfg.max_delta_iq_a);
		ret |= motor_set_param_checked("velocity_mpr_disturbance_ki_nm_per_rad_s",
					       cfg.disturbance_ki_nm_per_rad_s);
		if (ret != 0) {
			shell_error(sh, "Failed to apply velocity MPR bandwidth params (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity MPR bandwidth tuned: bw=%.2f Hz -> q=%.6f r=%.6f horizon=%u dIq=%.6f A/sample dist_ki=%.6f model=%s%s",
			    (double)result.applied_bandwidth_hz, (double)cfg.q_speed,
			    (double)cfg.r_delta_iq, cfg.horizon,
			    (double)cfg.max_delta_iq_a,
			    (double)cfg.disturbance_ki_nm_per_rad_s,
			    result.model_used ? "active" : "fallback",
			    result.clamped ? " clamped" : "");
		return 0;
	}

	shell_error(sh, "Usage: motor velocity mpr status | "
		    "motor velocity mpr set <q_speed> <r_delta_iq> <horizon> <max_delta_iq> [disturbance_ki] | "
		    "motor velocity mpr bandwidth <hz>");
	return -EINVAL;
}

/* motor velocity dob status
 * motor velocity dob defaults <safe|nominal>
 * motor velocity dob enable <0|1>
 * motor velocity dob gain <observer_gain_nm_per_rad_s>
 * motor velocity dob torque_limit <nm>
 * motor velocity dob iq_limit <a>
 */
int cmd_motor_velocity_dob(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || argc > 3) {
		shell_error(sh, "Usage: motor velocity dob status | "
			    "motor velocity dob defaults <safe|nominal> | "
			    "motor velocity dob enable <0|1> | "
			    "motor velocity dob gain <nm_per_rad_s> | "
			    "motor velocity dob torque_limit <nm> | "
			    "motor velocity dob iq_limit <a>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (strcmp(argv[1], "status") == 0) {
		if (argc != 2) {
			shell_error(sh, "Usage: motor velocity dob status");
			return -EINVAL;
		}
		shell_print(sh, "Velocity DOB:");
		shell_print(sh, "  Enabled:     %s",
			    g_motor_params->velocity_dob_cfg.enabled ? "YES" : "NO");
		const char *ready_reason = NULL;
		bool ready = motor_velocity_dob_ready(g_motor_params, &ready_reason);
		shell_print(sh, "  Ready:       %s (%s)",
			    ready ? "YES" : "NO", ready_reason);
		shell_print(sh, "  Gain:        %.6f Nm/(rad/s)",
			    (double)g_motor_params->velocity_dob_cfg.observer_gain_nm_per_rad_s);
		shell_print(sh, "  Torque limit %.6f Nm",
			    (double)g_motor_params->velocity_dob_cfg.torque_limit_nm);
		shell_print(sh, "  Iq FF limit: %.6f A",
			    (double)g_motor_params->velocity_dob_cfg.iq_ff_limit_a);
		shell_print(sh, "  State:       d=%.6f Nm, ff=%.6f A, res=%.6f rad/s",
			    (double)g_motor_params->live.velocity_dob_disturbance_nm,
			    (double)g_motor_params->live.velocity_dob_iq_ff_a,
			    (double)g_motor_params->live.velocity_dob_residual_rad_s);
		return 0;
	}

	if (strcmp(argv[1], "defaults") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor velocity dob defaults <safe|nominal>");
			return -EINVAL;
		}

		enum motor_gains_profile profile;
		if (motor_parse_gains_profile(argv[2], &profile) != 0) {
			shell_error(sh, "Profile must be 'safe' or 'nominal'");
			return -EINVAL;
		}

		float gain = 0.0f;
		float torque_limit = 0.0f;
		float iq_ff_limit = 0.0f;
		float kt = 0.0f;
		int ret = motor_compute_velocity_dob_defaults(g_motor_params, profile,
							      &gain, &torque_limit,
							      &iq_ff_limit, &kt);
		if (ret != 0) {
			shell_error(sh,
				    "Cannot compute DOB defaults; need valid active torque_gain and velocity current limit");
			return ret;
		}

		ret = motor_api_set_param("velocity_dob_enable", 0.0f);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_enable (err %d)", ret);
			return ret;
		}
		ret = motor_api_set_param("velocity_dob_observer_gain_nm_per_rad_s", gain);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_observer_gain_nm_per_rad_s (err %d)",
				    ret);
			return ret;
		}
		ret = motor_api_set_param("velocity_dob_torque_limit_nm", torque_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_torque_limit_nm (err %d)", ret);
			return ret;
		}
		ret = motor_api_set_param("velocity_dob_iq_ff_limit_a", iq_ff_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_iq_ff_limit_a (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity DOB %s defaults staged: enable=0 gain=%.6f Nm/(rad/s), torque_limit=%.6f Nm, iq_ff_limit=%.6f A (Kt=%.6f Nm/A). Run 'motor velocity dob enable 1' after velocity control is stable.",
			    (profile == MOTOR_GAINS_PROFILE_SAFE) ? "safe" : "nominal",
			    (double)gain, (double)torque_limit, (double)iq_ff_limit, (double)kt);
		return 0;
	}

	if (argc != 3) {
		shell_error(sh, "Usage: motor velocity dob defaults <safe|nominal> | "
			    "motor velocity dob <enable|gain|torque_limit|iq_limit> <value>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "enable") == 0) {
		bool enabled = false;
		if (!shell_parse_bool01(argv[2], &enabled)) {
			shell_error(sh, "enable value must be 0 or 1");
			return -EINVAL;
		}
		if (enabled) {
			const char *ready_reason = NULL;
			if (!motor_velocity_dob_ready(g_motor_params, &ready_reason)) {
				shell_error(sh, "Velocity DOB not ready: %s", ready_reason);
				return -EACCES;
			}
		}
		int ret = motor_api_set_param("velocity_dob_enable", enabled ? 1.0f : 0.0f);
		if (ret != 0) {
			shell_error(sh, "Failed to update velocity_dob_enable (err %d)", ret);
			return ret;
		}
		motor_dob_reset(&g_motor_params->velocity_dob_state,
				g_motor_params->live.velocity_rad_s);
		g_motor_params->live.velocity_dob_iq_ff_a = 0.0f;
		g_motor_params->live.velocity_dob_disturbance_nm = 0.0f;
		g_motor_params->live.velocity_dob_residual_rad_s = 0.0f;
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Velocity DOB %s", enabled ? "enabled" : "disabled");
		return 0;
	}

	float value = 0.0f;
	if (!shell_parse_finite_float(argv[2], &value)) {
		shell_error(sh, "value must be a finite number");
		return -EINVAL;
	}

	const char *param_name = NULL;
	const char *label = NULL;
	if (strcmp(argv[1], "gain") == 0) {
		if (value < 0.0f) {
			shell_error(sh, "gain must be >= 0");
			return -EINVAL;
		}
		param_name = "velocity_dob_observer_gain_nm_per_rad_s";
		label = "gain";
	} else if (strcmp(argv[1], "torque_limit") == 0) {
		if (value <= 0.0f) {
			shell_error(sh, "torque_limit must be > 0");
			return -EINVAL;
		}
		param_name = "velocity_dob_torque_limit_nm";
		label = "torque limit";
	} else if (strcmp(argv[1], "iq_limit") == 0) {
		if (value < 0.0f) {
			shell_error(sh, "iq_limit must be >= 0");
			return -EINVAL;
		}
		param_name = "velocity_dob_iq_ff_limit_a";
		label = "Iq FF limit";
	} else {
		shell_error(sh, "Unknown DOB field '%s' (use defaults|enable|gain|torque_limit|iq_limit)",
			    argv[1]);
		return -EINVAL;
	}

	int ret = motor_api_set_param(param_name, value);
	if (ret != 0) {
		shell_error(sh, "Failed to update %s (err %d)", param_name, ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Velocity DOB %s set to %.6f", label, (double)value);
	return 0;
}

/* motor position target <deg> */
