#include "shell_control_common.h"

/* Domain implementation split from shell_control.c. */

int cmd_motor_control_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Control System:");
	shell_print(sh, "  Regulator:   %s",
		    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ? "MPR" : "PI");
	shell_print(sh, "  Velocity dt: %.6f s (%u tick)",
		    (double)g_motor_params->velocity_mpr_cfg.dt_s,
		    MAX(OUTER_LOOP_DECIMATION_MIN, g_motor_params->velocity_loop_decimation));
	shell_print(sh, "  Position dt: %.6f s (%u tick)",
		    (double)g_motor_params->position_mpr_cfg.dt_s,
		    MAX(OUTER_LOOP_DECIMATION_MIN, g_motor_params->position_loop_decimation));
	shell_print(sh, "  Velocity PI: Kp=%.6f Ki=%.6f IqLim=%.6f A",
		    (double)g_motor_params->velocity_cl_kp_A_per_rad_s,
		    (double)g_motor_params->velocity_cl_ki_A_per_rad,
		    (double)g_motor_params->velocity_cl_iq_limit_A);
	shell_print(sh, "  Position PI: Kp=%.6f Ki=%.6f",
		    (double)g_motor_params->position_cl_kp_rad_s_per_rad,
		    (double)g_motor_params->position_cl_ki_rad_s2_per_rad);
	shell_print(sh, "  Velocity MPR: q=%.6f r=%.6f horizon=%u",
		    (double)g_motor_params->velocity_mpr_cfg.q_speed,
		    (double)g_motor_params->velocity_mpr_cfg.r_delta_iq,
		    g_motor_params->velocity_mpr_cfg.horizon);
	shell_print(sh, "  Position MPR: q_pos=%.6f q_vel=%.6f r=%.6f horizon=%u",
		    (double)g_motor_params->position_mpr_cfg.q_position,
		    (double)g_motor_params->position_mpr_cfg.q_velocity_ff,
		    (double)g_motor_params->position_mpr_cfg.r_delta_velocity,
		    g_motor_params->position_mpr_cfg.horizon);
	const char *dob_reason = NULL;
	bool dob_ready = motor_velocity_dob_ready(g_motor_params, &dob_reason);
	shell_print(sh, "  Feedforward DOB:    %s ready=%s (%s) ff=%.6f A dist=%.6f Nm",
		    g_motor_params->velocity_dob_cfg.enabled ? "ENABLED" : "DISABLED",
		    dob_ready ? "YES" : "NO",
		    dob_reason,
		    (double)g_motor_params->live.velocity_dob_iq_ff_a,
		    (double)g_motor_params->live.velocity_dob_disturbance_nm);
	shell_print(sh, "  Feedforward detent: %s gain=%.3f limit=%.6f A live=%.6f A",
		    g_motor_params->detent_map_cfg.enabled ? "ENABLED" : "DISABLED",
		    (double)g_motor_params->detent_map_cfg.gain,
		    (double)g_motor_params->detent_map_cfg.iq_ff_limit_a,
		    (double)g_motor_params->live.detent_iq_ff_a);
	return 0;
}

int cmd_motor_outer_status(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_motor_control_status(sh, argc, argv);
}

int cmd_motor_outer_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor outer mode <pi|mpr>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float mode_value;
	if (strcmp(argv[1], "pi") == 0) {
		mode_value = (float)MOTOR_OUTER_LOOP_MODE_PI;
	} else if (strcmp(argv[1], "mpr") == 0) {
		mode_value = (float)MOTOR_OUTER_LOOP_MODE_MPR;
	} else {
		shell_error(sh, "Mode must be 'pi' or 'mpr'");
		return -EINVAL;
	}

	int ret = motor_api_set_param("outer_loop_mode", mode_value);
	if (ret != 0) {
		shell_error(sh, "Failed to set outer loop mode (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Outer loop mode set to %s",
		    mode_value == (float)MOTOR_OUTER_LOOP_MODE_MPR ? "MPR" : "PI");
	return 0;
}

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
/* motor rls status */
int cmd_motor_rls_status(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	shell_print(sh, "RLS Estimator Status:");
	shell_print(sh, "");
	
	/* D-axis status */
	bool d_converged = rls_motor_est_is_converged(&g_motor_params->rls.d);
	float32_t d_residual = rls_motor_est_get_residual(&g_motor_params->rls.d);
	shell_print(sh, "D-axis:");
	shell_print(sh, "  Converged:     %s", d_converged ? "YES" : "NO");
	shell_print(sh, "  Residual:      %.6f V", (double)d_residual);
	shell_print(sh, "  Update count:  %u", g_motor_params->rls.d.num_updates);
	
	/* Q-axis status */
	bool q_converged = rls_motor_est_is_converged(&g_motor_params->rls.q);
	float32_t q_residual = rls_motor_est_get_residual(&g_motor_params->rls.q);
	shell_print(sh, "");
	shell_print(sh, "Q-axis:");
	shell_print(sh, "  Converged:     %s", q_converged ? "YES" : "NO");
	shell_print(sh, "  Residual:      %.6f V", (double)q_residual);
	shell_print(sh, "  Update count:  %u", g_motor_params->rls.q.num_updates);
	
	return 0;
}

/* motor rls params */
int cmd_motor_rls_params(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	/* D-axis estimates */
	float32_t Rs_d = rls_motor_est_get_Rs(&g_motor_params->rls.d);
	float32_t Ld = rls_motor_est_get_L(&g_motor_params->rls.d);
	float32_t Vbias_d = rls_motor_est_get_Vbias(&g_motor_params->rls.d);
	float32_t Vdt_d = rls_motor_est_get_Vdt_sign(&g_motor_params->rls.d);
	
	/* Q-axis estimates */
	float32_t Rs_q = rls_motor_est_get_Rs(&g_motor_params->rls.q);
	float32_t Lq = rls_motor_est_get_L(&g_motor_params->rls.q);
	float32_t Vbias_q = rls_motor_est_get_Vbias(&g_motor_params->rls.q);
	float32_t Vdt_q = rls_motor_est_get_Vdt_sign(&g_motor_params->rls.q);
	
	/* Averaged/synthesized values */
	float32_t Rs_avg = g_motor_params->Rs_measured_ohm;
	
	shell_print(sh, "RLS Parameter Estimates:");
	shell_print(sh, "");
	shell_print(sh, "Resistance:");
	shell_print(sh, "  Rs (d-axis):   %.6f Ω", (double)Rs_d);
	shell_print(sh, "  Rs (q-axis):   %.6f Ω", (double)Rs_q);
	shell_print(sh, "  Rs (averaged): %.6f Ω", (double)Rs_avg);
	shell_print(sh, "");
	shell_print(sh, "Inductance:");
	shell_print(sh, "  Ld:            %.6f H (%.3f mH)", (double)Ld, (double)(Ld * 1000.0f));
	shell_print(sh, "  Lq:            %.6f H (%.3f mH)", (double)Lq, (double)(Lq * 1000.0f));
	shell_print(sh, "  Saliency:      %.3f", (double)(Lq / Ld));
	shell_print(sh, "");
	shell_print(sh, "Nonlinearities:");
	shell_print(sh, "  Vbias (d):     %.6f V", (double)Vbias_d);
	shell_print(sh, "  Vbias (q):     %.6f V", (double)Vbias_q);
	shell_print(sh, "  Vdt*sign (d):  %.6f V", (double)Vdt_d);
	shell_print(sh, "  Vdt*sign (q):  %.6f V", (double)Vdt_q);
	
	return 0;
}

/* motor rls temp */
int cmd_motor_rls_temp(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	float32_t T_rls = g_motor_params->thermal.t_rls_c;
	float32_t T_thermal = thermal_model_get_temperature(&g_motor_params->thermal.model);
	float32_t P_loss = thermal_model_get_power_loss(&g_motor_params->thermal.model);
	float32_t T_delta = T_rls - T_thermal;
	
	shell_print(sh, "Temperature Estimates:");
	shell_print(sh, "");
	shell_print(sh, "  T_rls (from Rs):       %.2f °C", (double)T_rls);
	shell_print(sh, "  T_thermal (model):     %.2f °C", (double)T_thermal);
	shell_print(sh, "  Delta:                 %.2f °C", (double)T_delta);
	shell_print(sh, "");
	shell_print(sh, "Thermal Model:");
	shell_print(sh, "  Power loss:            %.3f W", (double)P_loss);
	shell_print(sh, "  Ambient temp:          %.2f °C", (double)g_motor_params->thermal.model.T_ambient);
	shell_print(sh, "  R_th:                  %.3f °C/W", (double)g_motor_params->thermal.model.R_th);
	shell_print(sh, "  C_th:                  %.1f J/°C", (double)g_motor_params->thermal.model.C_th);
	
	return 0;
}

/* motor rls gating */
int cmd_motor_rls_gating(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	shell_print(sh, "RLS Gating Conditions:");
	shell_print(sh, "");
	shell_print(sh, "Thresholds:");
	shell_print(sh, "  Min current:     %.3f A", (double)g_motor_params->rls.min_current_a);
	/* Convert rad/s to Hz for display */
	float32_t min_speed_hz = g_motor_params->rls.min_speed_rad_s / (2.0f * PI_F32);
	shell_print(sh, "  Min speed:       %.2f Hz", (double)min_speed_hz);
	shell_print(sh, "  Max voltage:     %.2f V", (double)g_motor_params->rls.max_voltage_v);
	shell_print(sh, "  Max residual:    %.4f V", (double)g_motor_params->rls.max_residual_v);
	shell_print(sh, "");
	shell_print(sh, "Current Status:");
	float32_t Id_abs = fabsf(g_motor_params->live.Id_A);
	float32_t Iq_abs = fabsf(g_motor_params->live.Iq_A);
	float32_t omega_hz = fabsf(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32));
	shell_print(sh, "  |Id|:             %.3f A %s", (double)Id_abs,
	            Id_abs > g_motor_params->rls.min_current_a ? "[OK]" : "[LOW]");
	shell_print(sh, "  |Iq|:             %.3f A %s", (double)Iq_abs,
	            Iq_abs > g_motor_params->rls.min_current_a ? "[OK]" : "[LOW]");
	shell_print(sh, "  Speed:           %.2f Hz %s", (double)omega_hz,
	            omega_hz > min_speed_hz ? "[OK]" : "[LOW]");
	shell_print(sh, "  D residual:      %.6f V %s", (double)g_motor_params->rls.d.residual,
	            fabsf(g_motor_params->rls.d.residual) < g_motor_params->rls.max_residual_v ? "[OK]" : "[HIGH]");
	shell_print(sh, "  Q residual:      %.6f V %s", (double)g_motor_params->rls.q.residual,
	            fabsf(g_motor_params->rls.q.residual) < g_motor_params->rls.max_residual_v ? "[OK]" : "[HIGH]");
	
	return 0;
}

/* motor rls reset */
int cmd_motor_rls_reset(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	/* Reset both RLS estimators */
	rls_motor_est_reset(&g_motor_params->rls.d);
	rls_motor_est_reset(&g_motor_params->rls.q);
	prbs_reset(&g_motor_params->rls.prbs_gen);

	/* Reset estimator side-state used by ISR update gating/derivative timing. */
	g_motor_params->rls.id_prev_a = 0.0f;
	g_motor_params->rls.iq_prev_a = 0.0f;
	g_motor_params->rls.d_prev_cycle = g_motor_params->control_loop_count;
	g_motor_params->rls.q_prev_cycle = g_motor_params->control_loop_count;
	g_motor_params->rls.d_prev_valid = 0u;
	g_motor_params->rls.q_prev_valid = 0u;

	/* Restore synthesized values to reset estimator baselines. */
	g_motor_params->rls.ld_est_h = rls_motor_est_get_L(&g_motor_params->rls.d);
	g_motor_params->rls.lq_est_h = rls_motor_est_get_L(&g_motor_params->rls.q);
	g_motor_params->Rs_measured_ohm = rls_motor_est_get_Rs(&g_motor_params->rls.d);
	g_motor_params->thermal.t_rls_c = THERMAL_T_AMBIENT;

	shell_print(sh, "RLS estimators fully reset (state, covariance, PRBS, side-state)");

	return 0;
}
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */


