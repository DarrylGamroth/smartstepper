/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "shell_commands.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"
#include "config.h"
#include "angle_wrap.h"
#include "shell_commands_motion.h"
#include "shell_commands_state.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(shell_commands, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set by main) */
struct motor_parameters *g_motor_params = NULL;

/**
 * @brief Set global motor parameters pointer
 * Called from main() to provide access to motor_parameters structure
 */
void shell_set_motor_params(struct motor_parameters *params)
{
	g_motor_params = params;
}

/**
 * @brief Get global motor parameters pointer
 * @return Pointer to motor parameters, or NULL if not set
 */
struct motor_parameters *shell_get_motor_params(void)
{
	return g_motor_params;
}

bool motor_control_is_armed(const struct motor_parameters *params)
{
	return params && (atomic_get(&params->control_armed) != 0);
}

void motor_command_feed_watchdog(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	params->last_command_update_ms = k_uptime_get_32();
	params->command_timeout_latched = false;
}

/*============================================================================
 * Shell Command Implementations
 *============================================================================*/

/* motor params get <name> */
static int cmd_motor_params_get(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor params get <name>");
		return -EINVAL;
	}

	const char *name = argv[1];
	float value;
	
	if (motor_api_get_param(name, &value) == 0) {
		shell_print(sh, "%s = %.6f", name, (double)value);
		return 0;
	} else {
		shell_error(sh, "Parameter '%s' not found", name);
		return -ENOENT;
	}
}

/* motor params set <name> <value> */
static int cmd_motor_params_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor params set <name> <value>");
		return -EINVAL;
	}

	const char *name = argv[1];
	float value = strtof(argv[2], NULL);
	
	if (motor_api_set_param(name, value) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Set %s = %.6f", name, (double)value);
		return 0;
	} else {
		shell_error(sh, "Parameter '%s' not found or read-only", name);
		return -ENOENT;
	}
}

/* motor params list */
static int cmd_motor_params_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	shell_print(sh, "Available motor parameters:");
	shell_print(sh, "%-25s %s", "Name", "Value");
	shell_print(sh, "%-25s %s", "----", "-----");
	
	size_t count = motor_api_get_param_count();
	for (size_t i = 0; i < count; i++) {
		const char *name = motor_api_get_param_name(i);
		float value;
		if (motor_api_get_param_by_index(i, &value) == 0) {
			shell_print(sh, "%-25s %.6f", name, (double)value);
		}
	}
	
	return 0;
}

/* motor current id <value> */
static int cmd_motor_current_id(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current id <amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float id_amps = strtof(argv[1], NULL);
	if (fabsf(id_amps) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero current commands.");
		return -EACCES;
	}
	
	if (motor_api_set_param("Id_setpoint_A", id_amps) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Id setpoint = %.3f A", (double)id_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set Id current");
		return -EIO;
	}
}

/* motor current iq <value> */
static int cmd_motor_current_iq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current iq <amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float iq_amps = strtof(argv[1], NULL);
	if (fabsf(iq_amps) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero current commands.");
		return -EACCES;
	}
	
	if (motor_api_set_param("Iq_setpoint_A", iq_amps) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Iq setpoint = %.3f A", (double)iq_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set Iq current");
		return -EIO;
	}
}

/* motor current dq <id> <iq> */
static int cmd_motor_current_dq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor current dq <id_amps> <iq_amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float id_amps = strtof(argv[1], NULL);
	float iq_amps = strtof(argv[2], NULL);
	if ((fabsf(id_amps) > 1e-6f || fabsf(iq_amps) > 1e-6f) &&
	    !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero current commands.");
		return -EACCES;
	}
	
	if (motor_api_set_currents(id_amps, iq_amps) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Id = %.3f A, Iq = %.3f A", (double)id_amps, (double)iq_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set DQ currents");
		return -EIO;
	}
}

/* motor velocity target <hz> */
static int cmd_motor_velocity_target(const struct shell *sh, size_t argc, char **argv)
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
	if (!motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_OPEN) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_CLOSED)) {
		shell_error(sh, "Velocity target requires velocity_open or velocity_closed mode.");
		return -EACCES;
	}

	float target_hz = strtof(argv[1], NULL);
	if (fabsf(target_hz) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero velocity commands.");
		return -EACCES;
	}

	/* Convert Hz to mechanical rad/s for trajectory */
	float target_rad_s = target_hz * 2.0f * PI_F32;

	/* Respect currently configured profile velocity limits. */
	float32_t target_clamped = clampf(target_rad_s,
					  -g_motor_params->profile_max_velocity_rad_s,
					  g_motor_params->profile_max_velocity_rad_s);

	/* Set trajectory target (thread-safe access) */
	traj_set_target_value(&g_motor_params->traj_velocity, target_clamped);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Velocity target set to %.2f Hz", (double)(target_clamped / (2.0f * PI_F32)));
	return 0;
}

/* motor velocity status */
static int cmd_motor_velocity_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct smf_state *mode = g_motor_params->state_for_isr;
	if (!motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_OPEN) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_CLOSED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_POSITION)) {
		shell_print(sh, "Velocity controller: INACTIVE");
		return 0;
	}

	/* Get values from trajectory/controller (in rad/s) and convert to Hz for display */
	float target_rad_s = traj_get_target_value(&g_motor_params->traj_velocity);
	float ref_rad_s = g_motor_params->velocity_ref_rad_s;
	float meas_rad_s = g_motor_params->velocity_rad_s;
	float target_hz = target_rad_s / (2.0f * PI_F32);
	float ref_hz = ref_rad_s / (2.0f * PI_F32);
	float meas_hz = meas_rad_s / (2.0f * PI_F32);
	float error_hz = ref_hz - meas_hz;
	bool at_target = traj_is_at_target(&g_motor_params->traj_velocity);

	/* Determine motion state */
	const char *motion_str;
	if (fabsf(meas_rad_s) < 0.1f && fabsf(ref_rad_s) < 0.1f) {
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
	shell_print(sh, "  At Target:  %s", at_target ? "YES" : "NO");
	shell_print(sh, "  Motion:     %s", motion_str);
	shell_print(sh, "  Kp:         %.5f A/(rad/s)", (double)g_motor_params->velocity_cl_kp_A_per_rad_s);
	shell_print(sh, "  Iq limit:   %.3f A", (double)g_motor_params->velocity_cl_iq_limit_A);

	return 0;
}

/* motor velocity gains <kp_a_per_rad_s> <iq_limit_a> */
static int cmd_motor_velocity_gains(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor velocity gains <kp_a_per_rad_s> <iq_limit_a>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float kp = strtof(argv[1], NULL);
	float iq_limit = strtof(argv[2], NULL);
	if (motor_api_set_param("velocity_cl_kp_A_per_rad_s", kp) != 0 ||
	    motor_api_set_param("velocity_cl_iq_limit_A", iq_limit) != 0) {
		shell_error(sh, "Failed to update velocity gains");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Velocity gains set: Kp=%.5f A/(rad/s), Iq limit=%.3f A",
		    (double)kp, (double)iq_limit);
	return 0;
}

/* motor position target <deg> */
static int cmd_motor_position_target(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor position target <deg>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		shell_error(sh, "Not in position mode. Use 'motor state mode position' first.");
		return -EACCES;
	}

	float target_deg = strtof(argv[1], NULL);
	float target_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	g_motor_params->profile_sequence_running = false;
	g_motor_params->profile_sequence_tick_counter = 0U;
	motion_profile_quintic_cancel(&g_motor_params->position_profile, target_rad);
	g_motor_params->position_target_rad = target_rad;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Position target set to %.2f deg", (double)(target_rad * 180.0f / PI_F32));
	return 0;
}

/* motor position status */
static int cmd_motor_position_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		shell_print(sh, "Position controller: INACTIVE");
		return 0;
	}

	float target_rad = g_motor_params->position_target_rad;
	float meas_rad = g_motor_params->position_rad;
	float err_rad = wrap_rad_pi(target_rad - meas_rad);

	shell_print(sh, "Position Controller Status:");
	shell_print(sh, "  Target:     %.2f deg", (double)(target_rad * 180.0f / PI_F32));
	shell_print(sh, "  Measured:   %.2f deg", (double)(meas_rad * 180.0f / PI_F32));
	shell_print(sh, "  Error:      %.2f deg", (double)(err_rad * 180.0f / PI_F32));
	shell_print(sh, "  Profile:    %s",
		    motion_profile_quintic_is_active(&g_motor_params->position_profile) ?
			    "ACTIVE" :
			    (g_motor_params->position_profile.valid ? "COMPLETE" : "OFF"));
	shell_print(sh, "  Kp:         %.5f (rad/s)/rad", (double)g_motor_params->position_cl_kp_rad_s_per_rad);
	return 0;
}

/* motor position gains <kp_rad_s_per_rad> */
static int cmd_motor_position_gains(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor position gains <kp_rad_s_per_rad>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float kp = strtof(argv[1], NULL);
	if (motor_api_set_param("position_cl_kp_rad_s_per_rad", kp) != 0) {
		shell_error(sh, "Failed to update position gain");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Position gain set: Kp=%.5f (rad/s)/rad", (double)kp);
	return 0;
}

/* motor pi get <controller> */
static int cmd_motor_pi_get(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor pi get <id|iq>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const char *controller = argv[1];
	float kp, ki;
	
	if (strcmp(controller, "id") == 0) {
		kp = g_motor_params->pi_Id.kp;
		ki = g_motor_params->pi_Id.ki;
	} else if (strcmp(controller, "iq") == 0) {
		kp = g_motor_params->pi_Iq.kp;
		ki = g_motor_params->pi_Iq.ki;
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}
	
	shell_print(sh, "PI_%s: Kp = %.6f, Ki = %.6f", controller, (double)kp, (double)ki);
	return 0;
}

/* motor pi set <controller> <kp> <ki> */
static int cmd_motor_pi_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "Usage: motor pi set <id|iq> <kp> <ki>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const char *controller = argv[1];
	float kp = strtof(argv[2], NULL);
	float ki = strtof(argv[3], NULL);
	
	/* TODO: PI controllers are NOT double buffered - need API extension */
	/* For now, write directly (safe because PI gains not used in ISR context) */
	if (strcmp(controller, "id") == 0) {
		g_motor_params->pi_Id.kp = kp;
		g_motor_params->pi_Id.ki = ki;
	} else if (strcmp(controller, "iq") == 0) {
		g_motor_params->pi_Iq.kp = kp;
		g_motor_params->pi_Iq.ki = ki;
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}
	
	shell_print(sh, "Set PI_%s: Kp = %.6f, Ki = %.6f", controller, (double)kp, (double)ki);
	return 0;
}

/* motor pi bandwidth <controller> <hz> */
static int cmd_motor_pi_bandwidth(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor pi bandwidth <id|iq> <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const char *controller = argv[1];
	float bw_hz = strtof(argv[2], NULL);
	
	/* Validate bandwidth */
	if (bw_hz <= 0.0f || bw_hz > CONTROL_LOOP_FREQUENCY_HZ / 2.0f) {
		shell_error(sh, "Bandwidth must be positive and below Nyquist (%.1f Hz)",
		            (double)(CONTROL_LOOP_FREQUENCY_HZ / 2.0f));
		return -EINVAL;
	}
	
	/* Calculate PI gains from bandwidth */
	float32_t bw_rps = 2.0f * PI_F32 * bw_hz;
	float32_t T_sample = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t kp, ki;
	
	if (strcmp(controller, "id") == 0) {
		kp = MOTOR_INDUCTANCE_D_H * bw_rps;
		ki = (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_D_H) * T_sample;
		g_motor_params->pi_Id.kp = kp;
		g_motor_params->pi_Id.ki = ki;
		shell_print(sh, "D-axis PI tuned to %.1f Hz bandwidth:", (double)bw_hz);
		shell_print(sh, "  Kp = %.6f", (double)kp);
		shell_print(sh, "  Ki = %.6f", (double)ki);
	} else if (strcmp(controller, "iq") == 0) {
		kp = MOTOR_INDUCTANCE_Q_H * bw_rps;
		ki = (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_Q_H) * T_sample;
		g_motor_params->pi_Iq.kp = kp;
		g_motor_params->pi_Iq.ki = ki;
		shell_print(sh, "Q-axis PI tuned to %.1f Hz bandwidth:", (double)bw_hz);
		shell_print(sh, "  Kp = %.6f", (double)kp);
		shell_print(sh, "  Ki = %.6f", (double)ki);
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}
	
	return 0;
}

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
/* motor rls status */
static int cmd_motor_rls_status(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	shell_print(sh, "RLS Estimator Status:");
	shell_print(sh, "");
	
	/* D-axis status */
	bool d_converged = rls_motor_est_is_converged(&g_motor_params->rls_d);
	float32_t d_residual = rls_motor_est_get_residual(&g_motor_params->rls_d);
	shell_print(sh, "D-axis:");
	shell_print(sh, "  Converged:     %s", d_converged ? "YES" : "NO");
	shell_print(sh, "  Residual:      %.6f V", (double)d_residual);
	shell_print(sh, "  Update count:  %u", g_motor_params->rls_d.num_updates);
	
	/* Q-axis status */
	bool q_converged = rls_motor_est_is_converged(&g_motor_params->rls_q);
	float32_t q_residual = rls_motor_est_get_residual(&g_motor_params->rls_q);
	shell_print(sh, "");
	shell_print(sh, "Q-axis:");
	shell_print(sh, "  Converged:     %s", q_converged ? "YES" : "NO");
	shell_print(sh, "  Residual:      %.6f V", (double)q_residual);
	shell_print(sh, "  Update count:  %u", g_motor_params->rls_q.num_updates);
	
	return 0;
}

/* motor rls params */
static int cmd_motor_rls_params(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	/* D-axis estimates */
	float32_t Rs_d = rls_motor_est_get_Rs(&g_motor_params->rls_d);
	float32_t Ld = rls_motor_est_get_L(&g_motor_params->rls_d);
	float32_t Vbias_d = rls_motor_est_get_Vbias(&g_motor_params->rls_d);
	float32_t Vdt_d = rls_motor_est_get_Vdt_sign(&g_motor_params->rls_d);
	
	/* Q-axis estimates */
	float32_t Rs_q = rls_motor_est_get_Rs(&g_motor_params->rls_q);
	float32_t Lq = rls_motor_est_get_L(&g_motor_params->rls_q);
	float32_t Vbias_q = rls_motor_est_get_Vbias(&g_motor_params->rls_q);
	float32_t Vdt_q = rls_motor_est_get_Vdt_sign(&g_motor_params->rls_q);
	
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
static int cmd_motor_rls_temp(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	float32_t T_rls = g_motor_params->T_rls_C;
	float32_t T_thermal = thermal_model_get_temperature(&g_motor_params->thermal);
	float32_t P_loss = thermal_model_get_power_loss(&g_motor_params->thermal);
	float32_t T_delta = T_rls - T_thermal;
	
	shell_print(sh, "Temperature Estimates:");
	shell_print(sh, "");
	shell_print(sh, "  T_rls (from Rs):       %.2f °C", (double)T_rls);
	shell_print(sh, "  T_thermal (model):     %.2f °C", (double)T_thermal);
	shell_print(sh, "  Delta:                 %.2f °C", (double)T_delta);
	shell_print(sh, "");
	shell_print(sh, "Thermal Model:");
	shell_print(sh, "  Power loss:            %.3f W", (double)P_loss);
	shell_print(sh, "  Ambient temp:          %.2f °C", (double)g_motor_params->thermal.T_ambient);
	shell_print(sh, "  R_th:                  %.3f °C/W", (double)g_motor_params->thermal.R_th);
	shell_print(sh, "  C_th:                  %.1f J/°C", (double)g_motor_params->thermal.C_th);
	
	return 0;
}

/* motor rls gating */
static int cmd_motor_rls_gating(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	shell_print(sh, "RLS Gating Conditions:");
	shell_print(sh, "");
	shell_print(sh, "Thresholds:");
	shell_print(sh, "  Min current:     %.3f A", (double)g_motor_params->rls_min_current_A);
	/* Convert rad/s to Hz for display */
	float32_t min_speed_hz = g_motor_params->rls_min_speed_rad_s / (2.0f * PI_F32);
	shell_print(sh, "  Min speed:       %.2f Hz", (double)min_speed_hz);
	shell_print(sh, "  Max voltage:     %.2f V", (double)g_motor_params->rls_max_voltage_V);
	shell_print(sh, "  Max residual:    %.4f V", (double)g_motor_params->rls_max_residual);
	shell_print(sh, "");
	shell_print(sh, "Current Status:");
	float32_t Id_abs = fabsf(g_motor_params->Id_A);
	float32_t Iq_abs = fabsf(g_motor_params->Iq_A);
	float32_t omega_hz = fabsf(g_motor_params->velocity_rad_s / (2.0f * PI_F32));
	shell_print(sh, "  |Id|:             %.3f A %s", (double)Id_abs,
	            Id_abs > g_motor_params->rls_min_current_A ? "[OK]" : "[LOW]");
	shell_print(sh, "  |Iq|:             %.3f A %s", (double)Iq_abs,
	            Iq_abs > g_motor_params->rls_min_current_A ? "[OK]" : "[LOW]");
	shell_print(sh, "  Speed:           %.2f Hz %s", (double)omega_hz,
	            omega_hz > min_speed_hz ? "[OK]" : "[LOW]");
	shell_print(sh, "  D residual:      %.6f V %s", (double)g_motor_params->rls_d.residual,
	            fabsf(g_motor_params->rls_d.residual) < g_motor_params->rls_max_residual ? "[OK]" : "[HIGH]");
	shell_print(sh, "  Q residual:      %.6f V %s", (double)g_motor_params->rls_q.residual,
	            fabsf(g_motor_params->rls_q.residual) < g_motor_params->rls_max_residual ? "[OK]" : "[HIGH]");
	
	return 0;
}

/* motor rls reset */
static int cmd_motor_rls_reset(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	/* Reset both RLS estimators */
	rls_motor_est_reset(&g_motor_params->rls_d);
	rls_motor_est_reset(&g_motor_params->rls_q);
	
	shell_print(sh, "RLS estimators reset");
	
	return 0;
}
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

/*============================================================================
 * Shell Command Tree
 *============================================================================*/

/* motor params subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_params,
	SHELL_CMD(get, NULL, "Get parameter value", cmd_motor_params_get),
	SHELL_CMD(set, NULL, "Set parameter value", cmd_motor_params_set),
	SHELL_CMD(list, NULL, "List all parameters", cmd_motor_params_list),
	SHELL_SUBCMD_SET_END
);

/* motor current subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_current,
	SHELL_CMD(id, NULL, "Set Id current (A)", cmd_motor_current_id),
	SHELL_CMD(iq, NULL, "Set Iq current (A)", cmd_motor_current_iq),
	SHELL_CMD(dq, NULL, "Set Id and Iq currents", cmd_motor_current_dq),
	SHELL_SUBCMD_SET_END
);

/* motor state mode subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state_mode,
	SHELL_CMD(torque, NULL, "Torque (Id/Iq) control mode", cmd_motor_state_mode_torque),
	SHELL_CMD(velocity_open, NULL, "Open-loop velocity control mode", cmd_motor_state_mode_velocity_open),
	SHELL_CMD(velocity_closed, NULL, "Closed-loop velocity control mode", cmd_motor_state_mode_velocity_closed),
	SHELL_CMD(position, NULL, "Closed-loop position control mode", cmd_motor_state_mode_position),
	SHELL_SUBCMD_SET_END
);

/* motor state subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state,
	SHELL_CMD(idle, NULL, "Transition to IDLE state", cmd_motor_state_idle),
	SHELL_CMD(offline, NULL, "Transition to OFFLINE state", cmd_motor_state_offline),
	SHELL_CMD(online, NULL, "Transition to ONLINE state", cmd_motor_state_online),
	SHELL_CMD(calibrate, NULL, "Run calibration sequence", cmd_motor_state_calibrate),
	SHELL_CMD(clear_error, NULL, "Clear error state", cmd_motor_state_clear_error),
	SHELL_CMD(status, NULL, "Show motor status", cmd_motor_state_status),
	SHELL_CMD(mode, &sub_motor_state_mode, "Switch control mode", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor pi subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_pi,
	SHELL_CMD(get, NULL, "Get PI gains", cmd_motor_pi_get),
	SHELL_CMD(set, NULL, "Set PI gains", cmd_motor_pi_set),
	SHELL_CMD(bandwidth, NULL, "Set bandwidth (auto-tune)", cmd_motor_pi_bandwidth),
	SHELL_SUBCMD_SET_END
);

/* motor info subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_info,
	SHELL_CMD(config, NULL, "Show motor config", cmd_motor_info_config),
	SHELL_CMD(measured, NULL, "Show measured params", cmd_motor_info_measured),
	SHELL_CMD(live, NULL, "Show live telemetry", cmd_motor_info_live),
	SHELL_CMD(stats, NULL, "Show statistics", cmd_motor_info_stats),
	SHELL_SUBCMD_SET_END
);

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
/* motor rls subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_rls,
	SHELL_CMD(status, NULL, "Show RLS status", cmd_motor_rls_status),
	SHELL_CMD(params, NULL, "Show RLS estimates", cmd_motor_rls_params),
	SHELL_CMD(temp, NULL, "Show temperature estimates", cmd_motor_rls_temp),
	SHELL_CMD(gating, NULL, "Show gating conditions", cmd_motor_rls_gating),
	SHELL_CMD(reset, NULL, "Reset RLS estimators", cmd_motor_rls_reset),
	SHELL_SUBCMD_SET_END
);
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

/* motor velocity subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_velocity,
	SHELL_CMD_ARG(target, NULL, "Set velocity target <hz>", cmd_motor_velocity_target, 2, 0),
	SHELL_CMD_ARG(gains, NULL, "Set velocity gains <kp_a_per_rad_s> <iq_limit_a>", cmd_motor_velocity_gains, 3, 0),
	SHELL_CMD(status, NULL, "Show velocity status", cmd_motor_velocity_status),
	SHELL_SUBCMD_SET_END
);

/* motor position subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_position,
	SHELL_CMD_ARG(target, NULL, "Set position target <deg>", cmd_motor_position_target, 2, 0),
	SHELL_CMD_ARG(gains, NULL, "Set position gain <kp_rad_s_per_rad>", cmd_motor_position_gains, 2, 0),
	SHELL_CMD(status, NULL, "Show position status", cmd_motor_position_status),
	SHELL_SUBCMD_SET_END
);

/* motor profile seq subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile_seq_trigger,
	SHELL_CMD_ARG(source, NULL, "Set trigger source <timer|external>",
		      cmd_motor_profile_seq_trigger_source, 2, 0),
	SHELL_CMD_ARG(edge, NULL, "Set external edge <rising|falling|both>",
		      cmd_motor_profile_seq_trigger_edge, 2, 0),
	SHELL_CMD_ARG(channel, NULL, "Set external capture channel <0..3>",
		      cmd_motor_profile_seq_trigger_channel, 2, 0),
	SHELL_CMD_ARG(min_interval_us, NULL, "Set minimum trigger spacing <us>",
		      cmd_motor_profile_seq_trigger_min_interval, 2, 0),
	SHELL_CMD(status, NULL, "Show trigger source/edge/filter status",
		  cmd_motor_profile_seq_trigger_status),
	SHELL_CMD(fire, NULL, "Inject one software external trigger", cmd_motor_profile_seq_trigger_fire),
	SHELL_SUBCMD_SET_END
);

/* motor profile seq subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile_seq,
	SHELL_CMD(clear, NULL, "Clear sequence points", cmd_motor_profile_seq_clear),
	SHELL_CMD_ARG(add, NULL, "Add sequence point <target_deg>", cmd_motor_profile_seq_add, 2, 0),
	SHELL_CMD_ARG(config, NULL,
		      "Set sequence config <period_ms> <move_ms> <end_vel_hz> <loop:0|1>",
		      cmd_motor_profile_seq_config, 5, 0),
	SHELL_CMD(trigger, &sub_motor_profile_seq_trigger, "Sequence trigger source config", NULL),
	SHELL_CMD(start, NULL, "Start sequence playback using configured trigger source",
		  cmd_motor_profile_seq_start),
	SHELL_CMD(stop, NULL, "Stop sequence playback", cmd_motor_profile_seq_stop),
	SHELL_CMD(status, NULL, "Show sequence status", cmd_motor_profile_seq_status),
	SHELL_CMD(list, NULL, "List sequence points", cmd_motor_profile_seq_list),
	SHELL_SUBCMD_SET_END
);

/* motor profile subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile,
	SHELL_CMD_ARG(set, NULL, "Set profile limits <max_hz> <max_accel_hz_s>", cmd_motor_profile_set, 3, 0),
	SHELL_CMD_ARG(move, NULL, "Plan quintic move <target_deg> <end_vel_hz> <duration_ms>", cmd_motor_profile_move, 4, 0),
	SHELL_CMD(cancel, NULL, "Cancel active motion profile", cmd_motor_profile_cancel),
	SHELL_CMD(seq, &sub_motor_profile_seq, "Sequence playback control", NULL),
	SHELL_CMD(status, NULL, "Show motion profile status", cmd_motor_profile_status),
	SHELL_SUBCMD_SET_END
);

/* motor chopper calib subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_chopper_calib,
	SHELL_CMD_ARG(start, NULL, "Capture edges <slots> <revs> <speed_hz>", cmd_motor_chopper_calib_start, 4, 0),
	SHELL_CMD(stop, NULL, "Stop active calibration capture", cmd_motor_chopper_calib_stop),
	SHELL_CMD(status, NULL, "Show calibration capture/midpoint status", cmd_motor_chopper_calib_status),
	SHELL_CMD(apply, NULL, "Apply midpoint table to profile sequence points", cmd_motor_chopper_calib_apply),
	SHELL_CMD(clear, NULL, "Clear calibration buffers and midpoint table", cmd_motor_chopper_calib_clear),
	SHELL_SUBCMD_SET_END
);

/* motor chopper subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_chopper,
	SHELL_CMD(calib, &sub_motor_chopper_calib, "Chopper midpoint calibration", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor encoder subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder,
	SHELL_CMD(alarm, NULL, "Read AEAT-9955 alarm byte (MHI/MLO)", cmd_motor_encoder_alarm),
	SHELL_SUBCMD_SET_END
);

/* motor safety subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_safety,
	SHELL_CMD_ARG(timeout, NULL, "Set command timeout <ms> (0 disables)", cmd_motor_safety_timeout, 2, 0),
	SHELL_CMD(status, NULL, "Show safety interlock/timeout status", cmd_motor_safety_status),
	SHELL_CMD(pet, NULL, "Refresh command watchdog timer", cmd_motor_safety_pet),
	SHELL_SUBCMD_SET_END
);

/* Top-level motor command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD(params, &sub_motor_params, "Parameter access", NULL),
	SHELL_CMD(current, &sub_motor_current, "Current control", NULL),
	SHELL_CMD(state, &sub_motor_state, "State machine control", NULL),
	SHELL_CMD(arm, NULL, "Arm torque-producing control output", cmd_motor_arm),
	SHELL_CMD(disarm, NULL, "Disarm control output and request IDLE", cmd_motor_disarm),
	SHELL_CMD(safety, &sub_motor_safety, "Safety interlock and timeout", NULL),
	SHELL_CMD(pi, &sub_motor_pi, "PI controller tuning", NULL),
	SHELL_CMD(info, &sub_motor_info, "Motor information", NULL),
#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
	SHELL_CMD(rls, &sub_motor_rls, "RLS parameter estimation", NULL),
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */
	SHELL_CMD(velocity, &sub_motor_velocity, "Velocity control", NULL),
	SHELL_CMD(position, &sub_motor_position, "Position control", NULL),
	SHELL_CMD(profile, &sub_motor_profile, "Motion profile settings", NULL),
	SHELL_CMD(chopper, &sub_motor_chopper, "Optical chopper utilities", NULL),
	SHELL_CMD(encoder, &sub_motor_encoder, "Encoder diagnostics", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &sub_motor, "Motor control commands", NULL);
