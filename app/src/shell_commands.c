/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <math.h>
#include "shell_commands.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "config.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(shell_commands, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set by main) */
static struct motor_parameters *g_motor_params = NULL;

/**
 * @brief Set global motor parameters pointer
 * Called from main() to provide access to motor_parameters structure
 */
void shell_set_motor_params(struct motor_parameters *params)
{
	g_motor_params = params;
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

	float id_amps = strtof(argv[1], NULL);
	
	if (motor_api_set_param("Id_setpoint_A", id_amps) == 0) {
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

	float iq_amps = strtof(argv[1], NULL);
	
	if (motor_api_set_param("Iq_setpoint_A", iq_amps) == 0) {
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

	float id_amps = strtof(argv[1], NULL);
	float iq_amps = strtof(argv[2], NULL);
	
	if (motor_api_set_currents(id_amps, iq_amps) == 0) {
		shell_print(sh, "Id = %.3f A, Iq = %.3f A", (double)id_amps, (double)iq_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set DQ currents");
		return -EIO;
	}
}

/* motor state start */
static int cmd_motor_state_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	motor_api_request_start();
	shell_print(sh, "Motor start requested");
	return 0;
}

/* motor state stop */
static int cmd_motor_state_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	motor_api_request_stop();
	shell_print(sh, "Motor stop requested");
	return 0;
}

/* motor state status */
static int cmd_motor_state_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	int state = motor_api_get_state();
	const char *state_str = motor_state_to_string(state);
	int error = motor_api_get_error();
	const char *error_str = motor_error_to_string(error);
	
	shell_print(sh, "Motor Status:");
	shell_print(sh, "  State: %s (%d)", state_str, state);
	shell_print(sh, "  Error: %s (%d)", error_str, error);
	
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
		ki = 0.25f * (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_D_H) * T_sample;
		g_motor_params->pi_Id.kp = kp;
		g_motor_params->pi_Id.ki = ki;
		shell_print(sh, "D-axis PI tuned to %.1f Hz bandwidth:", (double)bw_hz);
		shell_print(sh, "  Kp = %.6f", (double)kp);
		shell_print(sh, "  Ki = %.6f", (double)ki);
	} else if (strcmp(controller, "iq") == 0) {
		kp = MOTOR_INDUCTANCE_Q_H * bw_rps;
		ki = 0.25f * (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_Q_H) * T_sample;
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

/* motor info config */
static int cmd_motor_info_config(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	shell_print(sh, "Motor Configuration:");
	shell_print(sh, "  Pole pairs:     %d", MOTOR_POLE_PAIRS);
	shell_print(sh, "  Max current:    %.3f A", (double)MOTOR_MAX_CURRENT_A);
	shell_print(sh, "  Rated voltage:  %.3f V", (double)NOMINAL_VOLTAGE_V);
	shell_print(sh, "  Control freq:   %u Hz", (uint32_t)CONTROL_LOOP_FREQUENCY_HZ);
	shell_print(sh, "  PWM freq:       %u Hz", (uint32_t)PWM_FREQUENCY_HZ);
	shell_print(sh, "  Observer BW:    %.1f Hz", (double)ANGLE_OBSERVER_BANDWIDTH_HZ);
	shell_print(sh, "  PI Id BW:       %.1f Hz", (double)CURRENT_LOOP_BANDWIDTH_HZ);
	shell_print(sh, "  PI Iq BW:       %.1f Hz", (double)CURRENT_LOOP_BANDWIDTH_HZ);
	shell_print(sh, "  Overcurrent:    %.3f A", (double)OVERCURRENT_THRESHOLD_A);
	shell_print(sh, "  Overvoltage:    %.1f V", (double)VBUS_MAX_V);
	
	return 0;
}

/* motor info measured */
static int cmd_motor_info_measured(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	shell_print(sh, "Measured Parameters:");
	shell_print(sh, "  Rs:             %.6f Ohm", (double)g_motor_params->Rs_measured_ohm);
	shell_print(sh, "  L:              %.9f H", (double)g_motor_params->Ls_measured_H);
	shell_print(sh, "  R/L:            %.3f rad/s", (double)g_motor_params->R_over_L_measured);
	shell_print(sh, "  Ia offset:      %.6f A", (double)g_motor_params->Ia_offset);
	shell_print(sh, "  Ib offset:      %.6f A", (double)g_motor_params->Ib_offset);
	
	return 0;
}

/* motor info live */
static int cmd_motor_info_live(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	int state = motor_api_get_state();
	int error = motor_api_get_error();
	
	shell_print(sh, "Live Telemetry:");
	shell_print(sh, "  State:          %s", motor_state_to_string(state));
	shell_print(sh, "  Error:          %s", motor_error_to_string(error));
	shell_print(sh, "  Angle (mech):   %.1f deg", (double)(g_motor_params->position_rad * 180.0f / PI_F32));
	shell_print(sh, "  Angle (elec):   %.1f deg", (double)(g_motor_params->elec_angle_rad * 180.0f / PI_F32));
	shell_print(sh, "  Speed:          %.3f Hz (%.1f RPM)", 
		    (double)(g_motor_params->velocity_rad_s / (2.0f * PI_F32)),
		    (double)(g_motor_params->velocity_rad_s / (2.0f * PI_F32) * 60.0f));
	shell_print(sh, "  Id reference:   %.3f A", (double)g_motor_params->Id_ref_A);
	shell_print(sh, "  Iq reference:   %.3f A", (double)g_motor_params->Iq_ref_A);
	shell_print(sh, "  Id measured:    %.3f A", (double)g_motor_params->Id_A);
	shell_print(sh, "  Iq measured:    %.3f A", (double)g_motor_params->Iq_A);
	shell_print(sh, "  Ia:             %.3f A", (double)g_motor_params->Ia_A);
	shell_print(sh, "  Ib:             %.3f A", (double)g_motor_params->Ib_A);
	shell_print(sh, "  Vd:             %.3f V", (double)g_motor_params->Vd_V);
	shell_print(sh, "  Vq:             %.3f V", (double)g_motor_params->Vq_V);
	shell_print(sh, "  Vbus:           %.1f V", (double)g_motor_params->dc_bus_voltage_V);
	shell_print(sh, "  Encoder OK:     %s", g_motor_params->encoder_fault_counter == 0 ? "yes" : "no");
	shell_print(sh, "  Encoder faults: %u", g_motor_params->encoder_fault_counter);
	
	return 0;
}

/* motor info stats */
static int cmd_motor_info_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	uint32_t avg_cycles = 0;
	if (g_motor_params->control_loop_count > 0) {
		avg_cycles = g_motor_params->total_isr_cycles / g_motor_params->control_loop_count;
	}
	
	shell_print(sh, "Performance Statistics:");
	shell_print(sh, "  ISR count:              %u", g_motor_params->control_loop_count);
	shell_print(sh, "  ISR max cycles:         %u", g_motor_params->max_isr_cycles);
	shell_print(sh, "  ISR avg cycles:         %u", avg_cycles);
	shell_print(sh, "  Buffer swaps:           %u", g_motor_params->buffer_swap_count);
	shell_print(sh, "  Encoder faults:         %u", g_motor_params->encoder_fault_counter);
	
	return 0;
}

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

/* motor state subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state,
	SHELL_CMD(start, NULL, "Start motor", cmd_motor_state_start),
	SHELL_CMD(stop, NULL, "Stop motor", cmd_motor_state_stop),
	SHELL_CMD(status, NULL, "Show motor status", cmd_motor_state_status),
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

/* Top-level motor command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD(params, &sub_motor_params, "Parameter access", NULL),
	SHELL_CMD(current, &sub_motor_current, "Current control", NULL),
	SHELL_CMD(state, &sub_motor_state, "State machine control", NULL),
	SHELL_CMD(pi, &sub_motor_pi, "PI controller tuning", NULL),
	SHELL_CMD(info, &sub_motor_info, "Motor information", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &sub_motor, "Motor control commands", NULL);
