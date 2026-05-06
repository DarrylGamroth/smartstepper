#include "shell_control_common.h"

/* Domain implementation split from shell_control.c. */

int cmd_motor_current_id(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current id <amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float id_amps = 0.0f;
	if (!shell_parse_finite_float(argv[1], &id_amps)) {
		shell_error(sh, "id current must be a finite number");
		return -EINVAL;
	}
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
int cmd_motor_current_iq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current iq <amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float iq_amps = 0.0f;
	if (!shell_parse_finite_float(argv[1], &iq_amps)) {
		shell_error(sh, "iq current must be a finite number");
		return -EINVAL;
	}
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
int cmd_motor_current_dq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor current dq <id_amps> <iq_amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float id_amps = 0.0f;
	float iq_amps = 0.0f;
	if (!shell_parse_finite_float(argv[1], &id_amps) ||
	    !shell_parse_finite_float(argv[2], &iq_amps)) {
		shell_error(sh, "id/iq currents must be finite numbers");
		return -EINVAL;
	}
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

/* motor current gain get <id|iq> */
int cmd_motor_current_gain_get(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current gain get <id|iq>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const char *controller = argv[1];
	float kp = 0.0f;
	float ki = 0.0f;

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

	shell_print(sh, "Current PI %s: Kp=%.6f, Ki=%.6f",
		    controller, (double)kp, (double)ki);
	return 0;
}

/* motor current gain set <id|iq> <kp> <ki> */
int cmd_motor_current_gain_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "Usage: motor current gain set <id|iq> <kp> <ki>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing current PI gains.");
		return -EACCES;
	}

	const char *controller = argv[1];
	float kp = 0.0f;
	float ki = 0.0f;
	if (!shell_parse_finite_float(argv[2], &kp) ||
	    !shell_parse_finite_float(argv[3], &ki)) {
		shell_error(sh, "kp/ki must be finite numbers");
		return -EINVAL;
	}
	if (kp <= 0.0f || ki < 0.0f) {
		shell_error(sh, "kp must be > 0 and ki must be >= 0");
		return -EINVAL;
	}

	if (strcmp(controller, "id") == 0) {
		pi_set_gains(&g_motor_params->pi_Id, kp, ki);
	} else if (strcmp(controller, "iq") == 0) {
		pi_set_gains(&g_motor_params->pi_Iq, kp, ki);
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Current PI %s gains set: Kp=%.6f, Ki=%.6f",
		    controller, (double)kp, (double)ki);
	return 0;
}

/* motor current gain bandwidth <id|iq> <hz> */
int cmd_motor_current_gain_bandwidth(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor current gain bandwidth <id|iq> <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before tuning current PI bandwidth.");
		return -EACCES;
	}

	const char *controller = argv[1];
	float bw_hz = 0.0f;
	if (!shell_parse_finite_float(argv[2], &bw_hz)) {
		shell_error(sh, "Bandwidth must be a finite number.");
		return -EINVAL;
	}
	if (bw_hz <= 0.0f || bw_hz > CONTROL_LOOP_FREQUENCY_HZ / 2.0f) {
		shell_error(sh, "Bandwidth must be positive and below Nyquist (%.1f Hz)",
			    (double)(CONTROL_LOOP_FREQUENCY_HZ / 2.0f));
		return -EINVAL;
	}

	float32_t bw_rps = 2.0f * PI_F32 * bw_hz;
	float32_t t_sample = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t kp = 0.0f;
	float32_t ki = 0.0f;

	if (strcmp(controller, "id") == 0) {
		if (MOTOR_INDUCTANCE_D_H <= 0.0f) {
			shell_error(sh, "Invalid D-axis inductance in configuration");
			return -ERANGE;
		}
		kp = MOTOR_INDUCTANCE_D_H * bw_rps;
		ki = (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_D_H) * t_sample;
		pi_set_gains(&g_motor_params->pi_Id, kp, ki);
	} else if (strcmp(controller, "iq") == 0) {
		if (MOTOR_INDUCTANCE_Q_H <= 0.0f) {
			shell_error(sh, "Invalid Q-axis inductance in configuration");
			return -ERANGE;
		}
		kp = MOTOR_INDUCTANCE_Q_H * bw_rps;
		ki = (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_Q_H) * t_sample;
		pi_set_gains(&g_motor_params->pi_Iq, kp, ki);
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Current PI %s tuned to %.1f Hz: Kp=%.6f, Ki=%.6f",
		    controller, (double)bw_hz, (double)kp, (double)ki);
	return 0;
}

/* motor velocity target <hz> */

