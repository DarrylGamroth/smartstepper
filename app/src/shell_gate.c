#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor gate status */
int cmd_motor_gate_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	bool fault_a = false;
	bool fault_b = false;
	int ret = motor_hardware_get_gate_driver_faults(&fault_a, &fault_b);
	if (ret < 0) {
		shell_error(sh, "Failed to read gate-driver fault status: %d", ret);
		return ret;
	}

	shell_print(sh, "Gate Driver Status:");
	shell_print(sh, "  A cached fault: %s", fault_a ? "SET" : "clear");
	shell_print(sh, "  B cached fault: %s", fault_b ? "SET" : "clear");
	shell_print(sh, "  Recovery:       motor gate reset");
	return 0;
}

/* motor gate reset */
int cmd_motor_gate_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = motor_hardware_reset_gate_driver_faults_masked(g_motor_params);
	if (ret < 0) {
		shell_error(sh, "Gate-driver fault reset failed: %d", ret);
		return ret;
	}

	if (g_motor_params) {
		g_motor_params->recovery_status.gate_reset_done = true;
		g_motor_params->recovery_status.safe_idle_ready =
			(!g_motor_params->recovery_status.gate_reset_required ||
			 g_motor_params->recovery_status.gate_reset_done) &&
			(!g_motor_params->recovery_status.encoder_recovery_required ||
			 g_motor_params->recovery_status.encoder_recovery_done);
	}
	shell_print(sh, "Gate-driver fault reset pulse sent");
	return 0;
}


