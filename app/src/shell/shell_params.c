#include "shell_control_common.h"

/* Domain implementation split from shell_control.c. */

int cmd_motor_params_get(const struct shell *sh, size_t argc, char **argv)
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
int cmd_motor_params_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor params set <name> <value>");
		return -EINVAL;
	}

	const char *name = argv[1];
	float value = 0.0f;
	if (!shell_parse_finite_float(argv[2], &value)) {
		shell_error(sh, "value must be a finite number");
		return -EINVAL;
	}
	
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
int cmd_motor_params_list(const struct shell *sh, size_t argc, char **argv)
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

