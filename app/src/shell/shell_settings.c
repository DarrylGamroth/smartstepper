/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/shell/shell.h>

#include "config.h"
#include "motor_settings.h"
#include "motor_control_api.h"
#include "motor_state_utils.h"
#include "shell_commands_motion.h"
#include "shell_control.h"

#define MOTOR_SETTINGS_GROUP_USAGE \
	"[baseline|encoder|identity|model|limits|controllers|detent|all]"

static int parse_group(const char *arg, uint32_t *groups)
{
	if (arg == NULL || groups == NULL) {
		return -EINVAL;
	}
	if (strcmp(arg, "baseline") == 0 || strcmp(arg, "encoder") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_ENCODER;
		return 0;
	}
	if (strcmp(arg, "identity") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_IDENTITY;
		return 0;
	}
	if (strcmp(arg, "model") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_MODEL;
		return 0;
	}
	if (strcmp(arg, "limits") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_LIMITS;
		return 0;
	}
	if (strcmp(arg, "controllers") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_CONTROLLERS;
		return 0;
	}
	if (strcmp(arg, "detent") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_DETENT;
		return 0;
	}
	if (strcmp(arg, "all") == 0) {
		*groups = MOTOR_SETTINGS_GROUP_ALL;
		return 0;
	}
	return -EINVAL;
}

static const char *yesno(bool value)
{
	return value ? "YES" : "NO";
}

static void print_groups(const struct shell *sh, const char *label, uint32_t groups)
{
	shell_print(sh,
		    "%s: encoder=%s identity=%s model=%s limits=%s controllers=%s detent=%s",
		    label,
		    yesno((groups & MOTOR_SETTINGS_GROUP_ENCODER) != 0U),
		    yesno((groups & MOTOR_SETTINGS_GROUP_IDENTITY) != 0U),
		    yesno((groups & MOTOR_SETTINGS_GROUP_MODEL) != 0U),
		    yesno((groups & MOTOR_SETTINGS_GROUP_LIMITS) != 0U),
		    yesno((groups & MOTOR_SETTINGS_GROUP_CONTROLLERS) != 0U),
		    yesno((groups & MOTOR_SETTINGS_GROUP_DETENT) != 0U));
}

static bool mutation_safe(const struct motor_parameters *params)
{
	return params != NULL && !motor_control_is_armed(params) &&
	       !motor_state_ptr_is_online_control_state(params->state_for_isr) &&
	       !params->calibration.running;
}

static int require_mutation_safe(const struct shell *sh)
{
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!mutation_safe(g_motor_params)) {
		shell_error(sh, "Settings writes/load/clear require disarmed, non-online, non-calibrating state");
		return -EBUSY;
	}
	return 0;
}

static void print_snapshot(const struct shell *sh,
			   const struct motor_settings_snapshot *s,
			   uint32_t present_groups)
{
	shell_print(sh, "Motor Settings Preview:");
	shell_print(sh, "  Root:          %s", motor_settings_key_root());
	shell_print(sh, "  Schema:        %u%s", s->schema_version,
		    s->schema_version == MOTOR_SETTINGS_SCHEMA_VERSION ? "" : " (unsupported)");
	shell_print(sh, "  Generation:    %u", s->generation);
	print_groups(sh, "  Present groups", present_groups);
	print_groups(sh, "  Valid groups  ", s->valid_groups);

	if ((present_groups & MOTOR_SETTINGS_GROUP_ENCODER) != 0U) {
		shell_print(sh, "  Encoder:");
		shell_print(sh, "    direction_sign:        %d", s->encoder_direction_sign);
		shell_print(sh, "    commutation_offset:    %.6f rad (%.3f deg mech)",
			    (double)s->encoder_commutation_offset_mech_rad,
			    (double)(s->encoder_commutation_offset_mech_rad * 180.0f / PI_F32));
		shell_print(sh, "    trim:                  %.6f rad (%.3f deg elec)",
			    (double)s->encoder_trim_elec_rad,
			    (double)(s->encoder_trim_elec_rad * 180.0f / PI_F32));
		shell_print(sh, "    correlation/residual:  %.5f / %.6f rad",
			    (double)s->encoder_mapping_correlation,
			    (double)s->encoder_mapping_residual_rad);
	}
	if ((present_groups & MOTOR_SETTINGS_GROUP_IDENTITY) != 0U) {
		shell_print(sh, "  Identity:");
		shell_print(sh, "    pole_pairs:            %u", s->identity_pole_pairs);
		shell_print(sh, "    Note: identity mismatches are rejected until all hot paths are runtime-configured.");
	}
	if ((present_groups & MOTOR_SETTINGS_GROUP_MODEL) != 0U) {
		shell_print(sh, "  Model:");
		shell_print(sh, "    Rs=%.6f ohm Ld=%.9f H Lq=%.9f H",
			    (double)s->model_rs_ohm,
			    (double)s->model_ld_h,
			    (double)s->model_lq_h);
		shell_print(sh, "    psi_f=%.8f Wb Kt=%.8f Nm/A",
			    (double)s->model_flux_linkage_wb,
			    (double)s->model_kt_nm_per_a);
		shell_print(sh, "    J=%.9f kgm2 B=%.9f Nm/(rad/s) Tc=%.9f Nm",
			    (double)s->model_inertia_kgm2,
			    (double)s->model_viscous_friction_nm_per_rad_s,
			    (double)s->model_coulomb_friction_nm);
	}
	if ((present_groups & MOTOR_SETTINGS_GROUP_LIMITS) != 0U) {
		shell_print(sh, "  Limits:");
		shell_print(sh, "    nominal_voltage:       %.3f V",
			    (double)s->limits_nominal_voltage_v);
		shell_print(sh, "    max_current/brake:     %.6f A / %.6f A",
			    (double)s->limits_max_current_a,
			    (double)s->limits_brake_current_a);
		shell_print(sh, "    profile vel/accel:     %.6f Hz / %.6f Hz/s",
			    (double)s->limits_max_velocity_hz,
			    (double)s->limits_max_accel_hz_s);
		shell_print(sh, "    command_timeout:       %u ms",
			    s->limits_command_timeout_ms);
		shell_print(sh, "    Note: profile limits and timeout apply now; electrical safety limits must match this firmware image.");
	}
	if ((present_groups & MOTOR_SETTINGS_GROUP_CONTROLLERS) != 0U) {
		shell_print(sh, "  Controllers:");
		shell_print(sh, "    outer_loop:          %s",
			    s->ctrl_outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ? "MPR" : "PI");
		shell_print(sh, "    velocity bandwidth:  %.6f Hz",
			    (double)s->ctrl_velocity_bandwidth_hz);
		shell_print(sh, "    position bandwidth:  %.6f Hz",
			    (double)s->ctrl_position_bandwidth_hz);
		shell_print(sh, "    damping ratio:       %.6f",
			    (double)s->ctrl_damping_ratio);
		shell_print(sh, "    velocity iq_limit:   %.6f A",
			    (double)s->ctrl_velocity_iq_limit_a);
		shell_print(sh, "    DOB intent:          enabled=%s gain_scale=%.6f",
			    s->ctrl_velocity_dob_enabled ? "yes" : "no",
			    (double)s->ctrl_velocity_dob_gain_scale);
		shell_print(sh, "    Note: PI/MPR/DOB coefficients are recomputed from these values on load.");
	}
	if ((present_groups & MOTOR_SETTINGS_GROUP_DETENT) != 0U) {
		shell_print(sh, "  Detent metadata:");
		shell_print(sh, "    enabled=%s bins=%u phase=%d gain=%.6f iq_limit=%.6f crc=0x%08X",
			    s->detent_enabled ? "yes" : "no",
			    s->detent_bins,
			    s->detent_phase_advance_bins,
			    (double)s->detent_gain,
			    (double)s->detent_iq_ff_limit_a,
			    s->detent_table_crc32);
	}
	shell_print(sh, "  Note: ADC current offsets are intentionally boot-calibrated and not persisted.");
}

int cmd_motor_settings_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;
	int ret = motor_settings_read(&snap, &present);
	if (ret != 0) {
		shell_error(sh, "Failed to read settings (err %d)", ret);
		return ret;
	}

	shell_print(sh, "Motor Settings Status:");
	shell_print(sh, "  Backend:       Zephyr Settings/ZMS");
	shell_print(sh, "  Root:          %s", motor_settings_key_root());
	shell_print(sh, "  Autoload:      %s", motor_settings_autoload_enabled() ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Schema:        %u", snap.schema_version);
	shell_print(sh, "  Generation:    %u", snap.generation);
	print_groups(sh, "  Present groups", present);
	print_groups(sh, "  Valid groups  ", snap.valid_groups);
	shell_print(sh, "  Mutation safe: %s", yesno(mutation_safe(g_motor_params)));
	return 0;
}

int cmd_motor_settings_preview(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;
	int ret = motor_settings_read(&snap, &present);
	if (ret != 0) {
		shell_error(sh, "Failed to read settings (err %d)", ret);
		return ret;
	}
	if (present == 0U && snap.schema_version == 0U) {
		shell_print(sh, "No motor settings found; devicetree defaults remain active.");
		return 0;
	}
	print_snapshot(sh, &snap, present);
	return 0;
}

int cmd_motor_settings_save(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t groups = MOTOR_SETTINGS_GROUP_ALL;
	if (argc > 2) {
		shell_error(sh, "Usage: motor settings save " MOTOR_SETTINGS_GROUP_USAGE);
		return -EINVAL;
	}
	if (argc == 2 && parse_group(argv[1], &groups) != 0) {
		shell_error(sh, "Unknown settings group '%s'", argv[1]);
		return -EINVAL;
	}
	int ret = require_mutation_safe(sh);
	if (ret != 0) {
		return ret;
	}

	uint32_t saved = 0U;
	ret = motor_settings_save(g_motor_params, groups, &saved);
	if (ret != 0) {
		shell_error(sh, "Failed to save settings (err %d)", ret);
		return ret;
	}
	print_groups(sh, "Saved groups", saved);
	return 0;
}

int cmd_motor_settings_load(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t groups = MOTOR_SETTINGS_GROUP_ALL;
	if (argc > 2) {
		shell_error(sh, "Usage: motor settings load " MOTOR_SETTINGS_GROUP_USAGE);
		return -EINVAL;
	}
	if (argc == 2 && parse_group(argv[1], &groups) != 0) {
		shell_error(sh, "Unknown settings group '%s'", argv[1]);
		return -EINVAL;
	}
	int ret = require_mutation_safe(sh);
	if (ret != 0) {
		return ret;
	}

	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;
	ret = motor_settings_read(&snap, &present);
	if (ret != 0) {
		shell_error(sh, "Failed to read settings (err %d)", ret);
		return ret;
	}
	print_snapshot(sh, &snap, present);

	uint32_t loaded = 0U;
	ret = motor_settings_load(g_motor_params, groups, &loaded);
	if (ret != 0) {
		shell_error(sh, "Failed to load selected settings (err %d)", ret);
		return ret;
	}
	print_groups(sh, "Loaded groups", loaded);
	return 0;
}

int cmd_motor_settings_clear(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t groups = MOTOR_SETTINGS_GROUP_ALL;
	if (argc > 2) {
		shell_error(sh, "Usage: motor settings clear " MOTOR_SETTINGS_GROUP_USAGE);
		return -EINVAL;
	}
	if (argc == 2 && parse_group(argv[1], &groups) != 0) {
		shell_error(sh, "Unknown settings group '%s'", argv[1]);
		return -EINVAL;
	}
	int ret = require_mutation_safe(sh);
	if (ret != 0) {
		return ret;
	}

	ret = (groups == MOTOR_SETTINGS_GROUP_ALL) ? motor_settings_clear_all() :
					       motor_settings_clear(groups);
	if (ret != 0) {
		shell_error(sh, "Failed to clear settings (err %d)", ret);
		return ret;
	}
	print_groups(sh, "Cleared groups", groups);
	return 0;
}

int cmd_motor_settings_autoload_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Motor Settings Autoload:");
	shell_print(sh, "  State: DISABLED");
	shell_print(sh, "  Reason: explicit load only until HIL reboot/reset gates pass");
	return 0;
}
