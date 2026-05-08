#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

static inline double rad_s_to_hz(float32_t rad_s)
{
	return (double)(rad_s / (2.0f * PI_F32));
}

/* motor fault snapshot start [decimation] */
int cmd_motor_fault_snapshot_start(const struct shell *sh, size_t argc, char **argv)
{
#if !IS_ENABLED(CONFIG_MOTOR_ISR_FAULT_SNAPSHOT)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "Fault snapshot telemetry is not compiled in");
	return -ENOTSUP;
#else
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor fault snapshot start [decimation]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t decimation = 1U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &decimation) || decimation == 0U ||
		    decimation > UINT16_MAX) {
			shell_error(sh, "decimation must be in [1, %u]", UINT16_MAX);
			return -EINVAL;
		}
	}

	motor_fault_snapshot_reset(g_motor_params, false);
	g_motor_params->fault_snapshot.decimation = (uint16_t)decimation;
	g_motor_params->fault_snapshot.enabled = true;
	shell_print(sh,
		    "Fault snapshot started: decimation=%u, capacity=%u samples",
		    g_motor_params->fault_snapshot.decimation,
		    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	return 0;
#endif
}

/* motor fault snapshot stop */
int cmd_motor_fault_snapshot_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->fault_snapshot.enabled = false;
	shell_print(sh, "Fault snapshot stopped: stored=%u overrun=%u",
		    g_motor_params->fault_snapshot.count,
		    g_motor_params->fault_snapshot.overrun_count);
	return 0;
}

/* motor fault snapshot status */
int cmd_motor_fault_snapshot_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Fault snapshot:");
	shell_print(sh, "  Compiled:   %s",
		    IS_ENABLED(CONFIG_MOTOR_ISR_FAULT_SNAPSHOT) ? "YES" : "NO");
	shell_print(sh, "  Enabled:    %s", g_motor_params->fault_snapshot.enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->fault_snapshot.decimation);
	shell_print(sh, "  Latched:    %s", g_motor_params->fault_snapshot.latched ? "YES" : "NO");
	if (g_motor_params->fault_snapshot.latched) {
		shell_print(sh, "  Fault:      %s (%u)",
			    motor_error_to_string((int)g_motor_params->fault_snapshot.latch_error_code),
			    g_motor_params->fault_snapshot.latch_error_code);
		if (g_motor_params->fault_snapshot.latch_error_code == ERROR_ENCODER_FAULT) {
			shell_print(sh, "  Enc reason: %s (%u)",
				    motor_encoder_fault_reason_to_string(
					    g_motor_params->fault_snapshot.latch_encoder_fault_reason),
				    g_motor_params->fault_snapshot.latch_encoder_fault_reason);
		}
		shell_print(sh, "  Fault loop: %u", g_motor_params->fault_snapshot.latch_loop);
	}
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->fault_snapshot.count,
		    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->fault_snapshot.overrun_count);

	if (g_motor_params->fault_snapshot.count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->fault_snapshot.write_idx +
						  MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES - 1U) %
						 MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
		const struct motor_fault_snapshot_sample *newest =
			&g_motor_params->fault_snapshot.samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u reason=%s src=%s enc=%.3fdeg v_tgt=%.3fHz v_ref=%.3fHz v_meas=%.3fHz iq_ref=%.3fA iq=%.3fA ia=%.3fA ib=%.3fA fresh=%u warn=%u err=%u status=0x%02X",
			    newest->control_loop_count,
			    motor_encoder_fault_reason_to_string(newest->encoder_fault_reason),
			    motor_encoder_input_source_to_string(newest->input_source),
			    (double)newest->encoder_angle_deg,
			    rad_s_to_hz(newest->velocity_target_rad_s),
			    rad_s_to_hz(newest->velocity_ref_rad_s),
			    rad_s_to_hz(newest->velocity_mech_rad_s),
			    (double)newest->Iq_ref_A,
			    (double)newest->Iq_A,
			    (double)newest->Ia_A,
			    (double)newest->Ib_A,
			    newest->sample_fresh,
			    newest->sample_warning,
			    newest->sample_error,
			    newest->status);
	}

	return 0;
}

/* motor fault snapshot dump [count] */
int cmd_motor_fault_snapshot_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor fault snapshot dump [count]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t requested = 32U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &requested) || requested == 0U ||
		    requested > MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES) {
			shell_error(sh, "count must be in [1, %u]",
				    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
			return -EINVAL;
		}
	}

	uint16_t stored = g_motor_params->fault_snapshot.count;
	if (stored == 0U) {
		shell_print(sh, "No fault snapshot samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->fault_snapshot.write_idx +
				     MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES - count) %
				    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);

	if (motor_control_is_armed(g_motor_params)) {
		shell_warn(sh,
			   "Control is armed; snapshot dump may include concurrently updated samples.");
	}

	shell_print(sh,
		    "idx loop reason src fresh warn err status pqual enc_deg obs_in elec obs_we v_tgt_hz v_ref_hz v_meas_hz v_filt_hz id_ref iq_ref id iq ia ib vd vq");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
		const struct motor_fault_snapshot_sample *sample =
			&g_motor_params->fault_snapshot.samples[idx];
		shell_print(sh,
			    "%u %u %s %s %u %u %u 0x%02X 0x%02X %.3f %.6f %.6f %.6f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
			    i,
			    sample->control_loop_count,
			    motor_encoder_fault_reason_to_string(sample->encoder_fault_reason),
			    motor_encoder_input_source_to_string(sample->input_source),
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    sample->status,
			    sample->position_quality_flags,
			    (double)sample->encoder_angle_deg,
			    (double)sample->observer_input_rad,
			    (double)sample->elec_angle_rad,
			    (double)sample->observer_elec_speed_rad_s,
			    rad_s_to_hz(sample->velocity_target_rad_s),
			    rad_s_to_hz(sample->velocity_ref_rad_s),
			    rad_s_to_hz(sample->velocity_mech_rad_s),
			    rad_s_to_hz(sample->velocity_filtered_rad_s),
			    (double)sample->Id_ref_A,
			    (double)sample->Iq_ref_A,
			    (double)sample->Id_A,
			    (double)sample->Iq_A,
			    (double)sample->Ia_A,
			    (double)sample->Ib_A,
			    (double)sample->Vd_V,
			    (double)sample->Vq_V);
	}

	return 0;
}

/* motor fault snapshot clear */
int cmd_motor_fault_snapshot_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->fault_snapshot.enabled = false;
	motor_fault_snapshot_reset(g_motor_params, true);
	shell_print(sh, "Fault snapshot cleared");
	return 0;
}
