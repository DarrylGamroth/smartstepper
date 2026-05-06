#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor encoder trace start [decimation] */
int cmd_motor_encoder_trace_start(const struct shell *sh, size_t argc, char **argv)
{
#if !IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_RAW_TRACE)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "Encoder raw trace telemetry is not compiled in");
	return -ENOTSUP;
#else
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder trace start [decimation]");
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

	g_motor_params->encoder_raw_trace.decimation = (uint16_t)decimation;
	motor_encoder_raw_trace_reset(g_motor_params, false);
	g_motor_params->encoder_raw_trace.enabled = true;

	shell_print(sh, "Encoder raw trace started: decimation=%u, capacity=%u samples",
		    g_motor_params->encoder_raw_trace.decimation,
		    MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	return 0;
#endif
}

/* motor encoder trace stop */
int cmd_motor_encoder_trace_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_raw_trace.enabled = false;
	shell_print(sh, "Encoder raw trace stopped: stored=%u overrun=%u",
		    g_motor_params->encoder_raw_trace.count,
		    g_motor_params->encoder_raw_trace.overrun_count);
	return 0;
}

/* motor encoder trace clear */
int cmd_motor_encoder_trace_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_raw_trace.enabled = false;
	motor_encoder_raw_trace_reset(g_motor_params, true);
	shell_print(sh, "Encoder raw trace cleared");
	return 0;
}

/* motor encoder trace status */
int cmd_motor_encoder_trace_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Encoder raw trace:");
	shell_print(sh, "  Compiled:   %s",
		    IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_RAW_TRACE) ? "YES" : "NO");
	shell_print(sh, "  Enabled:    %s", g_motor_params->encoder_raw_trace.enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->encoder_raw_trace.decimation);
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->encoder_raw_trace.count,
		    MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->encoder_raw_trace.overrun_count);

	if (g_motor_params->encoder_raw_trace.count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->encoder_raw_trace.write_idx +
						  MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES - 1U) %
						 MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
		const struct motor_encoder_raw_trace_sample *newest =
			&g_motor_params->encoder_raw_trace.samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u src=%s raw_deg=%.3f ctrl_deg=%.3f gen_mech=%.3fdeg gen_elec=%.3fdeg q=0x%02X fresh=%u warn=%u err=%u io=%u status=0x%02X",
			    newest->control_loop_count,
			    motor_encoder_input_source_to_string(newest->input_source),
			    (double)newest->raw_angle_deg,
			    (double)newest->control_angle_deg,
			    motor_shell_rad_to_deg(newest->generated_mech_rad),
			    motor_shell_rad_to_deg(newest->generated_elec_rad),
			    newest->quality_flags,
			    newest->sample_fresh,
			    newest->sample_warning,
			    newest->sample_error,
			    newest->sample_io_fault,
			    newest->status);
	}

	return 0;
}

/* motor encoder trace summary */
int cmd_motor_encoder_trace_summary(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_raw_trace.count;
	if (stored == 0U) {
		shell_print(sh, "No raw trace samples");
		return 0;
	}

	uint16_t oldest_idx = motor_encoder_ring_oldest(g_motor_params->encoder_raw_trace.write_idx,
							stored,
							MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	const struct motor_encoder_raw_trace_sample *first = NULL;
	const struct motor_encoder_raw_trace_sample *prev = NULL;
	const struct motor_encoder_raw_trace_sample *last = NULL;
	double raw_delta_deg = 0.0;
	double ctrl_delta_deg = 0.0;
	double min_raw_deg = 0.0;
	double max_raw_deg = 0.0;
	uint32_t clean_count = 0U;
	uint32_t fresh_count = 0U;
	uint32_t warn_count = 0U;
	uint32_t err_count = 0U;
	uint32_t io_count = 0U;
	uint32_t enabled_count = 0U;
	uint32_t delta_drop_count = 0U;
	uint8_t status_or = 0U;
	uint8_t status_and = 0xFFU;

	for (uint16_t i = 0U; i < stored; i++) {
		uint16_t idx = motor_encoder_ring_index(oldest_idx, i,
							MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
		const struct motor_encoder_raw_trace_sample *sample =
			&g_motor_params->encoder_raw_trace.samples[idx];

		fresh_count += sample->sample_fresh ? 1U : 0U;
		warn_count += sample->sample_warning ? 1U : 0U;
		err_count += sample->sample_error ? 1U : 0U;
		io_count += sample->sample_io_fault ? 1U : 0U;
		enabled_count += sample->sample_enabled ? 1U : 0U;
		status_or |= sample->status;
		status_and &= sample->status;

		if (!motor_encoder_raw_trace_sample_clean(sample)) {
			continue;
		}

		if (first == NULL) {
			first = sample;
			min_raw_deg = (double)motor_encoder_trace_deg(sample->raw_angle_rad);
			max_raw_deg = min_raw_deg;
		} else {
			double sample_raw_deg =
				(double)motor_encoder_trace_deg(sample->raw_angle_rad);
			double prev_raw_deg =
				(double)motor_encoder_trace_deg(prev->raw_angle_rad);
			double sample_ctrl_deg =
				(double)motor_encoder_trace_deg(sample->control_angle_rad);
			double prev_ctrl_deg =
				(double)motor_encoder_trace_deg(prev->control_angle_rad);
			double raw_step_deg = motor_encoder_wrap_delta_deg(sample_raw_deg -
									   prev_raw_deg);
			double ctrl_step_deg = motor_encoder_wrap_delta_deg(sample_ctrl_deg -
									    prev_ctrl_deg);

			if (!isfinite(raw_step_deg)) {
				delta_drop_count++;
			} else {
				raw_delta_deg += raw_step_deg;
			}
			if (!isfinite(ctrl_step_deg)) {
				delta_drop_count++;
			} else {
				ctrl_delta_deg += ctrl_step_deg;
			}
			if (sample_raw_deg < min_raw_deg) {
				min_raw_deg = sample_raw_deg;
			}
			if (sample_raw_deg > max_raw_deg) {
				max_raw_deg = sample_raw_deg;
			}
		}
		clean_count++;
		prev = sample;
		last = sample;
	}

	shell_print(sh, "Encoder raw trace summary:");
	shell_print(sh, "  Stored:       %u / %u", stored, MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	shell_print(sh, "  Decimation:   %u", g_motor_params->encoder_raw_trace.decimation);
	shell_print(sh, "  Overrun:      %u", g_motor_params->encoder_raw_trace.overrun_count);
	if (clean_count > 0U) {
		shell_print(sh, "  Clean span:   %u -> %u (%u ticks, %u samples)",
			    first->control_loop_count, last->control_loop_count,
			    last->control_loop_count - first->control_loop_count,
			    clean_count);
		shell_print(sh, "  Raw first/last: %d -> %d mdeg",
			    motor_encoder_rad_to_mdeg(first->raw_angle_rad),
			    motor_encoder_rad_to_mdeg(last->raw_angle_rad));
		shell_print(sh, "  Raw delta:    %d mdeg, avg %d mHz",
			    motor_encoder_deg_to_mdeg(raw_delta_deg),
			    motor_encoder_avg_velocity_mhz_deg(raw_delta_deg,
							       first->control_loop_count,
							       last->control_loop_count));
		shell_print(sh, "  Ctrl delta:   %d mdeg, avg %d mHz",
			    motor_encoder_deg_to_mdeg(ctrl_delta_deg),
			    motor_encoder_avg_velocity_mhz_deg(ctrl_delta_deg,
							       first->control_loop_count,
							       last->control_loop_count));
	} else {
		shell_print(sh, "  Clean span:   none");
	}
	shell_print(sh, "  Counts:       clean=%u fresh=%u ctrl_en=%u warn=%u err=%u io=%u",
		    clean_count, fresh_count, enabled_count, warn_count, err_count, io_count);
	shell_print(sh, "  Delta drops:  %u", delta_drop_count);
	shell_print(sh, "  Status bits:  or=0x%02X and=0x%02X first=0x%02X last=0x%02X",
		    status_or, status_and,
		    g_motor_params->encoder_raw_trace.samples[oldest_idx].status,
		    g_motor_params->encoder_raw_trace.samples[
			    motor_encoder_ring_index(oldest_idx, stored - 1U,
						     MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES)].status);
	return 0;
}

/* motor encoder trace dump [count] | dump <offset> <count> */
int cmd_motor_encoder_trace_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_raw_trace.count;
	if (stored == 0U) {
		shell_print(sh, "No raw trace samples");
		return 0;
	}

	uint16_t start = 0U;
	uint16_t count = 0U;
	int ret = motor_encoder_parse_dump_window(sh, argc, argv, stored,
						  MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES,
						  g_motor_params->encoder_raw_trace.write_idx,
						  &start, &count);
	if (ret < 0) {
		return ret;
	}

	if (g_motor_params->encoder_raw_trace.enabled) {
		shell_warn(sh,
			   "Raw trace is still running; dump may include concurrently updated samples.");
	}

	shell_print(sh, "Raw trace dump: stored=%u count=%u max_chunk=%u",
		    stored, count, MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
	shell_print(sh,
		    "idx loop src raw_mdeg ctrl_mdeg obs_mech_mdeg obs_elec_mdeg gen_mech_mdeg gen_elec_mdeg q fresh warn err io status ctrl_en");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
		const struct motor_encoder_raw_trace_sample *sample =
			&g_motor_params->encoder_raw_trace.samples[idx];
		shell_print(sh,
			    "%u %u %u %d %d %d %d %d %d 0x%02X %u %u %u %u 0x%02X %u",
			    i,
			    sample->control_loop_count,
			    sample->input_source,
			    motor_encoder_rad_to_mdeg(sample->raw_angle_rad),
			    motor_encoder_deg_to_mdeg((double)sample->control_angle_deg),
			    motor_encoder_rad_to_mdeg(sample->observer_mech_rad),
			    motor_encoder_rad_to_mdeg(sample->observer_elec_rad),
			    motor_encoder_rad_to_mdeg(sample->generated_mech_rad),
			    motor_encoder_rad_to_mdeg(sample->generated_elec_rad),
			    sample->quality_flags,
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    sample->sample_io_fault,
			    sample->status,
			    sample->sample_enabled);
	}

	return 0;
}


