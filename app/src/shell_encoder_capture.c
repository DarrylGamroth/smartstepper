#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor encoder capture start [decimation] */
int cmd_motor_encoder_capture_start(const struct shell *sh, size_t argc, char **argv)
{
#if !IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_CAPTURE)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "Encoder capture telemetry is not compiled in");
	return -ENOTSUP;
#else
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder capture start [decimation]");
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

	g_motor_params->encoder_capture.decimation = (uint16_t)decimation;
	motor_encoder_capture_reset(g_motor_params, false);
	g_motor_params->encoder_capture.enabled = true;

	shell_print(sh,
		    "Encoder capture started: decimation=%u, capacity=%u samples",
		    g_motor_params->encoder_capture.decimation,
		    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	return 0;
#endif
}

/* motor encoder capture stop */
int cmd_motor_encoder_capture_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_capture.enabled = false;
	shell_print(sh, "Encoder capture stopped: stored=%u overrun=%u",
		    g_motor_params->encoder_capture.count,
		    g_motor_params->encoder_capture.overrun_count);
	return 0;
}

/* motor encoder capture clear */
int cmd_motor_encoder_capture_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_capture.enabled = false;
	motor_encoder_capture_reset(g_motor_params, true);
	shell_print(sh, "Encoder capture cleared");
	return 0;
}


/* motor encoder capture status */
int cmd_motor_encoder_capture_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Encoder capture:");
	shell_print(sh, "  Compiled:   %s",
		    IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_CAPTURE) ? "YES" : "NO");
	shell_print(sh, "  Enabled:    %s", g_motor_params->encoder_capture.enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->encoder_capture.decimation);
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->encoder_capture.count,
		    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->encoder_capture.overrun_count);

	if (g_motor_params->encoder_capture.count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->encoder_capture.write_idx +
						  MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - 1U) %
						 MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *newest =
			&g_motor_params->encoder_capture.samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u src=%s deg=%.3f fresh=%u warn=%u err=%u status=0x%02X ctrl_en=%u",
			    newest->control_loop_count,
			    motor_encoder_input_source_to_string(newest->input_source),
			    (double)newest->angle_deg,
			    newest->sample_fresh,
			    newest->sample_warning,
			    newest->sample_error,
			    newest->status,
			    newest->sample_enabled);
		if (newest->compare_valid) {
			shell_print(sh,
				    "  Compare:    gen_mech=%.3fdeg enc_mech=%.3fdeg d_mech=%.3fdeg d_elec=%.3fdeg",
				    (double)(newest->generated_mech_rad * (180.0f / PI_F32)),
				    (double)(newest->encoder_mech_rad * (180.0f / PI_F32)),
				    (double)(newest->mech_error_rad * (180.0f / PI_F32)),
				    (double)(newest->elec_error_rad * (180.0f / PI_F32)));
		} else {
			shell_print(sh, "  Compare:    unavailable (needs fresh clean encoder sample)");
		}
	}

	return 0;
}

/* motor encoder capture summary */
int cmd_motor_encoder_capture_summary(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_capture.count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t oldest_idx = motor_encoder_ring_oldest(g_motor_params->encoder_capture.write_idx,
							stored,
							MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	const struct motor_encoder_capture_sample *first = NULL;
	const struct motor_encoder_capture_sample *prev = NULL;
	const struct motor_encoder_capture_sample *last = NULL;
	float32_t angle_delta_rad = 0.0f;
	float32_t encoder_delta_rad = 0.0f;
	float32_t generated_delta_rad = 0.0f;
	float32_t min_angle_deg = 0.0f;
	float32_t max_angle_deg = 0.0f;
	uint32_t clean_count = 0U;
	uint32_t fresh_count = 0U;
	uint32_t warn_count = 0U;
	uint32_t err_count = 0U;
	uint32_t enabled_count = 0U;
	uint32_t compare_count = 0U;
	uint8_t status_or = 0U;
	uint8_t status_and = 0xFFU;

	for (uint16_t i = 0U; i < stored; i++) {
		uint16_t idx = motor_encoder_ring_index(oldest_idx, i,
							MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *sample =
			&g_motor_params->encoder_capture.samples[idx];

		fresh_count += sample->sample_fresh ? 1U : 0U;
		warn_count += sample->sample_warning ? 1U : 0U;
		err_count += sample->sample_error ? 1U : 0U;
		enabled_count += sample->sample_enabled ? 1U : 0U;
		compare_count += sample->compare_valid ? 1U : 0U;
		status_or |= sample->status;
		status_and &= sample->status;

		if (!motor_encoder_capture_sample_clean(sample)) {
			continue;
		}

		if (first == NULL) {
			first = sample;
			min_angle_deg = sample->angle_deg;
			max_angle_deg = sample->angle_deg;
		} else {
			angle_delta_rad += wrap_rad_pi(sample->angle_rad - prev->angle_rad);
			encoder_delta_rad += wrap_rad_pi(sample->encoder_mech_rad -
							 prev->encoder_mech_rad);
			generated_delta_rad += wrap_rad_pi(sample->generated_mech_rad -
							   prev->generated_mech_rad);
			min_angle_deg = MIN(min_angle_deg, sample->angle_deg);
			max_angle_deg = MAX(max_angle_deg, sample->angle_deg);
		}
		clean_count++;
		prev = sample;
		last = sample;
	}

	shell_print(sh, "Encoder capture summary:");
	shell_print(sh, "  Stored:       %u / %u", stored, MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	shell_print(sh, "  Decimation:   %u", g_motor_params->encoder_capture.decimation);
	shell_print(sh, "  Overrun:      %u", g_motor_params->encoder_capture.overrun_count);
	if (clean_count > 0U) {
		shell_print(sh, "  Clean span:   %u -> %u (%u ticks, %u samples)",
			    first->control_loop_count, last->control_loop_count,
			    last->control_loop_count - first->control_loop_count,
			    clean_count);
		shell_print(sh, "  Angle first/last: %.3f -> %.3f deg",
			    (double)first->angle_deg, (double)last->angle_deg);
		shell_print(sh, "  Angle delta:  %.3f deg, avg %.4f Hz",
			    (double)(angle_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(angle_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Enc delta:    %.3f deg, avg %.4f Hz",
			    (double)(encoder_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(encoder_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Gen delta:    %.3f deg, avg %.4f Hz",
			    (double)(generated_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(generated_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Angle min/max: %.3f / %.3f deg",
			    (double)min_angle_deg, (double)max_angle_deg);
	} else {
		shell_print(sh, "  Clean span:   none");
	}
	shell_print(sh, "  Counts:       clean=%u fresh=%u ctrl_en=%u warn=%u err=%u compare=%u",
		    clean_count, fresh_count, enabled_count, warn_count, err_count, compare_count);
	shell_print(sh, "  Status bits:  or=0x%02X and=0x%02X first=0x%02X last=0x%02X",
		    status_or, status_and,
		    g_motor_params->encoder_capture.samples[oldest_idx].status,
		    g_motor_params->encoder_capture.samples[
			    motor_encoder_ring_index(oldest_idx, stored - 1U,
						     MOTOR_ENCODER_CAPTURE_MAX_SAMPLES)].status);
	return 0;
}

/* motor encoder capture dump [count] | dump <offset> <count> */
int cmd_motor_encoder_capture_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_capture.count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t start = 0U;
	uint16_t count = 0U;
	int ret = motor_encoder_parse_dump_window(sh, argc, argv, stored,
						  MOTOR_ENCODER_CAPTURE_MAX_SAMPLES,
						  g_motor_params->encoder_capture.write_idx,
						  &start, &count);
	if (ret < 0) {
		return ret;
	}

	if (g_motor_params->encoder_capture.enabled) {
		shell_warn(sh,
			   "Capture is still running; dump may include concurrently updated samples.");
	}

	shell_print(sh, "Capture dump: stored=%u count=%u max_chunk=%u",
		    stored, count, MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
	shell_print(sh,
		    "idx loop source deg rad enc_rad obs_rad norm q31 fresh warn err status ctrl_en");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		struct motor_encoder_capture_sample sample =
			g_motor_params->encoder_capture.samples[idx];
		float32_t norm = motor_encoder_normalized_from_rad(sample.angle_rad);
		int32_t q31 = motor_encoder_q31_from_rad(sample.angle_rad);
		shell_print(sh,
			    "%u %u %s %.3f %.6f %.6f %.6f %.6f %d %u %u %u 0x%02X %u",
			    i,
			    sample.control_loop_count,
			    motor_encoder_input_source_to_string(sample.input_source),
			    (double)sample.angle_deg,
			    (double)sample.angle_rad,
			    (double)sample.encoder_mech_rad,
			    (double)sample.observer_mech_rad,
			    (double)norm,
			    q31,
			    sample.sample_fresh,
			    sample.sample_warning,
			    sample.sample_error,
			    sample.status,
			    sample.sample_enabled);
	}

	return 0;
}

/* motor encoder capture compare [count] [gen|obs] */
int cmd_motor_encoder_capture_compare(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 1U || argc > 3U) {
		shell_error(sh, "Usage: motor encoder capture compare [count] [gen|obs]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t requested = 32U;
	uint8_t ref_mode = MOTOR_ENCODER_COMPARE_REF_GENERATED;
	if (argc == 2U) {
		if (shell_parse_u32(argv[1], &requested)) {
			if (requested == 0U || requested > MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
				shell_error(sh, "count must be in [1, %u]",
					    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
				return -EINVAL;
			}
		} else if (!motor_encoder_compare_ref_parse(argv[1], &ref_mode)) {
			shell_error(sh, "Invalid ref '%s' (expected gen|obs)", argv[1]);
			return -EINVAL;
		}
	}
	if (argc == 3U) {
		if (!shell_parse_u32(argv[1], &requested) || requested == 0U ||
		    requested > MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
			shell_error(sh, "count must be in [1, %u]",
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
			return -EINVAL;
		}
		if (!motor_encoder_compare_ref_parse(argv[2], &ref_mode)) {
			shell_error(sh, "Invalid ref '%s' (expected gen|obs)", argv[2]);
			return -EINVAL;
		}
	}

	uint16_t stored = g_motor_params->encoder_capture.count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->encoder_capture.write_idx +
				     MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - count) %
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);

	if (g_motor_params->encoder_capture.enabled) {
		shell_warn(sh,
			   "Capture is still running; compare dump may include concurrently updated samples.");
	}

	shell_print(sh, "idx loop src fresh warn err cmp ref");
	shell_print(sh, "  mech: enc_deg ref_deg d_deg");
	shell_print(sh, "  elec: enc_deg ref_deg d_deg rel_phase_deg");
	bool rel_phase_init = false;
	float32_t rel_phase_base_rad = 0.0f;
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *sample =
			&g_motor_params->encoder_capture.samples[idx];
		float32_t enc_m_deg = sample->encoder_mech_rad * (180.0f / PI_F32);
		float32_t enc_e_deg = sample->encoder_elec_rad * (180.0f / PI_F32);
		float32_t ref_mech_rad =
			(ref_mode == MOTOR_ENCODER_COMPARE_REF_OBSERVER) ?
				sample->observer_mech_rad : sample->generated_mech_rad;
		float32_t ref_elec_rad =
			(ref_mode == MOTOR_ENCODER_COMPARE_REF_OBSERVER) ?
				sample->observer_elec_rad : sample->generated_elec_rad;
		float32_t ref_m_deg = ref_mech_rad * (180.0f / PI_F32);
		float32_t ref_e_deg = ref_elec_rad * (180.0f / PI_F32);
		float32_t d_m_deg = 0.0f;
		float32_t d_e_deg = 0.0f;
		float32_t rel_phase_deg = 0.0f;
		bool compare_valid =
			sample->compare_valid &&
			(sample->encoder_mech_rad == sample->encoder_mech_rad) &&
			fabsf(sample->encoder_mech_rad) <= 1.0e6f &&
			(ref_mech_rad == ref_mech_rad) &&
			fabsf(ref_mech_rad) <= 1.0e6f &&
			(sample->encoder_elec_rad == sample->encoder_elec_rad) &&
			fabsf(sample->encoder_elec_rad) <= 1.0e6f &&
			(ref_elec_rad == ref_elec_rad) &&
			fabsf(ref_elec_rad) <= 1.0e6f;
		if (compare_valid) {
			float32_t mech_error_rad =
				wrap_rad_pi(sample->encoder_mech_rad - ref_mech_rad);
			float32_t elec_error_rad =
				wrap_rad_pi(sample->encoder_elec_rad - ref_elec_rad);
			d_m_deg = mech_error_rad * (180.0f / PI_F32);
			d_e_deg = elec_error_rad * (180.0f / PI_F32);
			if (!rel_phase_init) {
				rel_phase_base_rad = elec_error_rad;
				rel_phase_init = true;
				rel_phase_deg = 0.0f;
			} else {
				rel_phase_deg =
					wrap_rad_pi(elec_error_rad - rel_phase_base_rad) *
					(180.0f / PI_F32);
			}
		}
		shell_print(sh,
			    "%u %u %s %u %u %u %u %s",
			    i,
			    sample->control_loop_count,
			    motor_encoder_input_source_to_string(sample->input_source),
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    compare_valid ? 1U : 0U,
			    motor_encoder_compare_ref_to_string(ref_mode));
		shell_print(sh,
			    "  mech: %.3f %.3f %.3f",
			    (double)enc_m_deg,
			    (double)ref_m_deg,
			    (double)d_m_deg);
		shell_print(sh,
			    "  elec: %.3f %.3f %.3f %.3f",
			    (double)enc_e_deg,
			    (double)ref_e_deg,
			    (double)d_e_deg,
			    (double)rel_phase_deg);
	}

	return 0;
}


