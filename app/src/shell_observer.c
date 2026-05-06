#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

int cmd_motor_observer_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Observer Status:");
	shell_print(sh, "  Trust:        %s",
		    motor_shell_feedback_trust_name(g_motor_params->live.position_trust_state));
	shell_print(sh, "  Quality:      flags=0x%02X valid=%s fresh=%s error=%s",
		    g_motor_params->live.position_quality_flags,
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) ?
			    "yes" : "no",
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) ?
			    "yes" : "no",
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) ?
			    "yes" : "no");
	shell_print(sh, "  Input source: %s fresh=%s",
		    motor_encoder_input_source_to_string(g_motor_params->live.encoder_input_source),
		    g_motor_params->live.encoder_sample_fresh ? "yes" : "no");
	shell_print(sh, "  Mech angle:   %.3f deg",
		    motor_shell_rad_to_deg(g_motor_params->live.observer_mech_rad));
	shell_print(sh, "  Elec angle:   %.3f deg pred=%.3f deg",
		    motor_shell_rad_to_deg(g_motor_params->live.observer_elec_rad),
		    motor_shell_rad_to_deg(g_motor_params->live.observer_elec_pred_rad));
	shell_print(sh, "  Velocity:     %.4f Hz",
		    (double)(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32)));
	shell_print(sh, "  Accel:        %.3f rad/s^2",
		    (double)g_motor_params->live.acceleration_rad_s2);
	shell_print(sh, "  Delay:        sample=%.2f pred_age=%.0f",
		    (double)g_motor_params->live.observer_delay_samples,
		    (double)g_motor_params->live.observer_prediction_age_samples);
	shell_print(sh, "  Offset:       base=%.3f deg active=%.3f deg trim_elec=%.3f deg",
		    motor_shell_rad_to_deg(g_motor_params->observer_alignment_offset_rad),
		    motor_shell_rad_to_deg(g_motor_params->observer.mech_angle_offset_rad),
		    motor_shell_rad_to_deg(g_motor_params->observer_elec_trim_rad));
	shell_print(sh, "  Stale counts: samples=%u events=%u glitches=%u",
		    g_motor_params->live.position_stale_count,
		    g_motor_params->live.position_stale_events,
		    g_motor_params->live.position_glitch_count);

	return 0;
}


