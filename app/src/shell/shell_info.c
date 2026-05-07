#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor info config */
int cmd_motor_info_config(const struct shell *sh, size_t argc, char **argv)
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
int cmd_motor_info_measured(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	shell_print(sh, "Measured Parameters:");
	shell_print(sh, "  Rs:             %.6f Ohm", (double)g_motor_params->Rs_measured_ohm);
	shell_print(sh, "  Lavg:           %.9f H", (double)g_motor_params->Ls_measured_H);
	shell_print(sh, "  Ld:             %.9f H", (double)g_motor_params->Ld_measured_H);
	shell_print(sh, "  Lq:             %.9f H", (double)g_motor_params->Lq_measured_H);
	shell_print(sh, "  R/L:            %.3f rad/s", (double)g_motor_params->R_over_L_measured);
	shell_print(sh, "  psi_f active:   %.8f Wb", (double)g_motor_params->flux_linkage_wb_active);
	shell_print(sh, "  Kt active:      %.8f Nm/A",
		    (double)motor_torque_gain_resolve_active(g_motor_params));
	shell_print(sh, "  J active:       %.8f kgm2", (double)g_motor_params->inertia_kgm2_active);
	shell_print(sh, "  B active:       %.8f Nm/(rad/s)",
		    (double)g_motor_params->viscous_friction_nm_per_rad_s_active);
	struct motor_voltage_speed_limit_result voltage_limit = {0};
	if (motor_shell_voltage_speed_limit(g_motor_params,
					    g_motor_params->velocity_cl_iq_limit_A,
					    &voltage_limit) == 0 && voltage_limit.valid) {
		shell_print(sh, "  Speed limit:    %.2f Hz command, %.2f Hz voltage model",
			    (double)motor_shell_velocity_command_limit_hz(g_motor_params),
			    (double)voltage_limit.max_mech_hz);
		shell_print(sh, "  Voltage limit:  %.2f V usable, BEMF %.2f V at limit",
			    (double)voltage_limit.voltage_limit_v,
			    (double)voltage_limit.bemf_at_limit_v);
	}
	shell_print(sh, "  Ia offset:      %.6f A", (double)g_motor_params->Ia_offset);
	shell_print(sh, "  Ib offset:      %.6f A", (double)g_motor_params->Ib_offset);
	shell_print(sh, "  Commissioned:   %s", g_motor_params->calibration.commissioning_complete ? "YES" : "NO");
	
	return 0;
}

/* motor info live */
int cmd_motor_info_live(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	int state = motor_api_get_state();
	int error = motor_api_get_error();
	float32_t live_position_rad = g_motor_params->live.position_rad;
	float32_t live_elec_angle_rad = g_motor_params->live.elec_angle_rad;
	float32_t obs_mech_rad = g_motor_params->live.observer_mech_rad;
	float32_t obs_elec_rad = g_motor_params->live.observer_elec_rad;
	float32_t obs_elec_pred_rad = g_motor_params->live.observer_elec_pred_rad;
	float32_t obs_offset_rad = g_motor_params->observer.mech_angle_offset_rad;
	float32_t align_offset_rad = g_motor_params->observer_alignment_offset_rad;
	float32_t enc_observer_input_rad = g_motor_params->live.encoder_observer_input_rad;
	float32_t enc_raw_deg = g_motor_params->live.encoder_raw_deg;
	float32_t enc_raw_rad = g_motor_params->live.encoder_raw_rad;
	
	shell_print(sh, "Live Telemetry:");
	shell_print(sh, "  State:          %s", motor_state_to_string(state));
	shell_print(sh, "  Error:          %s", motor_error_to_string(error));
	shell_print(sh, "  Angle (mech):   %.1f deg",
		    motor_shell_rad_to_deg(live_position_rad));
	shell_print(sh, "  Angle (elec):   %.1f deg",
		    motor_shell_rad_to_deg(live_elec_angle_rad));
	shell_print(sh, "  Obs angle:      mech=%.1f deg elec=%.1f deg pred=%.1f deg",
		    motor_shell_rad_to_deg(obs_mech_rad),
		    motor_shell_rad_to_deg(obs_elec_rad),
		    motor_shell_rad_to_deg(obs_elec_pred_rad));
	shell_print(sh, "  Obs raw rad:    mech=%.6f elec=%.6f pred=%.6f",
		    (double)obs_mech_rad,
		    (double)obs_elec_rad,
		    (double)obs_elec_pred_rad);
	shell_print(sh, "  Obs raw bits:   mech=0x%08X elec=0x%08X pred=0x%08X",
		    motor_shell_f32_bits(obs_mech_rad),
		    motor_shell_f32_bits(obs_elec_rad),
		    motor_shell_f32_bits(obs_elec_pred_rad));
	shell_print(sh, "  Obs timing:     delay=%.2f samples pred_age=%.0f samples",
		    (double)g_motor_params->live.observer_delay_samples,
		    (double)g_motor_params->live.observer_prediction_age_samples);
	shell_print(sh, "  Obs offset:     %.3f deg",
		    motor_shell_rad_to_deg(obs_offset_rad));
	shell_print(sh, "  Obs off raw:    %.6f rad bits=0x%08X",
		    (double)obs_offset_rad,
		    motor_shell_f32_bits(obs_offset_rad));
	shell_print(sh, "  Encoder offset: %.3f deg",
		    motor_shell_rad_to_deg(align_offset_rad));
	shell_print(sh, "  Enc raw:        %.3f deg (%.6f rad)",
		    (double)enc_raw_deg,
		    (double)enc_raw_rad);
	shell_print(sh, "  Enc dir sign:   %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	shell_print(sh, "  Enc trim:       elec=%.3f deg mech=%.4f deg",
		    (double)(g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32)),
		    (double)((g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32)) /
			     (float32_t)MOTOR_POLE_PAIRS));
	shell_print(sh, "  Enc used:       %.6f rad (%s, fresh=%s)",
		    (double)enc_observer_input_rad,
		    motor_encoder_input_source_to_string(g_motor_params->live.encoder_input_source),
		    g_motor_params->live.encoder_sample_fresh ? "yes" : "no");
	shell_print(sh, "  Enc used bits:  0x%08X", motor_shell_f32_bits(
		    enc_observer_input_rad));
	shell_print(sh, "  Enc flags:      status=0x%02X warn=%s err=%s",
		    g_motor_params->live.encoder_last_status,
		    g_motor_params->live.encoder_sample_warning ? "SET" : "CLEAR",
		    g_motor_params->live.encoder_sample_error ? "SET" : "CLEAR");
	shell_print(sh, "  Enc acquisition:   %s, %s",
		    motor_encoder_acquisition_is_enabled() ? "enabled" : "disabled",
		    motor_encoder_acquisition_is_busy() ? "busy" : "idle");
	shell_print(sh, "  Enc flag count: warn=%u err=%u",
		    g_motor_params->encoder_warning_count,
		    g_motor_params->encoder_error_count);
	shell_print(sh, "  Pos quality:    0x%02X trust=%s (valid=%s fresh=%s err=%s)",
		    g_motor_params->live.position_quality_flags,
		    motor_shell_feedback_trust_name(g_motor_params->live.position_trust_state),
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) ? "yes" : "no",
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) ? "yes" : "no",
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) ? "yes" : "no");
	shell_print(sh, "  Pos unwrapped:  %.6f rad", (double)g_motor_params->live.position_unwrapped_rad);
	shell_print(sh, "  Pos innovation: %.6f rad", (double)g_motor_params->live.position_innovation_rad);
	shell_print(sh, "  Pos accel:      %.3f rad/s^2", (double)g_motor_params->live.acceleration_rad_s2);
	shell_print(sh, "  Pos counts:     stale=%u events=%u",
		    g_motor_params->live.position_stale_count,
		    g_motor_params->live.position_stale_events);
	shell_print(sh, "  Speed:          %.3f Hz (%.1f RPM)", 
		    (double)(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32)),
		    (double)(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32) * 60.0f));
	shell_print(sh, "  Id reference:   %.3f A", (double)g_motor_params->live.Id_ref_A);
	shell_print(sh, "  Iq reference:   %.3f A", (double)g_motor_params->live.Iq_ref_A);
	shell_print(sh, "  Id measured:    %.3f A", (double)g_motor_params->live.Id_A);
	shell_print(sh, "  Iq measured:    %.3f A", (double)g_motor_params->live.Iq_A);
	shell_print(sh, "  Ia:             %.3f A", (double)g_motor_params->live.Ia_A);
	shell_print(sh, "  Ib:             %.3f A", (double)g_motor_params->live.Ib_A);
	shell_print(sh, "  Vd:             %.3f V", (double)g_motor_params->Vd_V);
	shell_print(sh, "  Vq:             %.3f V", (double)g_motor_params->Vq_V);
	shell_print(sh, "  Va:             %.3f V", (double)g_motor_params->live.Va_V);
	shell_print(sh, "  Vb:             %.3f V", (double)g_motor_params->live.Vb_V);
	shell_print(sh, "  Vmag(max):      %.3f V", (double)g_motor_params->max_voltage_magnitude_V);
	shell_print(sh, "  Vbus:           %.1f V", (double)g_motor_params->live.dc_bus_voltage_V);
	shell_print(sh, "  Encoder OK:     %s", g_motor_params->encoder_fault_counter == 0 ? "yes" : "no");
	shell_print(sh, "  Encoder faults: %u", g_motor_params->encoder_fault_counter);
	
	return 0;
}


/* motor info stats */
int cmd_motor_info_stats(const struct shell *sh, size_t argc, char **argv)
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
	shell_print(sh, "  Encoder faults:         %u", g_motor_params->encoder_fault_counter);
	shell_print(sh, "  Encoder warn count:     %u", g_motor_params->encoder_warning_count);
	shell_print(sh, "  Encoder error count:    %u", g_motor_params->encoder_error_count);
	
	return 0;
}
