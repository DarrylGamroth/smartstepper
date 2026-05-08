/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>

#include "shell_commands.h"
#include "shell_control.h"
#include "shell_commands_motion.h"
#include "shell_commands_commission.h"
#include "shell_commands_state.h"
#include "config.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(shell_commands, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set by main) */
struct motor_parameters *g_motor_params = NULL;

void shell_set_motor_params(struct motor_parameters *params)
{
	g_motor_params = params;
}

struct motor_parameters *shell_get_motor_params(void)
{
	return g_motor_params;
}

bool motor_control_is_armed(const struct motor_parameters *params)
{
	return params && (atomic_get(&params->control_armed) != 0);
}

void motor_command_feed_watchdog(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	params->last_command_update_ms = k_uptime_get_32();
	params->last_command_update_loop = params->control_loop_count;
	params->command_timeout_latched = false;
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
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_current_gain,
	SHELL_CMD(get, NULL, "Get current-loop PI gains <id|iq>", cmd_motor_current_gain_get),
	SHELL_CMD(set, NULL, "Set current-loop PI gains <id|iq> <kp> <ki>", cmd_motor_current_gain_set),
	SHELL_CMD(bandwidth, NULL, "Set current-loop PI bandwidth <id|iq> <hz>", cmd_motor_current_gain_bandwidth),
	SHELL_SUBCMD_SET_END
);

/* motor current subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_current,
	SHELL_CMD(id, NULL, "Set Id current (A)", cmd_motor_current_id),
	SHELL_CMD(iq, NULL, "Set Iq current (A)", cmd_motor_current_iq),
	SHELL_CMD(dq, NULL, "Set Id and Iq currents", cmd_motor_current_dq),
	SHELL_CMD(status, NULL, "Show current command and slew status", cmd_motor_current_status),
	SHELL_CMD(gain, &sub_motor_current_gain, "Current-loop PI gain tuning", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor state mode subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state_mode,
	SHELL_CMD(current_encoder, NULL, "Encoder-commutated direct Id/Iq current mode",
		  cmd_motor_state_mode_current_encoder),
	SHELL_CMD(velocity_generated, NULL, "Generated-angle velocity mode",
		  cmd_motor_state_mode_velocity_generated),
	SHELL_CMD(position_generated, NULL, "Generated-angle position/profile mode",
		  cmd_motor_state_mode_position_generated),
	SHELL_CMD(velocity_encoder, NULL, "Encoder-feedback velocity mode",
		  cmd_motor_state_mode_velocity_encoder),
	SHELL_CMD(position_encoder, NULL, "Encoder-feedback position/profile mode",
		  cmd_motor_state_mode_position_encoder),
	SHELL_SUBCMD_SET_END
);

/* motor state subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state,
	SHELL_CMD(idle, NULL, "Transition to IDLE state", cmd_motor_state_idle),
	SHELL_CMD(prepare, NULL, "Run prepare/calibration path before ONLINE",
		  cmd_motor_state_prepare_online),
	SHELL_CMD(online, NULL, "Transition to ONLINE state", cmd_motor_state_online),
	SHELL_CMD(calibrate, NULL, "Run fast boot calibration (current offsets only)",
		  cmd_motor_state_calibrate),
	SHELL_CMD(commission, NULL, "Run RoverL electrical bootstrap",
		  cmd_motor_state_commission),
	SHELL_CMD(clear_error, NULL, "Clear error state", cmd_motor_state_clear_error),
	SHELL_CMD(status, NULL, "Show motor status", cmd_motor_state_status),
	SHELL_CMD(policy, NULL, "Show active control policy", cmd_motor_state_policy),
	SHELL_CMD(transition, NULL, "Show latest state/mode transition result",
		  cmd_motor_state_transition),
	SHELL_CMD(recovery, NULL, "Show fault recovery status", cmd_motor_state_recovery),
	SHELL_CMD(mode, &sub_motor_state_mode, "Switch control mode", NULL),
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

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
/* motor rls subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_rls,
	SHELL_CMD(status, NULL, "Show RLS status", cmd_motor_rls_status),
	SHELL_CMD(params, NULL, "Show RLS estimates", cmd_motor_rls_params),
	SHELL_CMD(temp, NULL, "Show temperature estimates", cmd_motor_rls_temp),
	SHELL_CMD(gating, NULL, "Show gating conditions", cmd_motor_rls_gating),
	SHELL_CMD(reset, NULL, "Reset RLS estimators", cmd_motor_rls_reset),
	SHELL_SUBCMD_SET_END
);
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

/* motor outer subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_outer,
	SHELL_CMD(mode, NULL, "Select outer-loop regulator <pi|mpr>", cmd_motor_outer_mode),
	SHELL_CMD(status, NULL, "Show active outer-loop regulator and tuning", cmd_motor_outer_status),
	SHELL_SUBCMD_SET_END
);

/* motor control subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_control,
	SHELL_CMD(status, NULL, "Show control regulator, limits, and feedforward status",
		  cmd_motor_control_status),
	SHELL_SUBCMD_SET_END
);

/* motor observer subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_observer,
	SHELL_CMD(status, NULL, "Show observer trust, angles, and latency",
		  cmd_motor_observer_status),
	SHELL_SUBCMD_SET_END
);

/* motor velocity subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_velocity,
	SHELL_CMD_ARG(target, NULL, "Set velocity target <hz>", cmd_motor_velocity_target, 2, 0),
	SHELL_CMD_ARG(decimation, NULL, "Set velocity loop decimation <ticks>", cmd_motor_velocity_decimation, 2, 0),
	SHELL_CMD_ARG(pi, NULL,
		      "Velocity PI: status | set <kp> <ki> <iq_limit> | defaults <safe|nominal> | bandwidth <hz> [zeta]",
		      cmd_motor_velocity_pi, 2, 3),
	SHELL_CMD_ARG(mpr, NULL,
		      "Velocity MPR: status | set <q> <r> <horizon> <max_delta_iq> [dist_ki] | bandwidth <hz>",
		      cmd_motor_velocity_mpr, 2, 5),
	SHELL_CMD_ARG(dob, NULL,
		      "Velocity disturbance observer: status | defaults <safe|nominal> | enable <0|1> | gain <nm_per_rad_s> | torque_limit <nm> | iq_limit <a>",
		      cmd_motor_velocity_dob, 2, 1),
	SHELL_CMD(status, NULL, "Show velocity status", cmd_motor_velocity_status),
	SHELL_SUBCMD_SET_END
);

/* motor position subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_position,
	SHELL_CMD_ARG(target, NULL, "Set position target <deg>", cmd_motor_position_target, 2, 0),
	SHELL_CMD_ARG(decimation, NULL, "Set position loop decimation <ticks>", cmd_motor_position_decimation, 2, 0),
	SHELL_CMD_ARG(pi, NULL,
		      "Position PI: status | set <kp> <ki> | defaults <safe|nominal> | bandwidth <hz> [zeta]",
		      cmd_motor_position_pi, 2, 2),
	SHELL_CMD_ARG(mpr, NULL,
		      "Position MPR: status | set <q_pos> <q_vel> <r> <horizon> [max_delta_vel] | bandwidth <hz>",
		      cmd_motor_position_mpr, 2, 5),
	SHELL_CMD(status, NULL, "Show position status", cmd_motor_position_status),
	SHELL_SUBCMD_SET_END
);

/* motor profile seq subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile_seq_trigger,
	SHELL_CMD_ARG(source, NULL, "Set trigger source <timer|external> (alias: internal)",
		      cmd_motor_profile_seq_trigger_source, 2, 0),
	SHELL_CMD_ARG(edge, NULL, "Set external edge <rising|falling|both>",
		      cmd_motor_profile_seq_trigger_edge, 2, 0),
	SHELL_CMD_ARG(channel, NULL, "Set external capture channel <0..3>",
		      cmd_motor_profile_seq_trigger_channel, 2, 0),
	SHELL_CMD_ARG(min_interval_us, NULL, "Set minimum trigger spacing <us>",
		      cmd_motor_profile_seq_trigger_min_interval, 2, 0),
	SHELL_CMD(status, NULL, "Show trigger source/edge/filter status",
		  cmd_motor_profile_seq_trigger_status),
	SHELL_CMD(fire, NULL, "Inject one software external trigger", cmd_motor_profile_seq_trigger_fire),
	SHELL_SUBCMD_SET_END
);

/* motor profile seq subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile_seq,
	SHELL_CMD(clear, NULL, "Clear sequence points", cmd_motor_profile_seq_clear),
	SHELL_CMD_ARG(add, NULL, "Add sequence point <target_deg>", cmd_motor_profile_seq_add, 2, 0),
	SHELL_CMD_ARG(period_ms, NULL, "Set trigger period <ms>", cmd_motor_profile_seq_period_ms, 2, 0),
	SHELL_CMD_ARG(move_ms, NULL, "Set move duration <ms>", cmd_motor_profile_seq_move_ms, 2, 0),
	SHELL_CMD_ARG(end_vel_hz, NULL, "Set segment end velocity <hz>",
		      cmd_motor_profile_seq_end_vel_hz, 2, 0),
	SHELL_CMD_ARG(loop, NULL, "Set loop enable <0|1>", cmd_motor_profile_seq_loop, 2, 0),
	SHELL_CMD_ARG(config, NULL,
		      "Legacy bulk set <period_ms> <move_ms> <end_vel_hz> <loop:0|1>",
		      cmd_motor_profile_seq_config, 5, 0),
	SHELL_CMD(trigger, &sub_motor_profile_seq_trigger, "Sequence trigger source config", NULL),
	SHELL_CMD(start, NULL, "Start sequence playback using configured trigger source",
		  cmd_motor_profile_seq_start),
	SHELL_CMD(stop, NULL, "Stop sequence playback", cmd_motor_profile_seq_stop),
	SHELL_CMD(status, NULL, "Show sequence status", cmd_motor_profile_seq_status),
	SHELL_CMD(list, NULL, "List sequence points", cmd_motor_profile_seq_list),
	SHELL_SUBCMD_SET_END
);

/* motor profile subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile,
	SHELL_CMD_ARG(set, NULL, "Set profile limits <max_hz> <max_accel_hz_s>", cmd_motor_profile_set, 3, 0),
	SHELL_CMD_ARG(move, NULL, "Plan quintic move <target_deg> <end_vel_hz> <duration_ms>", cmd_motor_profile_move, 4, 0),
	SHELL_CMD(cancel, NULL, "Cancel active motion profile", cmd_motor_profile_cancel),
	SHELL_CMD(seq, &sub_motor_profile_seq, "Sequence playback control", NULL),
	SHELL_CMD(status, NULL, "Show motion profile status", cmd_motor_profile_status),
	SHELL_SUBCMD_SET_END
);

/* motor chopper calib subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_chopper_calib,
	SHELL_CMD_ARG(start, NULL, "Capture edges <slots> <revs> <speed_hz>", cmd_motor_chopper_calib_start, 4, 0),
	SHELL_CMD(stop, NULL, "Stop active calibration capture", cmd_motor_chopper_calib_stop),
	SHELL_CMD(status, NULL, "Show calibration capture/midpoint status", cmd_motor_chopper_calib_status),
	SHELL_CMD(apply, NULL, "Apply midpoint table to profile sequence points", cmd_motor_chopper_calib_apply),
	SHELL_CMD(clear, NULL, "Clear calibration buffers and midpoint table", cmd_motor_chopper_calib_clear),
	SHELL_SUBCMD_SET_END
);

/* motor chopper subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_chopper,
	SHELL_CMD(calib, &sub_motor_chopper_calib, "Chopper midpoint calibration", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor encoder subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_capture,
	SHELL_CMD_ARG(start, NULL, "Start capture [decimation]", cmd_motor_encoder_capture_start, 1, 1),
	SHELL_CMD(stop, NULL, "Stop capture", cmd_motor_encoder_capture_stop),
	SHELL_CMD(status, NULL, "Show capture buffer status", cmd_motor_encoder_capture_status),
	SHELL_CMD(summary, NULL, "Summarize capture buffer", cmd_motor_encoder_capture_summary),
	SHELL_CMD_ARG(dump, NULL, "Dump samples [count] or <offset> <count> (max 32 rows)",
		      cmd_motor_encoder_capture_dump, 1, 2),
	SHELL_CMD_ARG(compare, NULL, "Dump encoder comparison [count] [gen|obs]",
		      cmd_motor_encoder_capture_compare, 1, 2),
	SHELL_CMD(clear, NULL, "Clear capture buffer", cmd_motor_encoder_capture_clear),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_trace,
	SHELL_CMD_ARG(start, NULL, "Start raw trace [decimation]", cmd_motor_encoder_trace_start, 1, 1),
	SHELL_CMD(stop, NULL, "Stop raw trace", cmd_motor_encoder_trace_stop),
	SHELL_CMD(status, NULL, "Show raw trace buffer status", cmd_motor_encoder_trace_status),
	SHELL_CMD(summary, NULL, "Summarize raw trace buffer", cmd_motor_encoder_trace_summary),
	SHELL_CMD_ARG(dump, NULL, "Dump raw samples [count] or <offset> <count> (max 32 rows)",
		      cmd_motor_encoder_trace_dump, 1, 2),
	SHELL_CMD(clear, NULL, "Clear raw trace buffer", cmd_motor_encoder_trace_clear),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_protocol,
	SHELL_CMD(status, NULL, "Show AEAT-9955 protocol/config state",
		  cmd_motor_encoder_protocol_status),
	SHELL_CMD(spi4_8_volatile, NULL, "Switch AEAT to volatile SPI4-8 CRC16",
		  cmd_motor_encoder_protocol_spi4_8_volatile),
	SHELL_CMD(spi4_16_volatile, NULL, "Switch AEAT to volatile SPI4-16 parity",
		  cmd_motor_encoder_protocol_spi4_16_volatile),
	SHELL_CMD(detect, NULL, "Detect current AEAT SPI4 protocol",
		  cmd_motor_encoder_protocol_detect),
	SHELL_CMD(driver_spi4_8, NULL, "Set driver-only protocol to SPI4-8 CRC16",
		  cmd_motor_encoder_protocol_driver_spi4_8),
	SHELL_CMD(driver_spi4_16, NULL, "Set driver-only protocol to SPI4-16 parity",
		  cmd_motor_encoder_protocol_driver_spi4_16),
	SHELL_CMD_ARG(spi_mode, NULL, "Set RT SPI electrical mode <cpol 0|1> <cpha 0|1>",
		      cmd_motor_encoder_protocol_spi_mode, 3, 0),
	SHELL_CMD(raw_position, NULL, "Read one raw AEAT position frame",
		  cmd_motor_encoder_protocol_raw_position),
	SHELL_CMD_ARG(raw_reg, NULL, "Read one raw AEAT register frame <addr>",
		      cmd_motor_encoder_protocol_raw_reg, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_acquisition,
	SHELL_CMD(status, NULL, "Show encoder acquisition status/counters",
		  cmd_motor_encoder_acquisition),
	SHELL_CMD(reset, NULL, "Reset encoder acquisition counters",
		  cmd_motor_encoder_acquisition_reset),
	SHELL_CMD(recover, NULL, "Abort/reset encoder acquisition fault state",
		  cmd_motor_encoder_recover),
	SHELL_CMD_ARG(inject, NULL, "Fault inject mode [none|status|frame]",
		      cmd_motor_encoder_acquisition_inject, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_control,
	SHELL_CMD(status, NULL, "Show encoder-control readiness gate",
		  cmd_motor_encoder_control_status),
	SHELL_CMD_ARG(direction, NULL, "Get/set encoder direction sign [<1|-1>]",
		      cmd_motor_encoder_direction, 1, 1),
	SHELL_CMD_ARG(trim, NULL, "Get/set electrical commutation trim [<-180..180> deg]",
		      cmd_motor_encoder_trim, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_reg,
	SHELL_CMD_ARG(read, NULL, "Read AEAT-9955 register <addr>",
		      cmd_motor_encoder_reg_read, 2, 0),
	SHELL_CMD_ARG(write, NULL, "Write AEAT-9955 register <addr> <value>",
		      cmd_motor_encoder_reg_write, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder,
	SHELL_CMD(status, NULL, "Show concise encoder control status", cmd_motor_encoder_status),
	SHELL_CMD(alarm, NULL, "Read AEAT-9955 alarm byte (MHI/MLO)", cmd_motor_encoder_alarm),
	SHELL_CMD(fast, NULL, "Show fast encoder_rt driver status/counters",
		  cmd_motor_encoder_fast),
	SHELL_CMD(reg, &sub_motor_encoder_reg, "Encoder register access", NULL),
	SHELL_CMD_ARG(reg_read, NULL, "Read AEAT-9955 register <addr>",
		      cmd_motor_encoder_reg_read, 2, 0),
	SHELL_CMD_ARG(reg_write, NULL, "Write AEAT-9955 register <addr> <value>",
		      cmd_motor_encoder_reg_write, 3, 0),
	SHELL_CMD(protocol, &sub_motor_encoder_protocol, "AEAT-9955 protocol control", NULL),
	SHELL_CMD(control, &sub_motor_encoder_control, "Encoder control configuration/readiness",
		  NULL),
	SHELL_CMD_ARG(direction, NULL, "Get/set encoder direction sign [<1|-1>]",
		      cmd_motor_encoder_direction, 1, 1),
	SHELL_CMD_ARG(trim, NULL, "Get/set electrical commutation trim [<-180..180> deg]",
		      cmd_motor_encoder_trim, 1, 1),
	SHELL_CMD(capture, &sub_motor_encoder_capture, "Encoder sample capture buffer", NULL),
	SHELL_CMD(trace, &sub_motor_encoder_trace, "Raw encoder telemetry trace buffer", NULL),
	SHELL_CMD(acquisition, &sub_motor_encoder_acquisition, "Encoder acquisition diagnostics",
		  cmd_motor_encoder_acquisition),
	SHELL_CMD(acquisition_reset, NULL, "Reset encoder acquisition counters",
		  cmd_motor_encoder_acquisition_reset),
	SHELL_CMD(recover, NULL, "Abort/reset encoder acquisition fault state",
		  cmd_motor_encoder_recover),
	SHELL_CMD_ARG(acquisition_inject, NULL, "Acquisition fault inject mode [none|status|frame]",
		      cmd_motor_encoder_acquisition_inject, 1, 1),
	SHELL_CMD(control_status, NULL, "Show encoder-control readiness gate",
		  cmd_motor_encoder_control_status),
	SHELL_SUBCMD_SET_END
);

/* motor safety subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_safety,
	SHELL_CMD_ARG(timeout, NULL, "Set command timeout <ms> (0 disables)", cmd_motor_safety_timeout, 2, 0),
	SHELL_CMD(status, NULL, "Show safety interlock/timeout status", cmd_motor_safety_status),
	SHELL_CMD(pet, NULL, "Refresh command watchdog timer", cmd_motor_safety_pet),
	SHELL_SUBCMD_SET_END
);

/* motor gate-driver subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_gate,
	SHELL_CMD(status, NULL, "Show gate-driver cached fault status", cmd_motor_gate_status),
	SHELL_CMD(reset, NULL, "Pulse DRV8328 nSLEEP to clear latched faults", cmd_motor_gate_reset),
	SHELL_SUBCMD_SET_END
);

/* motor fault snapshot subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_fault_snapshot,
	SHELL_CMD_ARG(start, NULL, "Start fault snapshot capture [decimation]",
		      cmd_motor_fault_snapshot_start, 1, 1),
	SHELL_CMD(stop, NULL, "Stop fault snapshot capture", cmd_motor_fault_snapshot_stop),
	SHELL_CMD(status, NULL, "Show fault snapshot ring status", cmd_motor_fault_snapshot_status),
	SHELL_CMD_ARG(dump, NULL, "Dump latest fault snapshot rows [count]",
		      cmd_motor_fault_snapshot_dump, 1, 1),
	SHELL_CMD(clear, NULL, "Clear fault snapshot ring and latch", cmd_motor_fault_snapshot_clear),
	SHELL_SUBCMD_SET_END
);

/* motor fault subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_fault,
	SHELL_CMD(snapshot, &sub_motor_fault_snapshot, "ISR fault snapshot diagnostics", NULL),
	SHELL_CMD(recovery, NULL, "Show explicit fault recovery status", cmd_motor_fault_recovery),
	SHELL_SUBCMD_SET_END
);

/* motor commission flux subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_flux,
	SHELL_CMD_ARG(run, NULL,
		      "Run flux capture <min_hz> <max_hz> <steps> <settle_ms> <sample_ms> <iq_limit_a>",
		      cmd_motor_commission_flux_run, 7, 0),
	SHELL_SUBCMD_SET_END
);

/* motor commission mech subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_mech,
	SHELL_CMD_ARG(run, NULL,
		      "Run mechanical capture <base_hz> <dither_hz> <dither_period_ms> <duration_ms>",
		      cmd_motor_commission_mech_run, 5, 0),
	SHELL_CMD(status, NULL, "Show staged mechanical v2 fit and capture status",
		  cmd_motor_commission_status),
	SHELL_CMD(active, NULL, "Show active/staged mechanical model state",
		  cmd_motor_commission_status),
	SHELL_CMD(clear, NULL, "Clear staged mechanical capture data",
		  cmd_motor_commission_clear),
	SHELL_CMD(apply_staged, NULL, "Apply valid staged mechanical model",
		  cmd_motor_commission_apply),
	SHELL_SUBCMD_SET_END
);

/* motor commission motion subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_motion,
	SHELL_CMD_ARG(threshold, NULL,
		      "Find min moving current <start_a> <stop_a> <step_a> <hold_ms> [min_motion_deg]",
		      cmd_motor_commission_motion_threshold, 5, 1),
	SHELL_SUBCMD_SET_END
);

/* motor commission encoder subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_encoder,
	SHELL_CMD_ARG(run, NULL,
		      "Stage one-way generated-sweep commutation map <current_a> <mech_hz> <cycles>",
		      cmd_motor_commission_encoder_run, 4, 0),
	SHELL_CMD_ARG(robust, NULL,
		      "Stage robust commutation map <current_a> <mech_hz> <cycles> [bidirectional]",
		      cmd_motor_commission_encoder_robust, 4, 1),
	SHELL_CMD(status, NULL, "Show staged commutation map and quality metrics",
		  cmd_motor_commission_encoder_status),
	SHELL_CMD(apply, NULL, "Apply valid staged commutation map to runtime observer",
		  cmd_motor_commission_encoder_apply),
	SHELL_CMD(clear, NULL, "Clear staged commutation map",
		  cmd_motor_commission_encoder_clear),
	SHELL_SUBCMD_SET_END
);

/* motor commission detent subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_detent,
	SHELL_CMD_ARG(run, NULL,
		      "Run detent feedforward capture <mech_hz> <cycles> [decimation] [iq_limit_a]",
		      cmd_motor_commission_detent_run, 3, 2),
	SHELL_CMD(status, NULL, "Show staged detent feedforward table",
		  cmd_motor_commission_detent_status),
	SHELL_CMD_ARG(apply, NULL,
		      "Apply staged detent table [enable] [gain] [limit_a]",
		      cmd_motor_commission_detent_apply, 1, 3),
	SHELL_CMD_ARG(validate, NULL,
		      "Compare low-speed velocity ripple with detent off/on <mech_hz> <duration_ms>",
		      cmd_motor_commission_detent_validate, 3, 0),
	SHELL_CMD_ARG(dump, NULL,
		      "Dump staged detent bins [start_bin] [count]",
		      cmd_motor_commission_detent_dump, 1, 2),
	SHELL_CMD(clear, NULL, "Clear staged and runtime detent feedforward table",
		  cmd_motor_commission_detent_clear),
	SHELL_SUBCMD_SET_END
);

/* motor commission electrical subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_electrical_measure,
	SHELL_CMD_ARG(rs, NULL,
		      "Stage production Rs measurement [current_a] [samples] [settle_ms]",
		      cmd_motor_commission_electrical_measure_rs, 1, 3),
	SHELL_CMD_ARG(inductance, NULL,
		      "Stage production L measurement [pulse_v] [samples] [pulse_ms]",
		      cmd_motor_commission_electrical_measure_inductance, 1, 3),
	SHELL_CMD_ARG(demod, NULL,
		      "Stage cycle-counted demodulated L measurement [pulse_v] [samples] [half_cycles]",
		      cmd_motor_commission_electrical_measure_demod, 1, 3),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_electrical,
	SHELL_CMD(plan, NULL, "Show production electrical-ID workflow and limits",
		  cmd_motor_commission_electrical_plan),
	SHELL_CMD(measure, &sub_motor_commission_electrical_measure,
		  "Run one production electrical-ID measurement stage", NULL),
	SHELL_CMD_ARG(run, NULL,
		      "Run production electrical-ID sequence [rs_current_a] [l_pulse_v] [samples]",
		      cmd_motor_commission_electrical_run, 1, 3),
	SHELL_CMD_ARG(sweep, NULL,
		      "Sweep D-axis inductance pulses and stage stable scalar L [samples]",
		      cmd_motor_commission_electrical_sweep, 1, 1),
	SHELL_CMD_ARG(demod_sweep, NULL,
		      "Sweep D-axis demod frequencies and stage best scalar L [pulse_v] [samples]",
		      cmd_motor_commission_electrical_demod_sweep, 1, 2),
	SHELL_CMD_ARG(saliency_sweep, NULL,
		      "Measure diagnostic vector saliency [pulse_v] [vectors] [pairs] [revs] [half_cycles] [settle_ticks]",
		      cmd_motor_commission_electrical_saliency_sweep, 1, 6),
	SHELL_CMD(saliency_apply, NULL,
		  "Explicitly stage the last valid diagnostic saliency Ld/Lq",
		  cmd_motor_commission_electrical_saliency_apply),
	SHELL_CMD(status, NULL, "Show staged production electrical-ID result",
		  cmd_motor_commission_electrical_status),
	SHELL_CMD(apply, NULL, "Apply valid staged production electrical-ID result",
		  cmd_motor_commission_electrical_apply),
	SHELL_CMD_ARG(validate, NULL,
		      "Validate active current-loop step [current_a] [hold_ms] [max_error_a]",
		      cmd_motor_commission_electrical_validate, 1, 3),
	SHELL_CMD(clear, NULL, "Clear staged production electrical-ID result",
		  cmd_motor_commission_electrical_clear),
	SHELL_SUBCMD_SET_END
);

/* motor commission auto subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_auto,
	SHELL_CMD_ARG(run, NULL, "Plan or run identify+tune workflow [slow|confirm] [apply]",
		      cmd_motor_commission_auto_run, 1, 1),
	SHELL_CMD(status, NULL, "Show staged auto-tune defaults and reject flags",
		  cmd_motor_commission_auto_status),
	SHELL_CMD(apply, NULL, "Apply staged auto-tune defaults to active runtime parameters",
		  cmd_motor_commission_auto_apply),
	SHELL_CMD_ARG(validate, NULL,
		      "Apply staged tune and run velocity_encoder validation [max_hz] [hold_ms]",
		      cmd_motor_commission_auto_validate, 1, 2),
	SHELL_SUBCMD_SET_END
);

/* motor commission validate subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_validate,
	SHELL_CMD_ARG(current, NULL,
		      "Bounded current_encoder smoke test [iq_a] [hold_ms] [ramp_ms] [stop_deg]",
		      cmd_motor_commission_validate_current, 1, 4),
	SHELL_CMD_ARG(velocity, NULL,
		      "Smoke-test velocity_encoder response [max_hz] [hold_ms] [active]",
		      cmd_motor_commission_validate_velocity, 1, 3),
	SHELL_CMD_ARG(position, NULL, "Smoke-test position_encoder profiled move [delta_deg] [hold_ms]",
		      cmd_motor_commission_validate_position, 1, 2),
	SHELL_SUBCMD_SET_END
);

/* motor commission subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission,
	SHELL_CMD_ARG(run, NULL, "Run standard restart commissioning workflow [slow|confirm] [apply]",
		      cmd_motor_commission_run, 1, 2),
	SHELL_CMD(status, NULL, "Show commissioning status and capture stats", cmd_motor_commission_status),
	SHELL_CMD(clear, NULL, "Clear commissioning context and captured data", cmd_motor_commission_clear),
	SHELL_CMD(abort, NULL, "Abort active commissioning run", cmd_motor_commission_abort),
	SHELL_CMD(apply, NULL, "Apply valid commissioning estimates to active runtime params", cmd_motor_commission_apply),
	SHELL_CMD_ARG(boot, NULL,
		      "Run runtime boot gate: current offsets + commutation map/apply",
		      cmd_motor_commission_boot, 1, 3),
	SHELL_CMD(motion, &sub_motor_commission_motion, "Motion threshold commissioning", NULL),
	SHELL_CMD(flux, &sub_motor_commission_flux, "Flux-linkage commissioning", NULL),
	SHELL_CMD(mech, &sub_motor_commission_mech, "Mechanical commissioning", NULL),
	SHELL_CMD(encoder, &sub_motor_commission_encoder, "Encoder commutation mapping", NULL),
	SHELL_CMD(detent, &sub_motor_commission_detent, "Detent feedforward commissioning", NULL),
	SHELL_CMD(electrical, &sub_motor_commission_electrical,
		  "Production electrical identification", NULL),
	SHELL_CMD(auto, &sub_motor_commission_auto, "One-command identify+tune workflow", NULL),
	SHELL_CMD(validate, &sub_motor_commission_validate, "Encoder-mode smoke validation", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor settings autoload subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_settings_autoload,
	SHELL_CMD(status, NULL, "Show settings autoload status", cmd_motor_settings_autoload_status),
	SHELL_SUBCMD_SET_END
);

/* motor settings subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_settings,
	SHELL_CMD(status, NULL, "Show persisted motor settings status", cmd_motor_settings_status),
	SHELL_CMD(preview, NULL, "Preview persisted motor settings", cmd_motor_settings_preview),
	SHELL_CMD(save, NULL,
		  "Save settings [baseline|encoder|identity|model|limits|controllers|detent|all]",
		  cmd_motor_settings_save),
	SHELL_CMD(load, NULL,
		  "Load settings [baseline|encoder|identity|model|limits|controllers|detent|all]",
		  cmd_motor_settings_load),
	SHELL_CMD(clear, NULL,
		  "Clear settings [baseline|encoder|identity|model|limits|controllers|detent|all]",
		  cmd_motor_settings_clear),
	SHELL_CMD(autoload, &sub_motor_settings_autoload, "Settings autoload controls", NULL),
	SHELL_SUBCMD_SET_END
);

/* Top-level motor command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD(params, &sub_motor_params, "Parameter access", NULL),
	SHELL_CMD(current, &sub_motor_current, "Current control", NULL),
	SHELL_CMD(state, &sub_motor_state, "State machine control", NULL),
	SHELL_CMD(arm, NULL, "Arm torque-producing control output", cmd_motor_arm),
	SHELL_CMD(disarm, NULL, "Disarm control output and request IDLE", cmd_motor_disarm),
	SHELL_CMD(safety, &sub_motor_safety, "Safety interlock and timeout", NULL),
	SHELL_CMD(gate, &sub_motor_gate, "Gate-driver diagnostics/recovery", NULL),
	SHELL_CMD(fault, &sub_motor_fault, "Fault diagnostics", NULL),
	SHELL_CMD(info, &sub_motor_info, "Motor information", NULL),
	SHELL_CMD(settings, &sub_motor_settings, "Persistent motor settings", NULL),
#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
	SHELL_CMD(rls, &sub_motor_rls, "RLS parameter estimation", NULL),
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */
	SHELL_CMD(outer, &sub_motor_outer, "Outer-loop regulator selection", NULL),
	SHELL_CMD(control, &sub_motor_control, "Control regulator/feedforward status", NULL),
	SHELL_CMD(observer, &sub_motor_observer, "Observer status", NULL),
	SHELL_CMD(velocity, &sub_motor_velocity, "Velocity control", NULL),
	SHELL_CMD(position, &sub_motor_position, "Position control", NULL),
	SHELL_CMD(profile, &sub_motor_profile, "Motion profile settings", NULL),
	SHELL_CMD(chopper, &sub_motor_chopper, "Optical chopper utilities", NULL),
	SHELL_CMD(commission, &sub_motor_commission, "Commissioning workflows", NULL),
	SHELL_CMD(encoder, &sub_motor_encoder, "Encoder diagnostics", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &sub_motor, "Motor control commands", NULL);
