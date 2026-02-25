/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config.h"
#include "motor/telemetry/capture.h"

/* Phase P03 scaffold helper. Kept out of the ISR callback wiring for now. */
void motor_control_telemetry_refresh_diag(struct motor_parameters *params)
{
	if (params == NULL) {
		return;
	}

	struct motor_rt_diag_state *diag = &params->rt_diag;
	diag->state_counter = params->state_counter;
	diag->encoder_fault_counter = params->encoder_fault_counter;
	diag->encoder_warning_count = params->encoder_warning_count;
	diag->encoder_error_count = params->encoder_error_count;
	diag->max_isr_cycles = params->max_isr_cycles;
	diag->total_isr_cycles = params->total_isr_cycles;
	diag->overrun_count = params->overrun_count;
	diag->encoder_capture_overrun_count = params->encoder_capture_overrun_count;
	diag->fault_snapshot_overrun_count = params->fault_snapshot_overrun_count;
	diag->fault_snapshot_latch_loop = params->fault_snapshot_latch_loop;
	diag->fault_snapshot_latch_error_code = params->fault_snapshot_latch_error_code;
	diag->command_timeout_count = params->command_timeout_count;
	diag->profile_sequence_event_drop_count = params->profile_sequence_event_drop_count;
}

/* Optional capture-path hook for extended observer/debug payloads. */
void motor_control_telemetry_consume_capture(const struct motor_capture_feedback *capture)
{
	ARG_UNUSED(capture);
}
