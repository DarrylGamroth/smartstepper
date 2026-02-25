/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_DIAG_H_
#define MOTOR_RUNTIME_DIAG_H_

#include <stdint.h>

/**
 * @brief Runtime state intended for slow-rate diagnostics and shell visibility.
 *
 * Update cadence:
 * 1. Written in ISR and/or state thread, read mostly by shell/diagnostic paths.
 * 2. Not required to stay in the ISR hot set.
 */
struct motor_rt_diag_state {
	uint32_t state_counter;
	uint32_t encoder_fault_counter;
	uint32_t encoder_warning_count;
	uint32_t encoder_error_count;
	uint32_t max_isr_cycles;
	uint32_t total_isr_cycles;
	uint32_t overrun_count;
	uint32_t encoder_capture_overrun_count;
	uint32_t fault_snapshot_overrun_count;
	uint32_t fault_snapshot_latch_loop;
	uint32_t fault_snapshot_latch_error_code;
	uint32_t command_timeout_count;
	uint32_t profile_sequence_event_drop_count;
};

#endif /* MOTOR_RUNTIME_DIAG_H_ */
