/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_IO_H_
#define MOTOR_RUNTIME_IO_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/observers/feedback.h"
#include "motor/telemetry/capture.h"

struct motor_control_encoder_sample {
	bool enabled;
	bool fresh;
	bool warning;
	bool error;
	bool io_fault;
	uint8_t status;
	float32_t angle_deg;
};

struct motor_control_pwm_output {
	bool update_pwm;
	float32_t da_hb1_pu;
	float32_t da_hb2_pu;
	float32_t db_hb1_pu;
	float32_t db_hb2_pu;
};

struct motor_control_fault_snapshot {
	bool valid;
	float32_t encoder_angle_deg;
	float32_t observer_input_rad;
	float32_t elec_angle_rad;
	float32_t observer_elec_speed_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
	float32_t id_a;
	float32_t iq_a;
	float32_t ia_a;
	float32_t ib_a;
	float32_t vd_v;
	float32_t vq_v;
	uint8_t input_source;
	uint8_t sample_fresh;
	uint8_t sample_warning;
	uint8_t sample_error;
	uint8_t status;
	uint8_t position_quality_flags;
};

struct motor_control_step_report {
	bool error_pending;
	uint32_t error_code;
	bool encoder_capture_valid;
	bool encoder_raw_trace_valid;
	struct motor_control_encoder_sample encoder_sample;
	struct motor_control_feedback encoder_feedback;
	struct motor_capture_feedback encoder_capture;
	uint8_t position_quality_flags;
	struct motor_control_fault_snapshot fault_snapshot;
};

#endif /* MOTOR_RUNTIME_IO_H_ */
