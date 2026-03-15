/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_TELEMETRY_H_
#define MOTOR_CONTROL_TELEMETRY_H_

#include "motor/observers/feedback.h"
#include "motor/telemetry/capture.h"
#include "motor_control_loop.h"

struct motor_parameters;

void motor_control_telemetry_refresh_diag(struct motor_parameters *params);

void motor_control_telemetry_consume_capture(const struct motor_capture_feedback *capture);

void motor_control_telemetry_store_encoder_capture(struct motor_parameters *params,
						   const struct motor_capture_feedback *capture);

void motor_control_telemetry_store_encoder_raw_trace(
	struct motor_parameters *params,
	const struct motor_control_encoder_sample *raw_sample,
	const struct motor_control_feedback *control_fb,
	uint8_t position_quality_flags);

void motor_control_telemetry_store_fault_snapshot(
	struct motor_parameters *params,
	const struct motor_control_fault_snapshot *snapshot);

#endif /* MOTOR_CONTROL_TELEMETRY_H_ */
