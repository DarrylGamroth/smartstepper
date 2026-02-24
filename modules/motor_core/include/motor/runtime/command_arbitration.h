/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMMAND_ARBITRATION_H_
#define MOTOR_COMMAND_ARBITRATION_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

struct motor_command_arbitration_input {
	bool online_control_state;
	bool feature_angle_gen;
	bool feature_use_commanded_currents;
	bool feedback_valid;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t id_setpoint_a;
	float32_t iq_setpoint_a;
	float32_t id_ref_in_a;
	float32_t iq_ref_in_a;
};

struct motor_command_arbitration_output {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	bool reset_current_pi;
};

void motor_command_arbitration_apply(
	const struct motor_command_arbitration_input *in,
	struct motor_command_arbitration_output *out);

#endif /* MOTOR_COMMAND_ARBITRATION_H_ */
