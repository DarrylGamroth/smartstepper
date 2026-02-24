/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CURRENT_REF_POLICY_CORE_H_
#define MOTOR_CURRENT_REF_POLICY_CORE_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

struct motor_current_ref_policy_core_input {
	bool online_control_state;
	bool control_armed;
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

struct motor_current_ref_policy_core_output {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	bool reset_current_pi;
	bool disarmed_interlock_active;
};

void motor_current_ref_policy_core_apply(
	const struct motor_current_ref_policy_core_input *in,
	struct motor_current_ref_policy_core_output *out);

#endif /* MOTOR_CURRENT_REF_POLICY_CORE_H_ */
