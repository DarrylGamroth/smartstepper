/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CURRENT_REF_POLICY_H_
#define MOTOR_CURRENT_REF_POLICY_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

struct motor_parameters;

struct motor_current_ref_policy_inputs {
	bool online_control_state;
	bool feature_angle_gen;
	bool feature_use_commanded_currents;
	bool control_armed;
	float32_t speed_mech_filtered_rad_s;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

struct motor_current_ref_policy_outputs {
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

int motor_current_ref_apply_policy(struct motor_parameters *params,
				   const struct motor_current_ref_policy_inputs *in,
				   struct motor_current_ref_policy_outputs *out);

#endif /* MOTOR_CURRENT_REF_POLICY_H_ */
