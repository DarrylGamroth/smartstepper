/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_OUTER_LOOP_RUNTIME_H_
#define MOTOR_RUNTIME_OUTER_LOOP_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_parameters;

struct motor_outer_loop_inputs {
	bool position_active;
	bool velocity_active;
	bool feature_angle_gen;
	bool feature_velocity_traj;
	uint32_t velocity_loop_decimation;
	uint32_t position_loop_decimation;
	float32_t velocity_loop_dt_s;
	float32_t position_loop_dt_s;
	float32_t position_mech_rad;
	float32_t speed_mech_rad_s;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

struct motor_outer_loop_outputs {
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t speed_mech_filtered_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

int motor_outer_loop_runtime_step(struct motor_parameters *params,
				  const struct motor_outer_loop_inputs *in,
				  struct motor_outer_loop_outputs *out);

#endif /* MOTOR_RUNTIME_OUTER_LOOP_RUNTIME_H_ */
