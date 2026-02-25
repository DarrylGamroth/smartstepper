/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_DQ_DECOUPLING_H_
#define MOTOR_DQ_DECOUPLING_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

struct motor_dq_decoupling_enable_input {
	bool feature_enabled;
	bool online_control_state;
	bool control_armed;
	bool torque_mode_state;
	bool min_speed_reached;
	bool flux_valid;
	bool speed_valid;
	bool feedback_valid;
};

bool motor_dq_decoupling_is_enabled(const struct motor_dq_decoupling_enable_input *in);

struct motor_dq_decoupling_feedforward_input {
	bool enabled;
	float32_t electrical_speed_rad_s;
	float32_t ld_h;
	float32_t lq_h;
	float32_t flux_linkage_wb;
	float32_t id_a;
	float32_t iq_a;
	float32_t max_voltage_magnitude_v;
	float32_t flux_headroom_ratio;
	float32_t ff_limit_ratio;
};

struct motor_dq_decoupling_feedforward_output {
	float32_t vd_ff_v;
	float32_t vq_ff_v;
};

int motor_dq_decoupling_feedforward_step(const struct motor_dq_decoupling_feedforward_input *in,
					 struct motor_dq_decoupling_feedforward_output *out);

#endif /* MOTOR_DQ_DECOUPLING_H_ */
