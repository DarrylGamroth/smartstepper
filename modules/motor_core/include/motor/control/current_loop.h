/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CURRENT_LOOP_H_
#define MOTOR_CURRENT_LOOP_H_

#include <zephyr/dsp/types.h>

#include "motor/filters/pi.h"

struct motor_current_loop_input {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	float32_t id_a;
	float32_t iq_a;
	float32_t max_voltage_magnitude_v;
	float32_t vd_ff_v;
	float32_t vq_ff_v;
};

struct motor_current_loop_output {
	float32_t vd_v;
	float32_t vq_v;
	float32_t vq_limit_v;
};

int motor_current_loop_step(struct pi_f32 *pi_id, struct pi_f32 *pi_iq,
			    const struct motor_current_loop_input *in,
			    struct motor_current_loop_output *out);

#endif /* MOTOR_CURRENT_LOOP_H_ */
