/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CALIBRATION_ALIGN_H_
#define MOTOR_CALIBRATION_ALIGN_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/motion/traj.h"

struct motor_align_traj_plan {
	float32_t id_start_a;
	float32_t id_target_a;
	float32_t max_delta_a_per_tick;
	float32_t steps;
};

struct motor_align_sample_accum {
	float32_t sum_sin;
	float32_t sum_cos;
	uint16_t count;
};

int motor_align_plan_id_traj(struct traj_f32 *traj,
			     float32_t id_target_a,
			     float32_t inject_duration_s,
			     float32_t control_hz,
			     struct motor_align_traj_plan *out);

float32_t motor_align_offset_from_mech_sample(float32_t mech_angle_rad);

void motor_align_accum_reset(struct motor_align_sample_accum *acc);

void motor_align_accum_push(struct motor_align_sample_accum *acc, float32_t mech_angle_rad);

bool motor_align_circular_mean(const struct motor_align_sample_accum *acc, float32_t *mean_rad);

#endif /* MOTOR_CALIBRATION_ALIGN_H_ */
