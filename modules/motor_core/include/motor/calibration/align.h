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

struct motor_align_config {
	float32_t pole_pairs;
	float32_t opposed_elec_tol_rad;
};

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

struct motor_align_dual_result {
	bool valid;
	float32_t pos_mech_rad;
	float32_t neg_mech_rad;
	float32_t final_offset_rad;
	float32_t measured_delta_mech_rad;
	float32_t expected_delta_mech_rad;
};

struct motor_align_offset_result {
	bool dual_solution_available;
	bool dual_solution_valid;
	float32_t pos_mech_rad;
	float32_t neg_mech_rad;
	float32_t final_offset_rad;
	float32_t measured_delta_mech_rad;
	float32_t expected_delta_mech_rad;
};

int motor_align_plan_id_traj(struct traj_f32 *traj,
			     float32_t id_target_a,
			     float32_t inject_duration_s,
			     float32_t control_hz,
			     struct motor_align_traj_plan *out);

float32_t motor_align_fallback_offset_from_mech(float32_t mech_angle_rad);

void motor_align_accum_reset(struct motor_align_sample_accum *acc);

void motor_align_accum_push(struct motor_align_sample_accum *acc, float32_t mech_angle_rad);

bool motor_align_circular_mean(const struct motor_align_sample_accum *acc, float32_t *mean_rad);

bool motor_align_compute_dual_polarity(const struct motor_align_config *cfg,
				       const struct motor_align_sample_accum *pos,
				       const struct motor_align_sample_accum *neg,
				       struct motor_align_dual_result *out);

bool motor_align_resolve_offset(const struct motor_align_config *cfg,
				const struct motor_align_sample_accum *pos,
				const struct motor_align_sample_accum *neg,
				struct motor_align_offset_result *out);

#endif /* MOTOR_CALIBRATION_ALIGN_H_ */
