/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CALIBRATION_RL_IDENT_H_
#define MOTOR_CALIBRATION_RL_IDENT_H_

#include <errno.h>
#include <math.h>

#include <zephyr/dsp/types.h>

#include "motor/motion/angle_gen.h"
#include "motor/motion/traj.h"

struct motor_roverl_config {
	float32_t target_current_a;
	float32_t settling_s;
	float32_t excitation_hz;
	float32_t control_hz;
	float32_t pole_pairs;
};

struct motor_roverl_accumulator {
	float32_t vd_id_sum;
	float32_t vq_id_sum;
	float32_t id_sq_sum;
};

struct motor_roverl_result {
	float32_t rs_ohm;
	float32_t ls_h;
	float32_t r_over_l;
	float32_t tau_s;
};

int motor_roverl_plan(struct traj_f32 *traj,
		      angle_gen_t *angle_gen,
		      struct motor_roverl_accumulator *accum,
		      const struct motor_roverl_config *cfg);

int motor_roverl_prepare_scalars(struct traj_f32 *traj,
				 angle_gen_t *angle_gen,
				 const struct motor_roverl_config *cfg,
				 float32_t *vd_id_sum_out,
				 float32_t *vq_id_sum_out,
				 float32_t *id_sq_sum_out);

void motor_roverl_accumulate(struct motor_roverl_accumulator *accum,
			     float32_t vd_v,
			     float32_t vq_v,
			     float32_t id_a);

static inline void motor_roverl_accumulator_from_scalars(struct motor_roverl_accumulator *accum,
							 float32_t vd_id_sum,
							 float32_t vq_id_sum,
							 float32_t id_sq_sum)
{
	if (accum == NULL) {
		return;
	}

	accum->vd_id_sum = vd_id_sum;
	accum->vq_id_sum = vq_id_sum;
	accum->id_sq_sum = id_sq_sum;
}

static inline int motor_roverl_accumulator_to_scalars(const struct motor_roverl_accumulator *accum,
						       float32_t *vd_id_sum_out,
						       float32_t *vq_id_sum_out,
						       float32_t *id_sq_sum_out)
{
	if (accum == NULL || vd_id_sum_out == NULL ||
	    vq_id_sum_out == NULL || id_sq_sum_out == NULL) {
		return -EINVAL;
	}

	*vd_id_sum_out = accum->vd_id_sum;
	*vq_id_sum_out = accum->vq_id_sum;
	*id_sq_sum_out = accum->id_sq_sum;
	return 0;
}

static inline void motor_roverl_accumulate_scalars(float32_t *vd_id_sum_inout,
						   float32_t *vq_id_sum_inout,
						   float32_t *id_sq_sum_inout,
						   float32_t vd_v,
						   float32_t vq_v,
						   float32_t id_a)
{
	if (vd_id_sum_inout == NULL || vq_id_sum_inout == NULL ||
	    id_sq_sum_inout == NULL) {
		return;
	}
	if (!isfinite(vd_v) || !isfinite(vq_v) || !isfinite(id_a)) {
		return;
	}

	*vd_id_sum_inout += vd_v * id_a;
	*vq_id_sum_inout += vq_v * id_a;
	*id_sq_sum_inout += id_a * id_a;
}

int motor_roverl_finalize_from_scalars(float32_t vd_id_sum,
				       float32_t vq_id_sum,
				       float32_t id_sq_sum,
				       float32_t excitation_hz,
				       struct motor_roverl_result *out);

int motor_roverl_finalize(const struct motor_roverl_accumulator *accum,
			  float32_t excitation_hz,
			  struct motor_roverl_result *out);

#endif /* MOTOR_CALIBRATION_RL_IDENT_H_ */
