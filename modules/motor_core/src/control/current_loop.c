/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/current_loop.h"

#include <errno.h>
#include <math.h>

int motor_current_loop_step(struct pi_f32 *pi_id, struct pi_f32 *pi_iq,
			    const struct motor_current_loop_input *in,
			    struct motor_current_loop_output *out)
{
	if (pi_id == NULL || pi_iq == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}
	if (in->max_voltage_magnitude_v <= 0.0f) {
		return -EINVAL;
	}

	return motor_current_loop_step_fast(pi_id, pi_iq, in, out);
}

int motor_current_loop_step_fast(struct pi_f32 *pi_id, struct pi_f32 *pi_iq,
				 const struct motor_current_loop_input *in,
				 struct motor_current_loop_output *out)
{
	return motor_current_loop_step_fast_values(pi_id, pi_iq,
						   in->id_ref_a, in->iq_ref_a,
						   in->id_a, in->iq_a,
						   in->max_voltage_magnitude_v,
						   in->vd_ff_v, in->vq_ff_v,
						   &out->vd_v, &out->vq_v,
						   &out->vq_limit_v);
}

int motor_current_loop_step_fast_values(struct pi_f32 *pi_id,
					struct pi_f32 *pi_iq,
					float32_t id_ref_a,
					float32_t iq_ref_a,
					float32_t id_a,
					float32_t iq_a,
					float32_t max_voltage_magnitude_v,
					float32_t vd_ff_v,
					float32_t vq_ff_v,
					float32_t *vd_v,
					float32_t *vq_v,
					float32_t *vq_limit_v)
{
	pi_set_min_max(pi_id, -max_voltage_magnitude_v, max_voltage_magnitude_v);
	pi_run_series(pi_id, id_ref_a, id_a, vd_ff_v, vd_v);

	float32_t v_limit_sq = max_voltage_magnitude_v * max_voltage_magnitude_v;
	float32_t vd = *vd_v;
	float32_t vq_limit_sq = v_limit_sq - (vd * vd);
	if (vq_limit_sq < 0.0f) {
		vq_limit_sq = 0.0f;
	}
	float32_t vq_limit_for_pi = sqrtf(vq_limit_sq);
	pi_set_min_max(pi_iq, -vq_limit_for_pi, vq_limit_for_pi);
	pi_run_series(pi_iq, iq_ref_a, iq_a, vq_ff_v, vq_v);

	/* Final vector clamp keeps Vdq inside the available voltage circle. */
	vd = *vd_v;
	float32_t vq = *vq_v;
	float32_t v_norm_sq = (vd * vd) + (vq * vq);
	if (v_norm_sq > v_limit_sq && v_norm_sq > 0.0f) {
		float32_t scale = max_voltage_magnitude_v / sqrtf(v_norm_sq);
		vd *= scale;
		vq *= scale;
		*vd_v = vd;
		*vq_v = vq;
	}

	float32_t vq_headroom_sq = v_limit_sq - (vd * vd);
	if (vq_headroom_sq < 0.0f) {
		vq_headroom_sq = 0.0f;
	}
	if (vq_limit_v != NULL) {
		*vq_limit_v = sqrtf(vq_headroom_sq);
	}

	return 0;
}
