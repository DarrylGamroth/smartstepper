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
	if (!isfinite(in->id_ref_a) || !isfinite(in->iq_ref_a) || !isfinite(in->id_a) ||
	    !isfinite(in->iq_a) || !isfinite(in->vd_ff_v) || !isfinite(in->vq_ff_v) ||
	    !isfinite(in->max_voltage_magnitude_v) || in->max_voltage_magnitude_v <= 0.0f) {
		return -EINVAL;
	}

	pi_set_min_max(pi_id, -in->max_voltage_magnitude_v, in->max_voltage_magnitude_v);
	pi_run_series(pi_id, in->id_ref_a, in->id_a, in->vd_ff_v, &out->vd_v);
	if (!isfinite(out->vd_v)) {
		return -ERANGE;
	}

	float32_t vq_limit_sq = (in->max_voltage_magnitude_v * in->max_voltage_magnitude_v) -
				(out->vd_v * out->vd_v);
	if (vq_limit_sq < 0.0f) {
		vq_limit_sq = 0.0f;
	}
	out->vq_limit_v = sqrtf(vq_limit_sq);
	pi_set_min_max(pi_iq, -out->vq_limit_v, out->vq_limit_v);
	pi_run_series(pi_iq, in->iq_ref_a, in->iq_a, in->vq_ff_v, &out->vq_v);
	if (!isfinite(out->vq_v)) {
		return -ERANGE;
	}

	return 0;
}
