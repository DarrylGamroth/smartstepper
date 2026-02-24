/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/decoupling.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/math_constants.h"

#define DECOUPLING_FLUX_BACKEMF_HEADROOM_RATIO 0.60f
#define DECOUPLING_TOTAL_FEEDFORWARD_LIMIT_RATIO 0.70f
#define DECOUPLING_MIN_SPEED_FOR_FLUX_CLAMP_RAD_S 1.0f

bool motor_decoupling_is_enabled(const struct motor_decoupling_enable_input *in)
{
	if (in == NULL) {
		return false;
	}

	return in->feature_enabled &&
	       in->online_control_state &&
	       in->control_armed &&
	       !in->torque_mode_state &&
	       in->min_speed_reached &&
	       in->flux_valid &&
	       in->speed_valid &&
	       in->feedback_valid;
}

int motor_decoupling_feedforward_step(const struct motor_decoupling_feedforward_input *in,
				      struct motor_decoupling_feedforward_output *out)
{
	if (in == NULL || out == NULL) {
		return -EINVAL;
	}

	out->vd_ff_v = 0.0f;
	out->vq_ff_v = 0.0f;

	if (!in->enabled) {
		return 0;
	}
	if (!isfinite(in->electrical_speed_rad_s) || !isfinite(in->ld_h) || !isfinite(in->lq_h) ||
	    !isfinite(in->flux_linkage_wb) || !isfinite(in->id_a) || !isfinite(in->iq_a) ||
	    !isfinite(in->max_voltage_magnitude_v) || in->max_voltage_magnitude_v <= 0.0f) {
		return -EINVAL;
	}

	float32_t omega_elec = in->electrical_speed_rad_s;
	float32_t ld_h = fmaxf(in->ld_h, 0.0f);
	float32_t lq_h = fmaxf(in->lq_h, 0.0f);
	float32_t psi_f_wb = in->flux_linkage_wb;
	if (psi_f_wb < 0.0f) {
		psi_f_wb = 0.0f;
	}

	float32_t omega_abs = fabsf(omega_elec);
	if (omega_abs > DECOUPLING_MIN_SPEED_FOR_FLUX_CLAMP_RAD_S) {
		float32_t psi_f_max_wb = (DECOUPLING_FLUX_BACKEMF_HEADROOM_RATIO *
					  in->max_voltage_magnitude_v) /
					 omega_abs;
		psi_f_wb = fminf(psi_f_wb, psi_f_max_wb);
	}

	out->vd_ff_v = -(omega_elec * lq_h * in->iq_a);
	out->vq_ff_v = omega_elec * ((ld_h * in->id_a) + psi_f_wb);

	float32_t ff_limit_v =
		DECOUPLING_TOTAL_FEEDFORWARD_LIMIT_RATIO * in->max_voltage_magnitude_v;
	float32_t ff_mag_sq = (out->vd_ff_v * out->vd_ff_v) + (out->vq_ff_v * out->vq_ff_v);
	if (ff_limit_v > 0.0f && ff_mag_sq > (ff_limit_v * ff_limit_v)) {
		float32_t scale = ff_limit_v / sqrtf(ff_mag_sq);
		out->vd_ff_v *= scale;
		out->vq_ff_v *= scale;
	}

	return 0;
}
