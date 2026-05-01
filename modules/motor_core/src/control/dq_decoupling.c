/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/dq_decoupling.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/math_constants.h"

#define DQ_DECOUPLING_DEFAULT_FLUX_HEADROOM_RATIO 0.60f
#define DQ_DECOUPLING_DEFAULT_FF_LIMIT_RATIO 0.70f
#define DQ_DECOUPLING_MIN_SPEED_FOR_FLUX_CLAMP_RAD_S 1.0f

bool motor_dq_decoupling_is_enabled(const struct motor_dq_decoupling_enable_input *in)
{
	if (in == NULL) {
		return false;
	}

	return in->feature_enabled &&
	       in->online_control_state &&
	       in->control_armed &&
	       !in->current_encoder_mode_state &&
	       in->min_speed_reached &&
	       in->flux_valid &&
	       in->speed_valid &&
	       in->feedback_valid;
}

int motor_dq_decoupling_feedforward_step(const struct motor_dq_decoupling_feedforward_input *in,
					 struct motor_dq_decoupling_feedforward_output *out)
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

	return motor_dq_decoupling_feedforward_step_fast(in, out);
}

int motor_dq_decoupling_feedforward_step_fast(
	const struct motor_dq_decoupling_feedforward_input *in,
	struct motor_dq_decoupling_feedforward_output *out)
{
	if (in == NULL || out == NULL) {
		return -EINVAL;
	}

	return motor_dq_decoupling_feedforward_step_fast_values(in->enabled,
								in->electrical_speed_rad_s,
								in->ld_h,
								in->lq_h,
								in->flux_linkage_wb,
								in->id_a,
								in->iq_a,
								in->max_voltage_magnitude_v,
								in->flux_headroom_ratio,
								in->ff_limit_ratio,
								&out->vd_ff_v,
								&out->vq_ff_v);
}

int motor_dq_decoupling_feedforward_step_fast_values(bool enabled,
						     float32_t electrical_speed_rad_s,
						     float32_t ld_h,
						     float32_t lq_h,
						     float32_t flux_linkage_wb,
						     float32_t id_a,
						     float32_t iq_a,
						     float32_t max_voltage_magnitude_v,
						     float32_t flux_headroom_ratio,
						     float32_t ff_limit_ratio,
						     float32_t *vd_ff_v,
						     float32_t *vq_ff_v)
{
	*vd_ff_v = 0.0f;
	*vq_ff_v = 0.0f;

	if (!enabled) {
		return 0;
	}

	float32_t omega_elec = electrical_speed_rad_s;
	ld_h = fmaxf(ld_h, 0.0f);
	lq_h = fmaxf(lq_h, 0.0f);
	if (!isfinite(flux_headroom_ratio) || flux_headroom_ratio <= 0.0f ||
	    flux_headroom_ratio > 1.0f) {
		flux_headroom_ratio = DQ_DECOUPLING_DEFAULT_FLUX_HEADROOM_RATIO;
	}
	if (!isfinite(ff_limit_ratio) || ff_limit_ratio <= 0.0f || ff_limit_ratio > 1.0f) {
		ff_limit_ratio = DQ_DECOUPLING_DEFAULT_FF_LIMIT_RATIO;
	}
	float32_t psi_f_wb = flux_linkage_wb;
	if (psi_f_wb < 0.0f) {
		psi_f_wb = 0.0f;
	}

	float32_t omega_abs = fabsf(omega_elec);
	if (omega_abs > DQ_DECOUPLING_MIN_SPEED_FOR_FLUX_CLAMP_RAD_S) {
		float32_t psi_f_max_wb = (flux_headroom_ratio * max_voltage_magnitude_v) /
					 omega_abs;
		psi_f_wb = fminf(psi_f_wb, psi_f_max_wb);
	}

	*vd_ff_v = -(omega_elec * lq_h * iq_a);
	*vq_ff_v = omega_elec * ((ld_h * id_a) + psi_f_wb);

	float32_t ff_limit_v = ff_limit_ratio * max_voltage_magnitude_v;
	float32_t ff_mag_sq = (*vd_ff_v * *vd_ff_v) + (*vq_ff_v * *vq_ff_v);
	if (ff_limit_v > 0.0f && ff_mag_sq > (ff_limit_v * ff_limit_v)) {
		float32_t scale = ff_limit_v / sqrtf(ff_mag_sq);
		*vd_ff_v *= scale;
		*vq_ff_v *= scale;
	}

	return 0;
}
