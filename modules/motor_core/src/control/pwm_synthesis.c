/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/pwm_synthesis.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/control/pwmgen.h"
#include "motor/math/math_constants.h"

static inline float32_t clamp_unit_interval(float32_t value)
{
	return clampf(value, 0.0f, 1.0f);
}

int motor_pwm_synthesis_step(const struct motor_pwm_synthesis_input *in,
			     struct motor_pwm_synthesis_output *out)
{
	if (in == NULL || out == NULL) {
		return -EINVAL;
	}
	if (!isfinite(in->va_v) || !isfinite(in->vb_v) || !isfinite(in->vbus_v) || in->vbus_v <= 0.0f) {
		return -EINVAL;
	}
	if (in->braking_enabled &&
	    (!isfinite(in->braking_iq_ref_a) || !isfinite(in->braking_speed_rad_s) ||
	     !isfinite(in->braking_vbus_limit_v) || !isfinite(in->braking_vbus_margin_inv) ||
	     in->braking_vbus_limit_v < 0.0f || in->braking_vbus_margin_inv <= 0.0f)) {
		return -EINVAL;
	}

	float32_t vbus_inv = 1.0f / in->vbus_v;
	out->ua_pu = in->va_v * vbus_inv;
	out->ub_pu = in->vb_v * vbus_inv;
	if (!isfinite(out->ua_pu) || !isfinite(out->ub_pu)) {
		return -ERANGE;
	}

	pwmgen_spwm_2phase_f32(out->ua_pu, out->ub_pu, &out->da_pu, &out->db_pu);
	out->da_pu = clamp_unit_interval(out->da_pu);
	out->db_pu = clamp_unit_interval(out->db_pu);

	out->da_hb1_pu = out->da_pu;
	out->da_hb2_pu = 1.0f - out->da_pu;
	out->db_hb1_pu = out->db_pu;
	out->db_hb2_pu = 1.0f - out->db_pu;

	if (in->braking_enabled) {
		bool is_braking = (in->braking_iq_ref_a * in->braking_speed_rad_s) < 0.0f;

		if (is_braking && fabsf(in->braking_speed_rad_s) > 0.628f &&
		    in->vbus_v > in->braking_vbus_limit_v) {
			float32_t overvoltage = in->vbus_v - in->braking_vbus_limit_v;
			float32_t short_duty = fminf(1.0f, overvoltage * in->braking_vbus_margin_inv);
			float32_t scale = 1.0f - short_duty;

			out->da_hb1_pu *= scale;
			out->da_hb2_pu = out->da_hb2_pu * scale + short_duty;
			out->db_hb1_pu *= scale;
			out->db_hb2_pu = out->db_hb2_pu * scale + short_duty;
		}
	}
	out->da_hb1_pu = clamp_unit_interval(out->da_hb1_pu);
	out->da_hb2_pu = clamp_unit_interval(out->da_hb2_pu);
	out->db_hb1_pu = clamp_unit_interval(out->db_hb1_pu);
	out->db_hb2_pu = clamp_unit_interval(out->db_hb2_pu);

	return 0;
}
