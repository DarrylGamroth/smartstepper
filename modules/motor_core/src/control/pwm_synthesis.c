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

	return motor_pwm_synthesis_step_fast(in, out);
}

int motor_pwm_synthesis_step_fast(const struct motor_pwm_synthesis_input *in,
				  struct motor_pwm_synthesis_output *out)
{
	if (in == NULL || out == NULL) {
		return -EINVAL;
	}

	return motor_pwm_synthesis_step_fast_values(in->va_v,
						    in->vb_v,
						    in->vbus_v,
						    in->braking_enabled,
						    in->braking_iq_ref_a,
						    in->braking_speed_rad_s,
						    in->braking_vbus_limit_v,
						    in->braking_vbus_margin_inv,
						    &out->ua_pu,
						    &out->ub_pu,
						    &out->da_pu,
						    &out->db_pu,
						    &out->da_hb1_pu,
						    &out->da_hb2_pu,
						    &out->db_hb1_pu,
						    &out->db_hb2_pu);
}

int motor_pwm_synthesis_step_fast_values(float32_t va_v,
					 float32_t vb_v,
					 float32_t vbus_v,
					 bool braking_enabled,
					 float32_t braking_iq_ref_a,
					 float32_t braking_speed_rad_s,
					 float32_t braking_vbus_limit_v,
					 float32_t braking_vbus_margin_inv,
					 float32_t *ua_pu,
					 float32_t *ub_pu,
					 float32_t *da_pu,
					 float32_t *db_pu,
					 float32_t *da_hb1_pu,
					 float32_t *da_hb2_pu,
					 float32_t *db_hb1_pu,
					 float32_t *db_hb2_pu)
{
	float32_t vbus_inv = 1.0f / vbus_v;
	*ua_pu = va_v * vbus_inv;
	*ub_pu = vb_v * vbus_inv;

	pwmgen_spwm_2phase_f32(*ua_pu, *ub_pu, da_pu, db_pu);
	*da_pu = clamp_unit_interval(*da_pu);
	*db_pu = clamp_unit_interval(*db_pu);

	*da_hb1_pu = *da_pu;
	*da_hb2_pu = 1.0f - *da_pu;
	*db_hb1_pu = *db_pu;
	*db_hb2_pu = 1.0f - *db_pu;

	if (braking_enabled) {
		bool is_braking = (braking_iq_ref_a * braking_speed_rad_s) < 0.0f;

		if (is_braking && fabsf(braking_speed_rad_s) > 0.628f &&
		    vbus_v > braking_vbus_limit_v) {
			float32_t overvoltage = vbus_v - braking_vbus_limit_v;
			float32_t short_duty = fminf(1.0f, overvoltage * braking_vbus_margin_inv);
			float32_t scale = 1.0f - short_duty;

			*da_hb1_pu *= scale;
			*da_hb2_pu = *da_hb2_pu * scale + short_duty;
			*db_hb1_pu *= scale;
			*db_hb2_pu = *db_hb2_pu * scale + short_duty;
		}
	}
	*da_hb1_pu = clamp_unit_interval(*da_hb1_pu);
	*da_hb2_pu = clamp_unit_interval(*da_hb2_pu);
	*db_hb1_pu = clamp_unit_interval(*db_hb1_pu);
	*db_hb2_pu = clamp_unit_interval(*db_hb2_pu);

	return 0;
}
