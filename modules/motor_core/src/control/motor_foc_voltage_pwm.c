/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/motor_foc_voltage_pwm.h"

#include <errno.h>
#include <math.h>

#if defined(CONFIG_ARCH_POSIX)
/* Native_sim unit tests provide local stubs for these CMSIS DSP helpers. */
void arm_sin_cos_f32(float32_t theta, float32_t *pSinVal, float32_t *pCosVal);
void arm_inv_park_f32(float32_t id, float32_t iq, float32_t *pAlpha, float32_t *pBeta,
		      float32_t sinVal, float32_t cosVal);
#else
#include <dsp/controller_functions.h>
#endif

#include "motor/control/pwmgen.h"
#include "motor/math/math_constants.h"

#define DECOUPLING_FLUX_BACKEMF_HEADROOM_RATIO 0.60f
#define DECOUPLING_TOTAL_FEEDFORWARD_LIMIT_RATIO 0.70f
#define DECOUPLING_MIN_SPEED_FOR_FLUX_CLAMP_RAD_S 1.0f

static inline float32_t clamp_unit_interval(float32_t value)
{
	return clampf(value, 0.0f, 1.0f);
}

int motor_foc_voltage_pwm_step(struct pi_f32 *pi_id, struct pi_f32 *pi_iq,
			       const struct motor_foc_voltage_pwm_inputs *in,
			       struct motor_foc_voltage_pwm_outputs *out)
{
	if (pi_id == NULL || pi_iq == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}
	if (!isfinite(in->vbus_v) || in->vbus_v <= 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(in->max_modulation_index) || in->max_modulation_index <= 0.0f ||
	    in->max_modulation_index > 1.0f) {
		return -EINVAL;
	}
	if (!isfinite(in->inv_park_angle_rad) || !isfinite(in->id_ref_a) || !isfinite(in->iq_ref_a) ||
	    !isfinite(in->id_a) || !isfinite(in->iq_a)) {
		return -EINVAL;
	}
	if (in->decoupling_enabled && !isfinite(in->electrical_speed_rad_s)) {
		return -EINVAL;
	}
	if (in->braking_enabled &&
	    (!isfinite(in->braking_iq_ref_a) || !isfinite(in->braking_speed_rad_s) ||
	     !isfinite(in->braking_vbus_limit_v) || !isfinite(in->braking_vbus_margin_inv) ||
	     in->braking_vbus_limit_v < 0.0f || in->braking_vbus_margin_inv <= 0.0f)) {
		return -EINVAL;
	}

	float32_t sin_theta;
	float32_t cos_theta;
	float32_t ctrl_angle_deg;
	float32_t vd_ff_v = 0.0f;
	float32_t vq_ff_v = 0.0f;
	float32_t max_voltage_magnitude_v = in->max_modulation_index * in->vbus_v;
	if (!isfinite(max_voltage_magnitude_v) || max_voltage_magnitude_v <= 0.0f) {
		return -ERANGE;
	}

	if (in->decoupling_enabled) {
		float32_t omega_elec = in->electrical_speed_rad_s;
		float32_t ld_h = fmaxf(in->ld_h, 0.0f);
		float32_t lq_h = fmaxf(in->lq_h, 0.0f);
		float32_t psi_f_wb = in->flux_linkage_wb;

		if (!isfinite(psi_f_wb) || psi_f_wb < 0.0f) {
			psi_f_wb = 0.0f;
		}

		float32_t omega_abs = fabsf(omega_elec);
		if (omega_abs > DECOUPLING_MIN_SPEED_FOR_FLUX_CLAMP_RAD_S) {
			float32_t psi_f_max_wb =
				(DECOUPLING_FLUX_BACKEMF_HEADROOM_RATIO * max_voltage_magnitude_v) /
				omega_abs;
			psi_f_wb = fminf(psi_f_wb, psi_f_max_wb);
		}

		vd_ff_v = -(omega_elec * lq_h * in->iq_a);
		vq_ff_v = omega_elec * ((ld_h * in->id_a) + psi_f_wb);

		/* Keep feedforward bounded so PI still has regulation headroom. */
		float32_t ff_limit_v =
			DECOUPLING_TOTAL_FEEDFORWARD_LIMIT_RATIO * max_voltage_magnitude_v;
		float32_t ff_mag_sq = vd_ff_v * vd_ff_v + vq_ff_v * vq_ff_v;
		if (ff_limit_v > 0.0f && ff_mag_sq > (ff_limit_v * ff_limit_v)) {
			float32_t scale = ff_limit_v / sqrtf(ff_mag_sq);
			vd_ff_v *= scale;
			vq_ff_v *= scale;
		}
	}

	out->max_voltage_magnitude_v = max_voltage_magnitude_v;
	out->vd_ff_v = vd_ff_v;
	out->vq_ff_v = vq_ff_v;
	pi_set_min_max(pi_id, -out->max_voltage_magnitude_v, out->max_voltage_magnitude_v);
	pi_run_series(pi_id, in->id_ref_a, in->id_a, vd_ff_v, &out->vd_v);
	if (!isfinite(out->vd_v)) {
		return -ERANGE;
	}

	float32_t vq_limit_sq = (out->max_voltage_magnitude_v * out->max_voltage_magnitude_v) -
				(out->vd_v * out->vd_v);
	if (vq_limit_sq < 0.0f) {
		vq_limit_sq = 0.0f;
	}
	out->vq_limit_v = sqrtf(vq_limit_sq);
	pi_set_min_max(pi_iq, -out->vq_limit_v, out->vq_limit_v);
	pi_run_series(pi_iq, in->iq_ref_a, in->iq_a, vq_ff_v, &out->vq_v);
	if (!isfinite(out->vq_v)) {
		return -ERANGE;
	}

	/* CMSIS arm_sin_cos_f32 expects angle in degrees. */
	ctrl_angle_deg = in->inv_park_angle_rad * (180.0f / PI_F32);
	arm_sin_cos_f32(ctrl_angle_deg, &sin_theta, &cos_theta);
	arm_inv_park_f32(out->vd_v, out->vq_v, &out->va_v, &out->vb_v, sin_theta, cos_theta);
	if (!isfinite(out->va_v) || !isfinite(out->vb_v)) {
		return -ERANGE;
	}

	float32_t vbus_inv = 1.0f / in->vbus_v;
	out->ua_pu = out->va_v * vbus_inv;
	out->ub_pu = out->vb_v * vbus_inv;
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

		/* Ignore very low speed near zero crossing (0.628 rad/s = 0.1 Hz). */
		if (is_braking && fabsf(in->braking_speed_rad_s) > 0.628f &&
		    in->vbus_v > in->braking_vbus_limit_v) {
			float32_t overvoltage = in->vbus_v - in->braking_vbus_limit_v;
			float32_t short_duty =
				fminf(1.0f, overvoltage * in->braking_vbus_margin_inv);
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
