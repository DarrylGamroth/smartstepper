/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/transforms.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#if defined(CONFIG_ARCH_POSIX)
void arm_sin_cos_f32(float32_t theta, float32_t *pSinVal, float32_t *pCosVal);
void arm_park_f32(float32_t i_alpha, float32_t i_beta, float32_t *pId, float32_t *pIq,
		  float32_t sinVal, float32_t cosVal);
void arm_inv_park_f32(float32_t id, float32_t iq, float32_t *pAlpha, float32_t *pBeta,
		      float32_t sinVal, float32_t cosVal);
#else
#include <dsp/controller_functions.h>
#endif

#include "motor/math/math_constants.h"

int motor_transforms_park(float32_t ia_a,
			  float32_t ib_a,
			  float32_t elec_angle_rad,
			  float32_t *id_a,
			  float32_t *iq_a)
{
	if (!isfinite(ia_a) || !isfinite(ib_a) || !isfinite(elec_angle_rad) || id_a == NULL ||
	    iq_a == NULL) {
		return -EINVAL;
	}

	float32_t sin_theta = 0.0f;
	float32_t cos_theta = 0.0f;
	float32_t angle_deg = elec_angle_rad * (180.0f / PI_F32);
	arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
	arm_park_f32(ia_a, ib_a, id_a, iq_a, sin_theta, cos_theta);
	if (!isfinite(*id_a) || !isfinite(*iq_a)) {
		return -ERANGE;
	}

	return 0;
}

int motor_transforms_inv_park(float32_t vd_v,
			      float32_t vq_v,
			      float32_t elec_angle_rad,
			      float32_t *va_v,
			      float32_t *vb_v)
{
	if (!isfinite(vd_v) || !isfinite(vq_v) || !isfinite(elec_angle_rad) || va_v == NULL ||
	    vb_v == NULL) {
		return -EINVAL;
	}

	float32_t sin_theta = 0.0f;
	float32_t cos_theta = 0.0f;
	float32_t angle_deg = elec_angle_rad * (180.0f / PI_F32);
	arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);
	arm_inv_park_f32(vd_v, vq_v, va_v, vb_v, sin_theta, cos_theta);
	if (!isfinite(*va_v) || !isfinite(*vb_v)) {
		return -ERANGE;
	}

	return 0;
}
