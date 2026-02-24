/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <zephyr/dsp/types.h>

#include "motor/math/math_constants.h"

void arm_sin_cos_f32(float32_t theta, float32_t *pSinVal, float32_t *pCosVal)
{
	const float32_t theta_rad = theta * (PI_F32 / 180.0f);
	if (pSinVal != NULL) {
		*pSinVal = sinf(theta_rad);
	}
	if (pCosVal != NULL) {
		*pCosVal = cosf(theta_rad);
	}
}

void arm_inv_park_f32(float32_t id, float32_t iq, float32_t *pAlpha, float32_t *pBeta,
		      float32_t sinVal, float32_t cosVal)
{
	if (pAlpha != NULL) {
		*pAlpha = (id * cosVal) - (iq * sinVal);
	}
	if (pBeta != NULL) {
		*pBeta = (id * sinVal) + (iq * cosVal);
	}
}
