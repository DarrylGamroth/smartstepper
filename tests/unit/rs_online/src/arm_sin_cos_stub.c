/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stddef.h>
#include <zephyr/dsp/types.h>

#include "math_constants.h"

/*
 * Test-only replacement for CMSIS arm_sin_cos_f32.
 * rs_online uses degree input, matching the CMSIS helper contract.
 */
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
