/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANGLE_WRAP_H_
#define ANGLE_WRAP_H_

#include <zephyr/dsp/types.h>
#include "math_constants.h"

/**
 * @brief Wrap angle to (-pi, pi] radians
 *
 * @param angle_rad Input angle in radians
 * @return Wrapped angle in (-pi, pi]
 */
static inline float32_t wrap_rad_pi(float32_t angle_rad)
{
	while (angle_rad > PI_F32) {
		angle_rad -= 2.0f * PI_F32;
	}
	while (angle_rad <= -PI_F32) {
		angle_rad += 2.0f * PI_F32;
	}
	return angle_rad;
}

/**
 * @brief Wrap angle to [0, 2*pi) radians
 *
 * @param angle_rad Input angle in radians
 * @return Wrapped angle in [0, 2*pi)
 */
static inline float32_t wrap_rad_2pi(float32_t angle_rad)
{
	while (angle_rad >= 2.0f * PI_F32) {
		angle_rad -= 2.0f * PI_F32;
	}
	while (angle_rad < 0.0f) {
		angle_rad += 2.0f * PI_F32;
	}
	return angle_rad;
}

#endif /* ANGLE_WRAP_H_ */
