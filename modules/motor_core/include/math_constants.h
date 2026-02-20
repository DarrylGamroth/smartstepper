/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MATH_CONSTANTS_H_
#define MATH_CONSTANTS_H_

#include <math.h>

/**
 * @file math_constants.h
 * @brief Mathematical constants and utilities
 *
 * M_PI is not guaranteed by the C standard, so we define our own float version.
 */

#ifndef PI_F32
#define PI_F32 3.14159265358979323846f
#endif

/**
 * @brief Clamp a floating-point value between min and max
 *
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value in range [min, max]
 */
static inline float clampf(float value, float min, float max)
{
	return fmaxf(fminf(value, max), min);
}

#endif /* MATH_CONSTANTS_H_ */
