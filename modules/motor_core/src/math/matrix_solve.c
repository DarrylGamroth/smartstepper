/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/math/matrix_solve.h"

#include <math.h>

#define MOTOR_MATH_SOLVE_PIVOT_EPS 1.0e-9f

bool motor_math_solve_linear_4x4(float32_t A[4][4], float32_t b[4], float32_t x[4])
{
	float32_t aug[4][5];

	for (uint32_t i = 0U; i < 4U; i++) {
		for (uint32_t j = 0U; j < 4U; j++) {
			aug[i][j] = A[i][j];
		}
		aug[i][4] = b[i];
	}

	for (uint32_t col = 0U; col < 4U; col++) {
		uint32_t pivot = col;
		float32_t pivot_abs = fabsf(aug[pivot][col]);

		for (uint32_t row = col + 1U; row < 4U; row++) {
			const float32_t a = fabsf(aug[row][col]);
			if (a > pivot_abs) {
				pivot = row;
				pivot_abs = a;
			}
		}

		if (pivot_abs < MOTOR_MATH_SOLVE_PIVOT_EPS) {
			return false;
		}

		if (pivot != col) {
			for (uint32_t j = col; j < 5U; j++) {
				const float32_t tmp = aug[col][j];
				aug[col][j] = aug[pivot][j];
				aug[pivot][j] = tmp;
			}
		}

		const float32_t inv_pivot = 1.0f / aug[col][col];
		for (uint32_t j = col; j < 5U; j++) {
			aug[col][j] *= inv_pivot;
		}

		for (uint32_t row = 0U; row < 4U; row++) {
			if (row == col) {
				continue;
			}

			const float32_t f = aug[row][col];
			if (f == 0.0f) {
				continue;
			}

			for (uint32_t j = col; j < 5U; j++) {
				aug[row][j] -= f * aug[col][j];
			}
		}
	}

	for (uint32_t i = 0U; i < 4U; i++) {
		x[i] = aug[i][4];
	}

	return true;
}
