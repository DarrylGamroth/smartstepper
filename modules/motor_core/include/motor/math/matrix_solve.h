/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MATRIX_SOLVE_H_
#define MATRIX_SOLVE_H_

#include <stdbool.h>

#include <zephyr/dsp/utils.h>

bool motor_math_solve_linear_4x4(float32_t A[4][4], float32_t b[4], float32_t x[4]);

#endif /* MATRIX_SOLVE_H_ */
