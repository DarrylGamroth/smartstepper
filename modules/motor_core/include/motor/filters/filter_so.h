/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHOPPER_FILTER_SO_H_
#define CHOPPER_FILTER_SO_H_

#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/dsp/types.h>

#include "motor/math/math_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Second-order IIR filter object (biquad)
 *
 * Difference equation:
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]
 */
struct filter_so_f32 {
	float32_t a1;
	float32_t a2;
	float32_t b0;
	float32_t b1;
	float32_t b2;
	float32_t x1;
	float32_t x2;
	float32_t y1;
	float32_t y2;
};

/**
 * @brief Initialize filter state and coefficients to zero.
 *
 * @param filter Filter instance
 */
static inline void filter_so_init(struct filter_so_f32 *filter)
{
	memset(filter, 0, sizeof(struct filter_so_f32));
}

/**
 * @brief Set denominator coefficients.
 *
 * @param filter Filter instance
 * @param a1 Coefficient for y[n-1]
 * @param a2 Coefficient for y[n-2]
 */
static inline void filter_so_set_den_coeffs(struct filter_so_f32 *filter, float32_t a1, float32_t a2)
{
	filter->a1 = a1;
	filter->a2 = a2;
}

/**
 * @brief Set numerator coefficients.
 *
 * @param filter Filter instance
 * @param b0 Coefficient for x[n]
 * @param b1 Coefficient for x[n-1]
 * @param b2 Coefficient for x[n-2]
 */
static inline void filter_so_set_num_coeffs(struct filter_so_f32 *filter, float32_t b0, float32_t b1,
					    float32_t b2)
{
	filter->b0 = b0;
	filter->b1 = b1;
	filter->b2 = b2;
}

/**
 * @brief Set filter delay-state initial conditions.
 *
 * @param filter Filter instance
 * @param x1 x[n-1]
 * @param x2 x[n-2]
 * @param y1 y[n-1]
 * @param y2 y[n-2]
 */
static inline void filter_so_set_initial_conditions(struct filter_so_f32 *filter, float32_t x1,
						    float32_t x2, float32_t y1, float32_t y2)
{
	filter->x1 = x1;
	filter->x2 = x2;
	filter->y1 = y1;
	filter->y2 = y2;
}

/**
 * @brief Configure filter as a unity passthrough (y=x).
 *
 * @param filter Filter instance
 */
static inline void filter_so_set_passthrough(struct filter_so_f32 *filter)
{
	filter_so_set_den_coeffs(filter, 0.0f, 0.0f);
	filter_so_set_num_coeffs(filter, 1.0f, 0.0f, 0.0f);
}

/**
 * @brief Prime delay-state values to a constant to avoid entry transients.
 *
 * @param filter Filter instance
 * @param value Initial value for x[n-1], x[n-2], y[n-1], y[n-2]
 */
static inline void filter_so_prime(struct filter_so_f32 *filter, float32_t value)
{
	filter_so_set_initial_conditions(filter, value, value, value, value);
}

/**
 * @brief Run one second-order filter step.
 *
 * @param filter Filter instance
 * @param input_value New input sample x[n]
 * @return Output sample y[n]
 */
static inline float32_t filter_so_run(struct filter_so_f32 *filter, float32_t input_value)
{
	float32_t y0;

	y0 = (filter->b0 * input_value) + (filter->b1 * filter->x1) + (filter->b2 * filter->x2) +
	     (filter->a1 * filter->y1) + (filter->a2 * filter->y2);

	filter->x2 = filter->x1;
	filter->x1 = input_value;
	filter->y2 = filter->y1;
	filter->y1 = y0;

	return y0;
}

/**
 * @brief Configure biquad coefficients as a digital notch filter.
 *
 * This uses the standard notch transfer function mapped to digital form:
 * H(z) = (1 - 2*cos(w0)z^-1 + z^-2) / (1 + a1*z^-1 + a2*z^-2)
 *
 * @param filter Filter instance
 * @param sample_rate_hz Sample rate in Hz
 * @param notch_hz Notch center frequency in Hz (0, fs/2)
 * @param q Quality factor (> 0)
 * @return 0 on success, -EINVAL or -ERANGE on invalid parameters
 */
static inline int filter_so_config_notch(struct filter_so_f32 *filter, float32_t sample_rate_hz,
					 float32_t notch_hz, float32_t q)
{
	if (filter == NULL || !isfinite(sample_rate_hz) || !isfinite(notch_hz) || !isfinite(q) ||
	    sample_rate_hz <= 0.0f || notch_hz <= 0.0f || q <= 0.0f) {
		return -EINVAL;
	}

	if (notch_hz >= (0.5f * sample_rate_hz)) {
		return -EINVAL;
	}

	const float32_t w0 = 2.0f * PI_F32 * (notch_hz / sample_rate_hz);
	const float32_t cw = cosf(w0);
	const float32_t sw = sinf(w0);
	const float32_t alpha = sw / (2.0f * q);
	const float32_t a0 = 1.0f + alpha;

	if (!isfinite(a0) || fabsf(a0) < 1.0e-8f) {
		return -ERANGE;
	}

	const float32_t inv_a0 = 1.0f / a0;
	const float32_t b0 = inv_a0;
	const float32_t b1 = -2.0f * cw * inv_a0;
	const float32_t b2 = inv_a0;
	const float32_t a1_std = -2.0f * cw * inv_a0;
	const float32_t a2_std = (1.0f - alpha) * inv_a0;

	/* Convert from standard form (subtract denominator terms) to this API form. */
	filter_so_set_num_coeffs(filter, b0, b1, b2);
	filter_so_set_den_coeffs(filter, -a1_std, -a2_std);

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* CHOPPER_FILTER_SO_H_ */
