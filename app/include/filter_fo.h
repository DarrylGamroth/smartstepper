
/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHOPPER_FILTER_FO_H_
#define CHOPPER_FILTER_FO_H_

#include <zephyr/dsp/types.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief First-order filter object
 */
struct filter_fo_f32 {
	/** Denominator filter coefficient value for z^(-1) */
	float32_t a1;
	/** Numerator filter coefficient value for z^0 */
	float32_t b0;
	/** Numerator filter coefficient value for z^(-1) */
	float32_t b1;
	/** Input value at time sample n=-1 */
	float32_t x1;
	/** Output value at time sample n=-1 */
	float32_t y1;
};

/**
 * @brief Get the first-order filter denominator coefficient a1
 *
 * @param filter Filter instance
 * @return Filter coefficient value for z^(-1)
 */
static inline float32_t filter_fo_get_a1(const struct filter_fo_f32 *filter)
{
	return filter->a1;
}

/**
 * @brief Get the first-order filter numerator coefficient b0
 *
 * @param filter Filter instance
 * @return Filter coefficient value for z^0
 */
static inline float32_t filter_fo_get_b0(const struct filter_fo_f32 *filter)
{
	return filter->b0;
}

/**
 * @brief Get the first-order filter numerator coefficient b1
 *
 * @param filter Filter instance
 * @return Filter coefficient value for z^(-1)
 */
static inline float32_t filter_fo_get_b1(const struct filter_fo_f32 *filter)
{
	return filter->b1;
}

/**
 * @brief Get the first-order filter input value at time sample n=-1
 *
 * @param filter Filter instance
 * @return Input value at time sample n=-1
 */
static inline float32_t filter_fo_get_x1(const struct filter_fo_f32 *filter)
{
	return filter->x1;
}

/**
 * @brief Get the first-order filter output value at time sample n=-1
 *
 * @param filter Filter instance
 * @return Output value at time sample n=-1
 */
static inline float32_t filter_fo_get_y1(const struct filter_fo_f32 *filter)
{
	return filter->y1;
}

/**
 * @brief Get the first-order filter denominator coefficients
 *
 * @param filter Filter instance
 * @param a1 Pointer to store filter coefficient value for z^(-1)
 */
static inline void filter_fo_get_den_coeffs(const struct filter_fo_f32 *filter, float32_t *a1)
{
	*a1 = filter->a1;
}

/**
 * @brief Get the initial conditions of the first-order filter
 *
 * @param filter Filter instance
 * @param x1 Pointer to store input value at time sample n=-1
 * @param y1 Pointer to store output value at time sample n=-1
 */
static inline void filter_fo_get_initial_conditions(const struct filter_fo_f32 *filter,
							    float32_t *x1, float32_t *y1)
{
	*x1 = filter->x1;
	*y1 = filter->y1;
}                               

/**
 * @brief Get the first-order filter numerator coefficients
 *
 * @param filter Filter instance
 * @param b0 Pointer to store filter coefficient value for z^0
 * @param b1 Pointer to store filter coefficient value for z^(-1)
 */
static inline void filter_fo_get_num_coeffs(const struct filter_fo_f32 *filter, float32_t *b0,
						    float32_t *b1)
{
	*b0 = filter->b0;
	*b1 = filter->b1;
}

/**
 * @brief Initialize the first-order filter
 *
 * @param filter Filter instance to initialize
 */
static inline void filter_fo_init(struct filter_fo_f32 *filter)
{
	memset(filter, 0, sizeof(struct filter_fo_f32));
}

/**
 * @brief Run a first-order filter of the form y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
 *
 * @param filter Filter instance
 * @param input_value Input value to filter
 * @return Output value from the filter
 */
static inline float32_t filter_fo_run(struct filter_fo_f32 *filter, float32_t input_value)
{
	float32_t y0;

	/* Compute the output */
	y0 = (filter->b0 * input_value) + (filter->b1 * filter->x1) - (filter->a1 * filter->y1);

	/* Store values for next time */
	filter->x1 = input_value;
	filter->y1 = y0;

	return y0;
}

/**
 * @brief Run a first-order filter of the form y[n] = b0*x[n] - a1*y[n-1]
 *
 * Simplified form without the b1*x[n-1] term.
 *
 * @param filter Filter instance
 * @param input_value Input value to filter
 * @return Output value from the filter
 */
static inline float32_t filter_fo_run_form_0(struct filter_fo_f32 *filter, float32_t input_value)
{
	float32_t y0;

	/* Compute the output */
	y0 = (filter->b0 * input_value) - (filter->a1 * filter->y1);

	/* Store values for next time */
	filter->y1 = y0;

	return y0;
}

/**
 * @brief Set the first-order filter denominator coefficient a1
 *
 * @param filter Filter instance
 * @param a1 Filter coefficient value for z^(-1)
 */
static inline void filter_fo_set_a1(struct filter_fo_f32 *filter, float32_t a1)
{
	filter->a1 = a1;
}

/**
 * @brief Set the first-order filter numerator coefficient b0
 *
 * @param filter Filter instance
 * @param b0 Filter coefficient value for z^0
 */
static inline void filter_fo_set_b0(struct filter_fo_f32 *filter, float32_t b0)
{
	filter->b0 = b0;
}

/**
 * @brief Set the first-order filter numerator coefficient b1
 *
 * @param filter Filter instance
 * @param b1 Filter coefficient value for z^(-1)
 */
static inline void filter_fo_set_b1(struct filter_fo_f32 *filter, float32_t b1)
{
	filter->b1 = b1;
}

/**
 * @brief Set the first-order filter input value at time sample n=-1
 *
 * @param filter Filter instance
 * @param x1 Input value at time sample n=-1
 */
static inline void filter_fo_set_x1(struct filter_fo_f32 *filter, float32_t x1)
{
	filter->x1 = x1;
}

/**
 * @brief Set the first-order filter output value at time sample n=-1
 *
 * @param filter Filter instance
 * @param y1 Output value at time sample n=-1
 */
static inline void filter_fo_set_y1(struct filter_fo_f32 *filter, float32_t y1)
{
	filter->y1 = y1;
}

/**
 * @brief Set the first-order filter denominator coefficients
 *
 * @param filter Filter instance
 * @param a1 Filter coefficient value for z^(-1)
 */
static inline void filter_fo_set_den_coeffs(struct filter_fo_f32 *filter, float32_t a1)
{
	filter->a1 = a1;
}

/**
 * @brief Set the initial conditions of the first-order filter
 *
 * @param filter Filter instance
 * @param x1 Input value at time sample n=-1
 * @param y1 Output value at time sample n=-1
 */
static inline void filter_fo_set_initial_conditions(struct filter_fo_f32 *filter, float32_t x1,
							    float32_t y1)
{
	filter->x1 = x1;
	filter->y1 = y1;
}                               

/**
 * @brief Set the first-order filter numerator coefficients
 *
 * @param filter Filter instance
 * @param b0 Filter coefficient value for z^0
 * @param b1 Filter coefficient value for z^(-1)
 */
static inline void filter_fo_set_num_coeffs(struct filter_fo_f32 *filter, float32_t b0,
						    float32_t b1)
{
	filter->b0 = b0;
	filter->b1 = b1;
}

#ifdef __cplusplus
}
#endif

#endif /* CHOPPER_FILTER_FO_H_ */
