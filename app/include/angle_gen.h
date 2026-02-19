/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANGLE_GEN_H
#define ANGLE_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <zephyr/dsp/types.h>
#include "math_constants.h"

/**
 * @brief Angle generator object
 * 
 * Generates a continuously incrementing angle at a specified angular velocity.
 * Useful for open-loop excitation, test signals, and reference frame generation.
 */
typedef struct {
	float32_t omega_rad_s;          //!< Current angular velocity in rad/s
	float32_t angle_delta_factor;   //!< Precalculated: ctrl_period_sec
	float32_t angle_delta_rad;      //!< Angle increment per iteration, rad
	float32_t angle_rad;            //!< Current angle output value, rad [-pi, pi]
} angle_gen_t;

/**
 * @brief Initialize angle generator
 * 
 * @param gen Pointer to angle generator structure
 * @param ctrl_period_sec Control loop period in seconds
 */
static inline void angle_gen_init(angle_gen_t *gen, float32_t ctrl_period_sec)
{
	gen->omega_rad_s = 0.0f;
	gen->angle_delta_factor = ctrl_period_sec;
	gen->angle_delta_rad = 0.0f;
	gen->angle_rad = 0.0f;
}

/**
 * @brief Set angle generator angular velocity
 * 
 * @param gen Pointer to angle generator structure
 * @param omega_rad_s Desired angular velocity in rad/s
 */
static inline void angle_gen_set_velocity(angle_gen_t *gen, float32_t omega_rad_s)
{
	gen->omega_rad_s = omega_rad_s;
	gen->angle_delta_rad = omega_rad_s * gen->angle_delta_factor;
}

/**
 * @brief Set angle generator angle directly
 * 
 * @param gen Pointer to angle generator structure
 * @param angle_rad Desired angle in radians
 */
static inline void angle_gen_set_angle(angle_gen_t *gen, float32_t angle_rad)
{
	gen->angle_rad = angle_rad;
}

/**
 * @brief Get current angle
 * 
 * @param gen Pointer to angle generator structure
 * @return Current angle in radians [-pi, pi]
 */
static inline float32_t angle_gen_get_angle(const angle_gen_t *gen)
{
	return gen->angle_rad;
}

/**
 * @brief Get current angle in degrees
 * 
 * @param gen Pointer to angle generator structure
 * @return Current angle in degrees [-180, 180]
 */
static inline float32_t angle_gen_get_angle_deg(const angle_gen_t *gen)
{
	return gen->angle_rad * (180.0f / PI_F32);
}

/**
 * @brief Run angle generator (call once per control loop iteration)
 * 
 * Increments the angle by the precalculated delta and wraps to [-pi, pi].
 * 
 * @param gen Pointer to angle generator structure
 */
static inline void angle_gen_run(angle_gen_t *gen)
{
	float32_t angle_rad = gen->angle_rad + gen->angle_delta_rad;

	/* Wrap angle to [-pi, pi] range */
	if (angle_rad > PI_F32) {
		angle_rad -= 2.0f * PI_F32;
	} else if (angle_rad < -PI_F32) {
		angle_rad += 2.0f * PI_F32;
	}

	gen->angle_rad = angle_rad;
}

#ifdef __cplusplus
}
#endif

#endif /* ANGLE_GEN_H */
