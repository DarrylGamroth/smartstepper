/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHOPPER_TRAJ_H_
#define CHOPPER_TRAJ_H_

#include <zephyr/sys/util.h>
#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Trajectory generator object
 */
struct traj_f32 {
	/** Target value for the trajectory */
	float32_t target_value;
	/** Intermediate value along the trajectory */
	float32_t int_value;
	/** Minimum value for the trajectory generator */
	float32_t min_value;
	/** Maximum value for the trajectory generator */
	float32_t max_value;
	/** Maximum delta value for the trajectory generator */
	float32_t max_delta;
};

/**
 * @brief Get the intermediate value for the trajectory
 *
 * @param traj Trajectory instance
 * @return Intermediate value
 */
static inline float32_t traj_get_int_value(const struct traj_f32 *traj)
{
	return traj->int_value;
}

/**
 * @brief Get the maximum delta value for the trajectory
 *
 * @param traj Trajectory instance
 * @return Maximum delta value
 */
static inline float32_t traj_get_max_delta(const struct traj_f32 *traj)
{
	return traj->max_delta;
}

/**
 * @brief Get the maximum value for the trajectory
 *
 * @param traj Trajectory instance
 * @return Maximum value
 */
static inline float32_t traj_get_max_value(const struct traj_f32 *traj)
{
	return traj->max_value;
}

/**
 * @brief Get the minimum value for the trajectory
 *
 * @param traj Trajectory instance
 * @return Minimum value
 */
static inline float32_t traj_get_min_value(const struct traj_f32 *traj)
{
	return traj->min_value;
}

/**
 * @brief Get the target value for the trajectory
 *
 * @param traj Trajectory instance
 * @return Target value
 */
static inline float32_t traj_get_target_value(const struct traj_f32 *traj)
{
	return traj->target_value;
}

/**
 * @brief Check if trajectory has reached its target
 *
 * @param traj Trajectory instance
 * @return true if intermediate value equals target value, false otherwise
 */
static inline bool traj_is_at_target(const struct traj_f32 *traj)
{
	return traj->int_value == traj->target_value;
}

/**
 * @brief Initialize the trajectory generator
 *
 * @param traj Trajectory instance to initialize
 */
static inline void traj_init(struct traj_f32 *traj)
{
	memset(traj, 0, sizeof(struct traj_f32));
}

/**
 * @brief Set the intermediate value for the trajectory
 *
 * @param traj Trajectory instance
 * @param int_value Intermediate value
 */
static inline void traj_set_int_value(struct traj_f32 *traj, float32_t int_value)
{
	traj->int_value = int_value;
}

/**
 * @brief Set the maximum delta value for the trajectory
 *
 * @param traj Trajectory instance
 * @param max_delta Maximum delta value
 */
static inline void traj_set_max_delta(struct traj_f32 *traj, float32_t max_delta)
{
	traj->max_delta = max_delta;
}

/**
 * @brief Set the maximum value for the trajectory
 *
 * @param traj Trajectory instance
 * @param max_value Maximum value
 */
static inline void traj_set_max_value(struct traj_f32 *traj, float32_t max_value)
{
	traj->max_value = max_value;
}

/**
 * @brief Set the minimum value for the trajectory
 *
 * @param traj Trajectory instance
 * @param min_value Minimum value
 */
static inline void traj_set_min_value(struct traj_f32 *traj, float32_t min_value)
{
	traj->min_value = min_value;
}

/**
 * @brief Set the target value for the trajectory
 *
 * @param traj Trajectory instance
 * @param target_value Target value
 */
static inline void traj_set_target_value(struct traj_f32 *traj, float32_t target_value)
{
	traj->target_value = target_value;
}

/**
 * @brief Run the trajectory generator
 *
 * Increments the intermediate value toward the target value, limited by max_delta,
 * and clamps the result between min_value and max_value.
 *
 * @param traj Trajectory instance
 */
static inline void traj_run(struct traj_f32 *traj)
{
	float32_t error;
	float32_t int_value;

	error = traj->target_value - traj->int_value;

	/* Increment the value with saturation */
	int_value = traj->int_value + fmaxf(fminf(error, traj->max_delta), -traj->max_delta);

	/* Bound the value */
	traj->int_value = fmaxf(fminf(int_value, traj->max_value), traj->min_value);
}

#ifdef __cplusplus
}
#endif

#endif /* CHOPPER_TRAJ_H_ */
