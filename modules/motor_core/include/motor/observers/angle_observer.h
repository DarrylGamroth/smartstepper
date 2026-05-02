/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANGLE_OBSERVER_H_
#define ANGLE_OBSERVER_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>
#include "motor/math/math_constants.h"

/**
 * @brief Angle observer state structure
 *
 * Contains all state for the angle tracking observer, including current and
 * predicted mechanical/electrical angles and mechanical speed. All angles are
 * in radians internally for consistency with control math.
 *
 * This structure is not thread-safe; callers must ensure exclusive access.
 */
struct angle_observer_state {
	/* Current-cycle outputs */
	float32_t mech_angle_rad;        /**< Filtered mechanical angle [0, 2π) */
	float32_t elec_angle_rad;        /**< Filtered electrical angle [0, 2π) */
	float32_t mech_speed_rad_s;      /**< Mechanical speed in rad/s */

	/* One-step prediction outputs (for next control cycle) */
	float32_t mech_angle_pred_rad;   /**< Predicted mechanical angle [0, 2π) */
	float32_t elec_angle_pred_rad;   /**< Predicted electrical angle [0, 2π) */

	/* Internal state */
	float32_t angle_est_rad;         /**< Internal unwrapped angle estimate */
	float32_t speed_est_rad_s;       /**< Internal speed estimate in rad/s */

	/* Cached configuration and gains */
	float32_t sample_period_s;       /**< Cached control loop period */
	float32_t bandwidth_hz;          /**< Cached observer bandwidth */
	uint32_t  pole_pairs;            /**< Cached motor pole pairs */
	float32_t L1Ts;                  /**< Cached position gain × Ts (2*wo*Ts) */
	float32_t L2Ts;                  /**< Cached velocity gain × Ts (wo^2*Ts) */
	float32_t mech_angle_offset_rad; /**< Mechanical angle offset from alignment (for NV storage) */
	float32_t delay_samples;         /**< Encoder measurement delay in sample periods (e.g., 1.0 for pipelined SPI) */
};

/**
 * @brief Initialize the angle observer
 *
 * Must be called before angle_observer_update(). Sets all internal state and
 * outputs to zero and caches configuration parameters.
 *
 * @param obs Pointer to observer state structure
 * @param sample_period_s Control loop sample period in seconds
 * @param bandwidth_hz Observer bandwidth in Hz (typically 50-300 Hz)
 * @param pole_pairs Motor pole pairs for electrical angle conversion
 * @param delay_samples Encoder measurement delay in samples (use 1.0 for pipelined SPI4-16)
 */
void angle_observer_init(struct angle_observer_state *obs,
			 float32_t sample_period_s,
			 float32_t bandwidth_hz,
			 uint32_t pole_pairs,
			 float32_t delay_samples);

/**
 * @brief Set the mechanical angle offset
 *
 * Sets the mechanical angle offset determined during rotor alignment.
 * This offset is applied to the mechanical angle before computing the
 * electrical angle, allowing the electrical zero to be calibrated.
 * The offset can be stored in non-volatile memory for persistence.
 *
 * @param obs Pointer to observer state structure
 * @param offset_rad Mechanical angle offset in radians
 */
void angle_observer_set_offset(struct angle_observer_state *obs,
			       float32_t offset_rad);

/**
 * @brief Set encoder measurement delay compensation
 *
 * Configures delay compensation for the angle measurement source.
 * Use 0.0 for real-time sources (angle generator) and 1.0 for pipelined
 * SPI reads (e.g., SPI4-16) where the reading is one cycle old.
 * Fractional values can be used for fine-tuning if the actual delay
 * is between sample periods.
 *
 * @param obs Pointer to observer state structure
 * @param delay_samples Measurement delay in sample periods (0.0=no delay, 1.0=one cycle old)
 */
static inline void angle_observer_set_delay(struct angle_observer_state *obs,
					    float32_t delay_samples)
{
	obs->delay_samples = ((delay_samples == delay_samples) &&
			      delay_samples >= 0.0f &&
			      delay_samples <= 16.0f) ?
				      delay_samples :
				      0.0f;
}

/**
 * @brief Update the angle observer with a new encoder measurement
 *
 * Runs one iteration of the alpha-beta tracking observer using the provided
 * encoder angle. Updates all current and predicted angle/speed outputs.
 * Applies the mechanical angle offset before computing electrical angles.
 *
 * @param obs Pointer to observer state structure
 * @param encoder_angle_rad Encoder mechanical angle in radians (approx [-π, π])
 */
void angle_observer_update(struct angle_observer_state *obs,
			   float32_t encoder_angle_rad);

/**
 * @brief Reseed observer state for mode handoff
 *
 * Resets the internal tracking state to a known mechanical angle/speed while
 * preserving configured gains and angle offset.
 *
 * @param obs Pointer to observer state structure
 * @param mech_angle_rad Mechanical angle in radians
 * @param mech_speed_rad_s Mechanical speed in rad/s
 */
void angle_observer_reset_tracking(struct angle_observer_state *obs,
				   float32_t mech_angle_rad,
				   float32_t mech_speed_rad_s);

/**
 * @brief Get current mechanical angle
 *
 * @param obs Pointer to observer state structure
 * @return Mechanical angle in radians [0, 2π)
 */
static inline float32_t angle_observer_get_mech_angle(
	const struct angle_observer_state *obs)
{
	return obs->mech_angle_rad;
}

/**
 * @brief Get predicted mechanical angle for next control cycle
 *
 * @param obs Pointer to observer state structure
 * @return Predicted mechanical angle in radians [0, 2π)
 */
static inline float32_t angle_observer_get_mech_angle_pred(
	const struct angle_observer_state *obs)
{
	return obs->mech_angle_pred_rad;
}

/**
 * @brief Get current electrical angle
 *
 * @param obs Pointer to observer state structure
 * @return Electrical angle in radians [0, 2π)
 */
static inline float32_t angle_observer_get_elec_angle(
	const struct angle_observer_state *obs)
{
	return obs->elec_angle_rad;
}

/**
 * @brief Get predicted electrical angle for next control cycle
 *
 * Used for inverse Park transform voltage calculation.
 *
 * @param obs Pointer to observer state structure
 * @return Predicted electrical angle in radians [0, 2π)
 */
static inline float32_t angle_observer_get_elec_angle_pred(
	const struct angle_observer_state *obs)
{
	return obs->elec_angle_pred_rad;
}

/**
 * @brief Get mechanical angular velocity
 *
 * @param obs Pointer to observer state structure
 * @return Mechanical angular velocity in rad/s
 */
static inline float32_t angle_observer_get_mech_speed(
	const struct angle_observer_state *obs)
{
	return obs->mech_speed_rad_s;
}

/**
 * @brief Get electrical angular velocity in rad/s
 *
 * Converts mechanical speed to electrical angular velocity for RLS cross-coupling compensation.
 * Formula: ω_elec = ω_mech × pole_pairs
 *
 * @param obs Pointer to observer state structure
 * @return Electrical angular velocity in rad/s
 */
static inline float32_t angle_observer_get_elec_speed(
	const struct angle_observer_state *obs)
{
	return obs->mech_speed_rad_s * obs->pole_pairs;
}

#endif /* ANGLE_OBSERVER_H_ */
