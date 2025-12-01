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

/**
 * @brief Angle observer state structure
 *
 * Contains all state for the angle tracking observer, including current and
 * predicted mechanical/electrical angles and mechanical speed. All angles are
 * in degrees to match arm_sin_cos_f32 conventions.
 *
 * This structure is not thread-safe; callers must ensure exclusive access.
 */
struct angle_observer_state {
	/* Current-cycle outputs (directly usable with arm_sin_cos_f32) */
	float32_t mech_angle_deg;        /**< Filtered mechanical angle [0, 360) */
	float32_t elec_angle_deg;        /**< Filtered electrical angle [0, 360) */
	float32_t mech_speed_dps;        /**< Mechanical speed in deg/s */

	/* One-step prediction outputs (for next control cycle) */
	float32_t mech_angle_pred_deg;   /**< Predicted mechanical angle [0, 360) */
	float32_t elec_angle_pred_deg;   /**< Predicted electrical angle [0, 360) */

	/* Internal state */
	float32_t angle_est_deg;         /**< Internal unwrapped angle estimate */
	float32_t speed_est_dps;         /**< Internal speed estimate in deg/s */

	/* Cached configuration and gains */
	float32_t sample_period_s;       /**< Cached control loop period */
	float32_t bandwidth_hz;          /**< Cached observer bandwidth */
	uint32_t  pole_pairs;            /**< Cached motor pole pairs */
	float32_t L1Ts;                  /**< Cached position gain × Ts (2*wo*Ts) */
	float32_t L2Ts;                  /**< Cached velocity gain × Ts (wo^2*Ts) */
	float32_t mech_angle_offset_deg; /**< Mechanical angle offset from alignment (for NV storage) */
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
 */
void angle_observer_init(struct angle_observer_state *obs,
			 float32_t sample_period_s,
			 float32_t bandwidth_hz,
			 uint32_t pole_pairs);

/**
 * @brief Set the mechanical angle offset
 *
 * Sets the mechanical angle offset determined during rotor alignment.
 * This offset is applied to the mechanical angle before computing the
 * electrical angle, allowing the electrical zero to be calibrated.
 * The offset can be stored in non-volatile memory for persistence.
 *
 * @param obs Pointer to observer state structure
 * @param offset_deg Mechanical angle offset in degrees
 */
void angle_observer_set_offset(struct angle_observer_state *obs,
			       float32_t offset_deg);

/**
 * @brief Update the angle observer with a new encoder measurement
 *
 * Runs one iteration of the alpha-beta tracking observer using the provided
 * encoder angle. Updates all current and predicted angle/speed outputs.
 * Applies the mechanical angle offset before computing electrical angles.
 *
 * @param obs Pointer to observer state structure
 * @param encoder_angle_deg Encoder mechanical angle in degrees (approx [-180, 179])
 */
void angle_observer_update(struct angle_observer_state *obs,
			   float32_t encoder_angle_deg);

/**
 * @brief Get current mechanical angle
 *
 * @param obs Pointer to observer state structure
 * @return Mechanical angle in degrees [0, 360)
 */
static inline float32_t angle_observer_get_mech_angle_deg(
	const struct angle_observer_state *obs)
{
	return obs->mech_angle_deg;
}

/**
 * @brief Get predicted mechanical angle for next control cycle
 *
 * @param obs Pointer to observer state structure
 * @return Predicted mechanical angle in degrees [0, 360)
 */
static inline float32_t angle_observer_get_mech_angle_pred_deg(
	const struct angle_observer_state *obs)
{
	return obs->mech_angle_pred_deg;
}

/**
 * @brief Get current electrical angle
 *
 * @param obs Pointer to observer state structure
 * @return Electrical angle in degrees [0, 360)
 */
static inline float32_t angle_observer_get_elec_angle_deg(
	const struct angle_observer_state *obs)
{
	return obs->elec_angle_deg;
}

/**
 * @brief Get predicted electrical angle for next control cycle
 *
 * Used for inverse Park transform voltage calculation.
 *
 * @param obs Pointer to observer state structure
 * @return Predicted electrical angle in degrees [0, 360)
 */
static inline float32_t angle_observer_get_elec_angle_pred_deg(
	const struct angle_observer_state *obs)
{
	return obs->elec_angle_pred_deg;
}

/**
 * @brief Get mechanical speed in mechanical Hz (revolutions per second)
 *
 * Converts internal deg/s to mechanical Hz for velocity control.
 *
 * @param obs Pointer to observer state structure
 * @return Mechanical speed in Hz (rev/s)
 */
static inline float32_t angle_observer_get_mech_speed_hz(
	const struct angle_observer_state *obs)
{
	return obs->mech_speed_dps / 360.0f;
}

#endif /* ANGLE_OBSERVER_H_ */
