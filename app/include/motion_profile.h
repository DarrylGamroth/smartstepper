/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTION_PROFILE_H_
#define MOTION_PROFILE_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Single quintic position segment state.
 *
 * Position polynomial in time:
 *   p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 */
struct motion_profile_quintic {
	float32_t sample_period_s;
	float32_t duration_s;
	float32_t t_s;
	float32_t c[6];

	float32_t start_position_rad;
	float32_t end_position_rad;
	float32_t start_velocity_rad_s;
	float32_t end_velocity_rad_s;

	float32_t position_rad;
	float32_t velocity_rad_s;
	float32_t acceleration_rad_s2;

	bool active;
	bool valid;
};

/**
 * @brief Initialize a quintic motion profile.
 *
 * @param profile Profile instance
 * @param sample_period_s Control loop period in seconds
 */
void motion_profile_quintic_init(struct motion_profile_quintic *profile, float32_t sample_period_s);

/**
 * @brief Plan a quintic segment with arbitrary endpoint velocity and acceleration.
 *
 * @param profile Profile instance
 * @param pos_start_rad Start position (rad)
 * @param vel_start_rad_s Start velocity (rad/s)
 * @param acc_start_rad_s2 Start acceleration (rad/s^2)
 * @param pos_end_rad End position (rad)
 * @param vel_end_rad_s End velocity (rad/s)
 * @param acc_end_rad_s2 End acceleration (rad/s^2)
 * @param duration_s Segment duration (s), must be > 0
 * @return 0 on success, negative errno on failure
 */
int motion_profile_quintic_plan(struct motion_profile_quintic *profile,
				float32_t pos_start_rad, float32_t vel_start_rad_s,
				float32_t acc_start_rad_s2, float32_t pos_end_rad,
				float32_t vel_end_rad_s, float32_t acc_end_rad_s2,
				float32_t duration_s);

/**
 * @brief Advance profile by one control sample.
 *
 * If the profile reaches its end time, @p active is cleared and endpoint values are held.
 *
 * @param profile Profile instance
 */
void motion_profile_quintic_step(struct motion_profile_quintic *profile);

/**
 * @brief Cancel current profile and hold a position.
 *
 * @param profile Profile instance
 * @param hold_position_rad Position to hold after cancel
 */
void motion_profile_quintic_cancel(struct motion_profile_quintic *profile, float32_t hold_position_rad);

/**
 * @brief Check profile against velocity/acceleration limits.
 *
 * @param profile Profile instance
 * @param max_velocity_rad_s Positive limit in rad/s (<=0 disables check)
 * @param max_accel_rad_s2 Positive limit in rad/s^2 (<=0 disables check)
 * @param samples Number of uniform samples over the segment (minimum 2)
 * @param peak_velocity_rad_s Optional output peak absolute velocity
 * @param peak_accel_rad_s2 Optional output peak absolute acceleration
 * @return 0 if within limits, -ERANGE if limit exceeded, negative errno on invalid args
 */
int motion_profile_quintic_check_limits(const struct motion_profile_quintic *profile,
					float32_t max_velocity_rad_s, float32_t max_accel_rad_s2,
					uint32_t samples, float32_t *peak_velocity_rad_s,
					float32_t *peak_accel_rad_s2);

static inline bool motion_profile_quintic_is_active(const struct motion_profile_quintic *profile)
{
	return profile->active;
}

static inline float32_t motion_profile_quintic_get_position(
	const struct motion_profile_quintic *profile)
{
	return profile->position_rad;
}

static inline float32_t motion_profile_quintic_get_velocity(
	const struct motion_profile_quintic *profile)
{
	return profile->velocity_rad_s;
}

static inline float32_t motion_profile_quintic_get_accel(
	const struct motion_profile_quintic *profile)
{
	return profile->acceleration_rad_s2;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PROFILE_H_ */
