/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/motion/traj.h"
#include "motor/motion/motion_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure an acceleration-limited velocity planning module.
 */
void motor_velocity_plan_init(struct traj_f32 *traj,
	     float32_t max_velocity_rad_s,
	     float32_t max_accel_rad_s2,
	     float32_t dt_s,
	     float32_t initial_rad_s);

/**
 * @brief Update velocity planner limits and clamp internal state.
 */
void motor_velocity_plan_update_limits(struct traj_f32 *traj,
	      float32_t max_velocity_rad_s,
	      float32_t max_accel_rad_s2,
	      float32_t dt_s);

/**
 * @brief Execute one velocity planner step.
 */
void motor_velocity_plan_step(struct traj_f32 *traj,
	     float32_t *target_rad_s_out,
	     float32_t *ref_rad_s_out);

/**
 * @brief Plan a wrapped quintic position segment for sequence playback.
 */
int motor_position_move_plan_sequence_segment(struct motion_profile_quintic *profile,
	      float32_t start_pos_rad,
	      float32_t start_vel_rad_s,
	      float32_t target_wrapped_rad,
	      float32_t end_vel_rad_s,
	      float32_t duration_s,
	      float32_t max_velocity_rad_s,
	      float32_t max_accel_rad_s2);

/**
 * @brief Resolve active/completed position move into target/error/feedforward.
 *
 * @retval true  Profile contributes target/error/feedforward.
 * @retval false Profile is inactive and invalid (caller should use direct target).
 */
bool motor_position_move_resolve(struct motion_profile_quintic *profile,
			 bool advance,
			 float32_t measured_pos_rad,
			 float32_t *target_wrapped_rad_out,
			 float32_t *position_error_rad_out,
			 float32_t *velocity_ff_rad_s_out);

/**
 * @brief Select next wrapped target in position sequence plan.
 *
 * @retval 0       Success, `target_wrapped_rad_out` valid.
 * @retval -ENOENT No available point (empty sequence or end without loop).
 * @retval -EINVAL Invalid input.
 */
int motor_position_sequence_take_next(const float32_t *points_rad,
	     uint16_t point_count,
	     bool loop,
	     uint16_t *next_idx_io,
	     float32_t *target_wrapped_rad_out,
	     bool *complete_after_take_out);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PLANNER_H_ */
