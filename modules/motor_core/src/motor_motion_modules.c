/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_motion_modules.h"

#include <errno.h>

#include "angle_wrap.h"

void motor_velocity_plan_init(struct traj_f32 *traj,
	     float32_t max_velocity_rad_s,
	     float32_t max_accel_rad_s2,
	     float32_t dt_s,
	     float32_t initial_rad_s)
{
	if (traj == NULL) {
		return;
	}

	traj_init(traj);
	traj_set_min_value(traj, -max_velocity_rad_s);
	traj_set_max_value(traj, max_velocity_rad_s);
	traj_set_max_delta(traj, max_accel_rad_s2 * dt_s);
	traj_set_target_value(traj, initial_rad_s);
	traj_set_int_value(traj, initial_rad_s);
}

void motor_velocity_plan_update_limits(struct traj_f32 *traj,
	      float32_t max_velocity_rad_s,
	      float32_t max_accel_rad_s2,
	      float32_t dt_s)
{
	if (traj == NULL) {
		return;
	}

	traj_set_min_value(traj, -max_velocity_rad_s);
	traj_set_max_value(traj, max_velocity_rad_s);
	traj_set_max_delta(traj, max_accel_rad_s2 * dt_s);
	traj_set_target_value(traj,
			     clampf(traj_get_target_value(traj),
				    -max_velocity_rad_s,
				    max_velocity_rad_s));
	traj_set_int_value(traj,
			  clampf(traj_get_int_value(traj),
				 -max_velocity_rad_s,
				 max_velocity_rad_s));
}

void motor_velocity_plan_step(struct traj_f32 *traj,
	     float32_t *target_rad_s_out,
	     float32_t *ref_rad_s_out)
{
	if (traj == NULL) {
		return;
	}

	if (target_rad_s_out != NULL) {
		*target_rad_s_out = traj_get_target_value(traj);
	}

	traj_run(traj);

	if (ref_rad_s_out != NULL) {
		*ref_rad_s_out = traj_get_int_value(traj);
	}
}

int motor_position_move_plan_sequence_segment(struct motion_profile_quintic *profile,
	      float32_t start_pos_rad,
	      float32_t start_vel_rad_s,
	      float32_t target_wrapped_rad,
	      float32_t end_vel_rad_s,
	      float32_t duration_s,
	      float32_t max_velocity_rad_s,
	      float32_t max_accel_rad_s2)
{
	if (profile == NULL) {
		return -EINVAL;
	}

	float32_t delta_rad = wrap_rad_pi(target_wrapped_rad - start_pos_rad);
	float32_t end_pos_rad = start_pos_rad + delta_rad;

	int ret = motion_profile_quintic_plan(profile,
				      start_pos_rad, start_vel_rad_s, 0.0f,
				      end_pos_rad, end_vel_rad_s, 0.0f,
				      duration_s);
	if (ret != 0) {
		return ret;
	}

	ret = motion_profile_quintic_check_limits(profile,
					 max_velocity_rad_s,
					 max_accel_rad_s2,
					 64U,
					 NULL,
					 NULL);
	if (ret != 0) {
		motion_profile_quintic_cancel(profile, start_pos_rad);
		return ret;
	}

	return 0;
}

bool motor_position_move_resolve(struct motion_profile_quintic *profile,
			 bool advance,
			 float32_t measured_pos_rad,
			 float32_t *target_wrapped_rad_out,
			 float32_t *position_error_rad_out,
			 float32_t *velocity_ff_rad_s_out)
{
	if (profile == NULL || target_wrapped_rad_out == NULL ||
	    position_error_rad_out == NULL || velocity_ff_rad_s_out == NULL) {
		return false;
	}

	bool active = motion_profile_quintic_is_active(profile);
	if (active && advance) {
		motion_profile_quintic_step(profile);
	}

	if (!active && !profile->valid) {
		return false;
	}

	float32_t profile_pos_rad = motion_profile_quintic_get_position(profile);

	*target_wrapped_rad_out = wrap_rad_2pi(profile_pos_rad);
	*position_error_rad_out = wrap_rad_pi(profile_pos_rad - measured_pos_rad);
	*velocity_ff_rad_s_out = active ? motion_profile_quintic_get_velocity(profile) : 0.0f;

	return true;
}

int motor_position_sequence_take_next(const float32_t *points_rad,
	     uint16_t point_count,
	     bool loop,
	     uint16_t *next_idx_io,
	     float32_t *target_wrapped_rad_out,
	     bool *complete_after_take_out)
{
	if (points_rad == NULL || next_idx_io == NULL || target_wrapped_rad_out == NULL ||
	    complete_after_take_out == NULL) {
		return -EINVAL;
	}
	if (point_count == 0U) {
		return -ENOENT;
	}

	uint16_t idx = *next_idx_io;
	if (idx >= point_count) {
		if (loop) {
			idx = 0U;
		} else {
			return -ENOENT;
		}
	}

	*target_wrapped_rad_out = points_rad[idx];
	idx++;

	if (idx >= point_count) {
		if (loop) {
			idx = 0U;
			*complete_after_take_out = false;
		} else {
			*complete_after_take_out = true;
		}
	} else {
		*complete_after_take_out = false;
	}

	*next_idx_io = idx;

	return 0;
}
