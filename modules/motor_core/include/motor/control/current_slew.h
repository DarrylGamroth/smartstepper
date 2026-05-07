/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_CURRENT_SLEW_H_
#define MOTOR_CONTROL_CURRENT_SLEW_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

#include "motor/motion/traj.h"

#ifdef __cplusplus
extern "C" {
#endif

struct motor_current_slew_pair {
	struct traj_f32 *id;
	struct traj_f32 *iq;
};

static inline float32_t motor_current_slew_delta_a_per_tick(float32_t max_current_a,
							    float32_t ramp_time_s,
							    float32_t control_hz)
{
	float32_t denom = ramp_time_s * control_hz;

	if (max_current_a <= 0.0f || denom <= 0.0f) {
		return 1.0e-6f;
	}

	float32_t delta = max_current_a / denom;

	return delta > 1.0e-6f ? delta : 1.0e-6f;
}

static inline void motor_current_slew_axis_init(struct traj_f32 *axis,
						float32_t max_current_a,
						float32_t delta_a_per_tick)
{
	traj_init(axis);
	traj_set_min_value(axis, -max_current_a);
	traj_set_max_value(axis, max_current_a);
	traj_set_max_delta(axis, delta_a_per_tick);
}

static inline void motor_current_slew_pair_init(const struct motor_current_slew_pair *slew,
						float32_t max_current_a,
						float32_t delta_a_per_tick)
{
	motor_current_slew_axis_init(slew->id, max_current_a, delta_a_per_tick);
	motor_current_slew_axis_init(slew->iq, max_current_a, delta_a_per_tick);
}

static inline void motor_current_slew_set_delta(const struct motor_current_slew_pair *slew,
						float32_t delta_a_per_tick)
{
	traj_set_max_delta(slew->id, delta_a_per_tick);
	traj_set_max_delta(slew->iq, delta_a_per_tick);
}

static inline void motor_current_slew_set_target(const struct motor_current_slew_pair *slew,
						 float32_t id_a,
						 float32_t iq_a)
{
	traj_set_target_value(slew->id, id_a);
	traj_set_target_value(slew->iq, iq_a);
}

static inline void motor_current_slew_zero(const struct motor_current_slew_pair *slew)
{
	motor_current_slew_set_target(slew, 0.0f, 0.0f);
}

static inline void motor_current_slew_force(const struct motor_current_slew_pair *slew,
					    float32_t id_a,
					    float32_t iq_a)
{
	motor_current_slew_set_target(slew, id_a, iq_a);
	traj_set_int_value(slew->id, id_a);
	traj_set_int_value(slew->iq, iq_a);
}

static inline void motor_current_slew_force_zero(const struct motor_current_slew_pair *slew)
{
	motor_current_slew_force(slew, 0.0f, 0.0f);
}

static inline void motor_current_slew_run(const struct motor_current_slew_pair *slew,
					  float32_t *id_ref_a,
					  float32_t *iq_ref_a)
{
	traj_run(slew->id);
	traj_run(slew->iq);
	if (id_ref_a != NULL) {
		*id_ref_a = traj_get_int_value(slew->id);
	}
	if (iq_ref_a != NULL) {
		*iq_ref_a = traj_get_int_value(slew->iq);
	}
}

static inline bool motor_current_slew_at_target(const struct motor_current_slew_pair *slew)
{
	return traj_is_at_target(slew->id) && traj_is_at_target(slew->iq);
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_CURRENT_SLEW_H_ */
