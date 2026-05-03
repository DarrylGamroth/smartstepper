/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/calibration/align.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"

/* CMSIS-DSP helper provided by toolchain/SDK on target and by test stub on host. */
extern void arm_sin_cos_f32(float32_t theta, float32_t *pSinVal, float32_t *pCosVal);

void motor_align_accum_reset(struct motor_align_sample_accum *acc)
{
	if (acc == NULL) {
		return;
	}

	acc->sum_sin = 0.0f;
	acc->sum_cos = 0.0f;
	acc->count = 0U;
}

void motor_align_accum_push(struct motor_align_sample_accum *acc, float32_t mech_angle_rad)
{
	if (acc == NULL || !isfinite(mech_angle_rad)) {
		return;
	}

	float32_t angle_deg = wrap_rad_2pi(mech_angle_rad) * (180.0f / PI_F32);
	float32_t sin_theta = 0.0f;
	float32_t cos_theta = 0.0f;

	/* CMSIS helper takes angle in degrees. */
	arm_sin_cos_f32(angle_deg, &sin_theta, &cos_theta);

	acc->sum_sin += sin_theta;
	acc->sum_cos += cos_theta;
	if (acc->count < UINT16_MAX) {
		acc->count++;
	}
}

bool motor_align_circular_mean(const struct motor_align_sample_accum *acc, float32_t *mean_rad)
{
	if (acc == NULL || mean_rad == NULL || acc->count == 0U) {
		return false;
	}

	if (!isfinite(acc->sum_sin) || !isfinite(acc->sum_cos)) {
		return false;
	}

	if ((fabsf(acc->sum_sin) < 1e-6f) && (fabsf(acc->sum_cos) < 1e-6f)) {
		return false;
	}

	*mean_rad = wrap_rad_2pi(atan2f(acc->sum_sin, acc->sum_cos));
	return true;
}

int motor_align_plan_id_traj(struct traj_f32 *traj,
			     float32_t id_target_a,
			     float32_t inject_duration_s,
			     float32_t control_hz,
			     struct motor_align_traj_plan *out)
{
	if (traj == NULL || out == NULL || !isfinite(id_target_a) ||
	    !isfinite(inject_duration_s) || !isfinite(control_hz) ||
	    inject_duration_s <= 0.0f || control_hz <= 0.0f) {
		return -EINVAL;
	}

	const float32_t id_start_a = traj_get_int_value(traj);
	const float32_t steps = fmaxf(1.0f, inject_duration_s * control_hz);
	const float32_t max_delta_a_per_tick =
		fmaxf(fabsf(id_target_a - id_start_a) / steps, 1e-6f);

	traj_set_target_value(traj, id_target_a);
	traj_set_max_delta(traj, max_delta_a_per_tick);

	out->id_start_a = id_start_a;
	out->id_target_a = id_target_a;
	out->max_delta_a_per_tick = max_delta_a_per_tick;
	out->steps = steps;
	return 0;
}

float32_t motor_align_offset_from_mech_sample(float32_t mech_angle_rad)
{
	if (!(mech_angle_rad == mech_angle_rad) || fabsf(mech_angle_rad) > (2.0f * PI_F32)) {
		return 0.0f;
	}

	return wrap_rad_pi(-wrap_rad_2pi(mech_angle_rad));
}
