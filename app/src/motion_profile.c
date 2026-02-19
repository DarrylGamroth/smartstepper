/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motion_profile.h"

#include <errno.h>
#include <math.h>
#include <string.h>

static void motion_profile_quintic_eval(const struct motion_profile_quintic *profile, float32_t t_s,
					float32_t *position_rad, float32_t *velocity_rad_s,
					float32_t *acceleration_rad_s2)
{
	const float32_t t2 = t_s * t_s;
	const float32_t t3 = t2 * t_s;
	const float32_t t4 = t3 * t_s;
	const float32_t t5 = t4 * t_s;

	const float32_t c0 = profile->c[0];
	const float32_t c1 = profile->c[1];
	const float32_t c2 = profile->c[2];
	const float32_t c3 = profile->c[3];
	const float32_t c4 = profile->c[4];
	const float32_t c5 = profile->c[5];

	*position_rad = c0 + c1 * t_s + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
	*velocity_rad_s = c1 + 2.0f * c2 * t_s + 3.0f * c3 * t2 + 4.0f * c4 * t3 + 5.0f * c5 * t4;
	*acceleration_rad_s2 = 2.0f * c2 + 6.0f * c3 * t_s + 12.0f * c4 * t2 + 20.0f * c5 * t3;
}

void motion_profile_quintic_init(struct motion_profile_quintic *profile, float32_t sample_period_s)
{
	if (profile == NULL) {
		return;
	}

	memset(profile, 0, sizeof(*profile));
	profile->sample_period_s = sample_period_s;
}

int motion_profile_quintic_plan(struct motion_profile_quintic *profile,
				float32_t pos_start_rad, float32_t vel_start_rad_s,
				float32_t acc_start_rad_s2, float32_t pos_end_rad,
				float32_t vel_end_rad_s, float32_t acc_end_rad_s2,
				float32_t duration_s)
{
	if (profile == NULL) {
		return -EINVAL;
	}
	if (!isfinite(duration_s) || duration_s <= 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(profile->sample_period_s) || profile->sample_period_s <= 0.0f) {
		return -EINVAL;
	}

	const float32_t T = duration_s;
	const float32_t T2 = T * T;
	const float32_t T3 = T2 * T;
	const float32_t T4 = T3 * T;
	const float32_t T5 = T4 * T;

	/* Solve in normalized time s=t/T for numerical stability. */
	const float32_t a0 = pos_start_rad;
	const float32_t a1 = vel_start_rad_s * T;
	const float32_t a2 = 0.5f * acc_start_rad_s2 * T2;

	const float32_t d1 = pos_end_rad - (a0 + a1 + a2);
	const float32_t d2 = vel_end_rad_s * T - (a1 + 2.0f * a2);
	const float32_t d3 = acc_end_rad_s2 * T2 - (2.0f * a2);

	const float32_t a3 = 10.0f * d1 - 4.0f * d2 + 0.5f * d3;
	const float32_t a4 = -15.0f * d1 + 7.0f * d2 - d3;
	const float32_t a5 = 6.0f * d1 - 3.0f * d2 + 0.5f * d3;

	profile->duration_s = T;
	profile->t_s = 0.0f;

	profile->start_position_rad = pos_start_rad;
	profile->end_position_rad = pos_end_rad;
	profile->start_velocity_rad_s = vel_start_rad_s;
	profile->end_velocity_rad_s = vel_end_rad_s;

	/* Convert back to coefficients in real time t. */
	profile->c[0] = a0;
	profile->c[1] = a1 / T;
	profile->c[2] = a2 / T2;
	profile->c[3] = a3 / T3;
	profile->c[4] = a4 / T4;
	profile->c[5] = a5 / T5;

	profile->position_rad = pos_start_rad;
	profile->velocity_rad_s = vel_start_rad_s;
	profile->acceleration_rad_s2 = acc_start_rad_s2;
	profile->valid = true;
	profile->active = true;

	return 0;
}

void motion_profile_quintic_step(struct motion_profile_quintic *profile)
{
	if (profile == NULL || !profile->valid || !profile->active) {
		return;
	}

	profile->t_s += profile->sample_period_s;
	if (profile->t_s >= profile->duration_s) {
		profile->t_s = profile->duration_s;
		profile->active = false;
	}

	motion_profile_quintic_eval(profile, profile->t_s, &profile->position_rad,
				    &profile->velocity_rad_s, &profile->acceleration_rad_s2);
}

void motion_profile_quintic_cancel(struct motion_profile_quintic *profile, float32_t hold_position_rad)
{
	if (profile == NULL) {
		return;
	}

	profile->active = false;
	profile->valid = false;
	profile->t_s = 0.0f;
	profile->duration_s = 0.0f;
	profile->position_rad = hold_position_rad;
	profile->velocity_rad_s = 0.0f;
	profile->acceleration_rad_s2 = 0.0f;
}

int motion_profile_quintic_check_limits(const struct motion_profile_quintic *profile,
					float32_t max_velocity_rad_s, float32_t max_accel_rad_s2,
					uint32_t samples, float32_t *peak_velocity_rad_s,
					float32_t *peak_accel_rad_s2)
{
	if (profile == NULL || !profile->valid || profile->duration_s <= 0.0f) {
		return -EINVAL;
	}
	if (samples < 2U) {
		samples = 2U;
	}

	float32_t peak_v = 0.0f;
	float32_t peak_a = 0.0f;

	for (uint32_t i = 0U; i <= samples; i++) {
		const float32_t ratio = (float32_t)i / (float32_t)samples;
		const float32_t t_s = ratio * profile->duration_s;
		float32_t pos, vel, acc;

		motion_profile_quintic_eval(profile, t_s, &pos, &vel, &acc);
		(void)pos;
		peak_v = fmaxf(peak_v, fabsf(vel));
		peak_a = fmaxf(peak_a, fabsf(acc));
	}

	if (peak_velocity_rad_s != NULL) {
		*peak_velocity_rad_s = peak_v;
	}
	if (peak_accel_rad_s2 != NULL) {
		*peak_accel_rad_s2 = peak_a;
	}

	if ((max_velocity_rad_s > 0.0f && peak_v > max_velocity_rad_s) ||
	    (max_accel_rad_s2 > 0.0f && peak_a > max_accel_rad_s2)) {
		return -ERANGE;
	}

	return 0;
}
