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

static inline void motor_align_rad_to_sin_cos(float32_t angle_rad,
					       float32_t *sin_out,
					       float32_t *cos_out)
{
	float32_t angle_deg = wrap_rad_2pi(angle_rad) * (180.0f / PI_F32);
	arm_sin_cos_f32(angle_deg, sin_out, cos_out);
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

bool motor_align_compute_dual_polarity(const struct motor_align_config *cfg,
				       const struct motor_align_sample_accum *pos,
				       const struct motor_align_sample_accum *neg,
				       struct motor_align_dual_result *out)
{
	if (cfg == NULL || pos == NULL || neg == NULL || out == NULL ||
	    cfg->pole_pairs <= 0.0f || !isfinite(cfg->pole_pairs)) {
		return false;
	}

	float32_t pos_mean = 0.0f;
	float32_t neg_mean = 0.0f;
	if (!motor_align_circular_mean(pos, &pos_mean) ||
	    !motor_align_circular_mean(neg, &neg_mean)) {
		return false;
	}

	float32_t expected_delta_mech_rad = PI_F32 / cfg->pole_pairs;
	float32_t measured_delta_mech_rad = wrap_rad_pi(neg_mean - pos_mean);
	float32_t delta_tol_mech_rad = cfg->opposed_elec_tol_rad / cfg->pole_pairs;
	float32_t delta_abs_error_rad = fabsf(fabsf(measured_delta_mech_rad) - expected_delta_mech_rad);

	float32_t offset_plus_rad = wrap_rad_pi(-pos_mean);
	float32_t offset_minus_rad = wrap_rad_pi((PI_F32 / cfg->pole_pairs) - neg_mean);
	float32_t sin_plus = 0.0f;
	float32_t cos_plus = 0.0f;
	float32_t sin_minus = 0.0f;
	float32_t cos_minus = 0.0f;

	motor_align_rad_to_sin_cos(offset_plus_rad, &sin_plus, &cos_plus);
	motor_align_rad_to_sin_cos(offset_minus_rad, &sin_minus, &cos_minus);

	float32_t offset_sum_sin = sin_plus + sin_minus;
	float32_t offset_sum_cos = cos_plus + cos_minus;

	if ((fabsf(offset_sum_sin) < 1e-6f) && (fabsf(offset_sum_cos) < 1e-6f)) {
		return false;
	}

	out->valid = (delta_abs_error_rad <= delta_tol_mech_rad);
	out->pos_mech_rad = pos_mean;
	out->neg_mech_rad = neg_mean;
	out->final_offset_rad = wrap_rad_pi(atan2f(offset_sum_sin, offset_sum_cos));
	out->measured_delta_mech_rad = measured_delta_mech_rad;
	out->expected_delta_mech_rad = expected_delta_mech_rad;
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

float32_t motor_align_fallback_offset_from_mech(float32_t mech_angle_rad)
{
	if (!isfinite(mech_angle_rad)) {
		return 0.0f;
	}

	return wrap_rad_pi(-wrap_rad_2pi(mech_angle_rad));
}

bool motor_align_resolve_offset(const struct motor_align_config *cfg,
				const struct motor_align_sample_accum *pos,
				const struct motor_align_sample_accum *neg,
				struct motor_align_offset_result *out)
{
	if (cfg == NULL || pos == NULL || neg == NULL || out == NULL ||
	    cfg->pole_pairs <= 0.0f || !isfinite(cfg->pole_pairs)) {
		return false;
	}

	*out = (struct motor_align_offset_result){0};

	float32_t pos_mean_rad = 0.0f;
	if (!motor_align_circular_mean(pos, &pos_mean_rad)) {
		return false;
	}
	out->pos_mech_rad = pos_mean_rad;
	out->expected_delta_mech_rad = PI_F32 / cfg->pole_pairs;

	struct motor_align_dual_result dual = {0};
	if (!motor_align_compute_dual_polarity(cfg, pos, neg, &dual)) {
		out->final_offset_rad = motor_align_fallback_offset_from_mech(pos_mean_rad);
		return true;
	}

	out->dual_solution_available = true;
	out->dual_solution_valid = dual.valid;
	out->pos_mech_rad = dual.pos_mech_rad;
	out->neg_mech_rad = dual.neg_mech_rad;
	out->measured_delta_mech_rad = dual.measured_delta_mech_rad;
	out->expected_delta_mech_rad = dual.expected_delta_mech_rad;

	if (dual.valid) {
		out->final_offset_rad = dual.final_offset_rad;
	} else {
		out->final_offset_rad = motor_align_fallback_offset_from_mech(pos_mean_rad);
	}

	return true;
}
