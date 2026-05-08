/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/calibration/rl_ident.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>

#include "motor/math/math_constants.h"

#define MOTOR_RL_IDENT_MIN_ID_SQ_SUM 1.0e-9f
#define MOTOR_RL_IDENT_MIN_ABS_EST 1.0e-9f
#define MOTOR_MIN_TRAJ_DELTA_PER_TICK 1.0e-6f

static inline bool finite_positive(float32_t x)
{
	return isfinite(x) && (x > 0.0f);
}

static inline bool finite_nonzero(float32_t x)
{
	return isfinite(x) && (fabsf(x) > 0.0f);
}

int motor_roverl_plan(struct traj_f32 *traj,
		      angle_gen_t *angle_gen,
		      struct motor_roverl_accumulator *accum,
		      const struct motor_roverl_config *cfg)
{
	if (traj == NULL || angle_gen == NULL || accum == NULL || cfg == NULL) {
		return -EINVAL;
	}
	if (!finite_positive(cfg->settling_s) ||
	    !finite_positive(cfg->excitation_hz) ||
	    !finite_positive(cfg->control_hz) ||
	    !finite_positive(cfg->pole_pairs) ||
	    !isfinite(cfg->target_current_a)) {
		return -EINVAL;
	}

	const float32_t steps = cfg->settling_s * cfg->control_hz;
	if (!finite_positive(steps)) {
		return -EINVAL;
	}

	const float32_t max_delta_a = fmaxf(fabsf(cfg->target_current_a) / steps,
					    MOTOR_MIN_TRAJ_DELTA_PER_TICK);

	traj_set_target_value(traj, cfg->target_current_a);
	traj_set_max_delta(traj, max_delta_a);

	accum->vd_id_sum = 0.0f;
	accum->vq_id_sum = 0.0f;
	accum->id_sq_sum = 0.0f;

	angle_gen_init(angle_gen, 1.0f / cfg->control_hz);
	const float32_t omega_mech_rad_s =
		(cfg->excitation_hz * 2.0f * PI_F32) / cfg->pole_pairs;
	angle_gen_set_velocity(angle_gen, omega_mech_rad_s);

	return 0;
}

int motor_roverl_prepare_scalars(struct traj_f32 *traj,
				 angle_gen_t *angle_gen,
				 const struct motor_roverl_config *cfg,
				 float32_t *vd_id_sum_out,
				 float32_t *vq_id_sum_out,
				 float32_t *id_sq_sum_out)
{
	if (vd_id_sum_out == NULL || vq_id_sum_out == NULL || id_sq_sum_out == NULL) {
		return -EINVAL;
	}

	struct motor_roverl_accumulator accum = {0};
	int ret = motor_roverl_plan(traj, angle_gen, &accum, cfg);
	if (ret != 0) {
		return ret;
	}

	return motor_roverl_accumulator_to_scalars(&accum,
						   vd_id_sum_out,
						   vq_id_sum_out,
						   id_sq_sum_out);
}

void motor_roverl_accumulate(struct motor_roverl_accumulator *accum,
			     float32_t vd_v,
			     float32_t vq_v,
			     float32_t id_a)
{
	if (accum == NULL) {
		return;
	}
	if (!isfinite(vd_v) || !isfinite(vq_v) || !isfinite(id_a)) {
		return;
	}

	accum->vd_id_sum += vd_v * id_a;
	accum->vq_id_sum += vq_v * id_a;
	accum->id_sq_sum += id_a * id_a;
}

int motor_roverl_finalize_from_scalars(float32_t vd_id_sum,
				       float32_t vq_id_sum,
				       float32_t id_sq_sum,
				       float32_t excitation_hz,
				       struct motor_roverl_result *out)
{
	struct motor_roverl_accumulator accum = {0};

	motor_roverl_accumulator_from_scalars(&accum, vd_id_sum, vq_id_sum, id_sq_sum);
	return motor_roverl_finalize(&accum, excitation_hz, out);
}

int motor_roverl_finalize(const struct motor_roverl_accumulator *accum,
			  float32_t excitation_hz,
			  struct motor_roverl_result *out)
{
	if (accum == NULL || out == NULL || !finite_positive(excitation_hz)) {
		return -EINVAL;
	}
	if (!isfinite(accum->id_sq_sum) || (accum->id_sq_sum <= MOTOR_RL_IDENT_MIN_ID_SQ_SUM)) {
		return -ERANGE;
	}

	const float32_t r_est = accum->vd_id_sum / accum->id_sq_sum;
	const float32_t omega_l_est = accum->vq_id_sum / accum->id_sq_sum;
	const float32_t omega_rad_s = 2.0f * PI_F32 * excitation_hz;
	const float32_t l_est = omega_l_est / omega_rad_s;

	if (!finite_nonzero(r_est) || !finite_nonzero(l_est) ||
	    fabsf(r_est) <= MOTOR_RL_IDENT_MIN_ABS_EST ||
	    fabsf(l_est) <= MOTOR_RL_IDENT_MIN_ABS_EST) {
		return -ERANGE;
	}

	out->rs_ohm = r_est;
	out->ls_h = l_est;
	out->r_over_l = r_est / l_est;
	out->tau_s = l_est / r_est;
	return 0;
}
