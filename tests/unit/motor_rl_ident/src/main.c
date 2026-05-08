/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor/calibration/rl_ident.h"
#include "motor/math/math_constants.h"

ZTEST(motor_rl_ident, test_roverl_plan_configures_traj_and_angle_gen)
{
	struct traj_f32 traj = {0};
	angle_gen_t angle_gen = {0};
	struct motor_roverl_accumulator accum = {
		.vd_id_sum = 1.0f,
		.vq_id_sum = -2.0f,
		.id_sq_sum = 3.0f,
	};
	const struct motor_roverl_config cfg = {
		.target_current_a = 0.2f,
		.settling_s = 0.2f,
		.excitation_hz = 20.0f,
		.control_hz = 20000.0f,
		.pole_pairs = 50.0f,
	};

	zassert_ok(motor_roverl_plan(&traj, &angle_gen, &accum, &cfg), NULL);
	zassert_within(traj_get_target_value(&traj), 0.2f, 1e-7f, NULL);
	zassert_within(traj_get_max_delta(&traj), 0.2f / (0.2f * 20000.0f), 1e-8f, NULL);
	zassert_within(angle_gen.omega_rad_s, (2.0f * PI_F32 * 20.0f) / 50.0f, 1e-6f, NULL);
	zassert_within(accum.vd_id_sum, 0.0f, 1e-7f, NULL);
	zassert_within(accum.vq_id_sum, 0.0f, 1e-7f, NULL);
	zassert_within(accum.id_sq_sum, 0.0f, 1e-7f, NULL);
}

ZTEST(motor_rl_ident, test_roverl_prepare_scalars_configures_and_zeros)
{
	struct traj_f32 traj = {0};
	angle_gen_t angle_gen = {0};
	float32_t vd = 1.0f;
	float32_t vq = 2.0f;
	float32_t id2 = 3.0f;
	const struct motor_roverl_config cfg = {
		.target_current_a = 0.2f,
		.settling_s = 0.2f,
		.excitation_hz = 20.0f,
		.control_hz = 20000.0f,
		.pole_pairs = 50.0f,
	};

	zassert_ok(motor_roverl_prepare_scalars(&traj, &angle_gen, &cfg, &vd, &vq, &id2), NULL);
	zassert_within(vd, 0.0f, 1e-7f, NULL);
	zassert_within(vq, 0.0f, 1e-7f, NULL);
	zassert_within(id2, 0.0f, 1e-7f, NULL);
	zassert_within(traj_get_target_value(&traj), 0.2f, 1e-7f, NULL);
}

ZTEST(motor_rl_ident, test_roverl_accumulator_scalar_marshal_round_trip)
{
	struct motor_roverl_accumulator accum = {0};
	float32_t vd = 0.0f;
	float32_t vq = 0.0f;
	float32_t id2 = 0.0f;

	motor_roverl_accumulator_from_scalars(&accum, 1.25f, -0.75f, 2.5f);
	zassert_ok(motor_roverl_accumulator_to_scalars(&accum, &vd, &vq, &id2), NULL);
	zassert_within(vd, 1.25f, 1e-7f, NULL);
	zassert_within(vq, -0.75f, 1e-7f, NULL);
	zassert_within(id2, 2.5f, 1e-7f, NULL);
}

ZTEST(motor_rl_ident, test_roverl_accumulate_scalars_updates_products)
{
	float32_t vd = 0.0f;
	float32_t vq = 0.0f;
	float32_t id2 = 0.0f;

	motor_roverl_accumulate_scalars(&vd, &vq, &id2, 2.0f, -3.0f, 0.5f);
	zassert_within(vd, 1.0f, 1e-7f, NULL);
	zassert_within(vq, -1.5f, 1e-7f, NULL);
	zassert_within(id2, 0.25f, 1e-7f, NULL);
}

ZTEST(motor_rl_ident, test_roverl_finalize_extracts_r_l_and_tau)
{
	const float32_t freq_hz = 20.0f;
	const float32_t rs = 2.0f;
	const float32_t ls = 0.003f;
	const float32_t omega = 2.0f * PI_F32 * freq_hz;
	const float32_t id_sq_sum = 10.0f;
	const struct motor_roverl_accumulator accum = {
		.vd_id_sum = rs * id_sq_sum,
		.vq_id_sum = (omega * ls) * id_sq_sum,
		.id_sq_sum = id_sq_sum,
	};
	struct motor_roverl_result out = {0};

	zassert_ok(motor_roverl_finalize(&accum, freq_hz, &out), NULL);
	zassert_within(out.rs_ohm, rs, 1e-5f, NULL);
	zassert_within(out.ls_h, ls, 1e-6f, NULL);
	zassert_within(out.r_over_l, rs / ls, 1e-2f, NULL);
	zassert_within(out.tau_s, ls / rs, 1e-6f, NULL);
}

ZTEST(motor_rl_ident, test_roverl_finalize_from_scalars_matches_finalize)
{
	const float32_t freq_hz = 20.0f;
	const float32_t rs = 2.0f;
	const float32_t ls = 0.003f;
	const float32_t omega = 2.0f * PI_F32 * freq_hz;
	const float32_t id_sq_sum = 10.0f;
	struct motor_roverl_result out = {0};

	zassert_ok(motor_roverl_finalize_from_scalars(rs * id_sq_sum,
						      (omega * ls) * id_sq_sum,
						      id_sq_sum, freq_hz, &out), NULL);
	zassert_within(out.rs_ohm, rs, 1e-5f, NULL);
	zassert_within(out.ls_h, ls, 1e-6f, NULL);
}

ZTEST(motor_rl_ident, test_roverl_finalize_rejects_invalid_inputs)
{
	struct motor_roverl_result out = {0};
	const struct motor_roverl_accumulator small_sum = {
		.vd_id_sum = 1.0f,
		.vq_id_sum = 1.0f,
		.id_sq_sum = 1.0e-12f,
	};

	zassert_equal(motor_roverl_finalize(NULL, 20.0f, &out), -EINVAL, NULL);
	zassert_equal(motor_roverl_finalize(&small_sum, 20.0f, &out), -ERANGE, NULL);
	zassert_equal(motor_roverl_finalize(&small_sum, 0.0f, &out), -EINVAL, NULL);
}

ZTEST_SUITE(motor_rl_ident, NULL, NULL, NULL, NULL, NULL);
