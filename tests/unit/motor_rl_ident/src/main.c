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

ZTEST(motor_rl_ident, test_rs_est_plan_configures_traj_and_filters)
{
	struct traj_f32 traj = {0};
	struct filter_fo_f32 vf = {0};
	struct filter_fo_f32 ifilt = {0};
	const struct motor_rs_est_config cfg = {
		.target_current_a = 0.15f,
		.rampup_s = 1.0f,
		.filter_bw_hz = 5.0f,
		.control_hz = 20000.0f,
	};
	const float32_t a1_expected = expf(-2.0f * PI_F32 * 5.0f / 20000.0f);
	const float32_t b0_expected = 1.0f - a1_expected;

	zassert_ok(motor_rs_est_plan(&traj, &vf, &ifilt, &cfg), NULL);
	zassert_within(traj_get_target_value(&traj), 0.15f, 1e-7f, NULL);
	zassert_within(traj_get_max_delta(&traj), 0.15f / (1.0f * 20000.0f), 1e-9f, NULL);
	zassert_within(filter_fo_get_a1(&vf), a1_expected, 1e-8f, NULL);
	zassert_within(filter_fo_get_b0(&vf), b0_expected, 1e-8f, NULL);
	zassert_within(filter_fo_get_a1(&ifilt), a1_expected, 1e-8f, NULL);
	zassert_within(filter_fo_get_b0(&ifilt), b0_expected, 1e-8f, NULL);
}

ZTEST(motor_rl_ident, test_rs_est_step_filter_accumulates_only_at_target)
{
	struct traj_f32 traj = {0};
	struct filter_fo_f32 vf = {0};
	struct filter_fo_f32 ifilt = {0};
	float32_t id_ref = 0.0f;

	/* Direct-pass one-pole configuration for simple assertions. */
	filter_fo_init(&vf);
	filter_fo_set_den_coeffs(&vf, 0.0f);
	filter_fo_set_num_coeffs(&vf, 1.0f, 0.0f);
	filter_fo_set_initial_conditions(&vf, 0.0f, 0.0f);
	filter_fo_init(&ifilt);
	filter_fo_set_den_coeffs(&ifilt, 0.0f);
	filter_fo_set_num_coeffs(&ifilt, 1.0f, 0.0f);
	filter_fo_set_initial_conditions(&ifilt, 0.0f, 0.0f);

	traj_set_int_value(&traj, 0.0f);
	traj_set_min_value(&traj, -1.0f);
	traj_set_max_value(&traj, 1.0f);
	traj_set_target_value(&traj, 0.1f);
	traj_set_max_delta(&traj, 0.01f); /* Not yet at target after one step */
	motor_rs_est_step_filter(&traj, &vf, &ifilt, 2.0f, 0.5f, &id_ref);
	zassert_true(id_ref > 0.0f && id_ref < 0.1f, NULL);
	zassert_within(filter_fo_get_y1(&vf), 0.0f, 1e-7f, NULL);
	zassert_within(filter_fo_get_y1(&ifilt), 0.0f, 1e-7f, NULL);

	traj_set_int_value(&traj, 0.0f);
	traj_set_min_value(&traj, -1.0f);
	traj_set_max_value(&traj, 1.0f);
	traj_set_target_value(&traj, 0.1f);
	traj_set_max_delta(&traj, 0.1f); /* Reaches target in one step */
	motor_rs_est_step_filter(&traj, &vf, &ifilt, 2.0f, 0.5f, &id_ref);
	zassert_within(id_ref, 0.1f, 1e-7f, NULL);
	zassert_within(filter_fo_get_y1(&vf), 2.0f, 1e-6f, NULL);
	zassert_within(filter_fo_get_y1(&ifilt), 0.5f, 1e-6f, NULL);
}

ZTEST(motor_rl_ident, test_rs_est_prepare_initializes_stationary_angle_frame)
{
	struct traj_f32 traj = {0};
	angle_gen_t angle_gen = {0};
	struct filter_fo_f32 vf = {0};
	struct filter_fo_f32 ifilt = {0};
	const struct motor_rs_est_config cfg = {
		.target_current_a = 0.12f,
		.rampup_s = 0.5f,
		.filter_bw_hz = 8.0f,
		.control_hz = 10000.0f,
	};

	angle_gen.angle_rad = 1.0f;
	angle_gen.omega_rad_s = 2.0f;
	angle_gen.angle_delta_factor = 0.0f;

	zassert_ok(motor_rs_est_prepare(&traj, &angle_gen, &vf, &ifilt, &cfg), NULL);
	zassert_within(angle_gen.omega_rad_s, 0.0f, 1e-7f, NULL);
	zassert_within(angle_gen.angle_rad, 0.0f, 1e-7f, NULL);
	zassert_within(angle_gen.angle_delta_factor, 1.0f / cfg.control_hz, 1e-7f, NULL);
}

ZTEST(motor_rl_ident, test_rs_est_finalize_from_scalars_matches_filter_finalize)
{
	struct filter_fo_f32 vf = {0};
	struct filter_fo_f32 ifilt = {0};
	struct motor_rs_est_result from_filter = {0};
	struct motor_rs_est_result from_scalars = {0};

	filter_fo_set_y1(&vf, 0.9f);
	filter_fo_set_y1(&ifilt, 0.3f);

	zassert_ok(motor_rs_est_finalize(&vf, &ifilt, 0.003f, &from_filter), NULL);
	zassert_ok(motor_rs_est_finalize_from_scalars(0.9f, 0.3f, 0.003f, &from_scalars), NULL);
	zassert_within(from_filter.rs_ohm, from_scalars.rs_ohm, 1e-7f, NULL);
	zassert_within(from_filter.r_over_l, from_scalars.r_over_l, 1e-5f, NULL);
}

ZTEST(motor_rl_ident, test_rs_est_finalize_extracts_rs_and_r_over_l)
{
	struct filter_fo_f32 vf = {0};
	struct filter_fo_f32 ifilt = {0};
	struct motor_rs_est_result out = {0};

	filter_fo_set_y1(&vf, 0.9f);
	filter_fo_set_y1(&ifilt, 0.3f);

	zassert_ok(motor_rs_est_finalize(&vf, &ifilt, 0.003f, &out), NULL);
	zassert_within(out.rs_ohm, 3.0f, 1e-6f, NULL);
	zassert_within(out.r_over_l, 1000.0f, 1e-3f, NULL);
	zassert_within(out.v_est_v, 0.9f, 1e-7f, NULL);
	zassert_within(out.i_est_a, 0.3f, 1e-7f, NULL);
}

ZTEST(motor_rl_ident, test_rs_est_finalize_rejects_small_current)
{
	struct filter_fo_f32 vf = {0};
	struct filter_fo_f32 ifilt = {0};
	struct motor_rs_est_result out = {0};

	filter_fo_set_y1(&vf, 1.0f);
	filter_fo_set_y1(&ifilt, 1.0e-7f);

	zassert_equal(motor_rs_est_finalize(&vf, &ifilt, 0.003f, &out), -ERANGE, NULL);
}

ZTEST_SUITE(motor_rl_ident, NULL, NULL, NULL, NULL, NULL);
