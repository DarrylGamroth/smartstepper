/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <zephyr/ztest.h>

#include "motor/control/mpr.h"

#define EPS 1e-6f

static float32_t sign_with_deadband(float32_t value)
{
	const float32_t deadband = 1e-3f;

	if (value > deadband) {
		return 1.0f;
	}
	if (value < -deadband) {
		return -1.0f;
	}
	return 0.0f;
}

static float32_t plant_velocity_step(const struct motor_mpr_velocity_model *model,
				     float32_t dt_s,
				     float32_t omega_rad_s,
				     float32_t iq_a,
				     float32_t load_torque_nm)
{
	float32_t j = model->inertia_kgm2;
	float32_t b = model->viscous_friction_nm_per_rad_s;
	float32_t tau_c = model->coulomb_friction_nm * sign_with_deadband(omega_rad_s);
	float32_t net_torque = (model->torque_constant_nm_per_a * iq_a) -
			       load_torque_nm - tau_c;

	if (b > 1e-9f) {
		float32_t a = expf(-(b * dt_s) / j);
		float32_t one_minus_a = 1.0f - a;
		float32_t omega_ss = net_torque / b;

		return a * omega_rad_s + one_minus_a * omega_ss;
	}

	return omega_rad_s + (net_torque / j) * dt_s;
}

ZTEST(motor_mpr, test_velocity_validate_rejects_invalid_inputs)
{
	struct motor_mpr_velocity_config cfg = {
		.dt_s = 0.001f,
		.horizon = 8U,
		.q_speed = 1.0f,
		.r_delta_iq = 0.05f,
		.iq_limit_a = 5.0f,
		.max_delta_iq_a = 0.25f,
		.disturbance_ki_nm_per_rad_s = 0.02f,
	};
	struct motor_mpr_velocity_model model = {
		.inertia_kgm2 = 0.002f,
		.viscous_friction_nm_per_rad_s = 0.004f,
		.coulomb_friction_nm = 0.001f,
		.torque_constant_nm_per_a = 0.12f,
	};

	zassert_ok(motor_mpr_velocity_validate(&cfg, &model), NULL);
	zassert_equal(motor_mpr_velocity_validate(NULL, &model), -EINVAL, NULL);
	zassert_equal(motor_mpr_velocity_validate(&cfg, NULL), -EINVAL, NULL);

	cfg.dt_s = 0.0f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
	cfg.dt_s = 0.001f;
	cfg.horizon = 0U;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
	cfg.horizon = 8U;
	cfg.q_speed = 0.0f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
	cfg.q_speed = 1.0f;
	cfg.r_delta_iq = 0.0f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
	cfg.r_delta_iq = 0.05f;
	cfg.iq_limit_a = -1.0f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
	cfg.iq_limit_a = 5.0f;
	cfg.max_delta_iq_a = -0.1f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);

	cfg.max_delta_iq_a = 0.25f;
	model.inertia_kgm2 = 0.0f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
	model.inertia_kgm2 = 0.002f;
	model.torque_constant_nm_per_a = 0.0f;
	zassert_equal(motor_mpr_velocity_validate(&cfg, &model), -EINVAL, NULL);
}

ZTEST(motor_mpr, test_velocity_step_rejects_bad_inputs)
{
	struct motor_mpr_velocity_config cfg = {
		.dt_s = 0.001f,
		.horizon = 4U,
		.q_speed = 1.0f,
		.r_delta_iq = 0.1f,
		.iq_limit_a = 2.0f,
		.max_delta_iq_a = 0.2f,
		.disturbance_ki_nm_per_rad_s = 0.0f,
	};
	struct motor_mpr_velocity_model model = {
		.inertia_kgm2 = 0.001f,
		.viscous_friction_nm_per_rad_s = 0.003f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.1f,
	};
	struct motor_mpr_velocity_state state = {0};
	float32_t iq_cmd = 0.0f;

	zassert_equal(motor_mpr_velocity_step(NULL, &model, &state, 0.0f, 0.0f, &iq_cmd),
		      -EINVAL, NULL);
	zassert_equal(motor_mpr_velocity_step(&cfg, NULL, &state, 0.0f, 0.0f, &iq_cmd),
		      -EINVAL, NULL);
	zassert_equal(motor_mpr_velocity_step(&cfg, &model, NULL, 0.0f, 0.0f, &iq_cmd),
		      -EINVAL, NULL);
	zassert_equal(motor_mpr_velocity_step(&cfg, &model, &state, 0.0f, 0.0f, NULL),
		      -EINVAL, NULL);
	zassert_equal(motor_mpr_velocity_step(&cfg, &model, &state, NAN, 0.0f, &iq_cmd),
		      -EINVAL, NULL);
	zassert_equal(motor_mpr_velocity_step(&cfg, &model, &state, 0.0f, INFINITY, &iq_cmd),
		      -EINVAL, NULL);
}

ZTEST(motor_mpr, test_velocity_step_respects_iq_and_delta_limits)
{
	struct motor_mpr_velocity_config cfg = {
		.dt_s = 0.001f,
		.horizon = 8U,
		.q_speed = 1.0f,
		.r_delta_iq = 0.05f,
		.iq_limit_a = 2.0f,
		.max_delta_iq_a = 0.15f,
		.disturbance_ki_nm_per_rad_s = 0.0f,
	};
	struct motor_mpr_velocity_model model = {
		.inertia_kgm2 = 0.0015f,
		.viscous_friction_nm_per_rad_s = 0.002f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.09f,
	};
	struct motor_mpr_velocity_state state = {0};
	float32_t iq_cmd = 0.0f;
	float32_t omega = 0.0f;

	for (int i = 0; i < 1200; i++) {
		float32_t prev = state.iq_cmd_a;

		zassert_ok(motor_mpr_velocity_step(&cfg, &model, &state, omega, 200.0f, &iq_cmd), NULL);
		zassert_true(fabsf(iq_cmd) <= cfg.iq_limit_a + 1e-5f, NULL);
		zassert_true(fabsf(iq_cmd - prev) <= cfg.max_delta_iq_a + 1e-5f, NULL);
		omega = plant_velocity_step(&model, cfg.dt_s, omega, iq_cmd, 0.0f);
	}
}

ZTEST(motor_mpr, test_velocity_tracks_reference_without_load)
{
	struct motor_mpr_velocity_config cfg = {
		.dt_s = 0.001f,
		.horizon = 10U,
		.q_speed = 2.0f,
		.r_delta_iq = 0.05f,
		.iq_limit_a = 6.0f,
		.max_delta_iq_a = 0.2f,
		.disturbance_ki_nm_per_rad_s = 0.02f,
	};
	struct motor_mpr_velocity_model model = {
		.inertia_kgm2 = 0.003f,
		.viscous_friction_nm_per_rad_s = 0.010f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.18f,
	};
	struct motor_mpr_velocity_state state = {0};
	const float32_t omega_ref = 40.0f;
	float32_t omega = 0.0f;
	float32_t iq_cmd = 0.0f;
	float32_t mean_abs_err = 0.0f;
	int count = 0;

	for (int i = 0; i < 6000; i++) {
		zassert_ok(motor_mpr_velocity_step(&cfg, &model, &state, omega, omega_ref, &iq_cmd), NULL);
		omega = plant_velocity_step(&model, cfg.dt_s, omega, iq_cmd, 0.0f);
		if (i >= 5000) {
			mean_abs_err += fabsf(omega_ref - omega);
			count++;
		}
	}

	mean_abs_err /= (float32_t)count;
	zassert_true(mean_abs_err < 0.8f, "mean abs err too high: %f", (double)mean_abs_err);
}

ZTEST(motor_mpr, test_velocity_disturbance_observer_improves_steady_state_error)
{
	struct motor_mpr_velocity_config cfg_base = {
		.dt_s = 0.001f,
		.horizon = 10U,
		.q_speed = 1.8f,
		.r_delta_iq = 0.05f,
		.iq_limit_a = 6.0f,
		.max_delta_iq_a = 0.25f,
		.disturbance_ki_nm_per_rad_s = 0.0f,
	};
	struct motor_mpr_velocity_model model = {
		.inertia_kgm2 = 0.004f,
		.viscous_friction_nm_per_rad_s = 0.012f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.2f,
	};
	const float32_t omega_ref = 30.0f;
	const float32_t load_nm = 0.30f;
	float32_t err_no_obs = 0.0f;
	float32_t err_obs = 0.0f;

	for (int pass = 0; pass < 2; pass++) {
		struct motor_mpr_velocity_config cfg = cfg_base;
		struct motor_mpr_velocity_state state = {0};
		float32_t omega = 0.0f;
		float32_t iq_cmd = 0.0f;
		float32_t err_sum = 0.0f;
		int count = 0;

		if (pass == 1) {
			cfg.disturbance_ki_nm_per_rad_s = 0.05f;
		}

		for (int i = 0; i < 7000; i++) {
			zassert_ok(
				motor_mpr_velocity_step(&cfg, &model, &state, omega, omega_ref, &iq_cmd),
				NULL);
			omega = plant_velocity_step(&model, cfg.dt_s, omega, iq_cmd, load_nm);
			if (i >= 6000) {
				err_sum += fabsf(omega_ref - omega);
				count++;
			}
		}

		if (pass == 0) {
			err_no_obs = err_sum / (float32_t)count;
		} else {
			err_obs = err_sum / (float32_t)count;
		}
	}

	zassert_true(err_obs < err_no_obs, "observer did not improve error: no_obs=%f obs=%f",
		     (double)err_no_obs, (double)err_obs);
	zassert_true(err_obs < 1.2f, "observer error too high: %f", (double)err_obs);
}

ZTEST(motor_mpr, test_position_validate_rejects_invalid_inputs)
{
	struct motor_mpr_position_config cfg = {
		.dt_s = 0.001f,
		.horizon = 16U,
		.q_position = 2.0f,
		.q_velocity_ff = 0.5f,
		.r_delta_velocity = 0.2f,
		.velocity_limit_rad_s = 30.0f,
		.max_delta_velocity_rad_s = 1.0f,
	};

	zassert_ok(motor_mpr_position_validate(&cfg), NULL);
	zassert_equal(motor_mpr_position_validate(NULL), -EINVAL, NULL);

	cfg.dt_s = 0.0f;
	zassert_equal(motor_mpr_position_validate(&cfg), -EINVAL, NULL);
	cfg.dt_s = 0.001f;
	cfg.horizon = 0U;
	zassert_equal(motor_mpr_position_validate(&cfg), -EINVAL, NULL);
	cfg.horizon = 16U;
	cfg.q_position = 0.0f;
	cfg.q_velocity_ff = 0.0f;
	zassert_equal(motor_mpr_position_validate(&cfg), -EINVAL, NULL);
	cfg.q_position = 2.0f;
	cfg.r_delta_velocity = 0.0f;
	zassert_equal(motor_mpr_position_validate(&cfg), -EINVAL, NULL);
	cfg.r_delta_velocity = 0.2f;
	cfg.velocity_limit_rad_s = 0.0f;
	zassert_equal(motor_mpr_position_validate(&cfg), -EINVAL, NULL);
}

ZTEST(motor_mpr, test_position_step_respects_limits_and_delta)
{
	struct motor_mpr_position_config cfg = {
		.dt_s = 0.001f,
		.horizon = 12U,
		.q_position = 3.0f,
		.q_velocity_ff = 0.0f,
		.r_delta_velocity = 0.2f,
		.velocity_limit_rad_s = 8.0f,
		.max_delta_velocity_rad_s = 0.3f,
	};
	struct motor_mpr_position_state state = {0};
	float32_t vel_cmd = 0.0f;

	for (int i = 0; i < 200; i++) {
		float32_t prev = state.velocity_cmd_rad_s;

		zassert_ok(motor_mpr_position_step(&cfg, &state, 5.0f, 0.0f, &vel_cmd), NULL);
		zassert_true(fabsf(vel_cmd) <= cfg.velocity_limit_rad_s + EPS, NULL);
		zassert_true(fabsf(vel_cmd - prev) <= cfg.max_delta_velocity_rad_s + EPS, NULL);
	}
}

ZTEST(motor_mpr, test_position_reduces_error_in_closed_loop)
{
	struct motor_mpr_position_config cfg = {
		.dt_s = 0.001f,
		.horizon = 20U,
		.q_position = 20.0f,
		.q_velocity_ff = 0.2f,
		.r_delta_velocity = 0.05f,
		.velocity_limit_rad_s = 20.0f,
		.max_delta_velocity_rad_s = 2.0f,
	};
	struct motor_mpr_position_state state = {0};
	float32_t theta = 0.0f;
	const float32_t target = 1.4f;
	float32_t vel_cmd = 0.0f;

	for (int i = 0; i < 8000; i++) {
		float32_t err = target - theta;

		zassert_ok(motor_mpr_position_step(&cfg, &state, err, 0.0f, &vel_cmd), NULL);
		theta += vel_cmd * cfg.dt_s;
	}

	zassert_true(fabsf(target - theta) < 0.02f, "final error too large: %f",
		     (double)fabsf(target - theta));
}

ZTEST(motor_mpr, test_position_feedforward_drives_velocity_command)
{
	struct motor_mpr_position_config cfg = {
		.dt_s = 0.001f,
		.horizon = 10U,
		.q_position = 0.0f,
		.q_velocity_ff = 3.0f,
		.r_delta_velocity = 0.5f,
		.velocity_limit_rad_s = 15.0f,
		.max_delta_velocity_rad_s = 0.4f,
	};
	struct motor_mpr_position_state state = {0};
	float32_t vel_cmd = 0.0f;
	const float32_t ff = 6.0f;

	for (int i = 0; i < 80; i++) {
		zassert_ok(motor_mpr_position_step(&cfg, &state, 0.0f, ff, &vel_cmd), NULL);
	}

	zassert_true(vel_cmd > 0.0f, NULL);
	zassert_true(fabsf(vel_cmd - ff) < 1.0f, "feedforward mismatch: cmd=%f ff=%f",
		     (double)vel_cmd, (double)ff);
}

ZTEST(motor_mpr, test_position_step_rejects_bad_inputs)
{
	struct motor_mpr_position_config cfg = {
		.dt_s = 0.001f,
		.horizon = 8U,
		.q_position = 1.0f,
		.q_velocity_ff = 1.0f,
		.r_delta_velocity = 0.2f,
		.velocity_limit_rad_s = 10.0f,
		.max_delta_velocity_rad_s = 0.5f,
	};
	struct motor_mpr_position_state state = {0};
	float32_t vel_cmd = 0.0f;

	zassert_equal(motor_mpr_position_step(NULL, &state, 0.0f, 0.0f, &vel_cmd), -EINVAL, NULL);
	zassert_equal(motor_mpr_position_step(&cfg, NULL, 0.0f, 0.0f, &vel_cmd), -EINVAL, NULL);
	zassert_equal(motor_mpr_position_step(&cfg, &state, 0.0f, 0.0f, NULL), -EINVAL, NULL);
	zassert_equal(motor_mpr_position_step(&cfg, &state, NAN, 0.0f, &vel_cmd), -EINVAL, NULL);
	zassert_equal(motor_mpr_position_step(&cfg, &state, 0.0f, INFINITY, &vel_cmd), -EINVAL, NULL);
}

ZTEST_SUITE(motor_mpr, NULL, NULL, NULL, NULL, NULL);
