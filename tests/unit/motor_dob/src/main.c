/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor/control/dob.h"

static float32_t clampf32(float32_t value, float32_t min_value, float32_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

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

static float32_t plant_velocity_step(const struct motor_dob_model *model,
			     float32_t dt_s,
			     float32_t omega_rad_s,
			     float32_t iq_a,
			     float32_t load_torque_nm)
{
	float32_t j = model->inertia_kgm2;
	float32_t b = model->viscous_friction_nm_per_rad_s;
	float32_t tau_c = model->coulomb_friction_nm * sign_with_deadband(omega_rad_s);
	float32_t net_torque = (model->torque_constant_nm_per_a * iq_a) - load_torque_nm - tau_c;

	if (b > 1e-9f) {
		float32_t a = expf(-(b * dt_s) / j);
		float32_t one_minus_a = 1.0f - a;
		float32_t omega_ss = net_torque / b;

		return a * omega_rad_s + one_minus_a * omega_ss;
	}

	return omega_rad_s + (net_torque / j) * dt_s;
}

ZTEST(motor_dob, test_validate_rejects_invalid_inputs)
{
	struct motor_dob_config cfg = {
		.enabled = true,
		.dt_s = 0.001f,
		.observer_gain_nm_per_rad_s = 0.05f,
		.torque_limit_nm = 1.0f,
		.iq_ff_limit_a = 2.0f,
	};
	struct motor_dob_model model = {
		.inertia_kgm2 = 0.002f,
		.viscous_friction_nm_per_rad_s = 0.004f,
		.coulomb_friction_nm = 0.002f,
		.torque_constant_nm_per_a = 0.12f,
	};

	zassert_ok(motor_dob_validate(&cfg, &model), NULL);
	zassert_equal(motor_dob_validate(NULL, &model), -EINVAL, NULL);
	zassert_equal(motor_dob_validate(&cfg, NULL), -EINVAL, NULL);

	cfg.dt_s = 0.0f;
	zassert_equal(motor_dob_validate(&cfg, &model), -EINVAL, NULL);
	cfg.dt_s = 0.001f;
	cfg.observer_gain_nm_per_rad_s = -0.1f;
	zassert_equal(motor_dob_validate(&cfg, &model), -EINVAL, NULL);
	cfg.observer_gain_nm_per_rad_s = 0.05f;
	cfg.torque_limit_nm = 0.0f;
	zassert_equal(motor_dob_validate(&cfg, &model), -EINVAL, NULL);

	cfg.torque_limit_nm = 1.0f;
	model.inertia_kgm2 = 0.0f;
	zassert_equal(motor_dob_validate(&cfg, &model), -EINVAL, NULL);
	model.inertia_kgm2 = 0.002f;
	model.torque_constant_nm_per_a = 0.0f;
	zassert_equal(motor_dob_validate(&cfg, &model), -EINVAL, NULL);
}

ZTEST(motor_dob, test_disabled_mode_outputs_zero_feedforward)
{
	struct motor_dob_config cfg = {
		.enabled = false,
		.dt_s = 0.001f,
		.observer_gain_nm_per_rad_s = 0.1f,
		.torque_limit_nm = 1.0f,
		.iq_ff_limit_a = 1.0f,
	};
	struct motor_dob_model model = {
		.inertia_kgm2 = 0.002f,
		.viscous_friction_nm_per_rad_s = 0.003f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.1f,
	};
	struct motor_dob_state state = {0};
	float32_t iq_ff = NAN;

	zassert_ok(motor_dob_step(&cfg, &model, &state, 12.0f, 0.5f, &iq_ff), NULL);
	zassert_equal(iq_ff, 0.0f, NULL);
	zassert_equal(state.disturbance_nm, 0.0f, NULL);
	zassert_equal(state.iq_ff_a, 0.0f, NULL);
	zassert_true(state.initialized, NULL);
}

ZTEST(motor_dob, test_step_rejects_non_finite_inputs)
{
	struct motor_dob_config cfg = {
		.enabled = true,
		.dt_s = 0.001f,
		.observer_gain_nm_per_rad_s = 0.05f,
		.torque_limit_nm = 1.0f,
		.iq_ff_limit_a = 1.0f,
	};
	struct motor_dob_model model = {
		.inertia_kgm2 = 0.001f,
		.viscous_friction_nm_per_rad_s = 0.002f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.1f,
	};
	struct motor_dob_state state = {0};
	float32_t iq_ff = 0.0f;

	zassert_equal(motor_dob_step(&cfg, &model, &state, NAN, 0.0f, &iq_ff), -EINVAL, NULL);
	zassert_equal(motor_dob_step(&cfg, &model, &state, 0.0f, INFINITY, &iq_ff), -EINVAL, NULL);
}

ZTEST(motor_dob, test_dob_reduces_steady_state_error_under_constant_load)
{
	const struct motor_dob_model model = {
		.inertia_kgm2 = 0.003f,
		.viscous_friction_nm_per_rad_s = 0.010f,
		.coulomb_friction_nm = 0.001f,
		.torque_constant_nm_per_a = 0.18f,
	};
	const struct motor_dob_config cfg_base = {
		.enabled = true,
		.dt_s = 0.001f,
		.observer_gain_nm_per_rad_s = 0.08f,
		.torque_limit_nm = 1.2f,
		.iq_ff_limit_a = 1.5f,
	};
	const float32_t omega_ref = 30.0f;
	const float32_t iq_limit = 2.0f;
	const float32_t load_nm = 0.25f;
	const float32_t kp = 0.08f;
	float32_t err_no_dob = 0.0f;
	float32_t err_with_dob = 0.0f;

	for (int pass = 0; pass < 2; pass++) {
		struct motor_dob_config cfg = cfg_base;
		struct motor_dob_state state = {0};
		float32_t omega = 0.0f;
		float32_t err_sum = 0.0f;
		uint32_t count = 0U;

		cfg.enabled = (pass == 1);
		for (int i = 0; i < 7000; i++) {
			float32_t iq_base = kp * (omega_ref - omega);
			iq_base = clampf32(iq_base, -iq_limit, iq_limit);

			float32_t iq_ff = 0.0f;
			zassert_ok(motor_dob_step(&cfg, &model, &state, omega, iq_base, &iq_ff), NULL);

			float32_t iq_total = clampf32(iq_base + iq_ff, -iq_limit, iq_limit);
			omega = plant_velocity_step(&model, cfg.dt_s, omega, iq_total, load_nm);

			if (i >= 6000) {
				err_sum += fabsf(omega_ref - omega);
				count++;
			}
		}

		if (pass == 0) {
			err_no_dob = err_sum / (float32_t)count;
		} else {
			err_with_dob = err_sum / (float32_t)count;
		}
	}

	zassert_true(err_with_dob < err_no_dob,
		     "DOB did not improve error: no_dob=%f with_dob=%f",
		     (double)err_no_dob, (double)err_with_dob);
}

ZTEST(motor_dob, test_resisting_load_yields_positive_iq_feedforward)
{
	const struct motor_dob_model model = {
		.inertia_kgm2 = 0.002f,
		.viscous_friction_nm_per_rad_s = 0.006f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.12f,
	};
	const struct motor_dob_config cfg = {
		.enabled = true,
		.dt_s = 0.001f,
		.observer_gain_nm_per_rad_s = 0.08f,
		.torque_limit_nm = 0.8f,
		.iq_ff_limit_a = 1.0f,
	};
	struct motor_dob_state state = {0};
	float32_t omega = 20.0f;
	float32_t iq_ff = 0.0f;

	for (int i = 0; i < 3000; i++) {
		zassert_ok(motor_dob_step(&cfg, &model, &state, omega, 0.0f, &iq_ff), NULL);
		omega = plant_velocity_step(&model, cfg.dt_s, omega, 0.0f, 0.18f);
	}

	zassert_true(state.disturbance_nm < 0.0f, NULL);
	zassert_true(iq_ff > 0.0f, NULL);
}

ZTEST_SUITE(motor_dob, NULL, NULL, NULL, NULL, NULL);
