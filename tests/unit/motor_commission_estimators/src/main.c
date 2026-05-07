/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>

#include "motor/estimation/commission_estimators.h"
#include "motor/math/matrix_solve.h"

static float32_t flux_model_vq(const struct motor_flux_id_config *cfg,
			       float32_t elec_speed_rad_s,
			       float32_t id_a,
			       float32_t iq_a,
			       float32_t diq_dt_a_s,
			       float32_t psi_f_wb,
			       float32_t bias_v)
{
	const float32_t y = psi_f_wb * elec_speed_rad_s + bias_v;
	return y + cfg->rs_ohm * iq_a + cfg->lq_h * diq_dt_a_s + elec_speed_rad_s * cfg->ld_h * id_a;
}

static float32_t mech_sign_term(float32_t mech_speed_rad_s, float32_t deadband_rad_s)
{
	if (mech_speed_rad_s > deadband_rad_s) {
		return 1.0f;
	}
	if (mech_speed_rad_s < -deadband_rad_s) {
		return -1.0f;
	}
	return 0.0f;
}

static float32_t mech_model_torque(float32_t inertia_kgm2,
				   float32_t viscous_nm_per_rad_s,
				   float32_t coulomb_nm,
				   float32_t offset_nm,
				   float32_t mech_speed_rad_s,
				   float32_t mech_accel_rad_s2,
				   float32_t deadband_rad_s)
{
	const float32_t sign_term = mech_sign_term(mech_speed_rad_s, deadband_rad_s);

	return inertia_kgm2 * mech_accel_rad_s2 + viscous_nm_per_rad_s * mech_speed_rad_s +
	       coulomb_nm * sign_term + offset_nm;
}

ZTEST(motor_commission_estimators, test_validate_flux_config)
{
	struct motor_flux_id_config cfg = {
		.rs_ohm = 0.4f,
		.ld_h = 0.0011f,
		.lq_h = 0.0013f,
		.min_abs_speed_rad_s = 5.0f,
		.min_speed_span_rad_s = 10.0f,
		.min_samples = 4U,
		.min_r2 = 0.1f,
		.require_positive_psi = true,
	};

	zassert_ok(motor_flux_id_validate_config(&cfg), NULL);
	cfg.min_samples = 0U;
	zassert_equal(motor_flux_id_validate_config(&cfg), -EINVAL, NULL);
	cfg.min_samples = 4U;
	cfg.min_r2 = 1.1f;
	zassert_equal(motor_flux_id_validate_config(&cfg), -EINVAL, NULL);
}

ZTEST(motor_commission_estimators, test_validate_mech_config)
{
	struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.12f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 4U,
		.min_r2 = 0.2f,
		.require_positive_inertia = true,
		.require_nonnegative_viscous = true,
	};

	zassert_ok(motor_mech_id_validate_config(&cfg), NULL);
	cfg.kt_nm_per_a = 0.0f;
	zassert_equal(motor_mech_id_validate_config(&cfg), -EINVAL, NULL);
	cfg.kt_nm_per_a = 0.12f;
	cfg.min_r2 = -0.1f;
	zassert_equal(motor_mech_id_validate_config(&cfg), -EINVAL, NULL);
}

ZTEST(motor_commission_estimators, test_init_with_invalid_config_disables_estimator)
{
	const struct motor_flux_id_config flux_cfg_bad = {
		.rs_ohm = NAN,
		.ld_h = 0.001f,
		.lq_h = 0.001f,
		.min_abs_speed_rad_s = 1.0f,
		.min_speed_span_rad_s = 1.0f,
		.min_samples = 4U,
		.min_r2 = 0.0f,
		.require_positive_psi = false,
	};
	const struct motor_mech_id_config mech_cfg_bad = {
		.kt_nm_per_a = 0.0f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 4U,
		.min_r2 = 0.0f,
		.require_positive_inertia = false,
		.require_nonnegative_viscous = false,
	};
	struct motor_flux_id_state flux_state = {0};
	struct motor_flux_id_result flux_result = {0};
	struct motor_mech_id_state mech_state = {0};
	struct motor_mech_id_result mech_result = {0};

	motor_flux_id_init(&flux_state, &flux_cfg_bad);
	motor_mech_id_init(&mech_state, &mech_cfg_bad);

	zassert_false(motor_flux_id_accumulate(&flux_state, 10.0f, 0.0f, 0.0f, 0.0f, 1.0f), NULL);
	zassert_equal(motor_flux_id_finalize(&flux_state, &flux_result), -EINVAL, NULL);
	zassert_false(motor_mech_id_accumulate(&mech_state, 1.0f, 1.0f, 0.5f), NULL);
	zassert_equal(motor_mech_id_finalize(&mech_state, &mech_result), -EINVAL, NULL);
}

ZTEST(motor_commission_estimators, test_flux_init_and_rejects_bad_samples)
{
	const struct motor_flux_id_config cfg = {
		.rs_ohm = 0.4f,
		.ld_h = 0.0011f,
		.lq_h = 0.0013f,
		.min_abs_speed_rad_s = 5.0f,
		.min_speed_span_rad_s = 10.0f,
		.min_samples = 4U,
		.min_r2 = 0.1f,
		.require_positive_psi = true,
	};
	struct motor_flux_id_state state = {0};

	motor_flux_id_init(&state, &cfg);
	zassert_equal(state.sample_count, 0u, NULL);
	zassert_true(isinf(state.min_speed_rad_s), NULL);
	zassert_true(isinf(state.max_speed_rad_s), NULL);

	zassert_false(motor_flux_id_accumulate(&state, NAN, 0.0f, 0.0f, 0.0f, 0.0f), NULL);
	zassert_false(motor_flux_id_accumulate(&state, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f), NULL);
	zassert_true(motor_flux_id_accumulate(&state, 12.0f, 0.1f, 0.2f, 0.3f, 1.0f), NULL);
	zassert_equal(state.sample_count, 1u, NULL);
}

ZTEST(motor_commission_estimators, test_flux_identifies_nominal_model)
{
	const struct motor_flux_id_config cfg = {
		.rs_ohm = 0.45f,
		.ld_h = 0.0012f,
		.lq_h = 0.0015f,
		.min_abs_speed_rad_s = 5.0f,
		.min_speed_span_rad_s = 40.0f,
		.min_samples = 32U,
		.min_r2 = 0.95f,
		.require_positive_psi = true,
	};
	const float32_t psi_true = 0.0325f;
	const float32_t bias_true = -0.18f;
	struct motor_flux_id_state state = {0};
	struct motor_flux_id_result result = {0};

	motor_flux_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 64U; i++) {
		const float32_t speed_mag = 10.0f + 3.0f * (float32_t)(i % 32U);
		const float32_t speed = (i % 2U == 0U) ? speed_mag : -speed_mag;
		const float32_t id = 0.4f * sinf(0.11f * (float32_t)i);
		const float32_t iq = 0.7f * cosf(0.07f * (float32_t)i);
		const float32_t diq_dt = 5.0f * sinf(0.09f * (float32_t)i);
		const float32_t vq =
			flux_model_vq(&cfg, speed, id, iq, diq_dt, psi_true, bias_true);

		zassert_true(motor_flux_id_accumulate(&state, speed, id, iq, diq_dt, vq), NULL);
	}

	zassert_ok(motor_flux_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_equal(result.sample_count, 64u, NULL);
	zassert_within(result.psi_f_wb, psi_true, 1e-5f, NULL);
	zassert_within(result.bias_v, bias_true, 1e-5f, NULL);
	zassert_true(result.r2 > 0.9999f, NULL);
	zassert_true(result.residual_rms_v < 1e-3f, NULL);
}

ZTEST(motor_commission_estimators, test_flux_finalize_rejects_insufficient_samples)
{
	const struct motor_flux_id_config cfg = {
		.rs_ohm = 0.4f,
		.ld_h = 0.001f,
		.lq_h = 0.001f,
		.min_abs_speed_rad_s = 1.0f,
		.min_speed_span_rad_s = 1.0f,
		.min_samples = 6U,
		.min_r2 = 0.0f,
		.require_positive_psi = false,
	};
	struct motor_flux_id_state state = {0};
	struct motor_flux_id_result result = {0};

	motor_flux_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 4U; i++) {
		zassert_true(motor_flux_id_accumulate(&state, 10.0f + (float32_t)i,
						      0.0f, 0.0f, 0.0f,
						      1.0f + (float32_t)i), NULL);
	}

	zassert_equal(motor_flux_id_finalize(&state, &result), -ENODATA, NULL);
	zassert_false(result.valid, NULL);
	zassert_equal(result.sample_count, 4u, NULL);
}

ZTEST(motor_commission_estimators, test_flux_finalize_rejects_small_speed_span)
{
	const struct motor_flux_id_config cfg = {
		.rs_ohm = 0.4f,
		.ld_h = 0.001f,
		.lq_h = 0.001f,
		.min_abs_speed_rad_s = 1.0f,
		.min_speed_span_rad_s = 50.0f,
		.min_samples = 8U,
		.min_r2 = 0.0f,
		.require_positive_psi = false,
	};
	struct motor_flux_id_state state = {0};
	struct motor_flux_id_result result = {0};

	motor_flux_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 16U; i++) {
		const float32_t speed = 10.0f + 0.5f * (float32_t)i;
		const float32_t vq = flux_model_vq(&cfg, speed, 0.0f, 0.0f, 0.0f, 0.02f, 0.1f);
		zassert_true(motor_flux_id_accumulate(&state, speed, 0.0f, 0.0f, 0.0f, vq), NULL);
	}

	zassert_equal(motor_flux_id_finalize(&state, &result), -ENODATA, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_flux_finalize_flags_negative_psi_when_required)
{
	const struct motor_flux_id_config cfg = {
		.rs_ohm = 0.4f,
		.ld_h = 0.001f,
		.lq_h = 0.001f,
		.min_abs_speed_rad_s = 2.0f,
		.min_speed_span_rad_s = 40.0f,
		.min_samples = 20U,
		.min_r2 = 0.1f,
		.require_positive_psi = true,
	};
	struct motor_flux_id_state state = {0};
	struct motor_flux_id_result result = {0};

	motor_flux_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 32U; i++) {
		const float32_t speed = -62.0f + 4.0f * (float32_t)i;
		const float32_t vq = flux_model_vq(&cfg, speed, 0.0f, 0.0f, 0.0f, -0.03f, 0.2f);
		zassert_true(motor_flux_id_accumulate(&state, speed, 0.0f, 0.0f, 0.0f, vq), NULL);
	}

	zassert_ok(motor_flux_id_finalize(&state, &result), NULL);
	zassert_true(result.psi_f_wb < 0.0f, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_flux_finalize_flags_low_r2)
{
	const struct motor_flux_id_config cfg = {
		.rs_ohm = 0.4f,
		.ld_h = 0.001f,
		.lq_h = 0.001f,
		.min_abs_speed_rad_s = 2.0f,
		.min_speed_span_rad_s = 60.0f,
		.min_samples = 20U,
		.min_r2 = 0.85f,
		.require_positive_psi = false,
	};
	struct motor_flux_id_state state = {0};
	struct motor_flux_id_result result = {0};

	motor_flux_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 40U; i++) {
		const float32_t speed = -78.0f + 4.0f * (float32_t)i;
		const float32_t y = 0.01f * speed * speed + 1.0f;
		const float32_t vq = y + cfg.rs_ohm * 0.2f;
		zassert_true(motor_flux_id_accumulate(&state, speed, 0.0f, 0.2f, 0.0f, vq), NULL);
	}

	zassert_ok(motor_flux_id_finalize(&state, &result), NULL);
	zassert_true(result.r2 < cfg.min_r2, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_mech_init_and_rejects_bad_samples)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.12f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 4U,
		.min_r2 = 0.0f,
		.require_positive_inertia = true,
		.require_nonnegative_viscous = true,
	};
	struct motor_mech_id_state state = {0};

	motor_mech_id_init(&state, &cfg);
	zassert_equal(state.sample_count, 0u, NULL);
	zassert_false(motor_mech_id_accumulate(&state, NAN, 0.0f, 0.0f), NULL);
	zassert_true(motor_mech_id_accumulate(&state, 1.0f, 2.0f, 0.5f), NULL);
	zassert_equal(state.sample_count, 1u, NULL);

	state.cfg.kt_nm_per_a = 0.0f;
	zassert_false(motor_mech_id_accumulate(&state, 1.0f, 2.0f, 0.5f), NULL);
}

ZTEST(motor_commission_estimators, test_mech_identifies_nominal_model)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.18f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 64U,
		.min_r2 = 0.95f,
		.require_positive_inertia = true,
		.require_nonnegative_viscous = true,
	};
	const float32_t inertia_true = 0.0026f;
	const float32_t viscous_true = 0.015f;
	const float32_t coulomb_true = 0.075f;
	const float32_t offset_true = -0.012f;
	struct motor_mech_id_state state = {0};
	struct motor_mech_id_result result = {0};

	motor_mech_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 160U; i++) {
		const float32_t speed = ((float32_t)((int32_t)(i % 41U) - 20)) * 0.8f;
		const float32_t accel = ((float32_t)((int32_t)(i % 29U) - 14)) * 2.7f +
				       0.13f * speed;
		const float32_t torque =
			mech_model_torque(inertia_true, viscous_true, coulomb_true,
					  offset_true, speed, accel, cfg.sign_deadband_rad_s);
		const float32_t iq = torque / cfg.kt_nm_per_a;

		zassert_true(motor_mech_id_accumulate(&state, speed, accel, iq), NULL);
	}

	zassert_ok(motor_mech_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_within(result.inertia_kgm2, inertia_true, 1e-5f, NULL);
	zassert_within(result.viscous_friction_nm_per_rad_s, viscous_true, 1e-5f, NULL);
	zassert_within(result.coulomb_friction_nm, coulomb_true, 1e-5f, NULL);
	zassert_within(result.offset_friction_nm, offset_true, 1e-5f, NULL);
	zassert_true(result.r2 > 0.9999f, NULL);
	zassert_within(result.residual_rms_nm, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_commission_estimators, test_mech_finalize_rejects_insufficient_samples)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.1f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 8U,
		.min_r2 = 0.0f,
		.require_positive_inertia = false,
		.require_nonnegative_viscous = false,
	};
	struct motor_mech_id_state state = {0};
	struct motor_mech_id_result result = {0};

	motor_mech_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 6U; i++) {
		zassert_true(motor_mech_id_accumulate(&state, (float32_t)i, 1.0f, 0.5f), NULL);
	}

	zassert_equal(motor_mech_id_finalize(&state, &result), -ENODATA, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_mech_finalize_rejects_singular_matrix)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.1f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 4U,
		.min_r2 = 0.0f,
		.require_positive_inertia = false,
		.require_nonnegative_viscous = false,
	};
	struct motor_mech_id_state state = {0};
	struct motor_mech_id_result result = {0};

	motor_mech_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 8U; i++) {
		zassert_true(motor_mech_id_accumulate(&state, 0.0f, 0.0f, 1.0f), NULL);
	}

	zassert_equal(motor_mech_id_finalize(&state, &result), -ERANGE, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_mech_finalize_flags_negative_inertia)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.2f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 32U,
		.min_r2 = 0.0f,
		.require_positive_inertia = true,
		.require_nonnegative_viscous = false,
	};
	struct motor_mech_id_state state = {0};
	struct motor_mech_id_result result = {0};

	motor_mech_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 80U; i++) {
		const float32_t speed = ((float32_t)((int32_t)(i % 37U) - 18)) * 0.9f;
		const float32_t accel = ((float32_t)((int32_t)(i % 21U) - 10)) * 3.0f;
		const float32_t torque =
			mech_model_torque(-0.003f, 0.01f, 0.04f, 0.0f, speed, accel,
					  cfg.sign_deadband_rad_s);
		zassert_true(motor_mech_id_accumulate(&state, speed, accel,
						      torque / cfg.kt_nm_per_a), NULL);
	}

	zassert_ok(motor_mech_id_finalize(&state, &result), NULL);
	zassert_true(result.inertia_kgm2 < 0.0f, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_mech_finalize_flags_negative_viscous)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.2f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 32U,
		.min_r2 = 0.0f,
		.require_positive_inertia = false,
		.require_nonnegative_viscous = true,
	};
	struct motor_mech_id_state state = {0};
	struct motor_mech_id_result result = {0};

	motor_mech_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 96U; i++) {
		const float32_t speed = ((float32_t)((int32_t)(i % 45U) - 22)) * 0.7f;
		const float32_t accel = ((float32_t)((int32_t)(i % 17U) - 8)) * 2.5f;
		const float32_t torque =
			mech_model_torque(0.004f, -0.02f, 0.03f, 0.0f, speed, accel,
					  cfg.sign_deadband_rad_s);
		zassert_true(motor_mech_id_accumulate(&state, speed, accel,
						      torque / cfg.kt_nm_per_a), NULL);
	}

	zassert_ok(motor_mech_id_finalize(&state, &result), NULL);
	zassert_true(result.viscous_friction_nm_per_rad_s < 0.0f, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_mech_finalize_flags_low_r2)
{
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = 0.1f,
		.sign_deadband_rad_s = 0.5f,
		.min_samples = 32U,
		.min_r2 = 0.7f,
		.require_positive_inertia = false,
		.require_nonnegative_viscous = false,
	};
	struct motor_mech_id_state state = {0};
	struct motor_mech_id_result result = {0};

	motor_mech_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 80U; i++) {
		const float32_t speed = ((float32_t)((int32_t)(i % 39U) - 19)) * 0.6f;
		const float32_t accel = ((float32_t)((int32_t)(i % 23U) - 11)) * 1.9f;
		const float32_t iq = 2.0f;

		zassert_true(motor_mech_id_accumulate(&state, speed, accel, iq), NULL);
	}

	zassert_ok(motor_mech_id_finalize(&state, &result), NULL);
	zassert_true(result.r2 < cfg.min_r2, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_matrix_solve_3x3)
{
	float32_t A[3][3] = {
		{4.0f, 1.0f, 2.0f},
		{1.0f, 3.0f, 0.5f},
		{2.0f, 0.5f, 5.0f},
	};
	float32_t b[3] = {12.0f, 8.5f, 18.0f};
	float32_t x[3] = {0};

	zassert_true(motor_math_solve_linear_3x3(A, b, x), NULL);
	zassert_within(x[0], 1.0f, 1.0e-5f, NULL);
	zassert_within(x[1], 2.0f, 1.0e-5f, NULL);
	zassert_within(x[2], 3.0f, 1.0e-5f, NULL);
}

ZTEST(motor_commission_estimators, test_friction_plateau_identifies_model)
{
	const struct motor_mech_friction_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.max_abs_accel_rad_s2 = 0.2f,
		.min_samples = 16U,
		.min_samples_per_direction = 4U,
		.min_r2 = 0.95f,
		.require_nonnegative_viscous = true,
		.require_nonnegative_coulomb = true,
	};
	const float32_t b_true = 0.0018f;
	const float32_t tc_true = 0.021f;
	const float32_t t0_true = -0.0025f;
	struct motor_mech_friction_id_state state = {0};
	struct motor_mech_friction_id_result result = {0};

	motor_mech_friction_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 80U; i++) {
		float32_t speed = 2.0f + 0.15f * (float32_t)(i % 20U);
		if ((i & 1U) != 0U) {
			speed = -speed;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t torque = b_true * speed + tc_true * sign + t0_true;

		zassert_true(motor_mech_friction_id_accumulate(&state, speed, 0.0f,
							       torque / cfg.kt_nm_per_a,
							       0.0f), NULL);
	}

	zassert_ok(motor_mech_friction_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_within(result.viscous_friction_nm_per_rad_s, b_true, 1.0e-5f, NULL);
	zassert_within(result.coulomb_friction_nm, tc_true, 1.0e-5f, NULL);
	zassert_within(result.signed_coulomb_friction_nm, tc_true, 1.0e-5f, NULL);
	zassert_within(result.offset_friction_nm, t0_true, 1.0e-5f, NULL);
	zassert_true(result.residual_rms_nm < 1.0e-5f, NULL);
}

ZTEST(motor_commission_estimators, test_friction_plateau_preserves_signed_coulomb)
{
	const struct motor_mech_friction_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.max_abs_accel_rad_s2 = 0.2f,
		.min_samples = 16U,
		.min_samples_per_direction = 4U,
		.min_r2 = 0.95f,
		.require_nonnegative_viscous = true,
		.require_nonnegative_coulomb = true,
	};
	const float32_t tc_signed = -0.018f;
	struct motor_mech_friction_id_state state = {0};
	struct motor_mech_friction_id_result result = {0};

	motor_mech_friction_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 64U; i++) {
		float32_t speed = 2.0f + 0.1f * (float32_t)(i % 16U);
		if ((i & 1U) != 0U) {
			speed = -speed;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t torque = 0.001f * speed + tc_signed * sign;

		zassert_true(motor_mech_friction_id_accumulate(&state, speed, 0.0f,
							       torque / cfg.kt_nm_per_a,
							       0.0f), NULL);
	}

	zassert_ok(motor_mech_friction_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_within(result.coulomb_friction_nm, fabsf(tc_signed), 1.0e-5f, NULL);
	zassert_within(result.signed_coulomb_friction_nm, tc_signed, 1.0e-5f, NULL);
}

ZTEST(motor_commission_estimators, test_friction_plateau_rejects_negative_viscous)
{
	const struct motor_mech_friction_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.max_abs_accel_rad_s2 = 0.2f,
		.min_samples = 16U,
		.min_samples_per_direction = 4U,
		.min_r2 = 0.0f,
		.require_nonnegative_viscous = true,
		.require_nonnegative_coulomb = true,
	};
	struct motor_mech_friction_id_state state = {0};
	struct motor_mech_friction_id_result result = {0};

	motor_mech_friction_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 64U; i++) {
		float32_t speed = 2.0f + 0.1f * (float32_t)(i % 16U);
		if ((i & 1U) != 0U) {
			speed = -speed;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t torque = -0.002f * speed + 0.02f * sign;

		zassert_true(motor_mech_friction_id_accumulate(&state, speed, 0.0f,
							       torque / cfg.kt_nm_per_a,
							       0.0f), NULL);
	}

	zassert_ok(motor_mech_friction_id_finalize(&state, &result), NULL);
	zassert_false(result.valid, NULL);
	zassert_true(result.viscous_friction_nm_per_rad_s < 0.0f, NULL);
}

ZTEST(motor_commission_estimators, test_friction_plateau_zero_viscous_fallback)
{
	const struct motor_mech_friction_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.max_abs_accel_rad_s2 = 0.2f,
		.min_samples = 16U,
		.min_samples_per_direction = 4U,
		.min_r2 = 0.0f,
		.require_nonnegative_viscous = true,
		.require_nonnegative_coulomb = true,
	};
	struct motor_mech_friction_id_state state = {0};
	struct motor_mech_friction_id_result unconstrained = {0};
	struct motor_mech_friction_id_result zero_b = {0};

	motor_mech_friction_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 64U; i++) {
		float32_t speed = 2.0f + 0.1f * (float32_t)(i % 16U);
		if ((i & 1U) != 0U) {
			speed = -speed;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t torque = -0.002f * speed + 0.02f * sign;

		zassert_true(motor_mech_friction_id_accumulate(&state, speed, 0.0f,
							       torque / cfg.kt_nm_per_a,
							       0.0f), NULL);
	}

	zassert_ok(motor_mech_friction_id_finalize(&state, &unconstrained), NULL);
	zassert_false(unconstrained.valid, NULL);
	zassert_ok(motor_mech_friction_id_finalize_zero_viscous(&state, &zero_b), NULL);
	zassert_true(zero_b.valid, NULL);
	zassert_within(zero_b.viscous_friction_nm_per_rad_s, 0.0f, 1.0e-9f, NULL);
	zassert_true(zero_b.coulomb_friction_nm >= 0.0f, NULL);
}

ZTEST(motor_commission_estimators, test_friction_plateau_requires_directional_coverage)
{
	const struct motor_mech_friction_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.max_abs_accel_rad_s2 = 0.2f,
		.min_samples = 16U,
		.min_samples_per_direction = 4U,
		.min_r2 = 0.0f,
		.require_nonnegative_viscous = true,
		.require_nonnegative_coulomb = true,
	};
	struct motor_mech_friction_id_state state = {0};
	struct motor_mech_friction_id_result result = {0};

	motor_mech_friction_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 32U; i++) {
		zassert_true(motor_mech_friction_id_accumulate(&state, 3.0f, 0.0f,
							       0.1f, 0.0f), NULL);
	}

	zassert_equal(motor_mech_friction_id_finalize(&state, &result), -ENODATA, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_commission_estimators, test_inertia_transient_identifies_model)
{
	const struct motor_mech_inertia_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.min_abs_accel_rad_s2 = 1.0f,
		.viscous_friction_nm_per_rad_s = 0.0015f,
		.coulomb_friction_nm = 0.02f,
		.offset_friction_nm = -0.001f,
		.fallback_inertia_kgm2 = 5.7e-6f,
		.min_plausibility_ratio = 0.1f,
		.max_plausibility_ratio = 10.0f,
		.min_samples = 16U,
		.min_samples_per_accel_direction = 4U,
		.max_residual_rms_nm = 0.01f,
		.require_plausible = true,
	};
	const float32_t j_true = 6.0e-6f;
	struct motor_mech_inertia_id_state state = {0};
	struct motor_mech_inertia_id_result result = {0};

	motor_mech_inertia_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 80U; i++) {
		float32_t speed = 2.0f + 0.05f * (float32_t)(i % 20U);
		if ((i & 2U) != 0U) {
			speed = -speed;
		}
		float32_t accel = 20.0f + (float32_t)(i % 9U);
		if ((i & 1U) != 0U) {
			accel = -accel;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t torque = j_true * accel +
					 cfg.viscous_friction_nm_per_rad_s * speed +
					 cfg.coulomb_friction_nm * sign +
					 cfg.offset_friction_nm;

		zassert_true(motor_mech_inertia_id_accumulate(&state, speed, accel,
							      torque / cfg.kt_nm_per_a,
							      0.0f), NULL);
	}

	zassert_ok(motor_mech_inertia_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_within(result.inertia_kgm2, j_true, 1.0e-8f, NULL);
	zassert_true(result.residual_rms_nm < 1.0e-5f, NULL);
}

ZTEST(motor_commission_estimators, test_inertia_transient_allows_signed_coulomb_subtraction)
{
	const struct motor_mech_inertia_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.min_abs_accel_rad_s2 = 1.0f,
		.viscous_friction_nm_per_rad_s = 0.001f,
		.coulomb_friction_nm = -0.015f,
		.offset_friction_nm = 0.0005f,
		.fallback_inertia_kgm2 = 5.7e-6f,
		.min_plausibility_ratio = 0.1f,
		.max_plausibility_ratio = 10.0f,
		.min_samples = 16U,
		.min_samples_per_accel_direction = 4U,
		.max_residual_rms_nm = 0.01f,
		.require_plausible = true,
	};
	const float32_t j_true = 7.0e-6f;
	struct motor_mech_inertia_id_state state = {0};
	struct motor_mech_inertia_id_result result = {0};

	motor_mech_inertia_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 80U; i++) {
		float32_t speed = 2.0f + 0.05f * (float32_t)(i % 20U);
		if ((i & 2U) != 0U) {
			speed = -speed;
		}
		float32_t accel = 18.0f + (float32_t)(i % 7U);
		if ((i & 1U) != 0U) {
			accel = -accel;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t torque = j_true * accel +
					 cfg.viscous_friction_nm_per_rad_s * speed +
					 cfg.coulomb_friction_nm * sign +
					 cfg.offset_friction_nm;

		zassert_true(motor_mech_inertia_id_accumulate(&state, speed, accel,
							      torque / cfg.kt_nm_per_a,
							      0.0f), NULL);
	}

	zassert_ok(motor_mech_inertia_id_finalize(&state, &result), NULL);
	zassert_true(result.valid, NULL);
	zassert_within(result.inertia_kgm2, j_true, 1.0e-8f, NULL);
}

ZTEST(motor_commission_estimators, test_inertia_transient_rejects_low_accel)
{
	const struct motor_mech_inertia_id_config cfg = {
		.kt_nm_per_a = 0.32f,
		.sign_deadband_rad_s = 0.5f,
		.min_abs_accel_rad_s2 = 5.0f,
		.viscous_friction_nm_per_rad_s = 0.001f,
		.coulomb_friction_nm = 0.01f,
		.offset_friction_nm = 0.0f,
		.fallback_inertia_kgm2 = 5.7e-6f,
		.min_plausibility_ratio = 0.1f,
		.max_plausibility_ratio = 10.0f,
		.min_samples = 8U,
		.min_samples_per_accel_direction = 2U,
		.max_residual_rms_nm = 0.1f,
		.require_plausible = true,
	};
	struct motor_mech_inertia_id_state state = {0};
	struct motor_mech_inertia_id_result result = {0};

	motor_mech_inertia_id_init(&state, &cfg);
	for (uint32_t i = 0U; i < 32U; i++) {
		zassert_false(motor_mech_inertia_id_accumulate(&state, 3.0f, 1.0f,
							       0.1f, 0.0f), NULL);
	}

	zassert_equal(motor_mech_inertia_id_finalize(&state, &result), -ENODATA, NULL);
}

ZTEST(motor_commission_estimators, test_detent_subtraction_recovers_friction_fit)
{
	const struct motor_mech_friction_id_config cfg = {
		.kt_nm_per_a = 0.30f,
		.sign_deadband_rad_s = 0.5f,
		.max_abs_accel_rad_s2 = 0.2f,
		.min_samples = 16U,
		.min_samples_per_direction = 4U,
		.min_r2 = 0.90f,
		.require_nonnegative_viscous = true,
		.require_nonnegative_coulomb = true,
	};
	const float32_t b_true = 0.001f;
	const float32_t tc_true = 0.015f;
	struct motor_mech_friction_id_state raw = {0};
	struct motor_mech_friction_id_state corrected = {0};
	struct motor_mech_friction_id_result raw_result = {0};
	struct motor_mech_friction_id_result corrected_result = {0};

	motor_mech_friction_id_init(&raw, &cfg);
	motor_mech_friction_id_init(&corrected, &cfg);
	for (uint32_t i = 0U; i < 96U; i++) {
		float32_t speed = 2.0f + 0.02f * (float32_t)(i % 24U);
		if ((i & 1U) != 0U) {
			speed = -speed;
		}
		const float32_t sign = (speed > 0.0f) ? 1.0f : -1.0f;
		const float32_t detent = 0.004f * sinf(0.37f * (float32_t)i);
		const float32_t model = b_true * speed + tc_true * sign;
		const float32_t iq = (model + detent) / cfg.kt_nm_per_a;

		zassert_true(motor_mech_friction_id_accumulate(&raw, speed, 0.0f,
							       iq, 0.0f), NULL);
		zassert_true(motor_mech_friction_id_accumulate(&corrected, speed, 0.0f,
							       iq, detent), NULL);
	}

	zassert_ok(motor_mech_friction_id_finalize(&raw, &raw_result), NULL);
	zassert_ok(motor_mech_friction_id_finalize(&corrected, &corrected_result), NULL);
	zassert_true(corrected_result.valid, NULL);
	zassert_true(corrected_result.residual_rms_nm < raw_result.residual_rms_nm, NULL);
	zassert_within(corrected_result.viscous_friction_nm_per_rad_s, b_true, 1.0e-5f, NULL);
	zassert_within(corrected_result.coulomb_friction_nm, tc_true, 1.0e-5f, NULL);
}

ZTEST(motor_commission_estimators, test_accel_window_velocity_fit)
{
	float32_t speeds[11];
	uint32_t loops[11];
	const float32_t fs = 1000.0f;
	const float32_t accel = 12.5f;

	for (uint32_t i = 0U; i < ARRAY_SIZE(speeds); i++) {
		const float32_t t = ((float32_t)i - 5.0f) / fs;
		speeds[i] = 4.0f + accel * t;
		loops[i] = i;
	}

	float32_t estimated = 0.0f;
	zassert_true(motor_mech_accel_window_velocity_fit(speeds, loops, ARRAY_SIZE(speeds),
							  5U, 3U, fs, &estimated), NULL);
	zassert_within(estimated, accel, 1.0e-4f, NULL);
	zassert_false(motor_mech_accel_window_velocity_fit(speeds, loops, ARRAY_SIZE(speeds),
							   1U, 3U, fs, &estimated), NULL);
}

ZTEST_SUITE(motor_commission_estimators, NULL, NULL, NULL, NULL, NULL);
