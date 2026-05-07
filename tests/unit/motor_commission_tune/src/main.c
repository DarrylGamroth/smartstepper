/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/ztest.h>

#include "motor/runtime/commission_tune.h"

static struct motor_commission_tune_config default_cfg(void)
{
	struct motor_commission_tune_config cfg;

	zassert_ok(motor_commission_tune_config_default(&cfg,
							7.0f,
							0.00005f,
							2.0f,
							300.0f,
							3000.0f),
		   NULL);
	return cfg;
}

static struct motor_commission_fit_summary valid_fit(void)
{
	struct motor_commission_fit_summary fit = {
		.psi_f_valid = true,
		.psi_f_wb = 0.0035f,
		.psi_f_r2 = 0.92f,
		.psi_f_residual_rms_v = 0.3f,
		.psi_f_sample_count = 180U,
		.mech_valid = true,
		.inertia_kgm2 = 1.4e-4f,
		.viscous_friction_nm_per_rad_s = 3.0e-4f,
		.mech_r2 = 0.70f,
		.mech_residual_rms_nm = 0.06f,
		.mech_sample_count = 220U,
	};

	return fit;
}

ZTEST(motor_commission_tune, test_default_config_is_valid)
{
	struct motor_commission_tune_config cfg = default_cfg();

	zassert_ok(motor_commission_tune_validate_config(&cfg), NULL);
	zassert_true(cfg.velocity_bw_hz > 0.0f, NULL);
	zassert_true(cfg.position_bw_ratio > 0.0f, NULL);
	zassert_true(cfg.position_bw_ratio <= 0.2f, NULL);
	zassert_true(cfg.iq_limit_a > 0.0f, NULL);
	zassert_true(cfg.iq_limit_a <= cfg.max_current_a, NULL);
}

ZTEST(motor_commission_tune, test_validate_config_rejects_invalid_values)
{
	struct motor_commission_tune_config cfg = default_cfg();

	cfg.position_bw_ratio = 0.25f;
	zassert_equal(motor_commission_tune_validate_config(&cfg), -EINVAL, NULL);

	cfg = default_cfg();
	cfg.iq_limit_a = cfg.max_current_a + 0.1f;
	zassert_equal(motor_commission_tune_validate_config(&cfg), -EINVAL, NULL);

	cfg = default_cfg();
	cfg.min_flux_samples = 0U;
	zassert_equal(motor_commission_tune_validate_config(&cfg), -EINVAL, NULL);
}

ZTEST(motor_commission_tune, test_rejects_low_flux_r2)
{
	struct motor_commission_tune_config cfg = default_cfg();
	struct motor_commission_fit_summary fit = valid_fit();
	struct motor_commission_tune_output out = {0};

	fit.psi_f_r2 = 0.1f;

	int ret = motor_commission_tune_compute(&fit, &cfg, &out);
	zassert_equal(ret, -ERANGE, NULL);
	zassert_false(out.accepted, NULL);
	zassert_true((out.reject_flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_R2) != 0U, NULL);
}

ZTEST(motor_commission_tune, test_rejects_invalid_mech_fit)
{
	struct motor_commission_tune_config cfg = default_cfg();
	struct motor_commission_fit_summary fit = valid_fit();
	struct motor_commission_tune_output out = {0};

	fit.mech_valid = false;
	fit.mech_sample_count = 4U;
	fit.inertia_kgm2 = -1.0e-4f;

	int ret = motor_commission_tune_compute(&fit, &cfg, &out);
	zassert_equal(ret, -ERANGE, NULL);
	zassert_false(out.accepted, NULL);
	zassert_true((out.reject_flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_INVALID) != 0U, NULL);
	zassert_true((out.reject_flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_SAMPLES) != 0U, NULL);
	zassert_true((out.reject_flags & MOTOR_COMMISSION_TUNE_REJECT_INERTIA_SIGN) != 0U, NULL);
}

ZTEST(motor_commission_tune, test_accepts_and_produces_positive_gains)
{
	struct motor_commission_tune_config cfg = default_cfg();
	struct motor_commission_fit_summary fit = valid_fit();
	struct motor_commission_tune_output out = {0};

	zassert_ok(motor_commission_tune_compute(&fit, &cfg, &out), NULL);
	zassert_true(out.accepted, NULL);
	zassert_equal(out.reject_flags, MOTOR_COMMISSION_TUNE_REJECT_NONE, NULL);
	zassert_true(out.kt_nm_per_a > 0.0f, NULL);
	zassert_true(out.velocity_kp_a_per_rad_s > 0.0f, NULL);
	zassert_true(out.velocity_ki_a_per_rad > 0.0f, NULL);
	zassert_true(out.position_kp_rad_s_per_rad > 0.0f, NULL);
	zassert_true(out.position_ki_rad_s2_per_rad > 0.0f, NULL);
	zassert_true(out.velocity_mpr_q_speed >= 0.003f &&
		     out.velocity_mpr_q_speed <= 0.05f,
		     "MPR q default is outside safe range: %f",
		     (double)out.velocity_mpr_q_speed);
	zassert_true(out.velocity_mpr_r_delta_iq >= 20.0f &&
		     out.velocity_mpr_r_delta_iq <= 100.0f,
		     "MPR r default is outside safe range: %f",
		     (double)out.velocity_mpr_r_delta_iq);
	zassert_true(out.velocity_mpr_max_delta_iq_a >= 0.0005f &&
		     out.velocity_mpr_max_delta_iq_a <= 0.0020f,
		     "MPR dIq default is outside safe range: %f",
		     (double)out.velocity_mpr_max_delta_iq_a);
	zassert_equal(out.velocity_mpr_disturbance_ki_nm_per_rad_s, 0.0f,
		      "MPR internal disturbance integrator should default off");
	zassert_false(out.velocity_dob_enable, NULL);
	zassert_true(out.velocity_dob_torque_limit_nm > 0.0f, NULL);
}

ZTEST(motor_commission_tune, test_mt6835_like_model_uses_low_speed_authority_floor)
{
	struct motor_commission_tune_config cfg;
	struct motor_commission_fit_summary fit = {
		.psi_f_valid = true,
		.psi_f_wb = 0.00455f,
		.psi_f_r2 = 0.92f,
		.psi_f_residual_rms_v = 0.3f,
		.psi_f_sample_count = 180U,
		.mech_valid = true,
		.inertia_kgm2 = 1.0e-4f,
		.viscous_friction_nm_per_rad_s = 2.7e-3f,
		.mech_r2 = 0.70f,
		.mech_residual_rms_nm = 0.06f,
		.mech_sample_count = 220U,
	};
	struct motor_commission_tune_output out = {0};

	zassert_ok(motor_commission_tune_config_default(&cfg,
							50.0f,
							0.00005f,
							1.5f,
							2.0f * 3.14159265f * 5.0f,
							2.0f * 3.14159265f * 20.0f),
		   NULL);
	cfg.iq_limit_a = 0.25f;

	zassert_ok(motor_commission_tune_compute(&fit, &cfg, &out), NULL);
	zassert_true(out.accepted, NULL);
	zassert_within(out.kt_nm_per_a, 0.34125f, 1.0e-5f, NULL);
	zassert_within(out.velocity_kp_a_per_rad_s, 0.07958f, 1.0e-4f,
		       "Kp should provide roughly full Iq authority at 0.5 Hz");
	zassert_within(out.velocity_ki_a_per_rad,
		       2.0f * out.velocity_kp_a_per_rad_s, 1.0e-5f,
		       "Ki should be capped relative to Kp for low-speed hybrid stepper startup");
}

ZTEST_SUITE(motor_commission_tune, NULL, NULL, NULL, NULL, NULL);
