/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "motor_foc_voltage_pwm.h"
#include "motor/filters/pi.h"

static void init_zero_pi(struct pi_f32 *pi)
{
	pi_init(pi);
	pi_set_gains(pi, 0.0f, 0.0f);
	pi_set_min_max(pi, -1000.0f, 1000.0f);
}

static struct motor_foc_voltage_pwm_inputs make_base_inputs(void)
{
	struct motor_foc_voltage_pwm_inputs in = {
		.id_ref_a = 0.0f,
		.iq_ref_a = 0.0f,
		.id_a = 0.0f,
		.iq_a = 0.0f,
		.vbus_v = 24.0f,
		.max_modulation_index = 0.9f,
		.inv_park_angle_rad = 0.0f,
		.decoupling_enabled = false,
		.electrical_speed_rad_s = 0.0f,
		.ld_h = 0.001f,
		.lq_h = 0.001f,
		.flux_linkage_wb = 0.03f,
		.braking_enabled = false,
		.braking_iq_ref_a = 0.0f,
		.braking_speed_rad_s = 0.0f,
		.braking_vbus_limit_v = 1000.0f,
		.braking_vbus_margin_inv = 0.0f,
	};

	return in;
}

ZTEST(motor_foc_voltage_pwm, test_rejects_invalid_inputs)
{
	struct pi_f32 pi_d;
	struct pi_f32 pi_q;
	struct motor_foc_voltage_pwm_inputs in = make_base_inputs();
	struct motor_foc_voltage_pwm_outputs out = {0};

	init_zero_pi(&pi_d);
	init_zero_pi(&pi_q);

	zassert_equal(motor_foc_voltage_pwm_step(NULL, &pi_q, &in, &out), -EINVAL, NULL);
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, NULL, &in, &out), -EINVAL, NULL);
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, NULL, &out), -EINVAL, NULL);
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, NULL), -EINVAL, NULL);

	in.vbus_v = 0.0f;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.max_modulation_index = 0.0f;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.max_modulation_index = 1.1f;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.max_modulation_index = NAN;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.inv_park_angle_rad = NAN;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.id_ref_a = NAN;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.decoupling_enabled = true;
	in.electrical_speed_rad_s = NAN;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.braking_enabled = true;
	in.braking_vbus_margin_inv = 0.0f;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.braking_enabled = true;
	in.braking_vbus_margin_inv = 1.0f;
	in.braking_vbus_limit_v = -1.0f;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.braking_enabled = true;
	in.braking_vbus_margin_inv = NAN;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);

	in = make_base_inputs();
	in.braking_enabled = true;
	in.braking_vbus_margin_inv = 1.0f;
	in.braking_vbus_limit_v = NAN;
	zassert_equal(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), -EINVAL, NULL);
}

ZTEST(motor_foc_voltage_pwm, test_decoupling_disabled_keeps_feedforward_zero)
{
	struct pi_f32 pi_d;
	struct pi_f32 pi_q;
	struct motor_foc_voltage_pwm_inputs in = make_base_inputs();
	struct motor_foc_voltage_pwm_outputs out = {0};

	init_zero_pi(&pi_d);
	init_zero_pi(&pi_q);

	in.id_a = 2.0f;
	in.iq_a = -1.5f;
	in.electrical_speed_rad_s = 300.0f;
	in.decoupling_enabled = false;

	zassert_ok(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), NULL);
	zassert_within(out.vd_ff_v, 0.0f, 1e-6f, NULL);
	zassert_within(out.vq_ff_v, 0.0f, 1e-6f, NULL);
	zassert_within(out.vd_v, 0.0f, 1e-6f, NULL);
	zassert_within(out.vq_v, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_foc_voltage_pwm, test_decoupling_feedforward_terms_applied)
{
	struct pi_f32 pi_d;
	struct pi_f32 pi_q;
	struct motor_foc_voltage_pwm_inputs in = make_base_inputs();
	struct motor_foc_voltage_pwm_outputs out = {0};

	init_zero_pi(&pi_d);
	init_zero_pi(&pi_q);

	in.id_a = 2.0f;
	in.iq_a = 3.0f;
	in.ld_h = 0.001f;
	in.lq_h = 0.002f;
	in.flux_linkage_wb = 0.05f;
	in.electrical_speed_rad_s = 100.0f;
	in.decoupling_enabled = true;

	zassert_ok(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), NULL);
	zassert_within(out.vd_ff_v, -0.6f, 1e-5f, NULL);
	zassert_within(out.vq_ff_v, 5.2f, 1e-5f, NULL);
	zassert_within(out.vd_v, out.vd_ff_v, 1e-5f, NULL);
	zassert_within(out.vq_v, out.vq_ff_v, 1e-5f, NULL);
}

ZTEST(motor_foc_voltage_pwm, test_vq_is_limited_by_resulting_vd_headroom)
{
	struct pi_f32 pi_d;
	struct pi_f32 pi_q;
	struct motor_foc_voltage_pwm_inputs in = make_base_inputs();
	struct motor_foc_voltage_pwm_outputs out = {0};

	init_zero_pi(&pi_d);
	pi_init(&pi_q);
	pi_set_gains(&pi_q, 100.0f, 0.0f);
	pi_set_min_max(&pi_q, -1000.0f, 1000.0f);

	in.id_a = 0.0f;
	in.iq_ref_a = 10.0f;
	in.iq_a = 2.0f;
	in.ld_h = 0.0f;
	in.lq_h = 0.01f;
	in.flux_linkage_wb = 0.0f;
	in.electrical_speed_rad_s = 500.0f;
	in.decoupling_enabled = true;

	zassert_ok(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), NULL);

	const float32_t vmax = in.max_modulation_index * in.vbus_v;
	const float32_t expected_vd_ff = -10.0f;
	const float32_t expected_vq_limit = sqrtf((vmax * vmax) - (expected_vd_ff * expected_vd_ff));

	zassert_within(out.vd_v, expected_vd_ff, 1e-4f, NULL);
	zassert_within(out.vq_limit_v, expected_vq_limit, 1e-4f, NULL);
	zassert_within(out.vq_v, expected_vq_limit, 1e-4f, NULL);
}

ZTEST(motor_foc_voltage_pwm, test_pwm_outputs_stay_bounded_under_braking)
{
	struct pi_f32 pi_d;
	struct pi_f32 pi_q;
	struct motor_foc_voltage_pwm_inputs in = make_base_inputs();
	struct motor_foc_voltage_pwm_outputs out = {0};

	init_zero_pi(&pi_d);
	init_zero_pi(&pi_q);

	in.decoupling_enabled = true;
	in.id_a = 2.0f;
	in.iq_a = -3.0f;
	in.electrical_speed_rad_s = 600.0f;
	in.ld_h = 0.002f;
	in.lq_h = 0.002f;
	in.flux_linkage_wb = 0.1f;

	in.braking_enabled = true;
	in.braking_iq_ref_a = -0.5f;
	in.braking_speed_rad_s = 100.0f;
	in.braking_vbus_limit_v = 10.0f;
	in.braking_vbus_margin_inv = 1000.0f;

	zassert_ok(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), NULL);
	zassert_true(out.da_pu >= 0.0f && out.da_pu <= 1.0f, NULL);
	zassert_true(out.db_pu >= 0.0f && out.db_pu <= 1.0f, NULL);
	zassert_true(out.da_hb1_pu >= 0.0f && out.da_hb1_pu <= 1.0f, NULL);
	zassert_true(out.da_hb2_pu >= 0.0f && out.da_hb2_pu <= 1.0f, NULL);
	zassert_true(out.db_hb1_pu >= 0.0f && out.db_hb1_pu <= 1.0f, NULL);
	zassert_true(out.db_hb2_pu >= 0.0f && out.db_hb2_pu <= 1.0f, NULL);
}

ZTEST(motor_foc_voltage_pwm, test_pwm_outputs_stay_bounded_under_extreme_pi_drive)
{
	struct pi_f32 pi_d;
	struct pi_f32 pi_q;
	struct motor_foc_voltage_pwm_inputs in = make_base_inputs();
	struct motor_foc_voltage_pwm_outputs out = {0};

	pi_init(&pi_d);
	pi_set_gains(&pi_d, 2000.0f, 0.0f);
	pi_set_min_max(&pi_d, -1000.0f, 1000.0f);
	pi_init(&pi_q);
	pi_set_gains(&pi_q, 2000.0f, 0.0f);
	pi_set_min_max(&pi_q, -1000.0f, 1000.0f);

	in.id_ref_a = 50.0f;
	in.iq_ref_a = -50.0f;
	in.id_a = 0.0f;
	in.iq_a = 0.0f;
	in.inv_park_angle_rad = 1.1f;
	in.braking_enabled = true;
	in.braking_iq_ref_a = -1.0f;
	in.braking_speed_rad_s = 120.0f;
	in.braking_vbus_limit_v = 5.0f;
	in.braking_vbus_margin_inv = 10.0f;

	zassert_ok(motor_foc_voltage_pwm_step(&pi_d, &pi_q, &in, &out), NULL);
	zassert_true(out.da_pu >= 0.0f && out.da_pu <= 1.0f, NULL);
	zassert_true(out.db_pu >= 0.0f && out.db_pu <= 1.0f, NULL);
	zassert_true(out.da_hb1_pu >= 0.0f && out.da_hb1_pu <= 1.0f, NULL);
	zassert_true(out.da_hb2_pu >= 0.0f && out.da_hb2_pu <= 1.0f, NULL);
	zassert_true(out.db_hb1_pu >= 0.0f && out.db_hb1_pu <= 1.0f, NULL);
	zassert_true(out.db_hb2_pu >= 0.0f && out.db_hb2_pu <= 1.0f, NULL);
}

ZTEST_SUITE(motor_foc_voltage_pwm, NULL, NULL, NULL, NULL, NULL);
