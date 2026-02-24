/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <math.h>

#include "motor/filters/filter_fo.h"
#include "motor/filters/filter_so.h"
#include "motor/filters/pi.h"

ZTEST(pi_filter, test_filter_form0_response)
{
	struct filter_fo_f32 f = {0};
	filter_fo_init(&f);
	filter_fo_set_den_coeffs(&f, 0.5f);
	filter_fo_set_num_coeffs(&f, 0.5f, 0.0f);

	float32_t y1 = filter_fo_run_form_0(&f, 1.0f);
	float32_t y2 = filter_fo_run_form_0(&f, 1.0f);

	zassert_within(y1, 0.5f, 1e-6f, NULL);
	zassert_within(y2, 0.75f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_filter_full_form_uses_x1_term)
{
	struct filter_fo_f32 f = {0};
	filter_fo_init(&f);
	filter_fo_set_den_coeffs(&f, 0.0f);
	filter_fo_set_num_coeffs(&f, 1.0f, 1.0f);
	filter_fo_set_initial_conditions(&f, 2.0f, 0.0f);

	float32_t y = filter_fo_run(&f, 3.0f);

	zassert_within(y, 5.0f, 1e-6f, NULL);
	zassert_within(filter_fo_get_x1(&f), 3.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_filter_so_passthrough)
{
	struct filter_so_f32 f = {0};

	filter_so_init(&f);
	filter_so_set_passthrough(&f);

	zassert_within(filter_so_run(&f, 0.25f), 0.25f, 1e-6f, NULL);
	zassert_within(filter_so_run(&f, -0.75f), -0.75f, 1e-6f, NULL);
	zassert_within(filter_so_run(&f, 1.0f), 1.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_filter_so_notch_config_validation)
{
	struct filter_so_f32 f = {0};

	zassert_equal(filter_so_config_notch(NULL, 1000.0f, 50.0f, 2.0f), -EINVAL, NULL);
	zassert_equal(filter_so_config_notch(&f, 0.0f, 50.0f, 2.0f), -EINVAL, NULL);
	zassert_equal(filter_so_config_notch(&f, 1000.0f, 0.0f, 2.0f), -EINVAL, NULL);
	zassert_equal(filter_so_config_notch(&f, 1000.0f, 500.0f, 2.0f), -EINVAL, NULL);
	zassert_equal(filter_so_config_notch(&f, 1000.0f, 50.0f, 0.0f), -EINVAL, NULL);
}

ZTEST(pi_filter, test_filter_so_notch_reduces_center_frequency_gain)
{
	struct filter_so_f32 f = {0};
	const float32_t fs_hz = 1000.0f;
	const float32_t f0_hz = 50.0f;
	const float32_t q = 4.0f;
	const float32_t dt = 1.0f / fs_hz;
	float32_t t = 0.0f;
	float32_t input_energy = 0.0f;
	float32_t output_energy = 0.0f;

	zassert_ok(filter_so_config_notch(&f, fs_hz, f0_hz, q), NULL);
	filter_so_set_initial_conditions(&f, 0.0f, 0.0f, 0.0f, 0.0f);

	for (int i = 0; i < 2000; i++) {
		float32_t x = sinf(2.0f * PI_F32 * f0_hz * t);
		float32_t y = filter_so_run(&f, x);
		if (i > 400) {
			input_energy += x * x;
			output_energy += y * y;
		}
		t += dt;
	}

	zassert_true(output_energy < (0.2f * input_energy), NULL);
}

ZTEST(pi_filter, test_pi_parallel_no_integrator)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 2.0f, 0.0f);
	pi_set_min_max(&pi, -100.0f, 100.0f);
	pi_run_parallel(&pi, 3.0f, 1.0f, 0.0f, &out);

	zassert_within(out, 4.0f, 1e-6f, NULL);
	zassert_within(pi_get_ui(&pi), 0.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_pi_parallel_dynamic_clamp_blocks_positive_windup)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 2.0f, 1.0f);
	pi_set_min_max(&pi, -1.0f, 1.0f);
	pi_set_ui(&pi, 0.4f);

	/* Proportional path already saturates high; integrator must clamp to 0. */
	pi_run_parallel(&pi, 1.0f, 0.0f, 0.0f, &out);
	zassert_within(out, 1.0f, 1e-6f, NULL);
	zassert_within(pi_get_ui(&pi), 0.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_pi_parallel_integrator_clamp)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 0.0f, 1.0f);
	pi_set_min_max(&pi, -1.0f, 1.0f);

	for (int i = 0; i < 5; i++) {
		pi_run_parallel(&pi, 2.0f, 0.0f, 0.0f, &out);
	}

	zassert_within(out, 1.0f, 1e-6f, NULL);
	zassert_within(pi_get_ui(&pi), 1.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_pi_parallel_unwinds_after_saturation)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 0.0f, 1.0f);
	pi_set_min_max(&pi, -1.0f, 1.0f);

	for (int i = 0; i < 3; i++) {
		pi_run_parallel(&pi, 2.0f, 0.0f, 0.0f, &out);
	}
	zassert_within(pi_get_ui(&pi), 1.0f, 1e-6f, NULL);

	pi_run_parallel(&pi, -1.0f, 0.0f, 0.0f, &out);
	zassert_within(out, 0.0f, 1e-6f, NULL);
	zassert_within(pi_get_ui(&pi), 0.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_pi_series_and_pi_run_outputs)
{
	struct pi_f32 pi_series = {0};
	struct pi_f32 pi_basic = {0};
	float32_t out_series = 0.0f;
	float32_t out_basic = 0.0f;

	pi_init(&pi_series);
	pi_set_gains(&pi_series, 2.0f, 0.5f);
	pi_set_min_max(&pi_series, -10.0f, 10.0f);
	pi_run_series(&pi_series, 2.0f, 1.0f, 0.5f, &out_series);
	zassert_within(out_series, 3.5f, 1e-6f, NULL);

	pi_init(&pi_basic);
	pi_set_gains(&pi_basic, 2.0f, 0.5f);
	pi_set_min_max(&pi_basic, -10.0f, 10.0f);
	pi_run(&pi_basic, 2.0f, 1.0f, &out_basic);
	zassert_within(out_basic, 3.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_pi_series_integrates_proportional_term_per_step)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 2.0f, 0.5f);
	pi_set_min_max(&pi, -100.0f, 100.0f);

	pi_run_series(&pi, 3.0f, 2.0f, 0.0f, &out);
	zassert_within(out, 3.0f, 1e-6f, NULL);
	zassert_within(pi_get_ui(&pi), 1.0f, 1e-6f, NULL);

	pi_run_series(&pi, 3.0f, 2.0f, 0.0f, &out);
	zassert_within(out, 4.0f, 1e-6f, NULL);
	zassert_within(pi_get_ui(&pi), 2.0f, 1e-6f, NULL);
}

ZTEST(pi_filter, test_pi_run_clamps_negative_output)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 3.0f, 1.0f);
	pi_set_min_max(&pi, -1.0f, 1.0f);

	pi_run(&pi, 0.0f, 2.0f, &out);
	zassert_within(out, -1.0f, 1e-6f, NULL);
	zassert_true(pi_get_ui(&pi) <= 0.0f, NULL);
}

ZTEST(pi_filter, test_pi_stores_io_state_for_parallel_and_series)
{
	struct pi_f32 pi = {0};
	float32_t out = 0.0f;

	pi_init(&pi);
	pi_set_gains(&pi, 1.0f, 0.1f);
	pi_set_min_max(&pi, -10.0f, 10.0f);

	pi_run_parallel(&pi, 2.5f, -0.5f, 0.3f, &out);
	zassert_within(pi_get_ref_value(&pi), 2.5f, 1e-6f, NULL);
	zassert_within(pi_get_fback_value(&pi), -0.5f, 1e-6f, NULL);
	zassert_within(pi_get_ffwd_value(&pi), 0.3f, 1e-6f, NULL);

	pi_run_series(&pi, -1.0f, 0.4f, -0.2f, &out);
	zassert_within(pi_get_ref_value(&pi), -1.0f, 1e-6f, NULL);
	zassert_within(pi_get_fback_value(&pi), 0.4f, 1e-6f, NULL);
	zassert_within(pi_get_ffwd_value(&pi), -0.2f, 1e-6f, NULL);
}

ZTEST_SUITE(pi_filter, NULL, NULL, NULL, NULL, NULL);
