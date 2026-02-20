/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "filter_fo.h"
#include "pi.h"

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

ZTEST_SUITE(pi_filter, NULL, NULL, NULL, NULL, NULL);
