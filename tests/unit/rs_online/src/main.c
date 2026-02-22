/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "rs_online.h"

ZTEST(rs_online, test_init_sets_fields_and_filter_state)
{
	struct rs_online_estimator est = {0};
	rs_online_init(&est,
		       1.0f,   /* Rs_init */
		       0.2f,   /* Rs_min */
		       2.0f,   /* Rs_max */
		       0.001f, /* Ld */
		       0.0012f,/* Lq */
		       0.03f,  /* psi_f */
		       1000.0f,/* est_freq_hz */
		       2.0f,   /* probe_freq_hz */
		       0.05f,  /* gamma */
		       0.1f,   /* epsilon */
		       50.0f,  /* deriv_bw_hz */
		       5.0f);  /* Rs_lp_bw_hz */

	zassert_within(est.Rs_est, 1.0f, 1e-6f, NULL);
	zassert_within(est.Rs_min, 0.2f, 1e-6f, NULL);
	zassert_within(est.Rs_max, 2.0f, 1e-6f, NULL);
	zassert_within(est.probe_dtheta, 0.72f, 1e-3f, NULL);
	zassert_within(est.Ts, 0.001f, 1e-6f, NULL);
	zassert_within(rs_online_get_filtered(&est), 1.0f, 1e-6f, NULL);
}

ZTEST(rs_online, test_update_keeps_estimate_within_bounds)
{
	struct rs_online_estimator est = {0};
	rs_online_init(&est, 1.0f, 0.5f, 1.5f,
		       0.001f, 0.001f, 0.02f,
		       1000.0f, 1.0f, 5.0f, 1e-6f, 100.0f, 10.0f);

	for (int i = 0; i < 300; i++) {
		rs_online_update(&est,
				 30.0f, -25.0f, /* vd, vq */
				 10.0f, -8.0f,  /* id, iq */
				 62.831852f);   /* omega_e_rad_s */
	}

	zassert_true(est.Rs_est >= est.Rs_min && est.Rs_est <= est.Rs_max, NULL);
	zassert_true(rs_online_get_filtered(&est) >= est.Rs_min &&
		     rs_online_get_filtered(&est) <= est.Rs_max, NULL);
}

ZTEST(rs_online, test_probe_angle_wraps)
{
	struct rs_online_estimator est = {0};
	rs_online_init(&est, 1.0f, 0.1f, 5.0f,
		       0.001f, 0.001f, 0.02f,
		       1000.0f, 10.0f, 0.1f, 0.01f, 50.0f, 2.0f);

	for (int i = 0; i < 1000; i++) {
		rs_online_update(&est, 1.0f, 1.0f, 0.1f, 0.1f, 0.0f);
	}

	zassert_true(est.probe_angle >= 0.0f && est.probe_angle < 360.0f, NULL);
}

ZTEST(rs_online, test_zero_current_case_remains_finite)
{
	struct rs_online_estimator est = {0};
	rs_online_init(&est, 0.8f, 0.1f, 2.0f,
		       0.001f, 0.001f, 0.02f,
		       2000.0f, 1.0f, 0.2f, 0.1f, 100.0f, 5.0f);

	for (int i = 0; i < 100; i++) {
		rs_online_update(&est, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	zassert_true(est.Rs_est >= est.Rs_min && est.Rs_est <= est.Rs_max, NULL);
	zassert_true(rs_online_get_filtered(&est) >= est.Rs_min &&
		     rs_online_get_filtered(&est) <= est.Rs_max, NULL);
}

ZTEST_SUITE(rs_online, NULL, NULL, NULL, NULL, NULL);
