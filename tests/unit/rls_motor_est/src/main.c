/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "rls_motor_est.h"

ZTEST(rls_motor_est, test_init_and_reset_restore_state)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 20000.0f, 0.5f, 1.2f, 0.003f, 10.0f);

	zassert_within(rls.theta[0], 1.2f, 1e-6f, NULL);
	zassert_within(rls.theta[1], 0.003f, 1e-6f, NULL);
	zassert_within(rls.theta[2], 0.0f, 1e-6f, NULL);
	zassert_within(rls.theta[3], 0.0f, 1e-6f, NULL);
	zassert_equal(rls.num_updates, 0u, NULL);

	rls_motor_est_update(&rls, 2.0f, 0.4f, 0.3f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_true(rls.num_updates > 0u, NULL);

	rls_motor_est_reset(&rls);
	zassert_equal(rls.num_updates, 0u, NULL);
	zassert_equal(rls.num_rejected, 0u, NULL);
	zassert_false(rls.converged, NULL);
	zassert_within(rls.theta[0], 1.2f, 1e-6f, NULL);
	zassert_within(rls.theta[1], 0.003f, 1e-6f, NULL);
}

ZTEST(rls_motor_est, test_update_accepts_valid_sample)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 20000.0f, 0.5f, 1.0f, 0.002f, 50.0f);

	rls_motor_est_update(&rls, 1.5f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_equal(rls.num_updates, 1u, NULL);
	zassert_equal(rls.num_rejected, 0u, NULL);
}

ZTEST(rls_motor_est, test_update_rejects_invalid_sample_period_without_control_freq)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 0.0f, 0.5f, 1.0f, 0.002f, 50.0f);

	rls_motor_est_update(&rls, 1.0f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f);
	zassert_equal(rls.num_updates, 0u, NULL);
	zassert_equal(rls.num_rejected, 1u, NULL);
}

ZTEST(rls_motor_est, test_parameter_bounds_enforced)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 20000.0f, 0.5f, 1.0f, 0.002f, 100.0f);

	for (int i = 0; i < 200; i++) {
		rls_motor_est_update(&rls, 10000.0f, 20.0f, -20.0f, 1000.0f, 0.1f, 20.0f, 0.001f);
	}

	zassert_true(rls.theta[0] >= 0.1f && rls.theta[0] <= 50.0f, NULL);
	zassert_true(rls.theta[1] >= 0.0001f && rls.theta[1] <= 0.1f, NULL);
	zassert_true(rls.theta[2] >= -5.0f && rls.theta[2] <= 5.0f, NULL);
	zassert_true(rls.theta[3] >= -2.0f && rls.theta[3] <= 2.0f, NULL);
}

ZTEST_SUITE(rls_motor_est, NULL, NULL, NULL, NULL, NULL);
