/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>
#include <errno.h>
#include <zephyr/ztest.h>

#include "rls_motor_est.h"

static void clear_covariance(struct rls_motor_est *rls)
{
	for (size_t i = 0; i < 4U; i++) {
		for (size_t j = 0; j < 4U; j++) {
			rls->P[i][j] = 0.0f;
		}
	}
}

static float32_t trace_covariance(const struct rls_motor_est *rls)
{
	return rls->P[0][0] + rls->P[1][1] + rls->P[2][2] + rls->P[3][3];
}

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

ZTEST(rls_motor_est, test_validate_config)
{
	zassert_ok(rls_motor_est_validate_config(0.99f, 20000.0f, 0.5f, 1.0f, 0.002f, 10.0f), NULL);
	zassert_equal(rls_motor_est_validate_config(0.0f, 20000.0f, 0.5f, 1.0f, 0.002f, 10.0f),
		      -EINVAL, NULL);
	zassert_equal(rls_motor_est_validate_config(1.1f, 20000.0f, 0.5f, 1.0f, 0.002f, 10.0f),
		      -EINVAL, NULL);
	zassert_equal(rls_motor_est_validate_config(0.99f, 0.0f, 0.5f, 1.0f, 0.002f, 10.0f),
		      -EINVAL, NULL);
}

ZTEST(rls_motor_est, test_init_clamps_seed_parameters_and_covariance_floor)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 20000.0f, 0.5f, 0.0f, 1.0f, 0.0f);

	zassert_within(rls.theta[0], 0.1f, 1e-6f, NULL);
	zassert_within(rls.theta[1], 0.1f, 1e-6f, NULL);

	for (size_t i = 0U; i < 4U; i++) {
		zassert_true(rls.P[i][i] >= 1e-6f, NULL);
	}
}

ZTEST(rls_motor_est, test_update_accepts_valid_sample)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 20000.0f, 0.5f, 1.0f, 0.002f, 50.0f);

	rls_motor_est_update(&rls, 1.5f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_equal(rls.num_updates, 1u, NULL);
	zassert_equal(rls.num_rejected, 0u, NULL);
}

ZTEST(rls_motor_est, test_update_uses_control_freq_for_nonpositive_sample_period)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 1000.0f, 0.5f, 1.0f, 0.002f, 20.0f);

	rls_motor_est_update(&rls, 1.5f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f);
	rls_motor_est_update(&rls, 1.5f, 0.3f, 0.2f, 0.0f, 0.0f, 0.0f, -0.0001f);
	zassert_equal(rls.num_updates, 2u, NULL);
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

ZTEST(rls_motor_est, test_update_rejects_invalid_lambda)
{
	struct rls_motor_est rls = {0};
	float32_t theta_before[4];

	rls_motor_est_init(&rls, 0.99f, 1000.0f, 0.5f, 1.0f, 0.002f, 20.0f);
	memcpy(theta_before, rls.theta, sizeof(theta_before));
	rls.lambda = 0.0f;

	rls_motor_est_update(&rls, 1.2f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_equal(rls.num_updates, 0u, NULL);
	zassert_equal(rls.num_rejected, 1u, NULL);
	for (size_t i = 0U; i < 4U; i++) {
		zassert_within(rls.theta[i], theta_before[i], 1e-8f, NULL);
	}

	rls.lambda = 1.1f;
	rls_motor_est_update(&rls, 1.2f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_equal(rls.num_updates, 0u, NULL);
	zassert_equal(rls.num_rejected, 2u, NULL);
}

ZTEST(rls_motor_est, test_update_rejects_nonfinite_sample)
{
	struct rls_motor_est rls = {0};
	float32_t theta_before[4];

	rls_motor_est_init(&rls, 0.99f, 1000.0f, 0.5f, 1.0f, 0.002f, 20.0f);
	memcpy(theta_before, rls.theta, sizeof(theta_before));

	rls_motor_est_update(&rls, NAN, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_equal(rls.num_updates, 0u, NULL);
	zassert_equal(rls.num_rejected, 1u, NULL);
	for (size_t i = 0U; i < 4U; i++) {
		zassert_within(rls.theta[i], theta_before[i], 1e-8f, NULL);
	}
}

ZTEST(rls_motor_est, test_update_rejects_small_denominator_without_state_change)
{
	struct rls_motor_est rls = {0};
	float32_t theta_before[4];

	rls_motor_est_init(&rls, 0.1f, 20000.0f, 0.5f, 1.0f, 0.002f, 50.0f);
	clear_covariance(&rls);
	/* Force phiᵀPphi strongly negative so denom guard triggers. */
	rls.P[0][0] = -0.2f;
	memcpy(theta_before, rls.theta, sizeof(theta_before));

	rls_motor_est_update(&rls, 2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.001f);

	zassert_equal(rls.num_updates, 0u, NULL);
	zassert_equal(rls.num_rejected, 1u, NULL);
	for (size_t i = 0U; i < 4U; i++) {
		zassert_within(rls.theta[i], theta_before[i], 1e-8f, NULL);
	}
}

ZTEST(rls_motor_est, test_cross_coupling_compensation_changes_estimation)
{
	struct rls_motor_est base = {0};
	struct rls_motor_est compensated = {0};

	rls_motor_est_init(&base, 0.99f, 20000.0f, 0.5f, 1.0f, 0.002f, 50.0f);
	rls_motor_est_init(&compensated, 0.99f, 20000.0f, 0.5f, 1.0f, 0.002f, 50.0f);

	rls_motor_est_update(&base, 1.0f, 0.2f, 0.1f, 0.0f, 0.01f, 0.5f, 0.001f);
	rls_motor_est_update(&compensated, 1.0f, 0.2f, 0.1f, 200.0f, 0.01f, 0.5f, 0.001f);

	zassert_true(fabsf(base.residual - compensated.residual) > 0.1f, NULL);
}

ZTEST(rls_motor_est, test_convergence_and_divergence_hysteresis)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.99f, 20000.0f, 10.0f, 1.0f, 0.002f, 1.0f);

	rls_motor_est_update(&rls, 1.0f, 0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_true(rls.converged, NULL);
	zassert_equal(rls.convergence_count, 1u, NULL);
	zassert_true(trace_covariance(&rls) < rls.convergence_threshold, NULL);

	for (size_t i = 0U; i < 4U; i++) {
		for (size_t j = 0U; j < 4U; j++) {
			rls.P[i][j] = 0.0f;
		}
		rls.P[i][i] = 100.0f;
	}

	rls_motor_est_update(&rls, 1.1f, 0.3f, 0.2f, 0.0f, 0.0f, 0.0f, 0.001f);
	zassert_false(rls.converged, NULL);
	zassert_true(trace_covariance(&rls) > (rls.convergence_threshold * 2.0f), NULL);
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

ZTEST(rls_motor_est, test_covariance_diagonal_stays_positive_and_finite)
{
	struct rls_motor_est rls = {0};
	rls_motor_est_init(&rls, 0.95f, 20000.0f, 0.5f, 1.0f, 0.002f, 1e-5f);

	for (int i = 0; i < 256; i++) {
		float32_t i_meas = ((float32_t)(i % 7) - 3.0f) * 0.4f;
		float32_t i_prev = i_meas - 0.05f;
		rls_motor_est_update(&rls, 3.0f + (float32_t)(i % 11), i_meas, i_prev,
				     50.0f, 0.001f, 0.2f, 0.0005f);
	}

	for (size_t i = 0U; i < 4U; i++) {
		zassert_true(isfinite(rls.P[i][i]), NULL);
		zassert_true(rls.P[i][i] >= 1e-6f, NULL);
	}
}

ZTEST_SUITE(rls_motor_est, NULL, NULL, NULL, NULL, NULL);
