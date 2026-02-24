/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rls_motor_est.c
 * @brief RLS motor parameter estimator implementation
 *
 * Implements 4-parameter recursive least squares for motor parameter
 * identification using explicit matrix operations for efficiency.
 */

#include "motor/estimation/rls_motor_est.h"
#include "motor/math/math_constants.h"
#include <errno.h>
#include <zephyr/sys/util.h>
#include <math.h>

/* Parameter bounds (from devicetree or config.h) */
#define RS_MIN_OHM       0.1f
#define RS_MAX_OHM       50.0f
#define LD_MIN_H         0.0001f
#define LD_MAX_H         0.1f
#define VBIAS_MAX_V      5.0f
#define VDT_SIGN_MAX_V   2.0f
#define P_MIN            1e-6f
#define LAMBDA_MIN       1e-6f
#define LAMBDA_MAX       1.0f

int rls_motor_est_validate_config(float32_t lambda,
				  float32_t control_freq,
				  float32_t convergence_threshold,
				  float32_t Rs_init,
				  float32_t L_init,
				  float32_t P_init)
{
	if (!isfinite(lambda) || lambda <= LAMBDA_MIN || lambda > LAMBDA_MAX ||
	    !isfinite(control_freq) || control_freq <= 0.0f ||
	    !isfinite(convergence_threshold) || convergence_threshold <= 0.0f ||
	    !isfinite(Rs_init) || !isfinite(L_init) ||
	    !isfinite(P_init) || P_init <= 0.0f) {
		return -EINVAL;
	}

	return 0;
}

void rls_motor_est_init(struct rls_motor_est *rls,
                        float32_t lambda,
                        float32_t control_freq,
                        float32_t convergence_threshold,
                        float32_t Rs_init,
                        float32_t L_init,
                        float32_t P_init)
{
	if (rls == NULL) {
		return;
	}

	if (!isfinite(lambda) || lambda <= LAMBDA_MIN || lambda > LAMBDA_MAX) {
		lambda = 1.0f;
	}
	if (!isfinite(control_freq) || control_freq < 0.0f) {
		control_freq = 0.0f;
	}
	if (!isfinite(convergence_threshold) || convergence_threshold <= 0.0f) {
		convergence_threshold = 1.0f;
	}
	if (!isfinite(Rs_init)) {
		Rs_init = RS_MIN_OHM;
	}
	if (!isfinite(L_init)) {
		L_init = LD_MIN_H;
	}
	if (!isfinite(P_init)) {
		P_init = P_MIN;
	}

	/* Configuration */
	rls->lambda = lambda;
	rls->control_freq = control_freq;
	rls->convergence_threshold = convergence_threshold;
	rls->Rs_init = Rs_init;
	rls->L_init = L_init;
	rls->P_init = fmaxf(P_init, P_MIN);

	rls_motor_est_reset(rls);
}

void rls_motor_est_update(struct rls_motor_est *rls,
                          float32_t V_meas,
                          float32_t I,
                          float32_t I_prev,
                          float32_t omega,
                          float32_t L_cross,
                          float32_t I_cross,
                          float32_t sample_period_s)
{
	if (rls == NULL) {
		return;
	}
	if (!isfinite(rls->lambda) || rls->lambda <= LAMBDA_MIN || rls->lambda > LAMBDA_MAX) {
		rls->num_rejected++;
		return;
	}

	/* Compensate cross-coupling: V_compensated = V_meas + ω*L_cross*I_cross */
	float32_t V_compensated = V_meas + omega * L_cross * I_cross;
	if (!isfinite(V_compensated)) {
		rls->num_rejected++;
		return;
	}

	/* Calculate current derivative with effective elapsed sample period */
	float32_t Ts = sample_period_s;
	if (Ts <= 0.0f && rls->control_freq > 0.0f) {
		Ts = 1.0f / rls->control_freq;
	}
	if (!isfinite(Ts) || Ts <= 0.0f) {
		rls->num_rejected++;
		return;
	}
	float32_t dI_dt = (I - I_prev) / Ts;
	if (!isfinite(dI_dt)) {
		rls->num_rejected++;
		return;
	}

	/* Build regression vector φ[k] = [I, dI/dt, 1, sign(I)]ᵀ */
	float32_t phi[4];
	phi[0] = I;
	phi[1] = dI_dt;
	phi[2] = 1.0f;
	phi[3] = (I >= 0.0f) ? 1.0f : -1.0f;

	/* Predicted voltage: y_pred = φᵀ * θ */
	float32_t y_pred = phi[0] * rls->theta[0] +
	                   phi[1] * rls->theta[1] +
	                   phi[2] * rls->theta[2] +
	                   phi[3] * rls->theta[3];

	/* Prediction error */
	float32_t error = V_compensated - y_pred;

	/* Compute P * φ (4x4 * 4x1 = 4x1) */
	float32_t P_phi[4];
	P_phi[0] = rls->P[0][0] * phi[0] + rls->P[0][1] * phi[1] +
	           rls->P[0][2] * phi[2] + rls->P[0][3] * phi[3];
	P_phi[1] = rls->P[0][1] * phi[0] + rls->P[1][1] * phi[1] +
	           rls->P[1][2] * phi[2] + rls->P[1][3] * phi[3];
	P_phi[2] = rls->P[0][2] * phi[0] + rls->P[1][2] * phi[1] +
	           rls->P[2][2] * phi[2] + rls->P[2][3] * phi[3];
	P_phi[3] = rls->P[0][3] * phi[0] + rls->P[1][3] * phi[1] +
	           rls->P[2][3] * phi[2] + rls->P[3][3] * phi[3];

	/* Denominator: λ + φᵀ * P * φ */
	float32_t phi_P_phi = phi[0] * P_phi[0] + phi[1] * P_phi[1] +
		                      phi[2] * P_phi[2] + phi[3] * P_phi[3];
	float32_t denom = rls->lambda + phi_P_phi;

	/* Guard against numerical issues */
	if (!isfinite(denom) || denom < 1e-6f) {
		rls->num_rejected++;
		return;
	}

	float32_t denom_inv = 1.0f / denom;

	/* Kalman gain: K = (P * φ) / denom */
	float32_t K[4];
	K[0] = P_phi[0] * denom_inv;
	K[1] = P_phi[1] * denom_inv;
	K[2] = P_phi[2] * denom_inv;
	K[3] = P_phi[3] * denom_inv;

	/* Update parameters: θ = θ + K * error */
	rls->theta[0] += K[0] * error;
	rls->theta[1] += K[1] * error;
	rls->theta[2] += K[2] * error;
	rls->theta[3] += K[3] * error;

	/* Apply parameter bounds */
	rls->theta[0] = clampf(rls->theta[0], RS_MIN_OHM, RS_MAX_OHM);
	rls->theta[1] = clampf(rls->theta[1], LD_MIN_H, LD_MAX_H);
	rls->theta[2] = clampf(rls->theta[2], -VBIAS_MAX_V, VBIAS_MAX_V);
	rls->theta[3] = clampf(rls->theta[3], -VDT_SIGN_MAX_V, VDT_SIGN_MAX_V);

	/* Update covariance: P = (P - K * φᵀ * P) / λ
	 * Only update upper triangle (P is symmetric)
	 */
	float32_t lambda_inv = 1.0f / rls->lambda;

	/* Row 0 */
	rls->P[0][0] = (rls->P[0][0] - K[0] * P_phi[0]) * lambda_inv;
	rls->P[0][1] = (rls->P[0][1] - K[0] * P_phi[1]) * lambda_inv;
	rls->P[0][2] = (rls->P[0][2] - K[0] * P_phi[2]) * lambda_inv;
	rls->P[0][3] = (rls->P[0][3] - K[0] * P_phi[3]) * lambda_inv;

	/* Row 1 (upper triangle only) */
	rls->P[1][1] = (rls->P[1][1] - K[1] * P_phi[1]) * lambda_inv;
	rls->P[1][2] = (rls->P[1][2] - K[1] * P_phi[2]) * lambda_inv;
	rls->P[1][3] = (rls->P[1][3] - K[1] * P_phi[3]) * lambda_inv;

	/* Row 2 (upper triangle only) */
	rls->P[2][2] = (rls->P[2][2] - K[2] * P_phi[2]) * lambda_inv;
	rls->P[2][3] = (rls->P[2][3] - K[2] * P_phi[3]) * lambda_inv;

	/* Row 3 (diagonal only) */
	rls->P[3][3] = (rls->P[3][3] - K[3] * P_phi[3]) * lambda_inv;

	/* Enforce minimum covariance (prevent numerical collapse) */
	for (int i = 0; i < 4; i++) {
		rls->P[i][i] = fmaxf(rls->P[i][i], P_MIN);
	}

	/* Update statistics */
	rls->num_updates++;
	rls->residual_sum_sq += error * error;
	rls->residual = error;

	/* Convergence detection */
	float32_t trace_P = rls->P[0][0] + rls->P[1][1] + rls->P[2][2] + rls->P[3][3];

	if (!rls->converged) {
		if (trace_P < rls->convergence_threshold) {
			rls->converged = true;
			rls->convergence_count = rls->num_updates;
		}
	} else {
		/* Check divergence with hysteresis */
		if (trace_P > rls->convergence_threshold * 2.0f) {
			rls->converged = false;
		}
	}
}

void rls_motor_est_reset(struct rls_motor_est *rls)
{
	/* Restore estimator state to init-time values */
	rls->theta[0] = clampf(rls->Rs_init, RS_MIN_OHM, RS_MAX_OHM);
	rls->theta[1] = clampf(rls->L_init, LD_MIN_H, LD_MAX_H);
	rls->theta[2] = 0.0f;
	rls->theta[3] = 0.0f;

	/* Reset covariance as diagonal matrix (P = P_init * I) */
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			rls->P[i][j] = (i == j) ? rls->P_init : 0.0f;
		}
	}

	/* Reset statistics */
	rls->num_updates = 0;
	rls->num_rejected = 0;
	rls->residual = 0.0f;
	rls->residual_sum_sq = 0.0f;
	rls->converged = false;
	rls->convergence_count = 0;
}
