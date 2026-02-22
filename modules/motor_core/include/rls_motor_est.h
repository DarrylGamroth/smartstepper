/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RLS_MOTOR_EST_H_
#define RLS_MOTOR_EST_H_

#include <zephyr/dsp/utils.h>
#include <stdbool.h>

/**
 * @file rls_motor_est.h
 * @brief RLS-based motor parameter estimator
 *
 * Estimates motor parameters (Rs, Ld/Lq, Vbias, Vdt_sign) from voltage
 * and current measurements using recursive least squares.
 */

/**
 * @brief RLS motor parameter estimator state
 *
 * Estimates 4 parameters per axis using the motor voltage equation.
 * Separate instances used for d-axis and q-axis estimation.
 */
struct rls_motor_est {
	/* Estimated parameters: [Rs, L, Vbias, Vdt_sign] */
	float32_t theta[4];

	/* Covariance matrix (4x4 symmetric, only upper triangle used) */
	float32_t P[4][4];

	/* Configuration */
	float32_t lambda;                    /* Forgetting factor (0,1] (0.9999 typical) */
	float32_t control_freq;              /* Control loop frequency for dI/dt */
	float32_t convergence_threshold;     /* Trace(P) threshold for convergence */
	float32_t Rs_init;                   /* Initial Rs estimate for full reset */
	float32_t L_init;                    /* Initial L estimate for full reset */
	float32_t P_init;                    /* Initial covariance diagonal for full reset */

	/* Statistics and diagnostics */
	uint32_t num_updates;                /* Total RLS updates performed */
	uint32_t num_rejected;               /* Updates rejected (denominator too small) */
	float32_t residual;                  /* Latest prediction error */
	float32_t residual_sum_sq;           /* Sum of squared residuals */
	bool converged;                      /* True if trace(P) < threshold */
	uint32_t convergence_count;          /* Update count at first convergence */
};

/**
 * @brief Initialize RLS motor estimator
 *
 * @param rls RLS estimator state
 * @param lambda Forgetting factor (0.9999 typical)
 * @param control_freq Control loop frequency in Hz
 * @param convergence_threshold Trace(P) threshold for convergence detection
 * @param Rs_init Initial resistance estimate (ohms)
 * @param L_init Initial inductance estimate (henries)
 * @param P_init Initial covariance diagonal value
 */
void rls_motor_est_init(struct rls_motor_est *rls,
                        float32_t lambda,
                        float32_t control_freq,
                        float32_t convergence_threshold,
                        float32_t Rs_init,
                        float32_t L_init,
                        float32_t P_init);

/**
 * @brief Validate RLS configuration parameters.
 *
 * @param lambda Forgetting factor in range (0, 1]
 * @param control_freq Control loop frequency in Hz (> 0)
 * @param convergence_threshold Trace(P) threshold (> 0)
 * @param Rs_init Initial resistance estimate (ohms, finite)
 * @param L_init Initial inductance estimate (henries, finite)
 * @param P_init Initial covariance diagonal value (finite, > 0)
 *
 * @retval 0 Configuration is valid
 * @retval -EINVAL Configuration is invalid
 */
int rls_motor_est_validate_config(float32_t lambda,
				  float32_t control_freq,
				  float32_t convergence_threshold,
				  float32_t Rs_init,
				  float32_t L_init,
				  float32_t P_init);

/**
 * @brief Update RLS estimator with new measurement
 *
 * @param rls RLS estimator state
 * @param V_meas Measured voltage (from PI controller output)
 * @param I Current measurement
 * @param I_prev Previous current (for derivative calculation)
 * @param omega Electrical angular velocity (rad/s)
 * @param L_cross Cross-coupling inductance (Lq for d-axis, Ld for q-axis)
 * @param I_cross Cross-coupling current (Iq for d-axis, Id for q-axis)
 * @param sample_period_s Effective sample period between I and I_prev [s]
 */
void rls_motor_est_update(struct rls_motor_est *rls,
                          float32_t V_meas,
                          float32_t I,
                          float32_t I_prev,
                          float32_t omega,
                          float32_t L_cross,
                          float32_t I_cross,
                          float32_t sample_period_s);

/**
 * @brief Reset RLS estimator to configured initial state
 *
 * Restores initial parameters and covariance captured at init time,
 * and clears all convergence/residual statistics.
 *
 * @param rls RLS estimator state
 */
void rls_motor_est_reset(struct rls_motor_est *rls);

/* Parameter access inline functions */

static inline float32_t rls_motor_est_get_Rs(const struct rls_motor_est *rls)
{
	return rls->theta[0];
}

static inline float32_t rls_motor_est_get_L(const struct rls_motor_est *rls)
{
	return rls->theta[1];
}

static inline float32_t rls_motor_est_get_Vbias(const struct rls_motor_est *rls)
{
	return rls->theta[2];
}

static inline float32_t rls_motor_est_get_Vdt_sign(const struct rls_motor_est *rls)
{
	return rls->theta[3];
}

static inline bool rls_motor_est_is_converged(const struct rls_motor_est *rls)
{
	return rls->converged;
}

static inline float32_t rls_motor_est_get_residual(const struct rls_motor_est *rls)
{
	return rls->residual;
}

#endif /* RLS_MOTOR_EST_H_ */
