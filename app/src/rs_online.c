/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rs_online.h"
#include <zephyr/dsp/types.h>
#include <zephyr/dsp/dsp.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <math.h>

#ifndef PI_F32
#define PI_F32 3.14159265358979323846f
#endif

void rs_online_init(struct rs_online_estimator *est,
                    float32_t Rs_init,
                    float32_t Rs_min,
                    float32_t Rs_max,
                    float32_t Ld,
                    float32_t Lq,
                    float32_t psi_f,
                    float32_t est_freq_hz,
                    float32_t probe_freq_hz,
                    float32_t gamma,
                    float32_t epsilon,
                    float32_t deriv_bw_hz,
                    float32_t Rs_lp_bw_hz)
{
    est->Rs_est     = Rs_init;
    est->Rs_min     = Rs_min;
    est->Rs_max     = Rs_max;
    est->gamma      = gamma;
    est->epsilon    = epsilon;

    est->probe_angle  = 0.0f;
    // probe_dtheta in degrees per sample: freq [Hz] * 360 [deg/cycle] / sample_rate [Hz]
    est->probe_dtheta = probe_freq_hz * 360.0f / est_freq_hz;

    // Store motor parameters
    est->Ld    = Ld;
    est->Lq    = Lq;
    est->psi_f = psi_f;

    est->Ts        = 1.0f / est_freq_hz;
    est->id_prev   = 0.0f;
    est->iq_prev   = 0.0f;

    // --- Initialize derivative filters: y[n] = b0*x[n] + a1*y[n-1] ---
    // For first-order low-pass: alpha = 2*pi*bw*Ts, b0 = alpha, a1 = (1-alpha)
    float32_t deriv_alpha = 2.0f * PI_F32 * deriv_bw_hz * est->Ts;
    deriv_alpha = CLAMP(deriv_alpha, 0.0f, 1.0f);  // Stability limit

    filter_fo_init(&est->did_dt_filt);
    filter_fo_set_b0(&est->did_dt_filt, deriv_alpha);
    filter_fo_set_a1(&est->did_dt_filt, (1.0f - deriv_alpha));

    filter_fo_init(&est->diq_dt_filt);
    filter_fo_set_b0(&est->diq_dt_filt, deriv_alpha);
    filter_fo_set_a1(&est->diq_dt_filt, (1.0f - deriv_alpha));

    // --- Initialize Rs low-pass filter ---
    float32_t Rs_alpha = 2.0f * PI_F32 * Rs_lp_bw_hz * est->Ts;
    Rs_alpha = CLAMP(Rs_alpha, 0.0f, 1.0f);

    filter_fo_init(&est->Rs_filt);
    filter_fo_set_b0(&est->Rs_filt, Rs_alpha);
    filter_fo_set_a1(&est->Rs_filt, (1.0f - Rs_alpha));
    filter_fo_set_y1(&est->Rs_filt, Rs_init);  // Initialize to starting value
}

void rs_online_update(struct rs_online_estimator *est,
                      float32_t vd, float32_t vq,
                      float32_t id, float32_t iq,
                      float32_t omega_e_dps)
{
    /*
     * Online Stator Resistance Estimator
     * 
     * Uses gradient descent to minimize voltage prediction error by adjusting Rs.
     * The voltage error is projected onto a slowly rotating probe direction to
     * ensure persistent excitation and prevent the gradient from stalling.
     * 
     * Model (with ωe in deg/s, converted to rad/s for voltage equations):
     *   vd = Rs*id - (ωe*π/180)*Lq*iq + Ld*did/dt
     *   vq = Rs*iq + (ωe*π/180)*(Ld*id + ψf) + Lq*diq/dt
     * 
     * Update law: Rs_est += γ * (e_par * i_par) / (ε + i_par²)
     * where e_par and i_par are error and current projected onto probe direction
     */

    // --- 0. Update di/dt estimates (filtered derivatives) ---

    // Raw discrete derivative: (i_k - i_{k-1}) / Ts
    float32_t raw_did_dt = (id - est->id_prev) / est->Ts;
    float32_t raw_diq_dt = (iq - est->iq_prev) / est->Ts;

    est->id_prev = id;
    est->iq_prev = iq;

    // First-order low-pass filter on derivatives to reduce noise
    float32_t did_dt = filter_fo_run_form_0(&est->did_dt_filt, raw_did_dt);
    float32_t diq_dt = filter_fo_run_form_0(&est->diq_dt_filt, raw_diq_dt);

    // --- 1. Predict dq voltages using full model with di/dt terms ---
    // Convert electrical speed from deg/s to rad/s for voltage model
    float32_t omega_e_rads = omega_e_dps * PI_F32 / 180.0f;

    float32_t vd_hat = est->Rs_est * id
                     - omega_e_rads * est->Lq * iq
                     + est->Ld * did_dt;

    float32_t vq_hat = est->Rs_est * iq
                     + omega_e_rads * (est->Ld * id + est->psi_f)
                     + est->Lq * diq_dt;

    float32_t evd = vd - vd_hat;
    float32_t evq = vq - vq_hat;

    // --- 2. Slow rotating probe direction in dq-plane ---

    est->probe_angle += est->probe_dtheta;

    // Keep angle bounded to [0, 360) to avoid loss of precision over time
    if (est->probe_angle >= 360.0f) {
        est->probe_angle -= 360.0f;
    }
    if (est->probe_angle < 0.0f) {
        est->probe_angle += 360.0f;
    }

    float32_t s, c;
    arm_sin_cos_f32(est->probe_angle, &s, &c);

    // Project currents and error onto probe direction
    float32_t i_par =  id * c + iq * s;
    float32_t e_par = evd * c + evq * s;

    // --- 3. Gradient update on Rs ---
    float32_t denom = est->epsilon + i_par * i_par;
    float32_t dRs   = est->gamma * (e_par * i_par) / denom;

    est->Rs_est += dRs;

    // Clamp to valid range
    est->Rs_est = CLAMP(est->Rs_est, est->Rs_min, est->Rs_max);

    // --- 4. Low-pass filtered Rs output ---
    filter_fo_run_form_0(&est->Rs_filt, est->Rs_est);
}
