/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RS_ONLINE_ESTIMATOR_H_
#define RS_ONLINE_ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>
#include "filter_fo.h"

#ifdef __cplusplus
extern "C" {
#endif

struct rs_online_estimator {
    // Estimated stator resistance [Ohm]
    float32_t Rs_est;

    // Limits
    float32_t Rs_min;
    float32_t Rs_max;

    // Gradient gain
    float32_t gamma;

    // Small epsilon to avoid divide-by-zero
    float32_t epsilon;

    // Slow rotating probe angle [deg]
    float32_t probe_angle;
    float32_t probe_dtheta;

    // --- Motor parameters (fixed) ---
    float32_t Ld;         // d-axis inductance [H]
    float32_t Lq;         // q-axis inductance [H]
    float32_t psi_f;      // flux linkage [Wb]

    // --- di/dt compensation filters ---
    float32_t Ts;                      // estimator sample period [s]
    float32_t id_prev;                 // previous id for derivative
    float32_t iq_prev;                 // previous iq for derivative
    struct filter_fo_f32 did_dt_filt;  // low-pass filter for di_d/dt
    struct filter_fo_f32 diq_dt_filt;  // low-pass filter for di_q/dt

    // --- Optional low-pass filtered Rs output ---
    struct filter_fo_f32 Rs_filt;      // low-pass filter for Rs estimate
};

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
                    float32_t Rs_lp_bw_hz);

void rs_online_update(struct rs_online_estimator *est,
                      float32_t vd, float32_t vq,
                      float32_t id, float32_t iq,
                      float32_t omega_e_dps);

/**
 * @brief Get the filtered Rs estimate
 *
 * @param est Estimator instance
 * @return Low-pass filtered Rs value
 */
static inline float32_t rs_online_get_filtered(const struct rs_online_estimator *est)
{
    return filter_fo_get_y1(&est->Rs_filt);
}

#ifdef __cplusplus
}
#endif

#endif /* RS_ONLINE_ESTIMATOR_H_ */