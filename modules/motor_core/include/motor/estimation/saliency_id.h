/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_SALIENCY_ID_H_
#define MOTOR_SALIENCY_ID_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_SALIENCY_ID_FLAG_OFFSET_VALID   (1U << 0)
#define MOTOR_SALIENCY_ID_FLAG_AMPLITUDE_VALID (1U << 1)
#define MOTOR_SALIENCY_ID_FLAG_LD_VALID       (1U << 2)
#define MOTOR_SALIENCY_ID_FLAG_LQ_VALID       (1U << 3)
#define MOTOR_SALIENCY_ID_FLAG_LAVG_VALID     (1U << 4)
#define MOTOR_SALIENCY_ID_FLAG_PHASE_VALID    (1U << 5)

struct motor_saliency_id_config {
	uint16_t min_samples;
	bool fit_first_harmonic;
	bool fit_fourth_harmonic;
	float32_t scale_factor;
	float32_t min_inductance_h;
	float32_t max_inductance_h;
	float32_t max_residual_ratio;
	float32_t max_saliency_ratio;
	float32_t min_confidence;
};

struct motor_saliency_id_state {
	struct motor_saliency_id_config cfg;
	bool config_valid;
	uint32_t sample_count;
	uint32_t rejected_samples;
	float32_t gram[7][7];
	float32_t rhs[7];
	float32_t sum_yy;
};

struct motor_saliency_id_bin {
	float32_t theta_elec_rad;
	float32_t sum_inv_l;
	float32_t sum_inv_l2;
	uint32_t samples;
};

struct motor_saliency_id_result {
	float32_t l_avg_h;
	float32_t ld_h;
	float32_t lq_h;
	float32_t lq_minus_ld_h;
	float32_t saliency_ratio;
	float32_t phase_rad;
	float32_t inv_l_offset;
	float32_t inv_l_cos1;
	float32_t inv_l_sin1;
	float32_t inv_l_amplitude1;
	float32_t inv_l_cos2;
	float32_t inv_l_sin2;
	float32_t inv_l_amplitude;
	float32_t inv_l_cos4;
	float32_t inv_l_sin4;
	float32_t inv_l_amplitude4;
	float32_t residual_rms;
	float32_t residual_ratio;
	float32_t confidence;
	uint32_t sample_count;
	uint32_t rejected_samples;
	uint32_t flags;
	bool valid;
};

int motor_saliency_id_validate_config(const struct motor_saliency_id_config *cfg);
void motor_saliency_id_init(struct motor_saliency_id_state *state,
			    const struct motor_saliency_id_config *cfg);
bool motor_saliency_id_add(struct motor_saliency_id_state *state,
			   float32_t theta_elec_rad,
			   float32_t inv_l_h_inv);
int motor_saliency_id_finalize(const struct motor_saliency_id_state *state,
			       struct motor_saliency_id_result *result);
int motor_saliency_id_finalize_bins(const struct motor_saliency_id_bin *bins,
				    uint32_t bin_count,
				    uint32_t rejected_samples,
				    const struct motor_saliency_id_config *cfg,
				    struct motor_saliency_id_result *result);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SALIENCY_ID_H_ */
