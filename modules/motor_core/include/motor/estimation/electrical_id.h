/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ELECTRICAL_ID_H_
#define MOTOR_ELECTRICAL_ID_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_ELECTRICAL_ID_FLAG_RS_VALID        (1U << 0)
#define MOTOR_ELECTRICAL_ID_FLAG_LD_VALID        (1U << 1)
#define MOTOR_ELECTRICAL_ID_FLAG_LQ_VALID        (1U << 2)
#define MOTOR_ELECTRICAL_ID_FLAG_LAVG_VALID      (1U << 3)
#define MOTOR_ELECTRICAL_ID_FLAG_SALIENCY_VALID  (1U << 4)
#define MOTOR_ELECTRICAL_ID_FLAG_PI_VALID        (1U << 5)

struct motor_electrical_id_limits {
	float32_t rs_min_ohm;
	float32_t rs_max_ohm;
	float32_t l_min_h;
	float32_t l_max_h;
	float32_t max_axis_mismatch_ratio;
	float32_t min_confidence;
};

struct motor_electrical_id_rs_config {
	float32_t min_abs_current_a;
	uint16_t min_samples;
	float32_t max_residual_ratio;
};

struct motor_electrical_id_rs_accum {
	float32_t sum_i_a;
	float32_t sum_v_v;
	float32_t sum_i2_a2;
	float32_t sum_v2_v2;
	float32_t sum_iv_va;
	uint32_t samples;
	uint32_t rejected_low_current;
};

struct motor_electrical_id_rs_result {
	float32_t rs_ohm;
	float32_t avg_current_a;
	float32_t avg_voltage_v;
	float32_t residual_rms_v;
	float32_t residual_ratio;
	float32_t confidence;
	uint32_t samples;
	uint32_t rejected_low_current;
	bool valid;
};

struct motor_electrical_id_l_config {
	float32_t dt_s;
	float32_t min_abs_di_dt_a_per_s;
	float32_t min_abs_delta_current_a;
	uint16_t min_samples;
	float32_t max_residual_ratio;
};

struct motor_electrical_id_l_accum {
	float32_t sum_x2;
	float32_t sum_xy;
	float32_t sum_y2;
	uint32_t samples;
	uint32_t rejected_low_slew;
};

struct motor_electrical_id_l_result {
	float32_t inductance_h;
	float32_t residual_rms_v;
	float32_t residual_ratio;
	float32_t confidence;
	uint32_t samples;
	uint32_t rejected_low_slew;
	bool valid;
};

struct motor_electrical_id_demod_config {
	float32_t min_abs_flux_vs;
	float32_t max_spread_ratio;
	float32_t scale_factor;
	uint16_t min_samples;
};

struct motor_electrical_id_demod_accum {
	float32_t sum_inv_l;
	float32_t sum_inv_l2;
	uint32_t samples;
	uint32_t rejected_low_signal;
	uint32_t rejected_non_positive;
};

struct motor_electrical_id_demod_result {
	float32_t inductance_h;
	float32_t inv_l_mean;
	float32_t inv_l_stddev;
	float32_t spread_ratio;
	float32_t confidence;
	uint32_t samples;
	uint32_t rejected_low_signal;
	uint32_t rejected_non_positive;
	bool valid;
};

struct motor_electrical_id_result {
	float32_t rs_ohm;
	float32_t ld_h;
	float32_t lq_h;
	float32_t l_avg_h;
	float32_t lq_minus_ld_h;
	float32_t axis_mismatch_ratio;
	float32_t confidence;
	uint32_t flags;
	bool valid;
};

struct motor_electrical_id_pi_recommendation {
	float32_t bandwidth_hz;
	float32_t sample_time_s;
	float32_t kp_d;
	float32_t ki_d;
	float32_t kp_q;
	float32_t ki_q;
	bool valid;
};

void motor_electrical_id_rs_reset(struct motor_electrical_id_rs_accum *accum);
int motor_electrical_id_rs_add(struct motor_electrical_id_rs_accum *accum,
				       const struct motor_electrical_id_rs_config *cfg,
				       float32_t voltage_v,
				       float32_t current_a);
int motor_electrical_id_rs_finalize(const struct motor_electrical_id_rs_accum *accum,
					    const struct motor_electrical_id_rs_config *cfg,
					    const struct motor_electrical_id_limits *limits,
					    struct motor_electrical_id_rs_result *result);

void motor_electrical_id_l_reset(struct motor_electrical_id_l_accum *accum);
int motor_electrical_id_l_add(struct motor_electrical_id_l_accum *accum,
				      const struct motor_electrical_id_l_config *cfg,
				      float32_t voltage_v,
				      float32_t current_a,
				      float32_t previous_current_a,
				      float32_t rs_ohm);
int motor_electrical_id_l_add_integral(struct motor_electrical_id_l_accum *accum,
				       const struct motor_electrical_id_l_config *cfg,
				       float32_t flux_linkage_vs,
				       float32_t delta_current_a);
int motor_electrical_id_l_finalize(const struct motor_electrical_id_l_accum *accum,
					   const struct motor_electrical_id_l_config *cfg,
					   const struct motor_electrical_id_limits *limits,
					   struct motor_electrical_id_l_result *result);

void motor_electrical_id_demod_reset(struct motor_electrical_id_demod_accum *accum);
int motor_electrical_id_demod_add(struct motor_electrical_id_demod_accum *accum,
				  const struct motor_electrical_id_demod_config *cfg,
				  float32_t flux_linkage_vs,
				  float32_t delta_current_a);
int motor_electrical_id_demod_add_pair(struct motor_electrical_id_demod_accum *accum,
				       const struct motor_electrical_id_demod_config *cfg,
				       float32_t positive_flux_vs,
				       float32_t positive_delta_current_a,
				       float32_t negative_flux_vs,
				       float32_t negative_delta_current_a);
int motor_electrical_id_demod_finalize(const struct motor_electrical_id_demod_accum *accum,
				       const struct motor_electrical_id_demod_config *cfg,
				       const struct motor_electrical_id_limits *limits,
				       struct motor_electrical_id_demod_result *result);

int motor_electrical_id_combine(const struct motor_electrical_id_rs_result *rs,
					const struct motor_electrical_id_l_result *ld,
					const struct motor_electrical_id_l_result *lq,
					const struct motor_electrical_id_limits *limits,
					struct motor_electrical_id_result *result);

int motor_electrical_id_recommend_current_pi(float32_t rs_ohm,
						     float32_t ld_h,
						     float32_t lq_h,
						     float32_t bandwidth_hz,
						     float32_t sample_time_s,
						     struct motor_electrical_id_pi_recommendation *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ELECTRICAL_ID_H_ */
