/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ESTIMATION_RLS_RUNTIME_H_
#define MOTOR_ESTIMATION_RLS_RUNTIME_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/filters/pi.h"
#include "motor/math/prbs.h"
#include "motor/observers/angle_observer.h"
#include "motor/estimation/rls_motor_est.h"
#include "motor/estimation/thermal_model.h"

struct motor_rls_runtime_ctx {
	bool rls_feature_enabled;
	uint32_t control_loop_count;
	float32_t control_loop_frequency_hz;
	struct {
		struct prbs_gen *prbs_gen;
		struct rls_motor_est *d;
		struct rls_motor_est *q;
		uint32_t decimation;
		uint32_t stagger_offset;
		float32_t excitation_current_a;
		float32_t *ld_est_h;
		float32_t *lq_est_h;
		float32_t *id_prev_a;
		float32_t *iq_prev_a;
		uint32_t *d_prev_cycle;
		uint32_t *q_prev_cycle;
		uint8_t *d_prev_valid;
		uint8_t *q_prev_valid;
		float32_t min_current_a;
		float32_t min_speed_rad_s;
		float32_t max_residual_v;
		float32_t max_voltage_v;
	} rls;
	struct angle_observer_state *observer;
	float32_t vd_v;
	float32_t vq_v;
	struct pi_f32 *pi_id;
	struct pi_f32 *pi_iq;
	float32_t default_flux_linkage_wb;
	float32_t *rs_measured_ohm;
	struct {
		struct thermal_model *model;
		uint32_t decimation;
		float32_t rs_ref_ohm;
		float32_t rs_ref_temp_c;
		float32_t rs_temp_coeff;
		float32_t *t_rls_c;
	} thermal;
};

struct motor_rls_runtime_state {
	bool rls_control_enabled;
	bool rls_runtime_enabled;
	uint32_t rls_mask;
};

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
float32_t motor_rls_prepare_id_reference(struct motor_rls_runtime_ctx *ctx,
					 bool online_control_state,
					 bool control_armed,
					 bool fresh_encoder_sample,
					 bool encoder_frame_error,
					 uint8_t encoder_input_source,
					 float32_t id_ref_a,
					 struct motor_rls_runtime_state *runtime_state);

void motor_rls_update_estimators(struct motor_rls_runtime_ctx *ctx,
				 const struct motor_rls_runtime_state *runtime_state,
				 float32_t id_a,
				 float32_t iq_a);
#else
static inline float32_t motor_rls_prepare_id_reference(struct motor_rls_runtime_ctx *ctx,
						       bool online_control_state,
						       bool control_armed,
						       bool fresh_encoder_sample,
						       bool encoder_frame_error,
						       uint8_t encoder_input_source,
						       float32_t id_ref_a,
						       struct motor_rls_runtime_state *runtime_state)
{
	(void)ctx;
	(void)online_control_state;
	(void)control_armed;
	(void)fresh_encoder_sample;
	(void)encoder_frame_error;
	(void)encoder_input_source;
	if (runtime_state != NULL) {
		runtime_state->rls_control_enabled = false;
		runtime_state->rls_runtime_enabled = false;
		runtime_state->rls_mask = 0U;
	}
	return id_ref_a;
}

static inline void motor_rls_update_estimators(struct motor_rls_runtime_ctx *ctx,
					       const struct motor_rls_runtime_state *runtime_state,
					       float32_t id_a,
					       float32_t iq_a)
{
	(void)ctx;
	(void)runtime_state;
	(void)id_a;
	(void)iq_a;
}
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

#endif /* MOTOR_ESTIMATION_RLS_RUNTIME_H_ */
