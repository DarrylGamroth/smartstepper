/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RLS_RUNTIME_H_
#define MOTOR_RLS_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <zephyr/dsp/types.h>

struct motor_parameters;

struct motor_rls_runtime_state {
	bool rls_control_enabled;
	bool rls_runtime_enabled;
	uint32_t rls_mask;
};

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
float32_t motor_rls_prepare_id_reference(struct motor_parameters *params,
					 bool online_control_state,
					 bool control_armed,
					 bool fresh_encoder_sample,
					 bool encoder_frame_error,
					 uint8_t encoder_input_source,
					 float32_t id_ref_a,
					 struct motor_rls_runtime_state *runtime_state);

void motor_rls_update_estimators(struct motor_parameters *params,
				 const struct motor_rls_runtime_state *runtime_state,
				 float32_t id_a,
				 float32_t iq_a);
#else
static inline float32_t motor_rls_prepare_id_reference(struct motor_parameters *params,
						       bool online_control_state,
						       bool control_armed,
						       bool fresh_encoder_sample,
						       bool encoder_frame_error,
						       uint8_t encoder_input_source,
						       float32_t id_ref_a,
						       struct motor_rls_runtime_state *runtime_state)
{
	(void)params;
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

static inline void motor_rls_update_estimators(struct motor_parameters *params,
					       const struct motor_rls_runtime_state *runtime_state,
					       float32_t id_a,
					       float32_t iq_a)
{
	(void)params;
	(void)runtime_state;
	(void)id_a;
	(void)iq_a;
}
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

#endif /* MOTOR_RLS_RUNTIME_H_ */
