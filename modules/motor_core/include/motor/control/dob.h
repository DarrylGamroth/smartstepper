/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_DOB_H_
#define MOTOR_DOB_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file dob.h
 * @brief Disturbance observer for velocity-loop feedforward compensation.
 *
 * The observer runs on the same reduced mechanical model used by velocity control:
 * omega[k+1] = a * omega[k] + b_u * i_q[k] + b_d * (d[k] - tau_coulomb[k]).
 *
 * Each step:
 * 1. Predict omega using previous state and commanded i_q.
 * 2. Compute residual: omega_meas - omega_pred.
 * 3. Integrate residual into disturbance estimate (bounded by torque_limit_nm).
 * 4. Convert estimated disturbance to i_q feedforward (bounded by iq_ff_limit_a).
 *
 * Lifecycle:
 * - Call init() when entering the closed-loop mode or after config/model changes.
 * - Call step() in the loop.
 * - reset() clears dynamic state only; it does not configure/discretize the model.
 */

/**
 * @brief Mechanical speed plant parameters for the velocity disturbance observer.
 */
struct motor_dob_model {
	float32_t inertia_kgm2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
	float32_t torque_constant_nm_per_a;
};

/**
 * @brief Velocity DOB tuning and constraints.
 */
struct motor_dob_config {
	bool enabled;
	float32_t dt_s;
	float32_t observer_gain_nm_per_rad_s;
	float32_t torque_limit_nm;
	float32_t iq_ff_limit_a;
};

/**
 * @brief Velocity DOB runtime state.
 */
struct motor_dob_state {
	bool initialized;
	float32_t omega_model_rad_s;
	float32_t disturbance_nm;
	float32_t iq_ff_a;
	float32_t residual_rad_s;
	float32_t a;
	float32_t b_u;
	float32_t b_d;
};

enum motor_dob_readiness_reason {
	MOTOR_DOB_READY = 0,
	MOTOR_DOB_NOT_COMMISSIONED,
	MOTOR_DOB_ENCODER_MAPPING_MISSING,
	MOTOR_DOB_FEEDBACK_UNTRUSTED,
	MOTOR_DOB_FAULT_ACTIVE,
	MOTOR_DOB_MODEL_INVALID,
	MOTOR_DOB_LIMIT_INVALID,
	MOTOR_DOB_TUNING_INVALID,
};

struct motor_dob_readiness_input {
	bool commissioning_complete;
	bool encoder_mapping_complete;
	bool feedback_trusted;
	bool fault_active;
	float32_t torque_constant_nm_per_a;
	float32_t inertia_kgm2;
	float32_t velocity_iq_limit_a;
	const struct motor_dob_config *cfg;
};

struct motor_dob_readiness_result {
	bool ready;
	enum motor_dob_readiness_reason reason;
	const char *reason_str;
};

int motor_dob_validate(const struct motor_dob_config *cfg,
		       const struct motor_dob_model *model);

const char *motor_dob_readiness_reason_str(enum motor_dob_readiness_reason reason);

int motor_dob_readiness_check(const struct motor_dob_readiness_input *in,
			      struct motor_dob_readiness_result *out);

int motor_dob_init(const struct motor_dob_config *cfg,
	   const struct motor_dob_model *model,
	   struct motor_dob_state *state,
	   float32_t omega_initial_rad_s);

void motor_dob_reset(struct motor_dob_state *state,
		    float32_t omega_initial_rad_s);

void motor_dob_invalidate(struct motor_dob_state *state);

int motor_dob_step(const struct motor_dob_config *cfg,
	  const struct motor_dob_model *model,
	  struct motor_dob_state *state,
	  float32_t omega_meas_rad_s,
	  float32_t iq_cmd_a,
	  float32_t *iq_ff_a_out);

int motor_dob_step_fast(const struct motor_dob_config *cfg,
			const struct motor_dob_model *model,
			struct motor_dob_state *state,
			float32_t omega_meas_rad_s,
			float32_t iq_cmd_a,
			float32_t *iq_ff_a_out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DOB_H_ */
