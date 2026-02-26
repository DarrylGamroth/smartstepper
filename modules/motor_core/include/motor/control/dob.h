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

int motor_dob_validate(const struct motor_dob_config *cfg,
		       const struct motor_dob_model *model);

int motor_dob_init(const struct motor_dob_config *cfg,
	   const struct motor_dob_model *model,
	   struct motor_dob_state *state,
	   float32_t omega_initial_rad_s);

void motor_dob_reset(struct motor_dob_state *state,
		    float32_t omega_initial_rad_s);

int motor_dob_step(const struct motor_dob_config *cfg,
	  const struct motor_dob_model *model,
	  struct motor_dob_state *state,
	  float32_t omega_meas_rad_s,
	  float32_t iq_cmd_a,
	  float32_t *iq_ff_a_out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DOB_H_ */
