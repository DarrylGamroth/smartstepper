/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_MPR_H_
#define MOTOR_MPR_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mechanical speed plant parameters for velocity MPR.
 */
struct motor_mpr_velocity_model {
	float32_t inertia_kgm2;
	float32_t viscous_friction_nm_per_rad_s;
	float32_t coulomb_friction_nm;
	float32_t torque_constant_nm_per_a;
};

/**
 * @brief Velocity MPR tuning and constraints.
 */
struct motor_mpr_velocity_config {
	float32_t dt_s;
	uint16_t horizon;
	float32_t q_speed;
	float32_t r_delta_iq;
	float32_t iq_limit_a;
	float32_t max_delta_iq_a;
	float32_t disturbance_ki_nm_per_rad_s;
};

/**
 * @brief Velocity MPR runtime state.
 */
struct motor_mpr_velocity_state {
	bool initialized;
	float32_t iq_cmd_a;
	float32_t omega_model_rad_s;
	float32_t disturbance_nm;
	float32_t last_omega_error_rad_s;
	float32_t a;
	float32_t b_u;
	float32_t b_d;
};

/**
 * @brief Position MPR tuning and constraints.
 *
 * This regulator computes a velocity command from wrapped position error and an
 * optional velocity feedforward term.
 */
struct motor_mpr_position_config {
	float32_t dt_s;
	uint16_t horizon;
	float32_t q_position;
	float32_t q_velocity_ff;
	float32_t r_delta_velocity;
	float32_t velocity_limit_rad_s;
	float32_t max_delta_velocity_rad_s;
};

/**
 * @brief Position MPR runtime state.
 */
struct motor_mpr_position_state {
	bool initialized;
	float32_t velocity_cmd_rad_s;
	float32_t last_position_error_rad;
};

int motor_mpr_velocity_validate(const struct motor_mpr_velocity_config *cfg,
				const struct motor_mpr_velocity_model *model);

int motor_mpr_velocity_init(const struct motor_mpr_velocity_config *cfg,
			    const struct motor_mpr_velocity_model *model,
			    struct motor_mpr_velocity_state *state,
			    float32_t omega_initial_rad_s,
			    float32_t iq_initial_a);

void motor_mpr_velocity_reset(struct motor_mpr_velocity_state *state,
			      float32_t omega_initial_rad_s,
			      float32_t iq_initial_a);

int motor_mpr_velocity_step(const struct motor_mpr_velocity_config *cfg,
			    const struct motor_mpr_velocity_model *model,
			    struct motor_mpr_velocity_state *state,
			    float32_t omega_meas_rad_s,
			    float32_t omega_ref_rad_s,
			    float32_t *iq_cmd_a_out);

int motor_mpr_position_validate(const struct motor_mpr_position_config *cfg);

int motor_mpr_position_init(const struct motor_mpr_position_config *cfg,
			    struct motor_mpr_position_state *state,
			    float32_t velocity_initial_rad_s);

void motor_mpr_position_reset(struct motor_mpr_position_state *state,
			      float32_t velocity_initial_rad_s);

int motor_mpr_position_step(const struct motor_mpr_position_config *cfg,
			    struct motor_mpr_position_state *state,
			    float32_t position_error_rad,
			    float32_t velocity_ff_rad_s,
			    float32_t *velocity_cmd_rad_s_out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MPR_H_ */
