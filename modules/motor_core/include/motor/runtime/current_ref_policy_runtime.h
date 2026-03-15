/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_CURRENT_REF_POLICY_RUNTIME_H_
#define MOTOR_RUNTIME_CURRENT_REF_POLICY_RUNTIME_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>

#include "motor/filters/pi.h"
#include "motor/motion/traj.h"
#include "motor/motion/angle_gen.h"
#include "motor/control/dob.h"
#include "motor/control/mpr.h"
#include "motor/control/position_regulator.h"
#include "motor/control/velocity_regulator.h"

struct motor_current_ref_policy_ctx {
	uint8_t position_quality_flags;
	float32_t *id_setpoint_a;
	float32_t *iq_setpoint_a;
	struct pi_f32 *pi_id;
	struct pi_f32 *pi_iq;
	float32_t *live_velocity_target_rad_s;
	float32_t *live_velocity_ref_rad_s;
	float32_t *velocity_cl_i_term_a;
	float32_t *position_cl_i_term_rad_s;
	struct motor_velocity_regulator_state *velocity_reg_state;
	struct motor_position_regulator_state *position_reg_state;
	struct traj_f32 *traj_velocity;
	angle_gen_t *angle_gen;
	struct motor_mpr_velocity_state *velocity_mpr_state;
	struct motor_mpr_position_state *position_mpr_state;
	struct motor_dob_state *velocity_dob_state;
	float32_t *live_velocity_dob_iq_ff_a;
	float32_t *live_velocity_dob_disturbance_nm;
	float32_t *live_velocity_dob_residual_rad_s;
};

struct motor_current_ref_policy_inputs {
	bool online_control_state;
	bool feature_angle_gen;
	bool feature_use_commanded_currents;
	bool control_armed;
	float32_t speed_mech_filtered_rad_s;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

struct motor_current_ref_policy_outputs {
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

int motor_current_ref_apply_policy(struct motor_current_ref_policy_ctx *ctx,
				   const struct motor_current_ref_policy_inputs *in,
				   struct motor_current_ref_policy_outputs *out);

#endif /* MOTOR_RUNTIME_CURRENT_REF_POLICY_RUNTIME_H_ */
