/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_OUTER_LOOP_RUNTIME_H_
#define MOTOR_RUNTIME_OUTER_LOOP_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/filters/filter_so.h"
#include "motor/motion/traj.h"
#include "motor/motion/motion_profile.h"
#include "motor/control/dob.h"
#include "motor/control/mpr.h"
#include "motor/control/position_regulator.h"
#include "motor/control/velocity_regulator.h"
#include "motor/compensation/detent_map.h"
#include "motor/compensation/electrical_ripple_ff.h"

struct motor_outer_loop_runtime_ctx {
	uint8_t outer_loop_mode;
	struct motion_profile_quintic *position_profile;
	bool control_armed;
	float32_t *position_target_rad;
	float32_t profile_max_velocity_rad_s;
	float32_t profile_max_accel_rad_s2;
	struct motor_mpr_position_config *position_mpr_cfg;
	struct motor_mpr_position_state *position_mpr_state;
	float32_t position_cl_kp_rad_s_per_rad;
	float32_t position_cl_ki_rad_s2_per_rad;
	float32_t *position_cl_i_term_rad_s;
	struct motor_position_regulator_state *position_reg_state;
	struct traj_f32 *traj_velocity;
	struct filter_so_f32 *filter_velocity_notch;
	uint8_t position_quality_flags;
	float32_t *live_velocity_target_rad_s;
	float32_t *live_velocity_ref_rad_s;
	struct motor_velocity_regulator_state *velocity_reg_state;
	struct motor_mpr_velocity_config *velocity_mpr_cfg;
	struct motor_mpr_velocity_state *velocity_mpr_state;
	float32_t *velocity_cl_i_term_a;
	float32_t velocity_cl_kp_a_per_rad_s;
	float32_t velocity_cl_ki_a_per_rad;
	float32_t velocity_cl_iq_limit_a;
	float32_t id_setpoint_a;
	float32_t torque_gain_nm_per_a_active;
	float32_t flux_linkage_wb_active;
	float32_t default_flux_linkage_wb;
	uint16_t pole_pairs;
	float32_t inertia_kgm2_active;
	float32_t viscous_friction_nm_per_rad_s_active;
	float32_t coulomb_friction_nm_active;
	struct motor_dob_config *velocity_dob_cfg;
	struct motor_dob_state *velocity_dob_state;
	bool velocity_dob_ref_valid;
	float32_t velocity_dob_last_ref_rad_s;
	float32_t *live_velocity_dob_iq_ff_a;
	float32_t *live_velocity_dob_disturbance_nm;
	float32_t *live_velocity_dob_residual_rad_s;
	struct motor_electrical_ripple_ff_config *electrical_ripple_ff_cfg;
	struct motor_electrical_ripple_ff_state *electrical_ripple_ff_state;
	float32_t *live_electrical_ripple_iq_ff_a;
	struct motor_detent_map_config *detent_map_cfg;
	struct motor_detent_map_state *detent_map_state;
	float32_t *live_detent_iq_ff_a;
};

struct motor_outer_loop_inputs {
	bool position_active;
	bool velocity_active;
	bool feature_velocity_traj;
	bool velocity_loop_update;
	bool position_loop_update;
	float32_t velocity_loop_dt_s;
	float32_t position_loop_dt_s;
	float32_t position_mech_rad;
	float32_t electrical_angle_rad;
	float32_t speed_mech_rad_s;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

struct motor_outer_loop_outputs {
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t speed_mech_filtered_rad_s;
	float32_t id_ref_a;
	float32_t iq_ref_a;
};

int motor_outer_loop_runtime_step(struct motor_outer_loop_runtime_ctx *ctx,
				  const struct motor_outer_loop_inputs *in,
				  struct motor_outer_loop_outputs *out);

#endif /* MOTOR_RUNTIME_OUTER_LOOP_RUNTIME_H_ */
