/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RT_CONTROL_CONTEXT_H_
#define MOTOR_RT_CONTROL_CONTEXT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/estimation/rls_runtime.h"
#include "motor/observers/encoder_feedback.h"
#include "motor/runtime/commission_runtime.h"
#include "motor/runtime/control_refs.h"

struct motor_control_measurements {
	float32_t angle_control_degrees;
	float32_t ia_a;
	float32_t ib_a;
	float32_t vbus_v;
	float32_t id_a;
	float32_t iq_a;
	float32_t park_angle_rad;
	float32_t position_mech_rad;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;
	uint8_t encoder_input_source;
	bool fresh_encoder_sample;
	uint8_t encoder_frame_status;
	bool encoder_frame_warning;
	bool encoder_frame_error;
};

struct motor_encoder_stage_result {
	struct motor_encoder_feedback feedback;
	struct motor_control_feedback control_fb;
	uint8_t input_source;
	float32_t angle_control_deg;
	bool fresh;
	uint8_t frame_status;
	bool frame_warning;
	bool frame_error;
};

/*
 * Persistent scratch and compact config snapshot for one control-loop ISR.
 *
 * The broad app motor_parameters object owns long-lived controllers, filters,
 * observers, and state-machine data. This context holds the small per-tick
 * dataflow objects that are overwritten each ISR so the hot path does not need
 * large stack allocations.
 */
struct motor_rt_control_ctx {
	uint32_t mode_flags;
	bool feature_angle_gen;
	bool feature_pwm_output;
	bool feature_pi_control;
	bool feature_velocity_traj;
	bool feature_use_commanded_currents;
	bool feature_braking;
	bool online_control_state;
	bool control_armed;
	uint32_t velocity_loop_decimation;
	uint32_t position_loop_decimation;
	float32_t velocity_loop_dt_s;
	float32_t position_loop_dt_s;
	float32_t velocity_target_rad_s;
	float32_t velocity_ref_rad_s;
	float32_t position_mech_rad;
	float32_t speed_mech_rad_s;
	float32_t accel_mech_rad_s2;
	float32_t speed_mech_filtered_rad_s;

	struct motor_control_measurements meas;
	struct motor_motion_ref motion_ref;
	struct motor_feedback_ref feedback_ref;
	struct motor_angle_ref angle_ref;
	struct motor_current_ref current_ref;
	struct motor_commutation_ref commutation_ref;
	struct motor_encoder_stage_result enc_stage;
	struct motor_rls_runtime_state rls_runtime;
	struct motor_commission_observation commission_obs;
};

#endif /* MOTOR_RT_CONTROL_CONTEXT_H_ */
