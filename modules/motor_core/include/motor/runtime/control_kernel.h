/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_CONTROL_KERNEL_H_
#define MOTOR_RUNTIME_CONTROL_KERNEL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/runtime/control_policy.h"
#include "motor/runtime/control_refs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct motor_control_kernel_input {
	const struct motor_control_policy *policy;
	const struct motor_motion_ref *motion_ref;
	const struct motor_feedback_ref *feedback_ref;
	const struct motor_current_ref *current_ref;
	bool current_loop_enabled;
	uint16_t feedback_stale_count;
	uint32_t feedback_stale_limit;
	float32_t profile_max_velocity_rad_s;
};

struct motor_control_kernel_output {
	struct motor_servo_ref servo_ref;
	struct motor_actuator_ref actuator_ref;
	bool feedback_valid;
	bool feedback_sane;
	int actuator_status;
};

bool motor_control_kernel_feedback_valid(const struct motor_control_policy *policy,
						 const struct motor_feedback_ref *feedback_ref,
						 uint16_t stale_count,
						 uint32_t stale_limit);

bool motor_control_kernel_feedback_sane(const struct motor_control_policy *policy,
					       const struct motor_feedback_ref *feedback_ref,
					       float32_t profile_max_velocity_rad_s);

int motor_control_kernel_step_fast(const struct motor_control_kernel_input *in,
					   struct motor_control_kernel_output *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RUNTIME_CONTROL_KERNEL_H_ */
