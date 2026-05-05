/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_ACTUATOR_ADAPTER_H_
#define MOTOR_RUNTIME_ACTUATOR_ADAPTER_H_

#include <errno.h>
#include <stdbool.h>

#include <zephyr/dsp/types.h>

#include "motor/runtime/control_policy.h"
#include "motor/runtime/control_refs.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline void motor_actuator_ref_clear(struct motor_actuator_ref *ref,
					    enum motor_actuator_kind kind)
{
	if (ref == NULL) {
		return;
	}

	ref->kind = kind;
	ref->effort_kind = MOTOR_ACTUATOR_EFFORT_NONE;
	ref->enabled = false;
	ref->position_rad = 0.0f;
	ref->velocity_rad_s = 0.0f;
	ref->acceleration_rad_s2 = 0.0f;
	ref->torque_nm = 0.0f;
	ref->current_a = 0.0f;
	ref->voltage_v = 0.0f;
	ref->normalized_effort = 0.0f;
	ref->id_ref_a = 0.0f;
	ref->iq_ref_a = 0.0f;
}

static inline void motor_actuator_ref_set_foc_current(struct motor_actuator_ref *ref,
						      bool enabled,
						      float32_t position_rad,
						      float32_t velocity_rad_s,
						      float32_t acceleration_rad_s2,
						      float32_t id_ref_a,
						      float32_t iq_ref_a)
{
	if (ref == NULL) {
		return;
	}

	ref->kind = MOTOR_ACTUATOR_FOC_CURRENT;
	ref->effort_kind = MOTOR_ACTUATOR_EFFORT_CURRENT_DQ;
	ref->enabled = enabled;
	ref->position_rad = position_rad;
	ref->velocity_rad_s = velocity_rad_s;
	ref->acceleration_rad_s2 = acceleration_rad_s2;
	ref->torque_nm = 0.0f;
	ref->current_a = 0.0f;
	ref->voltage_v = 0.0f;
	ref->normalized_effort = 0.0f;
	ref->id_ref_a = id_ref_a;
	ref->iq_ref_a = iq_ref_a;
}

static inline void motor_actuator_ref_set_brushed_current(struct motor_actuator_ref *ref,
							  bool enabled,
							  float32_t position_rad,
							  float32_t velocity_rad_s,
							  float32_t acceleration_rad_s2,
							  float32_t current_a)
{
	if (ref == NULL) {
		return;
	}

	ref->kind = MOTOR_ACTUATOR_BRUSHED_CURRENT;
	ref->effort_kind = MOTOR_ACTUATOR_EFFORT_CURRENT_SCALAR;
	ref->enabled = enabled;
	ref->position_rad = position_rad;
	ref->velocity_rad_s = velocity_rad_s;
	ref->acceleration_rad_s2 = acceleration_rad_s2;
	ref->torque_nm = 0.0f;
	ref->current_a = current_a;
	ref->voltage_v = 0.0f;
	ref->normalized_effort = 0.0f;
	ref->id_ref_a = 0.0f;
	ref->iq_ref_a = 0.0f;
}

static inline void motor_actuator_ref_set_brushed_voltage(struct motor_actuator_ref *ref,
							  bool enabled,
							  float32_t position_rad,
							  float32_t velocity_rad_s,
							  float32_t acceleration_rad_s2,
							  float32_t voltage_v)
{
	if (ref == NULL) {
		return;
	}

	ref->kind = MOTOR_ACTUATOR_BRUSHED_VOLTAGE;
	ref->effort_kind = MOTOR_ACTUATOR_EFFORT_VOLTAGE_SCALAR;
	ref->enabled = enabled;
	ref->position_rad = position_rad;
	ref->velocity_rad_s = velocity_rad_s;
	ref->acceleration_rad_s2 = acceleration_rad_s2;
	ref->torque_nm = 0.0f;
	ref->current_a = 0.0f;
	ref->voltage_v = voltage_v;
	ref->normalized_effort = 0.0f;
	ref->id_ref_a = 0.0f;
	ref->iq_ref_a = 0.0f;
}

static inline void motor_actuator_ref_set_step_dir(struct motor_actuator_ref *ref,
						   bool enabled,
						   float32_t position_rad,
						   float32_t velocity_rad_s,
						   float32_t acceleration_rad_s2)
{
	if (ref == NULL) {
		return;
	}

	ref->kind = MOTOR_ACTUATOR_STEP_DIR;
	ref->effort_kind = MOTOR_ACTUATOR_EFFORT_STEP_DIR;
	ref->enabled = enabled;
	ref->position_rad = position_rad;
	ref->velocity_rad_s = velocity_rad_s;
	ref->acceleration_rad_s2 = acceleration_rad_s2;
	ref->torque_nm = 0.0f;
	ref->current_a = 0.0f;
	ref->voltage_v = 0.0f;
	ref->normalized_effort = 0.0f;
	ref->id_ref_a = 0.0f;
	ref->iq_ref_a = 0.0f;
}

static inline int motor_actuator_ref_from_servo(const struct motor_control_policy *policy,
						const struct motor_servo_ref *servo,
						struct motor_actuator_ref *actuator)
{
	if (policy == NULL || servo == NULL || actuator == NULL) {
		return -EINVAL;
	}

	if (!servo->enabled) {
		motor_actuator_ref_clear(actuator, policy->actuator_kind);
		return 0;
	}

	switch (policy->actuator_kind) {
	case MOTOR_ACTUATOR_FOC_CURRENT:
		if (servo->effort_kind != MOTOR_SERVO_EFFORT_CURRENT_DQ) {
			motor_actuator_ref_clear(actuator, policy->actuator_kind);
			return -ENOTSUP;
		}
		motor_actuator_ref_set_foc_current(actuator, true,
						   servo->position_rad,
						   servo->velocity_rad_s,
						   servo->acceleration_rad_s2,
						   servo->id_ref_a,
						   servo->iq_ref_a);
		return 0;
	case MOTOR_ACTUATOR_BRUSHED_CURRENT:
		if (servo->effort_kind != MOTOR_SERVO_EFFORT_CURRENT) {
			motor_actuator_ref_clear(actuator, policy->actuator_kind);
			return -ENOTSUP;
		}
		motor_actuator_ref_set_brushed_current(actuator, true,
						       servo->position_rad,
						       servo->velocity_rad_s,
						       servo->acceleration_rad_s2,
						       servo->current_a);
		return 0;
	case MOTOR_ACTUATOR_BRUSHED_VOLTAGE:
		if (servo->effort_kind != MOTOR_SERVO_EFFORT_VOLTAGE) {
			motor_actuator_ref_clear(actuator, policy->actuator_kind);
			return -ENOTSUP;
		}
		motor_actuator_ref_set_brushed_voltage(actuator, true,
						       servo->position_rad,
						       servo->velocity_rad_s,
						       servo->acceleration_rad_s2,
						       servo->voltage_v);
		return 0;
	case MOTOR_ACTUATOR_STEP_DIR:
		if (servo->effort_kind != MOTOR_SERVO_EFFORT_NONE) {
			motor_actuator_ref_clear(actuator, policy->actuator_kind);
			return -ENOTSUP;
		}
		motor_actuator_ref_set_step_dir(actuator, true,
						servo->position_rad,
						servo->velocity_rad_s,
						servo->acceleration_rad_s2);
		return 0;
	default:
		motor_actuator_ref_clear(actuator, policy->actuator_kind);
		return -ENOTSUP;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RUNTIME_ACTUATOR_ADAPTER_H_ */
