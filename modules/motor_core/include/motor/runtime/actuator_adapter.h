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

	*ref = (struct motor_actuator_ref){
		.kind = kind,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_NONE,
		.enabled = false,
	};
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

	*ref = (struct motor_actuator_ref){
		.kind = MOTOR_ACTUATOR_FOC_CURRENT,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_CURRENT_DQ,
		.enabled = enabled,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.id_ref_a = id_ref_a,
		.iq_ref_a = iq_ref_a,
	};
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

	*ref = (struct motor_actuator_ref){
		.kind = MOTOR_ACTUATOR_BRUSHED_CURRENT,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_CURRENT_SCALAR,
		.enabled = enabled,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.current_a = current_a,
	};
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

	*ref = (struct motor_actuator_ref){
		.kind = MOTOR_ACTUATOR_BRUSHED_VOLTAGE,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_VOLTAGE_SCALAR,
		.enabled = enabled,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
		.voltage_v = voltage_v,
	};
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

	*ref = (struct motor_actuator_ref){
		.kind = MOTOR_ACTUATOR_STEP_DIR,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_STEP_DIR,
		.enabled = enabled,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.acceleration_rad_s2 = acceleration_rad_s2,
	};
}

static inline int motor_actuator_ref_from_servo(const struct motor_control_policy *policy,
						const struct motor_servo_ref *servo,
						struct motor_actuator_ref *actuator)
{
	if (policy == NULL || servo == NULL || actuator == NULL) {
		return -EINVAL;
	}

	motor_actuator_ref_clear(actuator, policy->actuator_kind);
	if (!servo->enabled) {
		return 0;
	}

	switch (policy->actuator_kind) {
	case MOTOR_ACTUATOR_FOC_CURRENT:
		if (servo->effort_kind != MOTOR_SERVO_EFFORT_CURRENT_DQ) {
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
			return -ENOTSUP;
		}
		motor_actuator_ref_set_brushed_voltage(actuator, true,
						       servo->position_rad,
						       servo->velocity_rad_s,
						       servo->acceleration_rad_s2,
						       servo->voltage_v);
		return 0;
	case MOTOR_ACTUATOR_STEP_DIR:
		motor_actuator_ref_set_step_dir(actuator, true,
						servo->position_rad,
						servo->velocity_rad_s,
						servo->acceleration_rad_s2);
		return 0;
	default:
		return -ENOTSUP;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RUNTIME_ACTUATOR_ADAPTER_H_ */
