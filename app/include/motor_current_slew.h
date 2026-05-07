/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_MOTOR_CURRENT_SLEW_H_
#define APP_MOTOR_CURRENT_SLEW_H_

#include <math.h>

#include "config.h"
#include "motor/control/current_slew.h"

static inline struct motor_current_slew_pair
motor_current_slew_from_params(struct motor_parameters *params)
{
	return (struct motor_current_slew_pair){
		.id = &params->traj_Id,
		.iq = &params->traj_Iq,
	};
}

static inline void motor_current_slew_params_set_target_ramp(struct motor_parameters *params,
							     float32_t id_a,
							     float32_t iq_a,
							     float32_t ramp_s)
{
	params->Id_setpoint_A = id_a;
	params->Iq_setpoint_A = iq_a;
	struct motor_current_slew_pair slew = motor_current_slew_from_params(params);
	float32_t ramp_ticks = ramp_s * CONTROL_LOOP_FREQUENCY_HZ;
	float32_t id_step_a = fabsf(id_a - traj_get_int_value(slew.id));
	float32_t iq_step_a = fabsf(iq_a - traj_get_int_value(slew.iq));
	float32_t step_a = fmaxf(id_step_a, iq_step_a);
	float32_t delta_a = (step_a > 0.0f && ramp_ticks > 0.0f) ?
		fmaxf(step_a / ramp_ticks, 1.0e-6f) :
		motor_current_slew_delta_a_per_tick(MOTOR_MAX_CURRENT_A,
						    CURRENT_COMMAND_RAMP_S,
						    CONTROL_LOOP_FREQUENCY_HZ);

	motor_current_slew_set_delta(&slew, delta_a);
	motor_current_slew_set_target(&slew, id_a, iq_a);
}

static inline void motor_current_slew_params_set_target(struct motor_parameters *params,
							float32_t id_a,
							float32_t iq_a)
{
	motor_current_slew_params_set_target_ramp(params, id_a, iq_a,
						 CURRENT_COMMAND_RAMP_S);
}

static inline void motor_current_slew_params_zero(struct motor_parameters *params)
{
	motor_current_slew_params_set_target(params, 0.0f, 0.0f);
}

static inline void motor_current_slew_params_force_zero(struct motor_parameters *params)
{
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	params->live.Id_ref_A = 0.0f;
	params->live.Iq_ref_A = 0.0f;
	struct motor_current_slew_pair slew = motor_current_slew_from_params(params);

	motor_current_slew_force_zero(&slew);
}

#endif /* APP_MOTOR_CURRENT_SLEW_H_ */
