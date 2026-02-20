/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_autonomy.h"

#include "config.h"
#include "motion_profile.h"
#include "motor_state_utils.h"

bool motor_autonomous_keepalive_active(const struct motor_parameters *params,
				       const struct smf_state *state,
				       bool control_armed)
{
	if (params == NULL || state == NULL || !control_armed) {
		return false;
	}

	return motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_OPEN) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_CLOSED) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_POSITION) ||
	       params->profile_sequence_running || params->chopper_cal_active ||
	       motion_profile_quintic_is_active(&params->position_profile);
}
