/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_STATE_UTILS_H_
#define MOTOR_STATE_UTILS_H_

#include <stdbool.h>
#include <zephyr/smf.h>
#include "motor_states.h"

/**
 * @brief Return true when state is an ONLINE submode (not parent ONLINE).
 */
static inline bool motor_state_is_online_submode(enum motor_state state)
{
	return state == MOTOR_STATE_ONLINE_TORQUE ||
	       state == MOTOR_STATE_ONLINE_VELOCITY_OPEN ||
	       state == MOTOR_STATE_ONLINE_PROFILE_OPEN ||
	       state == MOTOR_STATE_ONLINE_VELOCITY_CLOSED ||
	       state == MOTOR_STATE_ONLINE_POSITION;
}

/**
 * @brief Return true when an SMF state pointer matches a target state enum.
 */
static inline bool motor_state_ptr_is_mode(const struct smf_state *state, enum motor_state mode)
{
	return state == &motor_states[mode];
}

/**
 * @brief Return true when an SMF state pointer is ONLINE parent or submode.
 */
static inline bool motor_state_ptr_is_online_control_state(const struct smf_state *state)
{
	return motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_TORQUE) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_OPEN) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_PROFILE_OPEN) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_CLOSED) ||
	       motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_POSITION);
}

#endif /* MOTOR_STATE_UTILS_H_ */
