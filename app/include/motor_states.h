/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_STATES_H
#define MOTOR_STATES_H

#include <stdint.h>
#include <zephyr/smf.h>
#include "motor_events.h"

/* Forward declarations */
struct motor_parameters;

/* Motor control state machine states */
enum motor_state {
	MOTOR_STATE_HW_INIT,
	MOTOR_STATE_CTRL_INIT,
	MOTOR_STATE_CALIBRATION,
	MOTOR_STATE_OFFSET_MEAS,
	MOTOR_STATE_RS_EST,
	MOTOR_STATE_ROVERL_MEAS,
	MOTOR_STATE_ALIGN,
	MOTOR_STATE_IDLE,
	MOTOR_STATE_OFFLINE,
	MOTOR_STATE_ONLINE,
	MOTOR_STATE_ERROR,
};

/* Error codes for fault handling */
enum motor_error {
	ERROR_NONE = 0,
	ERROR_HARDWARE_BREAK = 1,
	ERROR_OVERCURRENT = 2,
	ERROR_ENCODER_FAULT = 3,
	ERROR_OVERVOLTAGE = 4,
	ERROR_EMERGENCY_STOP = 5,
};

/* State machine table - exposed for shell access */
extern const struct smf_state motor_states[];

/* Include config.h after event definitions to resolve circular dependency */
#include "config.h"

/**
 * @brief Get current state enum from SMF context
 *
 * @param params Motor parameters with SMF context
 * @return Current motor_state enum value, or -1 if unknown
 */
int motor_states_get_current(const struct motor_parameters *params);

/**
 * @brief Convert motor state enum to string
 *
 * @param state Motor state enum value
 * @return String representation of the state
 */
const char *motor_state_to_string(int state);

/**
 * @brief Convert motor error enum to string
 *
 * @param error Motor error enum value
 * @return String representation of the error
 */
const char *motor_error_to_string(int error);

/**
 * @brief Start motor state machine thread
 *
 * Creates and starts the motor state machine thread with event loop.
 * Call this once during application initialization.
 */
void motor_sm_thread_run(void);

/**
 * @brief Get pointer to motor parameters structure
 *
 * @return Pointer to motor_parameters used by state machine
 */
struct motor_parameters *motor_sm_get_params(void);

#endif /* MOTOR_STATES_H */
