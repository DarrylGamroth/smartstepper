/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_EVENTS_H
#define MOTOR_EVENTS_H

#include <stdint.h>

/* Motor control event types */
enum motor_event_type {
	MOTOR_EVENT_INIT,                /* Initial event to start state machine */
	MOTOR_EVENT_RUN,                 /* Generic run event for state transitions */
	MOTOR_EVENT_START_REQUEST,       /* Request to start motor */
	MOTOR_EVENT_STOP_REQUEST,        /* Request to stop motor */
	MOTOR_EVENT_CALIBRATE_REQUEST,   /* Request calibration sequence */
	MOTOR_EVENT_PARAM_UPDATE,        /* Update control parameter */
	MOTOR_EVENT_CLEAR_ERROR,         /* Clear error condition */
	MOTOR_EVENT_ERROR,               /* Error event  */
	MOTOR_EVENT_TIMEOUT,             /* State timeout expired */
    MOTOR_EVENT_NONE                 /* No event (used for polling) */
};

/* Motor control event message */
struct motor_event {
	enum motor_event_type type;
	union {
		struct {
			uint16_t param_offset;   /* Offset into motor_control_params */
			float value;
		} param_update;
		uint32_t error_code;         /* Error code for EMERGENCY_STOP events */
	};
};

#endif /* MOTOR_EVENTS_H */
