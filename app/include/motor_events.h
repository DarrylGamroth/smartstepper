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
	MOTOR_EVENT_IDLE,                /* Request transition to IDLE state */
	MOTOR_EVENT_OFFLINE,             /* Request transition to OFFLINE state */
	MOTOR_EVENT_ONLINE,              /* Request transition to ONLINE state */
	MOTOR_EVENT_CALIBRATE_REQUEST,   /* Request calibration sequence */
	MOTOR_EVENT_COMMISSION_REQUEST,  /* Request commissioning sequence */
	MOTOR_EVENT_MODE_CHANGE,         /* Request control mode change */
	MOTOR_EVENT_PARAM_UPDATE,        /* Update control parameter */
	MOTOR_EVENT_CLEAR_ERROR,         /* Clear error condition */
	MOTOR_EVENT_ERROR,               /* Error event  */
	MOTOR_EVENT_PROFILE_SEQ_TICK,    /* Profile sequence timer tick */
	MOTOR_EVENT_TIMEOUT,             /* State timeout expired */
    MOTOR_EVENT_NONE                 /* No event (used for polling) */
};

/* Motor control event message */
struct motor_event {
		enum motor_event_type type;
		union {
			struct {
				uint8_t param_id;        /* Parameter ID (see motor_control_api.c parameter table) */
				float value;
			} param_update;
		uint32_t error_code;         /* Error code for EMERGENCY_STOP events */
		int target_mode;             /* Target mode for MODE_CHANGE events (enum motor_state) */
	};
};

/**
 * @brief Convert motor event type enum to string
 *
 * @param event_type Motor event type enum value
 * @return String representation of the event type
 */
const char *motor_event_to_string(enum motor_event_type event_type);

#endif /* MOTOR_EVENTS_H */
