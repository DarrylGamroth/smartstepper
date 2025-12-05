/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_API_H
#define MOTOR_CONTROL_API_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include "config.h"

/* Forward declaration */
struct motor_parameters;

/**
 * @file motor_control_api.h
 * @brief Thread-safe motor control API for multi-protocol access
 * 
 * This API provides thread-safe access to motor control from any context
 * (Shell, MODBUS, CAN, MQTT, HTTP, etc.) using a message queue to the
 * state machine thread. All state transitions and parameter updates are
 * serialized through the state machine thread to avoid race conditions.
 */

/* Motor event queue - exposed for main loop access */
extern struct k_msgq motor_event_queue;

/**
 * @brief Initialize motor control API
 * 
 * Creates message queue for thread-safe parameter updates and state requests.
 * Sets the motor parameters pointer for API access.
 * Must be called before any motor_api_* functions are used.
 * 
 * @param params Pointer to motor_parameters structure (must remain valid)
 * @return 0 on success, negative errno on failure
 */
int motor_control_api_init(struct motor_parameters *params);

/**
 * @brief Request motor start (thread-safe)
 * 
 * Posts start request to state machine. State machine will transition
 * from IDLE to ONLINE if conditions are met.
 * 
 * @return 0 on success, negative errno on failure
 */
int motor_api_request_start(void);

/**
 * @brief Request motor stop (thread-safe)
 * 
 * Posts stop request to state machine. State machine will transition
 * to IDLE state.
 * 
 * @return 0 on success, negative errno on failure
 */
int motor_api_request_stop(void);

/**
 * @brief Request calibration sequence (thread-safe)
 * 
 * Posts calibration request to state machine. State machine will
 * start the calibration sequence if in IDLE state.
 * 
 * @return 0 on success, negative errno on failure
 */
int motor_api_request_calibrate(void);

/**
 * @brief Update control parameter by offset (thread-safe)
 * 
 * Posts parameter update to state machine thread via message queue.
 * Parameter is written to shadow buffer and swap is triggered.
 * 
 * @param param_offset Offset into motor_control_params structure
 * @param value New parameter value
 * @return 0 on success, negative errno on failure
 * 
 * @note This function is non-blocking. Update will occur asynchronously
 *       in the state machine thread, then ISR will swap buffers.
 */
int motor_api_update_param(uint16_t param_offset, float value);

/**
 * @brief Update control parameter by name (thread-safe)
 * 
 * Looks up parameter by name in the parameter table and updates it.
 * 
 * @param name Parameter name string
 * @param value New parameter value
 * @return 0 on success, -EINVAL if parameter not found
 */
int motor_api_set_param(const char *name, float value);

/**
 * @brief Get control parameter by name (thread-safe read)
 * 
 * Reads parameter from write buffer (protocol's view of pending parameters).
 * 
 * @param name Parameter name string
 * @param value Pointer to store parameter value
 * @return 0 on success, -EINVAL if parameter not found, -ENODEV if not initialized
 */
int motor_api_get_param(const char *name, float *value);

/**
 * @brief Get number of parameters in the parameter table
 * 
 * @return Number of parameters available
 */
size_t motor_api_get_param_count(void);

/**
 * @brief Get parameter name by index
 * 
 * @param index Parameter index (0 to count-1)
 * @return Parameter name string, or NULL if index out of range
 */
const char *motor_api_get_param_name(size_t index);

/**
 * @brief Get parameter value by index
 * 
 * @param index Parameter index (0 to count-1)
 * @param value Pointer to store parameter value
 * @return 0 on success, -EINVAL if index out of range, -ENODEV if not initialized
 */
int motor_api_get_param_by_index(size_t index, float *value);

/**
 * @brief Set current setpoints (thread-safe convenience wrapper)
 * 
 * @param id_A D-axis current setpoint (amps)
 * @param iq_A Q-axis current setpoint (amps)
 * @return 0 on success, negative errno on failure
 */
int motor_api_set_currents(float id_A, float iq_A);

/**
 * @brief Clear error condition (thread-safe)
 * 
 * Posts error clear request to state machine thread.
 * State machine will validate current state is ERROR before clearing.
 * 
 * @return 0 on success, negative errno on failure
 */
int motor_api_clear_error(void);

/**
 * @brief Emergency stop (thread-safe, high priority)
 * 
 * Posts emergency stop request with priority flag.
 * State machine will immediately disable PWM and transition to ERROR state.
 * 
 * @return 0 on success, negative errno on failure
 * 
 * @note This is the only way to safely stop the motor from any thread context.
 */
int motor_api_emergency_stop(void);

/**
 * @brief Post error event from ISR (non-blocking)
 * 
 * Posts emergency stop event with specific error code to the message queue.
 * Safe to call from ISR context. Non-blocking.
 * 
 * @param error_code Error code to post (from motor_error enum)
 * @return 0 on success, negative errno if queue is full
 */
int motor_api_post_error(uint32_t error_code);

/**
 * @brief Get current motor state (thread-safe read-only)
 * 
 * Reads current state machine state. This is safe because it's a read-only
 * operation and the state enum is updated atomically by SMF.
 * 
 * @return Current motor state enum value, or -1 if not initialized
 */
int motor_api_get_state(void);

/**
 * @brief Get last error code (thread-safe read-only)
 * 
 * Returns the error code that caused the last transition to ERROR state.
 * Only updated when entering ERROR state, not on every error event.
 * 
 * @return Last error code, or ERROR_NONE if no error has occurred
 */
int motor_api_get_error(void);

/**
 * @brief Read telemetry snapshot (thread-safe read-only)
 * 
 * Atomically reads telemetry values from motor_parameters structure.
 * Safe because these are updated by ISR and read by protocols.
 * 
 * @param id_meas Output: Measured d-axis current (A)
 * @param iq_meas Output: Measured q-axis current (A)
 * @param speed_hz Output: Mechanical speed (Hz)
 * @param vbus_V Output: Bus voltage (V)
 */
void motor_api_get_telemetry(float *id_meas, float *iq_meas, 
                             float *speed_hz, float *vbus_V);

/**
 * @brief Check for pending event (non-blocking)
 * 
 * Checks if there's a pending event in the queue. This should be called
 * by the main loop before smf_run_state() to allow state machine to
 * process events.
 * 
 * @return true if event is pending, false otherwise
 */
bool motor_api_has_event(void);

/**
 * @brief Get pending event (non-blocking)
 * 
 * Retrieves the next pending event from the queue without removing it.
 * The event will be consumed when the state machine processes it.
 * 
 * @param evt Pointer to store event
 * @return 0 on success, -ENOMSG if no event pending
 */
int motor_api_peek_event(struct motor_event *evt);

/**
 * @brief Apply parameter update from current event
 * 
 * Writes the parameter value to the shadow buffer and signals the ISR
 * to swap buffers. Only valid when current event is MOTOR_EVENT_PARAM_UPDATE.
 * 
 * @param params Pointer to motor_parameters containing the current event
 */
void motor_api_apply_param_update(struct motor_parameters *params);

/**
 * @brief Consume current event
 * 
 * Removes the current event from the queue after processing.
 * Should be called by state machine after handling the event.
 * For MOTOR_EVENT_PARAM_UPDATE, call motor_api_apply_param_update() first.
 */
void motor_api_consume_event(void);

#endif /* MOTOR_CONTROL_API_H */
