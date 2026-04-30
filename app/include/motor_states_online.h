/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_STATES_ONLINE_H
#define MOTOR_STATES_ONLINE_H

#include <zephyr/smf.h>

void motor_state_online_entry(void *obj);
enum smf_state_result motor_state_online_run(void *obj);
void motor_state_online_exit(void *obj);

void motor_state_online_torque_entry(void *obj);
enum smf_state_result motor_state_online_torque_run(void *obj);
void motor_state_online_torque_exit(void *obj);

void motor_state_online_velocity_open_entry(void *obj);
enum smf_state_result motor_state_online_velocity_open_run(void *obj);
void motor_state_online_velocity_open_exit(void *obj);

void motor_state_online_profile_open_entry(void *obj);
enum smf_state_result motor_state_online_profile_open_run(void *obj);
void motor_state_online_profile_open_exit(void *obj);

void motor_state_online_velocity_closed_entry(void *obj);
enum smf_state_result motor_state_online_velocity_closed_run(void *obj);
void motor_state_online_velocity_closed_exit(void *obj);

void motor_state_online_position_entry(void *obj);
enum smf_state_result motor_state_online_position_run(void *obj);
void motor_state_online_position_exit(void *obj);

#endif /* MOTOR_STATES_ONLINE_H */
