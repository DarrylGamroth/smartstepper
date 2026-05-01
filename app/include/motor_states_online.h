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

void motor_state_online_current_encoder_entry(void *obj);
enum smf_state_result motor_state_online_current_encoder_run(void *obj);
void motor_state_online_current_encoder_exit(void *obj);

void motor_state_online_velocity_generated_entry(void *obj);
enum smf_state_result motor_state_online_velocity_generated_run(void *obj);
void motor_state_online_velocity_generated_exit(void *obj);

void motor_state_online_position_generated_entry(void *obj);
enum smf_state_result motor_state_online_position_generated_run(void *obj);
void motor_state_online_position_generated_exit(void *obj);

void motor_state_online_velocity_encoder_entry(void *obj);
enum smf_state_result motor_state_online_velocity_encoder_run(void *obj);
void motor_state_online_velocity_encoder_exit(void *obj);

void motor_state_online_position_encoder_entry(void *obj);
enum smf_state_result motor_state_online_position_encoder_run(void *obj);
void motor_state_online_position_encoder_exit(void *obj);

#endif /* MOTOR_STATES_ONLINE_H */
