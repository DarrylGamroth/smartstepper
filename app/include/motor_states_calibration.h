/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_STATES_CALIBRATION_H
#define MOTOR_STATES_CALIBRATION_H

#include <zephyr/smf.h>

void motor_state_calibration_entry(void *obj);
enum smf_state_result motor_state_calibration_run(void *obj);
void motor_state_calibration_exit(void *obj);

void motor_state_offset_meas_entry(void *obj);
enum smf_state_result motor_state_offset_meas_run(void *obj);
void motor_state_offset_meas_exit(void *obj);

void motor_state_rs_est_entry(void *obj);
enum smf_state_result motor_state_rs_est_run(void *obj);
void motor_state_rs_est_exit(void *obj);

void motor_state_roverl_meas_entry(void *obj);
enum smf_state_result motor_state_roverl_meas_run(void *obj);
void motor_state_roverl_meas_exit(void *obj);

void motor_state_align_entry(void *obj);
enum smf_state_result motor_state_align_run(void *obj);
void motor_state_align_exit(void *obj);

void motor_state_align_sample_entry(void *obj);
enum smf_state_result motor_state_align_sample_run(void *obj);
void motor_state_align_sample_exit(void *obj);

#endif /* MOTOR_STATES_CALIBRATION_H */
