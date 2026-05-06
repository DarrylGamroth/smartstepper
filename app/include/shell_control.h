/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_CONTROL_H_
#define SHELL_CONTROL_H_

#include <zephyr/shell/shell.h>

int cmd_motor_params_get(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_params_set(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_params_list(const struct shell *sh, size_t argc, char **argv);

int cmd_motor_current_id(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_current_iq(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_current_dq(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_current_gain_get(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_current_gain_set(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_current_gain_bandwidth(const struct shell *sh, size_t argc, char **argv);

int cmd_motor_velocity_target(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_velocity_decimation(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_velocity_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_velocity_pi(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_velocity_mpr(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_velocity_dob(const struct shell *sh, size_t argc, char **argv);

int cmd_motor_position_target(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_position_decimation(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_position_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_position_pi(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_position_mpr(const struct shell *sh, size_t argc, char **argv);

int cmd_motor_control_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_outer_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_outer_mode(const struct shell *sh, size_t argc, char **argv);

int cmd_motor_rls_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_rls_params(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_rls_temp(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_rls_gating(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_rls_reset(const struct shell *sh, size_t argc, char **argv);

#endif /* SHELL_CONTROL_H_ */
