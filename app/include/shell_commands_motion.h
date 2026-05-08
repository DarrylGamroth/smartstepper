/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMANDS_MOTION_H_
#define SHELL_COMMANDS_MOTION_H_

#include <stdbool.h>
#include <zephyr/shell/shell.h>

struct motor_parameters;

/* Shared shell state from shell_commands.c */
extern struct motor_parameters *g_motor_params;
bool motor_control_is_armed(const struct motor_parameters *params);
void motor_command_feed_watchdog(struct motor_parameters *params);

/* Motion profile commands */
int cmd_motor_profile_set(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_move(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_cancel(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_status(const struct shell *sh, size_t argc, char **argv);

/* Chopper calibration commands */
int cmd_motor_chopper_geometry(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_sensor(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_calib_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_calib_start(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_calib_bidir(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_calib_stop(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_calib_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_chopper_calib_apply(const struct shell *sh, size_t argc, char **argv);

/* Sequence playback commands */
int cmd_motor_profile_seq_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_add(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_period_ms(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_move_ms(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_end_vel_hz(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_loop(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_config(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_trigger_source(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_trigger_edge(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_trigger_channel(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_trigger_min_interval(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_trigger_fire(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_trigger_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_start(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_stop(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_profile_seq_list(const struct shell *sh, size_t argc, char **argv);

#endif /* SHELL_COMMANDS_MOTION_H_ */
