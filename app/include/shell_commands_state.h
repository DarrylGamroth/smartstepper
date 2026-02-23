/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMANDS_STATE_H_
#define SHELL_COMMANDS_STATE_H_

#include <zephyr/shell/shell.h>

/* State mode and transition commands */
int cmd_motor_state_offline(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_idle(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_online(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_calibrate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_commission(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_clear_error(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_torque(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_velocity_open(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_velocity_closed(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_position(const struct shell *sh, size_t argc, char **argv);

/* Arm/disarm and safety commands */
int cmd_motor_arm(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_disarm(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_safety_timeout(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_safety_pet(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_safety_status(const struct shell *sh, size_t argc, char **argv);

/* Info and encoder diagnostics commands */
int cmd_motor_info_config(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_info_measured(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_info_live(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_info_stats(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_alarm(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_pipeline(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_pipeline_reset(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_direction(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trim(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_start(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_stop(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_clear(const struct shell *sh, size_t argc, char **argv);

#endif /* SHELL_COMMANDS_STATE_H_ */
