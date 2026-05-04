/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMANDS_STATE_H_
#define SHELL_COMMANDS_STATE_H_

#include <zephyr/shell/shell.h>

/* State mode and transition commands */
int cmd_motor_state_prepare_online(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_idle(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_online(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_calibrate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_commission(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_clear_error(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_policy(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_current_encoder(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_velocity_generated(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_position_generated(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_velocity_encoder(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_state_mode_position_encoder(const struct shell *sh, size_t argc, char **argv);

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
int cmd_motor_encoder_fast(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_reg_read(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_reg_write(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_spi4_8_volatile(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_spi4_16_volatile(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_driver_spi4_8(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_driver_spi4_16(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_spi_mode(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_raw_position(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_protocol_raw_reg(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_pipeline(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_pipeline_reset(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_pipeline_inject(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_direction(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trim(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_start(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_stop(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_summary(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_compare(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_capture_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trace_start(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trace_stop(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trace_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trace_summary(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trace_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_encoder_trace_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_start(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_fault_snapshot_stop(const struct shell *sh, size_t argc, char **argv);

#endif /* SHELL_COMMANDS_STATE_H_ */
