/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMANDS_COMMISSION_H_
#define SHELL_COMMANDS_COMMISSION_H_

#include <zephyr/shell/shell.h>

int cmd_motor_commission_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_abort(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_boot(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_motion_threshold(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_flux_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_mech_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_encoder_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_encoder_robust(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_encoder_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_encoder_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_encoder_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_detent_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_detent_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_detent_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_detent_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_validate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_validate_current(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_validate_velocity(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_validate_position(const struct shell *sh, size_t argc, char **argv);

#endif /* SHELL_COMMANDS_COMMISSION_H_ */
