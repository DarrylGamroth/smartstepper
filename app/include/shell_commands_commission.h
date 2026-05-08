/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMANDS_COMMISSION_H_
#define SHELL_COMMANDS_COMMISSION_H_

#include <stdbool.h>

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
int cmd_motor_commission_detent_validate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_detent_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_detent_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_ripple_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_ripple_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_ripple_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_ripple_validate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_ripple_dump(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_ripple_clear(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_plan(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_measure_rs(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_measure_inductance(const struct shell *sh, size_t argc,
						       char **argv);
int cmd_motor_commission_electrical_measure_demod(const struct shell *sh, size_t argc,
						  char **argv);
int cmd_motor_commission_electrical_demod_sweep(const struct shell *sh, size_t argc,
						char **argv);
int cmd_motor_commission_electrical_saliency_sweep(const struct shell *sh, size_t argc,
						   char **argv);
int cmd_motor_commission_electrical_saliency_apply(const struct shell *sh, size_t argc,
						   char **argv);
int cmd_motor_commission_electrical_sweep(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_validate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_electrical_clear(const struct shell *sh, size_t argc, char **argv);
int motor_commission_electrical_reapply_if_staged(const struct shell *sh, bool *applied);
int cmd_motor_commission_auto_run(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_status(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_apply(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_auto_validate(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_validate_current(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_validate_velocity(const struct shell *sh, size_t argc, char **argv);
int cmd_motor_commission_validate_position(const struct shell *sh, size_t argc, char **argv);

#endif /* SHELL_COMMANDS_COMMISSION_H_ */
