/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMANDS_H_
#define SHELL_COMMANDS_H_

#include <zephyr/kernel.h>

/* Forward declaration */
struct motor_parameters;

/**
 * @brief Set global motor parameters pointer for shell access
 * Must be called from main() before shell commands can access motor data
 */
void shell_set_motor_params(struct motor_parameters *params);

#endif /* SHELL_COMMANDS_H_ */
