/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/smf.h>

#include "motor_states.h"
#include "motor_isr.h"
#include "motor_hardware.h"
#include "motor_control_api.h"
#include "config.h"
#include "shell_commands.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
	/* Get motor parameters pointer from state machine */
	struct motor_parameters *params = motor_sm_get_params();

	/* Set shell access to motor parameters */
	shell_set_motor_params(params);

	/* Create and start state machine thread */
	motor_sm_thread_run();

	LOG_INF("Motor state machine thread started");

	while (1) {
		k_sleep(K_SECONDS(10));
		LOG_INF("Main thread alive");
	}

	return 0;
}
