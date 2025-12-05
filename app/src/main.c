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

/* Global device handles - accessed by ISR and state functions */
const struct device *const pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
const struct device *const pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
const struct device *const pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));
const struct device *const encoder1 = DEVICE_DT_GET(DT_ALIAS(encoder1));
const struct device *const adc1 = DEVICE_DT_GET(DT_NODELABEL(adc1));

int main(void)
{
	/* Get motor parameters pointer from state machine */
	struct motor_parameters *params = motor_sm_get_params();

	/* Set shell access to motor parameters */
	shell_set_motor_params(params);

	/* Create and start state machine thread */
	motor_sm_thread_run();

	LOG_INF("Motor state machine thread started");

	return 0;
}
