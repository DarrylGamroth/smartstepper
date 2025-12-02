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
	/* Motor parameters structure - initialized by motor_control_api_init() */
	static struct motor_parameters motor_params;

	/* Print configuration parameters */
	config_print_parameters();

	/* Initialize motor control API and shell access to motor parameters */
	if (motor_control_api_init(&motor_params) < 0) {
		LOG_ERR("Failed to initialize motor control API");
		return 0;
	}
	shell_set_motor_params(&motor_params);

	/* Initialize hardware */
	if (motor_hardware_init_gpio() < 0) {
		LOG_ERR("Failed to initialize GPIO");
		return 0;
	}

	if (motor_hardware_check_devices() < 0) {
		LOG_ERR("Device readiness check failed");
		return 0;
	}

	/* Initialize state machine */
	smf_set_initial(SMF_CTX(&motor_params), &motor_states[MOTOR_STATE_HW_INIT]);

	/* Main control loop */
	while (1) {
		/* Check for pending events from API (Shell, MODBUS, CAN, etc.)
		 * Events are peeked (not consumed) so state machine can examine them.
		 * State run() functions should call motor_api_consume_event() after handling.
		 */
		struct motor_event evt;
		if (motor_api_peek_event(&evt) == 0) {
			LOG_DBG("Event pending: type=%d", evt.type);
		}

		/* Run state machine (states will check for and handle events) */
		if (smf_run_state(SMF_CTX(&motor_params)) < 0) {
			LOG_ERR("State machine error");
			break;
		}

		k_sleep(K_MSEC(100));
	}

	return 0;
}
