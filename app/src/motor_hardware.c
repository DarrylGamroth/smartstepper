/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "motor_hardware.h"
#include "motor_isr.h"

/* GPIO specifications */
static const struct gpio_dt_spec pi_enable = GPIO_DT_SPEC_GET(DT_PATH(photo_interruptor_enable), gpios);
const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(DT_PATH(trig), gpios);

/* Hardware device instances */
const struct device *const pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
const struct device *const pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
const struct device *const pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));
const struct device *const encoder1 = DEVICE_DT_GET(DT_ALIAS(encoder1));
const struct device *const adc1 = DEVICE_DT_GET(DT_NODELABEL(adc1));
const struct device *const gate_driver_a = DEVICE_DT_GET(DT_ALIAS(gate_driver_a));
const struct device *const gate_driver_b = DEVICE_DT_GET(DT_ALIAS(gate_driver_b));

int motor_hardware_init_gpio(void)
{
	/* Check GPIO device readiness */
	if (!gpio_is_ready_dt(&pi_enable)) {
		printk("GPIO PE1 device is not ready\n");
		return -ENODEV;
	}

	/* Configure GPIO pins */
	int ret = gpio_pin_configure_dt(&pi_enable, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Failed to configure pi_enable GPIO\n");
		return ret;
	}

	ret = gpio_pin_configure_dt(&trig, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Failed to configure trig GPIO\n");
		return ret;
	}

	return 0;
}

int motor_hardware_check_devices(void)
{
	/* Check PWM device readiness */
	if (!device_is_ready(pwm1)) {
		printk("PWM1 device is not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(pwm3)) {
		printk("PWM3 device is not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(pwm8)) {
		printk("PWM8 device is not ready\n");
		return -ENODEV;
	}

	/* Check encoder readiness */
	if (!device_is_ready(encoder1)) {
		printk("encoder1 device is not ready\n");
		return -ENODEV;
	}

	/* Check ADC readiness */
	if (!device_is_ready(adc1)) {
		printk("ADC1 device is not ready\n");
		return -ENODEV;
	}

	/* Check gate driver readiness */
	if (!device_is_ready(gate_driver_a)) {
		printk("Gate driver A device is not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(gate_driver_b)) {
		printk("Gate driver B device is not ready\n");
		return -ENODEV;
	}

	return 0;
}
