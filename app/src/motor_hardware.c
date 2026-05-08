/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <drivers/mcpwm.h>
#include <drivers/adc_injected.h>
#include <drivers/gate_driver/ti_drv8328.h>

#include "motor_hardware.h"
#include "motor_isr.h"

/* GPIO specifications */
static const struct gpio_dt_spec pi_enable = GPIO_DT_SPEC_GET(DT_PATH(photo_interruptor_enable), gpios);
const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(DT_PATH(trig), gpios);

/* Hardware device instances */
const struct device *const pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
const struct device *const pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
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

	if (!device_is_ready(pwm8)) {
		printk("PWM8 device is not ready\n");
		return -ENODEV;
	}

	/* Check encoder or fast-path encoder transport readiness */
	if (!device_is_ready(encoder1)) {
		printk("encoder1 device/transport is not ready\n");
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

int motor_hardware_restart_pwm_adc_trigger(void)
{
	int ret;

	ret = adc_injected_enable(adc1);
	if (ret < 0) {
		return ret;
	}

	/* Channel 4 on pwm1 is the injected-ADC trigger. Keep it enabled even
	 * when phase outputs are disabled so the control ISR can keep telemetry
	 * and safety state fresh in powered modes.
	 */
	ret = mcpwm_enable(pwm1, 4);
	if (ret < 0) {
		return ret;
	}

	/* Start the slave before the master. `mcpwm_start()` also restores MOE
	 * on advanced timers, which is required after break/fault recovery.
	 */
	ret = mcpwm_start(pwm8);
	if (ret < 0) {
		return ret;
	}

	return mcpwm_start(pwm1);
}

int motor_hardware_reset_gate_driver_faults(void)
{
	int ret_a = drv8328_reset_fault(gate_driver_a);
	int ret_b = drv8328_reset_fault(gate_driver_b);

	if (ret_a < 0) {
		return ret_a;
	}

	return ret_b;
}

int motor_hardware_reset_gate_driver_faults_masked(void *break_user_data)
{
	int ret_mask_a = mcpwm_set_break_callback(pwm1, NULL, NULL);
	int ret_mask_b = mcpwm_set_break_callback(pwm8, NULL, NULL);

	int ret_reset = motor_hardware_reset_gate_driver_faults();

	int ret_restore_a = mcpwm_set_break_callback(pwm1, gate_driver_a_break_callback,
						     break_user_data);
	int ret_restore_b = mcpwm_set_break_callback(pwm8, gate_driver_b_break_callback,
						     break_user_data);

	if (ret_mask_a < 0) {
		return ret_mask_a;
	}
	if (ret_mask_b < 0) {
		return ret_mask_b;
	}
	if (ret_reset < 0) {
		return ret_reset;
	}
	if (ret_restore_a < 0) {
		return ret_restore_a;
	}
	return ret_restore_b;
}

int motor_hardware_get_gate_driver_faults(bool *fault_a, bool *fault_b)
{
	if (fault_a == NULL || fault_b == NULL) {
		return -EINVAL;
	}

	int ret = drv8328_get_fault_status(gate_driver_a, fault_a);
	if (ret < 0) {
		return ret;
	}

	return drv8328_get_fault_status(gate_driver_b, fault_b);
}

int motor_hardware_set_photo_interruptor_enable(bool enable)
{
	return gpio_pin_set_dt(&pi_enable, enable ? 1 : 0);
}

int motor_hardware_get_photo_interruptor_enable(bool *enabled)
{
	if (enabled == NULL) {
		return -EINVAL;
	}

	int ret = gpio_pin_get_dt(&pi_enable);
	if (ret < 0) {
		return ret;
	}
	*enabled = ret != 0;
	return 0;
}
