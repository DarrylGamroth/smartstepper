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
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <drivers/mcpwm.h>
#include <drivers/spi_tt.h>
#include <dt-bindings/pwm/stm32-mcpwm.h>

#include <drivers/pwm/mcpwm_stm32.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
	struct sensor_value val;
	int rc;
	const struct device *const pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
	const struct device *const pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
	const struct device *const pwm5 = DEVICE_DT_GET(DT_NODELABEL(pwm5));
	const struct device *const pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	if (!device_is_ready(pwm1))
	{
		printk("PWM1 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm3))
	{
		printk("PWM2 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm8))
	{
		printk("PWM8 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm5))
	{
		printk("PWM5 device is not ready\n");
		return 0;
	}

	mcpwm_configure(pwm1, 1, 0);
	mcpwm_configure(pwm1, 4, STM32_PWM_OC_MODE_PWM2);
	mcpwm_configure(pwm8, 1, 0);
	mcpwm_configure(pwm5, 1, 0);
	mcpwm_configure(pwm3, 1, STM32_PWM_OC_MODE_PWM2);

	mcpwm_set_duty_cycle(pwm1, 1, 0x20000000); // 25% duty cycle
	mcpwm_set_duty_cycle(pwm1, 4, 0x01000000); // 50% duty cycle
	mcpwm_set_duty_cycle(pwm8, 1, 0x20000000); // 25% duty cycle
	// mcpwm_stm32_set_duty_cycle_fast_f32(pwm1, 1, 0.25f); // 25% duty cycle
	// mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1, 0.25f, 0.5f); // 25% and 50% duty cycle
	// mcpwm_stm32_set_duty_cycle_2phase(pwm1, 0x60000000, 0x40000000); // 25% and 50% duty cycle
	mcpwm_set_duty_cycle(pwm5, 1, 0x01000000);
	mcpwm_set_duty_cycle(pwm3, 1, 0x01000000);

	mcpwm_enable(pwm8, 1);
	mcpwm_enable(pwm1, 1);
	mcpwm_enable(pwm1, 4);
	mcpwm_enable(pwm5, 1);
	mcpwm_enable(pwm3, 1);

	mcpwm_start(pwm3);

	return 0;
}
