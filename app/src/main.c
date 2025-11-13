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

const struct device *const pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
const struct device *const pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
const struct device *const pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));
const struct device *const aeat9955 = DEVICE_DT_GET(DT_NODELABEL(aeat9955));

SENSOR_DT_READ_IODEV(aeat_iodev, DT_NODELABEL(aeat9955), {SENSOR_CHAN_ROTATION, 0});
RTIO_DEFINE_WITH_MEMPOOL(rtio_ctx, 8, 8, 16, 16, sizeof(void *));

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static void mcpwm_callback(const struct device *dev, uint32_t channel, void *user_data)
{
	(void)sensor_read_async_mempool(&aeat_iodev, &rtio_ctx, (void *)&aeat_iodev);
}

/* ADC ISR callback - process completed sensor reads */
static void adc_callback(const struct device *dev, void *user_data)
{
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;
	struct sensor_q31_data position = {0};
	int rc;

	/* Non-blocking: check if a completion is ready */
	cqe = rtio_cqe_consume(&rtio_ctx);
	if (cqe == NULL) {
		/* No completion ready yet */
		return;
	}

	if (cqe->result != 0) {
		printk("Error reading sensor: %d\n", cqe->result);
		rtio_cqe_release(&rtio_ctx, cqe);
		return;
	}

	rc = rtio_cqe_get_mempool_buffer(&rtio_ctx, cqe, &buf, &buf_len);
	if (rc != 0) {
		rtio_cqe_release(&rtio_ctx, cqe);
		printk("Failed to get buffer from cqe: %d\n", rc);
		return;
	}

	struct sensor_decode_context pos_decode =
		SENSOR_DECODE_CONTEXT_INIT(SENSOR_DECODER_DT_GET(DT_NODELABEL(aeat9955)),
					   buf, SENSOR_CHAN_ROTATION, 0);

	rtio_cqe_release(&rtio_ctx, cqe);

	sensor_decode(&pos_decode, &position, 1);
	rtio_release_buffer(&rtio_ctx, buf, buf_len);

	/* Do something with position data in ISR context */
	// Use position.readings[0].angle or whatever you need
}

int main(void)
{
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;

	struct sensor_q31_data position = {0};

	int rc;

	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	if (!device_is_ready(pwm1)) {
		printk("PWM1 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm3)) {
		printk("PWM3 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm8)) {
		printk("PWM8 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(aeat9955)) {
		printk("AEAT9955 device is not ready\n");
		return 0;
	}

	mcpwm_set_compare_callback(pwm3, 1, mcpwm_callback, NULL);

	mcpwm_configure(pwm1, 1, 0);
	mcpwm_configure(pwm1, 2, 0);
	mcpwm_configure(pwm1, 4, STM32_PWM_OC_MODE_PWM2);

	mcpwm_configure(pwm8, 1, 0);
	mcpwm_configure(pwm8, 2, 0);

	mcpwm_configure(pwm3, 1, 0);

	mcpwm_set_duty_cycle(pwm1, 1, 0x20000000); // 25% duty cycle
	mcpwm_set_duty_cycle(pwm1, 2, 0x40000000); // 50% duty cycle
	mcpwm_set_duty_cycle(pwm1, 4, 0x01000000);

	mcpwm_set_duty_cycle(pwm8, 1, 0x28000000);
	mcpwm_set_duty_cycle(pwm8, 2, 0x30000000);

	mcpwm_set_duty_cycle(pwm3, 1, 0x47AE147A); // 56% duty cycle

	mcpwm_enable(pwm8, 1);
	mcpwm_enable(pwm8, 2);

	mcpwm_enable(pwm3, 1);

	mcpwm_enable(pwm1, 1);
	mcpwm_enable(pwm1, 2);
	mcpwm_enable(pwm1, 4);	

	mcpwm_start(pwm1);

	while (1) {
		cqe = rtio_cqe_consume_block(&rtio_ctx);

		if (cqe->result != 0) {
			printk("Error reading sensor: %d\n", cqe->result);
			rtio_cqe_release(&rtio_ctx, cqe);
			continue;
		}

		rc = rtio_cqe_get_mempool_buffer(&rtio_ctx, cqe, &buf, &buf_len);
		if (rc != 0) {
			rtio_cqe_release(&rtio_ctx, cqe);
			printk("Failed to get buffer from cqe: %d\n", rc);
			continue;
		}

		struct sensor_decode_context pos_decode =
			SENSOR_DECODE_CONTEXT_INIT(SENSOR_DECODER_DT_GET(DT_NODELABEL(aeat9955)),
						   buf, SENSOR_CHAN_ROTATION, 0);

		rtio_cqe_release(&rtio_ctx, cqe);

		sensor_decode(&pos_decode, &position, 1);
		rtio_release_buffer(&rtio_ctx, buf, buf_len);

		// printk("Position: " PRIsensor_q31_data "\n", PRIsensor_q31_data_arg(position, 0));
	}

	return 0;
}
