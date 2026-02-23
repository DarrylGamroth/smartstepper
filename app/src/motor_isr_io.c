/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/timing/timing.h>
#include <zephyr/sys/atomic.h>
#include <drivers/pwm/mcpwm_stm32.h>
#include <drivers/gate_driver/ti_drv8328.h>

#include "motor_control_loop.h"
#include "motor_control_api.h"
#include "motor_isr.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "config.h"
#include "motor_encoder_pipeline.h"

LOG_MODULE_REGISTER(motor_isr, CONFIG_APP_LOG_LEVEL);

void gate_driver_a_break_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* Hardware has already disabled PWM via break input
	 * This interrupt fires when BKIN pin goes active (overcurrent, external fault).
	 */
	drv8328_disable_all_channels(gate_driver_a);

	/* Post error after driver handles disable all channels. */
	motor_api_post_error(ERROR_HARDWARE_BREAK);
}

void gate_driver_b_break_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* Hardware has already disabled PWM via break input
	 * This interrupt fires when BKIN pin goes active (overcurrent, external fault).
	 */
	drv8328_disable_all_channels(gate_driver_b);

	/* Post error after driver handles disable all channels. */
	motor_api_post_error(ERROR_HARDWARE_BREAK);
}

void adc_callback(const struct device *dev, const q31_t *values,
		  uint8_t count, void *user_data)
{
	ARG_UNUSED(dev);

	struct motor_parameters *params = (struct motor_parameters *)user_data;
	if (params == NULL) {
		return;
	}

	gpio_pin_set_dt(&trig, 1);
	timing_t cycles_start = timing_counter_get();

	struct motor_control_encoder_sample encoder_sample = {
		.enabled = false,
		.fresh = false,
		.warning = false,
		.error = false,
		.io_fault = false,
		.status = 0U,
		.angle_deg = 0.0f,
	};

	bool encoder_enabled =
		atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ENCODER_READ);
	bool encoder_capture_enabled = params->encoder_capture_enabled;
	bool encoder_sampling_enabled = encoder_enabled || encoder_capture_enabled;
	/* Publish policy then always collect once to drain any completed CQE/buffer,
	 * even if encoder reads were just disabled this cycle.
	 */
	motor_encoder_pipeline_set_enabled(encoder_sampling_enabled);

	struct motor_encoder_sample sample = {0};
	int ret = motor_encoder_pipeline_collect(&sample);

	encoder_sample.enabled = encoder_enabled;
	if (encoder_sampling_enabled) {
		encoder_sample.angle_deg = sample.angle_deg;
		encoder_sample.status = sample.status;
		encoder_sample.warning = sample.warning;
		encoder_sample.error = sample.error;
		encoder_sample.fresh = (ret == 0);
		encoder_sample.io_fault =
			(ret < 0 && ret != -EAGAIN && ret != -ENODATA);
	}

	/* Schedule one async read per control ISR (pipeline style). This avoids
	 * every-other-cycle submission from the separate timer callback and keeps
	 * encoder latency to the intended single-cycle pipeline depth.
	 */
	if (encoder_sampling_enabled) {
		int req_ret = motor_encoder_pipeline_request_sample();
		if (req_ret < 0 && req_ret != -EALREADY) {
			params->encoder_fault_counter++;
		}
	}

	struct motor_control_pwm_output pwm_out = {0};
	motor_control_loop_step(params, values, count, &encoder_sample, &pwm_out);

	if (pwm_out.update_pwm) {
		mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1, pwm_out.da_hb1_pu, pwm_out.da_hb2_pu);
		mcpwm_stm32_set_duty_cycle_2phase_f32(pwm8, pwm_out.db_hb1_pu, pwm_out.db_hb2_pu);
	}

	/* Measure ISR execution time. */
	timing_t cycles_end = timing_counter_get();
	uint64_t cycles_elapsed = timing_cycles_get(&cycles_start, &cycles_end);
	params->total_isr_cycles += (uint32_t)cycles_elapsed;
	if (cycles_elapsed > params->max_isr_cycles) {
		params->max_isr_cycles = (uint32_t)cycles_elapsed;
	}

	gpio_pin_set_dt(&trig, 0);
}

void encoder1_callback(const struct device *dev, uint32_t channel,
		       void *user_data)
{
	struct motor_parameters *params = (struct motor_parameters *)user_data;
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	if (params == NULL) {
		return;
	}

	/* Hardware-timer-driven position-sequence tick source. */
	if (params->profile_sequence_running &&
	    atomic_get(&params->control_armed) != 0 &&
	    params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL &&
	    motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		uint32_t period_ticks = params->profile_sequence_period_ticks;
		if (period_ticks == 0U) {
			period_ticks = 1U;
		}

		uint32_t tick_counter = params->profile_sequence_tick_counter + 1U;
		if (tick_counter >= period_ticks) {
			struct motor_event evt = {
				.type = MOTOR_EVENT_PROFILE_SEQ_TICK,
			};

			params->profile_sequence_tick_counter = 0U;
			int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
			if (ret != 0) {
				params->profile_sequence_event_drop_count++;
			}
		} else {
			params->profile_sequence_tick_counter = tick_counter;
		}
	} else {
		params->profile_sequence_tick_counter = 0U;
	}
}
