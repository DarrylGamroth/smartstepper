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
#include "motor_encoder_acquisition.h"
#include "motor/runtime/keepalive_policy.h"
#include "motor/protection/interlocks.h"
#include "motor/motion/motion_profile.h"
#include "motor_control_telemetry.h"

LOG_MODULE_REGISTER(motor_isr, CONFIG_APP_LOG_LEVEL);

struct motor_adc_collect_stage {
	struct motor_control_encoder_sample encoder_sample;
};

struct motor_adc_process_stage {
	struct motor_control_pwm_output pwm_out;
	struct motor_control_step_report step_report;
};

static void motor_adc_publish_step_report(struct motor_parameters *params,
					  const struct motor_adc_process_stage *process)
{
	const struct motor_control_step_report *report = &process->step_report;

	if (report->encoder_capture_valid) {
		motor_control_telemetry_store_encoder_capture(params, &report->encoder_capture);
	}
	if (report->encoder_raw_trace_valid) {
		motor_control_telemetry_store_encoder_raw_trace(
			params, &report->encoder_sample, &report->encoder_feedback,
			report->position_quality_flags);
	}
	if (report->fault_snapshot.valid) {
		motor_control_telemetry_store_fault_snapshot(params, &report->fault_snapshot);
	}
	if (report->error_pending) {
		params->fault_snapshot.latched = 1U;
		params->fault_snapshot.latch_error_code = report->error_code;
		params->fault_snapshot.latch_loop = params->rt_fast.control_loop_count;
		struct motor_event evt = {
			.type = MOTOR_EVENT_ERROR,
			.error_code = report->error_code,
		};
		(void)motor_api_enqueue_event_from_isr(&evt);
	}
}

static inline uint32_t motor_adc_timeout_ticks_from_ms(uint32_t timeout_ms)
{
	uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)CONTROL_LOOP_FREQUENCY_HZ_U + 999ULL) /
			 1000ULL;

	if (ticks == 0ULL) {
		return 1U;
	}
	if (ticks > UINT32_MAX) {
		return UINT32_MAX;
	}
	return (uint32_t)ticks;
}

static void motor_adc_apply_keepalive_and_timeout(struct motor_parameters *params)
{
	bool control_armed = atomic_get(&params->control_armed) != 0;
	const struct smf_state *state = params->state_for_isr;
	bool online_control_state = motor_state_ptr_is_online_control_state(state);
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) ||
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_POSITION_GENERATED) ||
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) ||
		motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_POSITION_ENCODER);
	bool profile_active = motion_profile_quintic_is_active(&params->position_profile);
	bool autonomous_keepalive = motor_keepalive_policy_should_keepalive(
		control_armed, autonomous_mode_active, params->profile_seq.running,
		params->chopper_cal.active, profile_active);

	if (autonomous_keepalive) {
		params->last_command_update_loop = params->control_loop_count;
		params->command_timeout_latched = false;
	}

	if (params->command_timeout_ms == 0U) {
		return;
	}

	uint32_t timeout_ticks = motor_adc_timeout_ticks_from_ms(params->command_timeout_ms);
	uint32_t command_age_ticks = params->control_loop_count - params->last_command_update_loop;
	if (!online_control_state || !control_armed || autonomous_keepalive ||
	    command_age_ticks <= timeout_ticks) {
		return;
	}

	atomic_set(&params->control_armed, 0);
	if (!params->command_timeout_latched) {
		params->command_timeout_latched = true;
		params->command_timeout_count++;
	}
}

static void motor_adc_stage_collect(struct motor_parameters *params,
					 struct motor_adc_collect_stage *collect)
{
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
	bool encoder_capture_enabled = params->encoder_capture.enabled;
	bool encoder_raw_trace_enabled = params->encoder_raw_trace.enabled;
	bool encoder_sampling_enabled =
		encoder_enabled || encoder_capture_enabled || encoder_raw_trace_enabled;

	/* Publish policy then always collect once to drain any completed CQE/buffer,
	 * even if encoder reads were just disabled this cycle.
	 */
	motor_encoder_acquisition_set_enabled(encoder_sampling_enabled);

	struct motor_encoder_sample sample = {0};
	int ret = motor_encoder_acquisition_collect(&sample);

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

	collect->encoder_sample = encoder_sample;
}

static void motor_adc_stage_process(struct motor_parameters *params,
				    const q31_t *values,
				    uint8_t count,
				    const struct motor_adc_collect_stage *collect,
				    struct motor_adc_process_stage *process)
{
	motor_adc_apply_keepalive_and_timeout(params);

	/* Hardware-timer-driven position-sequence tick source.
	 * Keep event posting out of encoder1_callback (direct ISR context).
	 */
	if (params->profile_seq.running &&
	    atomic_get(&params->control_armed) != 0 &&
	    params->profile_seq.trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL &&
	    (motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER) ||
	     motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_GENERATED))) {
		uint32_t period_ticks = params->profile_seq.period_ticks;
		if (period_ticks == 0U) {
			period_ticks = 1U;
		}

		uint32_t tick_counter = params->profile_seq.tick_counter + 1U;
		if (tick_counter >= period_ticks) {
			struct motor_event evt = {
				.type = MOTOR_EVENT_PROFILE_SEQ_TICK,
			};

			params->profile_seq.tick_counter = 0U;
			int ret = motor_api_enqueue_event_from_isr(&evt);
			if (ret != 0) {
				params->profile_seq.event_drop_count++;
			}
		} else {
			params->profile_seq.tick_counter = tick_counter;
		}
	} else {
		params->profile_seq.tick_counter = 0U;
	}

	motor_control_loop_step(params,
				values,
				count,
				&collect->encoder_sample,
				&process->pwm_out,
				&process->step_report);

	motor_adc_publish_step_report(params, process);
}

static void motor_adc_stage_apply(const struct motor_adc_process_stage *process)
{
	/* Apply only finalized modulation commands produced by Process stage. */
	if (process->pwm_out.update_pwm) {
		mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1,
						      process->pwm_out.da_hb1_pu,
						      process->pwm_out.da_hb2_pu);
		mcpwm_stm32_set_duty_cycle_2phase_f32(pwm8,
						      process->pwm_out.db_hb1_pu,
						      process->pwm_out.db_hb2_pu);
	}
}

static void motor_adc_stage_telemetry(struct motor_parameters *params,
				      timing_t cycles_start)
{
	/* Measure ISR execution time. */
	timing_t cycles_end = timing_counter_get();
	uint64_t cycles_elapsed = timing_cycles_get(&cycles_start, &cycles_end);
	params->total_isr_cycles += (uint32_t)cycles_elapsed;
	if (cycles_elapsed > params->max_isr_cycles) {
		params->max_isr_cycles = (uint32_t)cycles_elapsed;
	}
}

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

	struct motor_adc_collect_stage collect = {0};
	struct motor_adc_process_stage process = {0};

	/* Stage: Collect */
	motor_adc_stage_collect(params, &collect);

	/* Stage: Process */
	motor_adc_stage_process(params, values, count, &collect, &process);

	/* Stage: Apply */
	motor_adc_stage_apply(&process);

	/* Stage: Telemetry */
	motor_adc_stage_telemetry(params, cycles_start);

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

	/* Trigger continuous encoder reads when feature is enabled. */
	bool encoder_enabled =
		atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ENCODER_READ);
	bool encoder_capture_enabled = params->encoder_capture.enabled;
	bool encoder_raw_trace_enabled = params->encoder_raw_trace.enabled;
	bool encoder_sampling_enabled =
		encoder_enabled || encoder_capture_enabled || encoder_raw_trace_enabled;

	motor_encoder_acquisition_set_enabled(encoder_sampling_enabled);
	if (encoder_sampling_enabled) {
		int ret = motor_encoder_acquisition_request_sample();
		if (ret < 0 && ret != -EALREADY) {
			params->encoder_fault_counter++;
		}
	}
}
