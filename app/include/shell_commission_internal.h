/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_COMMISSION_INTERNAL_H_
#define SHELL_COMMISSION_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>
#include <zephyr/shell/shell.h>

#include "config.h"
#include "motor_states.h"
#include "motor/runtime/commission_runtime.h"
#include "motor_control_telemetry.h"

#define MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS 8000U
#define MOTOR_COMMISSION_AUTO_VALIDATE_MIN_HOLD_MS 500U
#define MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS 10000U
#define MOTOR_COMMISSION_MOTION_MODE_TIMEOUT_MS 3000U
#define MOTOR_COMMISSION_MOTION_SAMPLE_MS 5U
#define MOTOR_COMMISSION_MOTION_MIN_SAMPLES 4U
#define MOTOR_COMMISSION_MOTION_ZERO_SETTLE_MS 80U
#define MOTOR_COMMISSION_ENCODER_MAX_ERROR_SAMPLES 4U
#define MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_IQ_A COMMISSION_VALIDATE_CURRENT_DEFAULT_IQ_A
#define MOTOR_COMMISSION_VALIDATE_CURRENT_MAX_IQ_A 0.150f
#define MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_HOLD_MS 1500U
#define MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_RAMP_MS 1000U
#define MOTOR_COMMISSION_VALIDATE_CURRENT_DEFAULT_STOP_DEG 5.0f
#define MOTOR_COMMISSION_VALIDATE_CURRENT_MAX_SPEED_HZ 8.0f
#define MOTOR_COMMISSION_VALIDATE_POSITION_DEFAULT_DELTA_DEG 5.0f
#define MOTOR_COMMISSION_VALIDATE_POSITION_DEFAULT_HOLD_MS 2000U
#define MOTOR_COMMISSION_VALIDATE_POSITION_MIN_DURATION_S 0.20f

struct motor_commission_motion_measurement {
	float32_t iq_a;
	float32_t start_angle_rad;
	float32_t end_angle_rad;
	float32_t net_motion_rad;
	float32_t abs_motion_rad;
	uint16_t sample_count;
	uint16_t warning_count;
	uint16_t error_count;
	float32_t max_abs_velocity_rad_s;
	bool stopped_on_motion;
	bool stopped_on_velocity;
	bool valid;
};

struct motor_commission_encoder_trace_guard {
	bool raw_trace_enabled;
	uint16_t raw_trace_decimation;
	uint16_t raw_trace_phase;
};

int motor_post_mode_change(enum motor_state target_mode);
enum motor_state motor_commission_set_requested_online_mode(enum motor_state mode);
void motor_commission_restore_requested_online_mode(enum motor_state saved_mode);
int motor_commission_request_online_mode(enum motor_state mode);
int motor_commission_wait_for_mode(enum motor_state mode, uint32_t timeout_ms);
int motor_commission_wait_ms_or_fault(uint32_t hold_ms);
int motor_commission_wait_for_control_loop(uint32_t timeout_ms);
void motor_commission_ctx_from_global(struct motor_commission_runtime_ctx *ctx);
int motor_commission_run_motion_threshold(const struct shell *sh,
					  float32_t start_a,
					  float32_t stop_a,
					  float32_t step_a,
					  uint32_t hold_ms,
					  float32_t min_motion_rad);
void motor_commission_set_velocity_target_hz(float32_t target_hz);
void motor_commission_motion_stop_current(void);
int motor_commission_prepare_idle_zero_current(uint32_t settle_ms);
int motor_commission_request_idle_disarmed(void);
void motor_commission_encoder_clear_result(void);
int motor_commission_motion_measure_current(float32_t signed_iq_a,
					    uint32_t hold_ms,
					    float32_t min_motion_rad,
					    struct motor_commission_motion_measurement *out);
int motor_commission_motion_measure_current_bounded(
	float32_t signed_iq_a,
	uint32_t hold_ms,
	uint32_t ramp_ms,
	float32_t min_motion_rad,
	float32_t stop_motion_rad,
	float32_t max_velocity_rad_s,
	struct motor_commission_motion_measurement *out);
void motor_commission_print_velocity_validation_sample(const struct shell *sh,
					       float32_t target_hz);
void motor_commission_encoder_trace_force_on_decimated(
	struct motor_commission_encoder_trace_guard *guard,
	uint16_t decimation);
void motor_commission_encoder_trace_force_on(
	struct motor_commission_encoder_trace_guard *guard);
void motor_commission_encoder_trace_restore(
	const struct motor_commission_encoder_trace_guard *guard);
bool motor_commission_encoder_latest_raw_trace_after(
	uint32_t min_loop,
	struct motor_encoder_raw_trace_sample *out);

#endif /* SHELL_COMMISSION_INTERNAL_H_ */
