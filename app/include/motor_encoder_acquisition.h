/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_ACQUISITION_H_
#define MOTOR_ENCODER_ACQUISITION_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>

struct motor_encoder_sample {
	float32_t angle_deg;
	float32_t angle_rad;
	uint8_t status;
	bool warning;
	bool error;
	bool frame_status_error;
	bool frame_parity_error;
	bool frame_crc_error;
	bool fresh;
};

struct motor_encoder_acquisition_stats {
	uint32_t request_ok;
	uint32_t request_busy;
	uint32_t request_disabled;
	uint32_t request_error;
	uint32_t collect_ok;
	uint32_t collect_pending;
	uint32_t collect_empty;
	uint32_t collect_error;
	uint32_t collect_transport_error;
	uint32_t collect_frame_error;
	uint32_t collect_frame_parity_error;
	uint32_t collect_frame_crc_error;
	uint32_t collect_frame_status_error;
	uint32_t collect_frame_glitch_error;
};

enum motor_encoder_test_inject_mode {
	MOTOR_ENCODER_TEST_INJECT_NONE = 0,
	MOTOR_ENCODER_TEST_INJECT_STATUS = 1,
	MOTOR_ENCODER_TEST_INJECT_FRAME = 2,
};

/**
 * @brief Enable or disable async encoder read requests.
 *
 * When disabled, request calls are rejected with -ESHUTDOWN. Completion
 * collection remains valid and should still be called to drain any in-flight
 * transaction that was started before disable.
 */
void motor_encoder_acquisition_set_enabled(bool enabled);

/**
 * @brief Return true when async encoder requests are currently enabled.
 */
bool motor_encoder_acquisition_is_enabled(void);

/**
 * @brief Return true when one async encoder read is currently in flight.
 */
bool motor_encoder_acquisition_is_busy(void);

/**
 * @brief Get a snapshot of encoder acquisition counters.
 */
void motor_encoder_acquisition_get_stats(struct motor_encoder_acquisition_stats *stats);

/**
 * @brief Reset encoder acquisition counters to zero.
 */
void motor_encoder_acquisition_reset_stats(void);

/**
 * @brief Abort any in-flight encoder transport request and clear busy state.
 *
 * This is intended for thread-context recovery paths before starting a new
 * commissioning or telemetry capture.
 */
void motor_encoder_acquisition_abort(void);

/**
 * @brief Request one async encoder read if enabled and not already in flight.
 *
 * @return 0 when a new read was queued, -EALREADY if a read is already in
 * flight, -ESHUTDOWN when disabled, or a negative errno from
 * the encoder_rt backend.
 */
int motor_encoder_acquisition_request_sample(void);

/**
 * @brief Collect one completed async encoder sample if available.
 *
 * Call this from the control ISR every cycle, even when reads are disabled.
 * This guarantees the low-latency encoder transport state is drained promptly.
 *
 * @param sample Output sample structure
 * @return 0 when a fresh sample is returned, -EAGAIN when a read is still in
 * flight, -ENODATA when no read is pending or complete, or -EIO on transfer or
 * frame errors.
 */
int motor_encoder_acquisition_collect(struct motor_encoder_sample *sample);

/**
 * @brief Set encoder test injection mode applied to collected frames.
 *
 * This is intended for on-target validation of warning/error handling paths.
 * Injection remains active until set back to `MOTOR_ENCODER_TEST_INJECT_NONE`.
 */
void motor_encoder_acquisition_set_test_inject_mode(enum motor_encoder_test_inject_mode mode);

/**
 * @brief Get currently armed one-shot test injection mode.
 */
enum motor_encoder_test_inject_mode motor_encoder_acquisition_get_test_inject_mode(void);

#endif /* MOTOR_ENCODER_ACQUISITION_H_ */
