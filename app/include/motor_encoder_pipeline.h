/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_PIPELINE_H_
#define MOTOR_ENCODER_PIPELINE_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/dsp/types.h>

struct motor_encoder_sample {
	float32_t angle_deg;
	float32_t angle_rad;
	uint8_t status;
	bool warning;
	bool error;
	bool fresh;
};

/**
 * @brief Enable or disable async encoder read requests.
 *
 * When disabled, request calls are rejected with -ESHUTDOWN. Completion
 * collection remains valid and should still be called to drain any in-flight
 * transaction that was started before disable.
 */
void motor_encoder_pipeline_set_enabled(bool enabled);

/**
 * @brief Return true when async encoder requests are currently enabled.
 */
bool motor_encoder_pipeline_is_enabled(void);

/**
 * @brief Return true when one async encoder read is currently in flight.
 */
bool motor_encoder_pipeline_is_busy(void);

/**
 * @brief Request one async encoder read if enabled and not already in flight.
 *
 * @return 0 when a new read was queued, -EALREADY if a read is already in
 * flight, -ESHUTDOWN when disabled, or a negative errno from
 * sensor_read_async_mempool().
 */
int motor_encoder_pipeline_request_sample(void);

/**
 * @brief Collect one completed async encoder sample if available.
 *
 * Call this from the control ISR every cycle, even when reads are disabled.
 * This guarantees RTIO CQEs and buffers are released promptly.
 *
 * @param sample Output sample structure
 * @return 0 when a fresh sample is returned, -EAGAIN when a read is still in
 * flight, -ENODATA when no read is pending or complete, or -EIO on transfer or
 * frame errors.
 */
int motor_encoder_pipeline_collect(struct motor_encoder_sample *sample);

#endif /* MOTOR_ENCODER_PIPELINE_H_ */
