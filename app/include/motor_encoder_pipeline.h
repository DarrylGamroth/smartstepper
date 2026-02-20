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
 * @brief Start an async encoder read if one is not already in flight.
 *
 * @return 0 when a new read was queued, -EALREADY if a read is already in
 * flight, or a negative errno from sensor_read_async_mempool().
 */
int motor_encoder_pipeline_kick(void);

/**
 * @brief Consume one completed async encoder sample if available.
 *
 * @param sample Output sample structure
 * @return 0 when a fresh sample is returned, -EAGAIN when a read is still in
 * flight, -ENODATA when no read is pending or complete, or -EIO on transfer or
 * frame errors.
 */
int motor_encoder_pipeline_poll(struct motor_encoder_sample *sample);

#endif /* MOTOR_ENCODER_PIPELINE_H_ */
