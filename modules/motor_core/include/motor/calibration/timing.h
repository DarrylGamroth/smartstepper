/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CALIBRATION_TIMING_H_
#define MOTOR_CALIBRATION_TIMING_H_

#include <stdbool.h>

#include <zephyr/dsp/types.h>
#include <zephyr/kernel.h>

int motor_calibration_timer_start_s(struct k_timer *timer, float32_t duration_s);

bool motor_calibration_timer_has_elapsed(struct k_timer *timer,
					 bool timeout_event_seen,
					 bool *stale_expiry_out);

#endif /* MOTOR_CALIBRATION_TIMING_H_ */
