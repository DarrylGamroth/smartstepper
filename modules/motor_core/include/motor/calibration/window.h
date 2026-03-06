/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CALIBRATION_WINDOW_H_
#define MOTOR_CALIBRATION_WINDOW_H_

#include <stdint.h>

enum motor_calibration_window_action {
	MOTOR_CALIBRATION_WINDOW_COMPLETE = 0,
	MOTOR_CALIBRATION_WINDOW_EXTEND = 1,
	MOTOR_CALIBRATION_WINDOW_FALLBACK = 2,
};

struct motor_calibration_window_policy {
	uint16_t min_samples;
	uint8_t max_retries;
};

enum motor_calibration_window_action
motor_calibration_window_evaluate(const struct motor_calibration_window_policy *policy,
				  uint16_t sample_count,
				  uint8_t *retry_count_inout);

#endif /* MOTOR_CALIBRATION_WINDOW_H_ */
