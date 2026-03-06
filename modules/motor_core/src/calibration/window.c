/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/calibration/window.h"

#include <stddef.h>

enum motor_calibration_window_action
motor_calibration_window_evaluate(const struct motor_calibration_window_policy *policy,
				  uint16_t sample_count,
				  uint8_t *retry_count_inout)
{
	if (policy == NULL || retry_count_inout == NULL) {
		return MOTOR_CALIBRATION_WINDOW_FALLBACK;
	}

	if (sample_count >= policy->min_samples) {
		return MOTOR_CALIBRATION_WINDOW_COMPLETE;
	}

	if (*retry_count_inout < policy->max_retries) {
		(*retry_count_inout)++;
		return MOTOR_CALIBRATION_WINDOW_EXTEND;
	}

	return MOTOR_CALIBRATION_WINDOW_FALLBACK;
}
