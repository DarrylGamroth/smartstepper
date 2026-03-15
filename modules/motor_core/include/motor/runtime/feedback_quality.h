/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_FEEDBACK_QUALITY_H_
#define MOTOR_RUNTIME_FEEDBACK_QUALITY_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor/observers/feedback_quality.h"

static inline bool motor_velocity_feedback_is_valid(uint8_t quality_flags)
{
	const uint8_t required = MOTOR_FEEDBACK_QUALITY_VALID;
	const uint8_t forbidden = MOTOR_FEEDBACK_QUALITY_ERROR;

	return ((quality_flags & required) != 0U) &&
	       ((quality_flags & forbidden) == 0U);
}

#endif /* MOTOR_RUNTIME_FEEDBACK_QUALITY_H_ */
