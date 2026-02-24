/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_QUALITY_H_
#define MOTOR_CONTROL_QUALITY_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_position_convert.h"

static inline bool motor_velocity_feedback_is_valid(uint8_t quality_flags)
{
	/* Encoder updates may not be fresh every ISR tick (RTIO completion cadence),
	 * but feedback is still usable while quality remains VALID.
	 */
	const uint8_t required = MOTOR_POSITION_CONVERT_QUALITY_VALID;
	const uint8_t forbidden = MOTOR_POSITION_CONVERT_QUALITY_ERROR |
				  MOTOR_POSITION_CONVERT_QUALITY_GLITCH;

	return ((quality_flags & required) != 0U) &&
	       ((quality_flags & forbidden) == 0U);
}

#endif /* MOTOR_CONTROL_QUALITY_H_ */
