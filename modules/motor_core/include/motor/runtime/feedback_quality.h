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
	return motor_feedback_quality_is_usable(quality_flags);
}

static inline bool motor_velocity_feedback_is_trusted(uint8_t quality_flags)
{
	return motor_feedback_quality_is_trusted(quality_flags);
}

#endif /* MOTOR_RUNTIME_FEEDBACK_QUALITY_H_ */
