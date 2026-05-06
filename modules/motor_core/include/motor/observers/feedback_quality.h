/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FEEDBACK_QUALITY_H_
#define FEEDBACK_QUALITY_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

enum motor_feedback_quality {
	MOTOR_FEEDBACK_QUALITY_VALID = BIT(0),
	MOTOR_FEEDBACK_QUALITY_FRESH = BIT(1),
	MOTOR_FEEDBACK_QUALITY_ERROR = BIT(2),
};

enum motor_feedback_trust_state {
	MOTOR_FEEDBACK_TRUST_FAULT = 0U,
	MOTOR_FEEDBACK_TRUST_PREDICTED = 1U,
	MOTOR_FEEDBACK_TRUST_TRUSTED = 2U,
};

static inline enum motor_feedback_trust_state
motor_feedback_quality_trust_state(uint8_t quality_flags)
{
	if ((quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) != 0U ||
	    (quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) == 0U) {
		return MOTOR_FEEDBACK_TRUST_FAULT;
	}

	if ((quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) != 0U) {
		return MOTOR_FEEDBACK_TRUST_TRUSTED;
	}

	return MOTOR_FEEDBACK_TRUST_PREDICTED;
}

static inline bool motor_feedback_quality_is_usable(uint8_t quality_flags)
{
	return motor_feedback_quality_trust_state(quality_flags) !=
	       MOTOR_FEEDBACK_TRUST_FAULT;
}

static inline bool motor_feedback_quality_is_trusted(uint8_t quality_flags)
{
	return motor_feedback_quality_trust_state(quality_flags) ==
	       MOTOR_FEEDBACK_TRUST_TRUSTED;
}

#endif /* FEEDBACK_QUALITY_H_ */
