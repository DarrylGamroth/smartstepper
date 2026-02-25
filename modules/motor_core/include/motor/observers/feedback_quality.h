/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FEEDBACK_QUALITY_H_
#define FEEDBACK_QUALITY_H_

#include <zephyr/sys/util.h>

enum motor_feedback_quality {
	MOTOR_FEEDBACK_QUALITY_VALID = BIT(0),
	MOTOR_FEEDBACK_QUALITY_FRESH = BIT(1),
	MOTOR_FEEDBACK_QUALITY_ERROR = BIT(2),
};

#endif /* FEEDBACK_QUALITY_H_ */
