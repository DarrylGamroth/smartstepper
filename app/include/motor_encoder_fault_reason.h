/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ENCODER_FAULT_REASON_H_
#define MOTOR_ENCODER_FAULT_REASON_H_

#include <stdint.h>

enum motor_encoder_fault_reason {
	MOTOR_ENCODER_FAULT_REASON_NONE = 0,
	MOTOR_ENCODER_FAULT_REASON_READ_EIO,
	MOTOR_ENCODER_FAULT_REASON_NO_FEEDBACK,
	MOTOR_ENCODER_FAULT_REASON_FRAME_OR_IO_ERROR,
	MOTOR_ENCODER_FAULT_REASON_STALE,
	MOTOR_ENCODER_FAULT_REASON_QUALITY,
	MOTOR_ENCODER_FAULT_REASON_PROPAGATED,
	MOTOR_ENCODER_FAULT_REASON_FEEDBACK_INVALID,
	MOTOR_ENCODER_FAULT_REASON_VELOCITY_NAN,
	MOTOR_ENCODER_FAULT_REASON_VELOCITY_SPIKE,
};

static inline const char *motor_encoder_fault_reason_to_string(uint8_t reason)
{
	switch ((enum motor_encoder_fault_reason)reason) {
	case MOTOR_ENCODER_FAULT_REASON_NONE:
		return "none";
	case MOTOR_ENCODER_FAULT_REASON_READ_EIO:
		return "read_eio";
	case MOTOR_ENCODER_FAULT_REASON_NO_FEEDBACK:
		return "no_feedback";
	case MOTOR_ENCODER_FAULT_REASON_FRAME_OR_IO_ERROR:
		return "frame_or_io_error";
	case MOTOR_ENCODER_FAULT_REASON_STALE:
		return "stale";
	case MOTOR_ENCODER_FAULT_REASON_QUALITY:
		return "quality";
	case MOTOR_ENCODER_FAULT_REASON_PROPAGATED:
		return "propagated";
	case MOTOR_ENCODER_FAULT_REASON_FEEDBACK_INVALID:
		return "feedback_invalid";
	case MOTOR_ENCODER_FAULT_REASON_VELOCITY_NAN:
		return "velocity_nan";
	case MOTOR_ENCODER_FAULT_REASON_VELOCITY_SPIKE:
		return "velocity_spike";
	default:
		return "unknown";
	}
}

#endif /* MOTOR_ENCODER_FAULT_REASON_H_ */
