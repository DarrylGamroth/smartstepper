/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_OPERATING_MODE_H_
#define MOTOR_OPERATING_MODE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/atomic.h>

#include "motor_states.h"
#include "motor/runtime/control_policy.h"

enum motor_mode_reset_policy {
	MOTOR_MODE_RESET_NONE = 0,
	MOTOR_MODE_RESET_DIRECT_CURRENT,
	MOTOR_MODE_RESET_GENERATED_VELOCITY,
	MOTOR_MODE_RESET_GENERATED_POSITION,
	MOTOR_MODE_RESET_ENCODER_VELOCITY,
	MOTOR_MODE_RESET_ENCODER_POSITION,
};

struct motor_operating_mode_descriptor {
	enum motor_state state;
	const char *name;
	enum motor_control_policy_mode policy_mode;
	atomic_val_t feature_mask;
	bool requires_encoder_setup;
	bool generated_mode;
	enum motor_mode_reset_policy entry_reset;
	enum motor_mode_reset_policy exit_reset;
};

const struct motor_operating_mode_descriptor *
motor_operating_mode_descriptor_get(enum motor_state state);

atomic_val_t motor_operating_mode_feature_mask(enum motor_state state);

#endif /* MOTOR_OPERATING_MODE_H_ */
