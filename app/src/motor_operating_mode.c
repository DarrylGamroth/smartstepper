/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_operating_mode.h"

#include <stddef.h>

#include <zephyr/sys/util.h>

static const struct motor_operating_mode_descriptor mode_descriptors[] = {
	{
		.state = MOTOR_STATE_ONLINE_CURRENT_ENCODER,
		.name = "current_encoder",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_CURRENT_ENCODER,
		.feature_mask = BIT(MOTOR_FEATURE_ENCODER_READ),
		.requires_encoder_setup = true,
		.generated_mode = false,
		.entry_reset = MOTOR_MODE_RESET_DIRECT_CURRENT,
		.exit_reset = MOTOR_MODE_RESET_DIRECT_CURRENT,
	},
	{
		.state = MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
		.name = "velocity_generated",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_GENERATED,
		.feature_mask = BIT(MOTOR_FEATURE_ANGLE_GEN) |
				BIT(MOTOR_FEATURE_VELOCITY_TRAJ),
		.requires_encoder_setup = false,
		.generated_mode = true,
		.entry_reset = MOTOR_MODE_RESET_GENERATED_VELOCITY,
		.exit_reset = MOTOR_MODE_RESET_GENERATED_VELOCITY,
	},
	{
		.state = MOTOR_STATE_ONLINE_POSITION_GENERATED,
		.name = "position_generated",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_POSITION_GENERATED,
		.feature_mask = BIT(MOTOR_FEATURE_ANGLE_GEN),
		.requires_encoder_setup = false,
		.generated_mode = true,
		.entry_reset = MOTOR_MODE_RESET_GENERATED_POSITION,
		.exit_reset = MOTOR_MODE_RESET_GENERATED_POSITION,
	},
	{
		.state = MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
		.name = "velocity_encoder",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER,
		.feature_mask = BIT(MOTOR_FEATURE_ENCODER_READ) |
				BIT(MOTOR_FEATURE_VELOCITY_TRAJ),
		.requires_encoder_setup = true,
		.generated_mode = false,
		.entry_reset = MOTOR_MODE_RESET_ENCODER_VELOCITY,
		.exit_reset = MOTOR_MODE_RESET_ENCODER_VELOCITY,
	},
	{
		.state = MOTOR_STATE_ONLINE_POSITION_ENCODER,
		.name = "position_encoder",
		.policy_mode = MOTOR_CONTROL_POLICY_MODE_POSITION_ENCODER,
		.feature_mask = BIT(MOTOR_FEATURE_ENCODER_READ) |
				BIT(MOTOR_FEATURE_VELOCITY_TRAJ),
		.requires_encoder_setup = true,
		.generated_mode = false,
		.entry_reset = MOTOR_MODE_RESET_ENCODER_POSITION,
		.exit_reset = MOTOR_MODE_RESET_ENCODER_POSITION,
	},
};

const struct motor_operating_mode_descriptor *
motor_operating_mode_descriptor_get(enum motor_state state)
{
	for (size_t i = 0U; i < ARRAY_SIZE(mode_descriptors); i++) {
		if (mode_descriptors[i].state == state) {
			return &mode_descriptors[i];
		}
	}

	return NULL;
}

atomic_val_t motor_operating_mode_feature_mask(enum motor_state state)
{
	const struct motor_operating_mode_descriptor *desc =
		motor_operating_mode_descriptor_get(state);

	return (desc != NULL) ? desc->feature_mask : 0;
}
