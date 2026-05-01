/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONFIG_SNAPSHOT_H_
#define CONFIG_SNAPSHOT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/smf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "motor/runtime/control_policy.h"

struct motor_rt_config_snapshot {
	uint32_t epoch;
	const struct smf_state *state;
	atomic_val_t feature_flags;
	uint32_t mode_flags;
	uint32_t velocity_loop_decimation;
	uint32_t position_loop_decimation;
	bool profile_sequence_running;
	bool profile_sequence_loop;
	uint8_t profile_sequence_trigger_source;
	uint8_t profile_sequence_trigger_edge;
	uint8_t profile_sequence_trigger_channel;
	uint32_t profile_sequence_period_ticks;
	uint32_t profile_sequence_period_ms;
	bool control_policy_valid;
	struct motor_control_policy_input control_policy_input;
	struct motor_control_policy control_policy;
};

enum motor_rt_mode_flag {
	MOTOR_RT_MODE_OFFSET_MEAS = BIT(0),
	MOTOR_RT_MODE_RS_EST = BIT(1),
	MOTOR_RT_MODE_ROVERL_MEAS = BIT(2),
	MOTOR_RT_MODE_ALIGN_POS_INJECT = BIT(3),
	MOTOR_RT_MODE_ALIGN_POS_SAMPLE = BIT(4),
	MOTOR_RT_MODE_ALIGN_NEG_INJECT = BIT(5),
	MOTOR_RT_MODE_ALIGN_NEG_SAMPLE = BIT(6),
	MOTOR_RT_MODE_ONLINE_CONTROL = BIT(7),
	MOTOR_RT_MODE_ONLINE_VELOCITY_OPEN = BIT(8),
	MOTOR_RT_MODE_ONLINE_TORQUE = BIT(9),
	MOTOR_RT_MODE_ONLINE_VELOCITY_CLOSED = BIT(10),
	MOTOR_RT_MODE_ONLINE_POSITION = BIT(11),
	MOTOR_RT_MODE_ONLINE_PROFILE_OPEN = BIT(12),
	MOTOR_RT_MODE_ERROR = BIT(13),
};

void motor_config_snapshot_init(void);
void motor_config_snapshot_publish(const struct motor_rt_config_snapshot *snapshot);
bool motor_config_snapshot_read(struct motor_rt_config_snapshot *snapshot_out);

#endif /* CONFIG_SNAPSHOT_H_ */
