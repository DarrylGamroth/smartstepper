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

struct motor_rt_config_snapshot {
	uint32_t epoch;
	const struct smf_state *state;
	atomic_val_t feature_flags;
	uint32_t velocity_loop_decimation;
	uint32_t position_loop_decimation;
	bool profile_sequence_running;
	bool profile_sequence_loop;
	uint8_t profile_sequence_trigger_source;
	uint8_t profile_sequence_trigger_edge;
	uint8_t profile_sequence_trigger_channel;
	uint32_t profile_sequence_period_ticks;
	uint32_t profile_sequence_period_ms;
};

void motor_config_snapshot_init(void);
void motor_config_snapshot_publish(const struct motor_rt_config_snapshot *snapshot);
bool motor_config_snapshot_read(struct motor_rt_config_snapshot *snapshot_out);

#endif /* CONFIG_SNAPSHOT_H_ */
