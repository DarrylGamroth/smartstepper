/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/runtime/config_snapshot.h"

#include <string.h>

static struct motor_rt_config_snapshot g_cfg_slots[2];
static atomic_t g_cfg_active_slot;
static atomic_t g_cfg_epoch;
static atomic_t g_cfg_published;

void motor_config_snapshot_init(void)
{
	memset(g_cfg_slots, 0, sizeof(g_cfg_slots));
	atomic_set(&g_cfg_active_slot, 0);
	atomic_set(&g_cfg_epoch, 0);
	atomic_set(&g_cfg_published, 0);
}

void motor_config_snapshot_publish(const struct motor_rt_config_snapshot *snapshot)
{
	if (snapshot == NULL) {
		return;
	}

	uint32_t active = (uint32_t)atomic_get(&g_cfg_active_slot) & 1U;
	uint32_t publish_slot = active ^ 1U;
	struct motor_rt_config_snapshot copy = *snapshot;
	uint32_t next_epoch = (uint32_t)atomic_add(&g_cfg_epoch, 1) + 1U;
	copy.epoch = next_epoch;
	g_cfg_slots[publish_slot] = copy;

	/* Publish complete snapshot atomically by flipping active slot index. */
	atomic_set(&g_cfg_active_slot, (atomic_val_t)publish_slot);
	atomic_set(&g_cfg_published, 1);
}

bool motor_config_snapshot_read(struct motor_rt_config_snapshot *snapshot_out)
{
	if (snapshot_out == NULL) {
		return false;
	}

	if (atomic_get(&g_cfg_published) == 0) {
		return false;
	}

	uint32_t active = (uint32_t)atomic_get(&g_cfg_active_slot) & 1U;
	*snapshot_out = g_cfg_slots[active];
	return true;
}
