/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMPENSATION_ELECTRICAL_RIPPLE_FF_H_
#define MOTOR_COMPENSATION_ELECTRICAL_RIPPLE_FF_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_electrical_ripple_ff_config {
	bool enabled;
	float32_t *table_iq_a;
	uint16_t table_len;
	int16_t phase_advance_bins;
	float32_t gain;
	float32_t iq_ff_limit_a;
};

struct motor_electrical_ripple_ff_state {
	bool initialized;
	uint16_t last_index;
	float32_t last_iq_ff_a;
};

int motor_electrical_ripple_ff_init(
	const struct motor_electrical_ripple_ff_config *cfg,
	struct motor_electrical_ripple_ff_state *state);

void motor_electrical_ripple_ff_reset(
	struct motor_electrical_ripple_ff_state *state);

void motor_electrical_ripple_ff_clear(
	const struct motor_electrical_ripple_ff_config *cfg);

int motor_electrical_ripple_ff_set_bin(
	const struct motor_electrical_ripple_ff_config *cfg,
	uint16_t index,
	float32_t iq_ff_a);

int motor_electrical_ripple_ff_mean(
	const struct motor_electrical_ripple_ff_config *cfg,
	float32_t *mean_iq_a);

int motor_electrical_ripple_ff_remove_mean(
	const struct motor_electrical_ripple_ff_config *cfg,
	float32_t *removed_mean_iq_a);

int motor_electrical_ripple_ff_lookup(
	const struct motor_electrical_ripple_ff_config *cfg,
	float32_t electrical_angle_rad,
	float32_t *iq_ff_a);

int motor_electrical_ripple_ff_step_fast(
	const struct motor_electrical_ripple_ff_config *cfg,
	struct motor_electrical_ripple_ff_state *state,
	float32_t electrical_angle_rad,
	float32_t *iq_ff_a);

#endif /* MOTOR_COMPENSATION_ELECTRICAL_RIPPLE_FF_H_ */
