/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_COMPENSATION_DETENT_MAP_H_
#define MOTOR_COMPENSATION_DETENT_MAP_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_detent_map_config {
	bool enabled;
	float32_t *table_iq_a;
	uint16_t table_len;
	int16_t phase_advance_bins;
	float32_t gain;
	float32_t iq_ff_limit_a;
};

struct motor_detent_map_state {
	bool initialized;
	uint16_t last_index;
	float32_t last_iq_ff_a;
};

int motor_detent_map_init(const struct motor_detent_map_config *cfg,
			  struct motor_detent_map_state *state);

void motor_detent_map_reset(struct motor_detent_map_state *state);

void motor_detent_map_clear(const struct motor_detent_map_config *cfg);

int motor_detent_map_set_bin(const struct motor_detent_map_config *cfg,
			     uint16_t index,
			     float32_t iq_ff_a);

int motor_detent_map_learn_sample(const struct motor_detent_map_config *cfg,
				  float32_t mech_angle_rad,
				  float32_t iq_sample_a,
				  float32_t alpha);

int motor_detent_map_step_fast(const struct motor_detent_map_config *cfg,
			       struct motor_detent_map_state *state,
			       float32_t mech_angle_rad,
			       float32_t *iq_ff_a);

#endif /* MOTOR_COMPENSATION_DETENT_MAP_H_ */

