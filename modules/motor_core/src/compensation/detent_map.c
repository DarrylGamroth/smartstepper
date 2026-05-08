/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/compensation/detent_map.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"

static bool motor_detent_map_config_valid(const struct motor_detent_map_config *cfg)
{
	return cfg != NULL &&
	       cfg->table_iq_a != NULL &&
	       cfg->table_len >= 2U &&
	       isfinite(cfg->gain) &&
	       isfinite(cfg->iq_ff_limit_a) &&
	       cfg->iq_ff_limit_a >= 0.0f;
}

static uint16_t motor_detent_map_wrap_index(int32_t index, uint16_t table_len)
{
	while (index < 0) {
		index += (int32_t)table_len;
	}
	while (index >= (int32_t)table_len) {
		index -= (int32_t)table_len;
	}

	return (uint16_t)index;
}

int motor_detent_map_init(const struct motor_detent_map_config *cfg,
			  struct motor_detent_map_state *state)
{
	if (!motor_detent_map_config_valid(cfg) || state == NULL) {
		return -EINVAL;
	}

	state->initialized = true;
	state->last_index = 0U;
	state->last_iq_ff_a = 0.0f;

	return 0;
}

void motor_detent_map_reset(struct motor_detent_map_state *state)
{
	if (state == NULL) {
		return;
	}

	state->initialized = false;
	state->last_index = 0U;
	state->last_iq_ff_a = 0.0f;
}

void motor_detent_map_clear(const struct motor_detent_map_config *cfg)
{
	if (!motor_detent_map_config_valid(cfg)) {
		return;
	}

	for (uint16_t i = 0U; i < cfg->table_len; i++) {
		cfg->table_iq_a[i] = 0.0f;
	}
}

int motor_detent_map_set_bin(const struct motor_detent_map_config *cfg,
			     uint16_t index,
			     float32_t iq_ff_a)
{
	if (!motor_detent_map_config_valid(cfg) ||
	    index >= cfg->table_len ||
	    !isfinite(iq_ff_a)) {
		return -EINVAL;
	}

	cfg->table_iq_a[index] = iq_ff_a;
	return 0;
}

int motor_detent_map_mean(const struct motor_detent_map_config *cfg,
			  float32_t *mean_iq_a)
{
	if (!motor_detent_map_config_valid(cfg) || mean_iq_a == NULL) {
		return -EINVAL;
	}

	float32_t sum = 0.0f;
	for (uint16_t i = 0U; i < cfg->table_len; i++) {
		float32_t iq = cfg->table_iq_a[i];
		if (!isfinite(iq)) {
			return -EINVAL;
		}
		sum += iq;
	}

	*mean_iq_a = sum / (float32_t)cfg->table_len;
	return 0;
}

int motor_detent_map_remove_mean(const struct motor_detent_map_config *cfg,
				 float32_t *removed_mean_iq_a)
{
	float32_t mean = 0.0f;
	int ret = motor_detent_map_mean(cfg, &mean);

	if (ret != 0) {
		return ret;
	}

	for (uint16_t i = 0U; i < cfg->table_len; i++) {
		cfg->table_iq_a[i] -= mean;
	}

	if (removed_mean_iq_a != NULL) {
		*removed_mean_iq_a = mean;
	}

	return 0;
}

int motor_detent_map_learn_sample(const struct motor_detent_map_config *cfg,
				  float32_t mech_angle_rad,
				  float32_t iq_sample_a,
				  float32_t alpha)
{
	if (!motor_detent_map_config_valid(cfg) ||
	    !isfinite(mech_angle_rad) ||
	    !isfinite(iq_sample_a) ||
	    !isfinite(alpha) ||
	    alpha < 0.0f ||
	    alpha > 1.0f) {
		return -EINVAL;
	}

	float32_t wrapped = wrap_rad_2pi(mech_angle_rad);
	float32_t scaled = wrapped * ((float32_t)cfg->table_len / (2.0f * PI_F32));
	uint16_t index = (uint16_t)floorf(scaled);

	if (index >= cfg->table_len) {
		index = 0U;
	}

	float32_t old = cfg->table_iq_a[index];
	cfg->table_iq_a[index] = old + alpha * (iq_sample_a - old);

	return 0;
}

int motor_detent_map_lookup(const struct motor_detent_map_config *cfg,
			    float32_t mech_angle_rad,
			    float32_t *iq_ff_a)
{
	if (!motor_detent_map_config_valid(cfg) ||
	    iq_ff_a == NULL ||
	    !isfinite(mech_angle_rad)) {
		return -EINVAL;
	}

	if (!cfg->enabled || cfg->iq_ff_limit_a == 0.0f || cfg->gain == 0.0f) {
		*iq_ff_a = 0.0f;
		return 0;
	}

	float32_t wrapped = wrap_rad_2pi(mech_angle_rad);
	float32_t scaled = wrapped * ((float32_t)cfg->table_len / (2.0f * PI_F32));
	int32_t base = (int32_t)floorf(scaled);
	float32_t frac = scaled - floorf(scaled);

	uint16_t idx0 = motor_detent_map_wrap_index(base + cfg->phase_advance_bins,
						    cfg->table_len);
	uint16_t idx1 = motor_detent_map_wrap_index(base + 1 + cfg->phase_advance_bins,
						    cfg->table_len);
	float32_t y0 = cfg->table_iq_a[idx0];
	float32_t y1 = cfg->table_iq_a[idx1];
	float32_t interp = y0 + frac * (y1 - y0);
	float32_t out = interp * cfg->gain;

	if (out > cfg->iq_ff_limit_a) {
		out = cfg->iq_ff_limit_a;
	} else if (out < -cfg->iq_ff_limit_a) {
		out = -cfg->iq_ff_limit_a;
	}

	*iq_ff_a = out;
	return 0;
}

int motor_detent_map_step_fast(const struct motor_detent_map_config *cfg,
			       struct motor_detent_map_state *state,
			       float32_t mech_angle_rad,
			       float32_t *iq_ff_a)
{
	if (!motor_detent_map_config_valid(cfg) ||
	    state == NULL ||
	    iq_ff_a == NULL ||
	    !state->initialized ||
	    !isfinite(mech_angle_rad)) {
		return -EINVAL;
	}

	float32_t out = 0.0f;
	int ret = motor_detent_map_lookup(cfg, mech_angle_rad, &out);
	if (ret != 0) {
		return ret;
	}
	float32_t wrapped = wrap_rad_2pi(mech_angle_rad);
	float32_t scaled = wrapped * ((float32_t)cfg->table_len / (2.0f * PI_F32));
	int32_t base = (int32_t)floorf(scaled);
	uint16_t idx0 = motor_detent_map_wrap_index(base + cfg->phase_advance_bins,
						    cfg->table_len);

	state->last_index = idx0;
	state->last_iq_ff_a = out;
	*iq_ff_a = out;

	return 0;
}
