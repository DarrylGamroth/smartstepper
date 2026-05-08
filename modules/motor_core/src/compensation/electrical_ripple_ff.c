/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/compensation/electrical_ripple_ff.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/angle_wrap.h"

static bool motor_electrical_ripple_ff_config_valid(
	const struct motor_electrical_ripple_ff_config *cfg)
{
	return cfg != NULL &&
	       cfg->table_iq_a != NULL &&
	       cfg->table_len >= 2U &&
	       isfinite(cfg->gain) &&
	       isfinite(cfg->iq_ff_limit_a) &&
	       cfg->iq_ff_limit_a >= 0.0f;
}

static uint16_t motor_electrical_ripple_ff_wrap_index(int32_t index,
						      uint16_t table_len)
{
	while (index < 0) {
		index += (int32_t)table_len;
	}
	while (index >= (int32_t)table_len) {
		index -= (int32_t)table_len;
	}

	return (uint16_t)index;
}

int motor_electrical_ripple_ff_init(
	const struct motor_electrical_ripple_ff_config *cfg,
	struct motor_electrical_ripple_ff_state *state)
{
	if (!motor_electrical_ripple_ff_config_valid(cfg) || state == NULL) {
		return -EINVAL;
	}

	state->initialized = true;
	state->last_index = 0U;
	state->last_iq_ff_a = 0.0f;

	return 0;
}

void motor_electrical_ripple_ff_reset(
	struct motor_electrical_ripple_ff_state *state)
{
	if (state == NULL) {
		return;
	}

	state->initialized = false;
	state->last_index = 0U;
	state->last_iq_ff_a = 0.0f;
}

void motor_electrical_ripple_ff_clear(
	const struct motor_electrical_ripple_ff_config *cfg)
{
	if (!motor_electrical_ripple_ff_config_valid(cfg)) {
		return;
	}

	for (uint16_t i = 0U; i < cfg->table_len; i++) {
		cfg->table_iq_a[i] = 0.0f;
	}
}

int motor_electrical_ripple_ff_set_bin(
	const struct motor_electrical_ripple_ff_config *cfg,
	uint16_t index,
	float32_t iq_ff_a)
{
	if (!motor_electrical_ripple_ff_config_valid(cfg) ||
	    index >= cfg->table_len ||
	    !isfinite(iq_ff_a)) {
		return -EINVAL;
	}

	cfg->table_iq_a[index] = iq_ff_a;
	return 0;
}

int motor_electrical_ripple_ff_mean(
	const struct motor_electrical_ripple_ff_config *cfg,
	float32_t *mean_iq_a)
{
	if (!motor_electrical_ripple_ff_config_valid(cfg) || mean_iq_a == NULL) {
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

int motor_electrical_ripple_ff_remove_mean(
	const struct motor_electrical_ripple_ff_config *cfg,
	float32_t *removed_mean_iq_a)
{
	float32_t mean = 0.0f;
	int ret = motor_electrical_ripple_ff_mean(cfg, &mean);

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

int motor_electrical_ripple_ff_lookup(
	const struct motor_electrical_ripple_ff_config *cfg,
	float32_t electrical_angle_rad,
	float32_t *iq_ff_a)
{
	if (!motor_electrical_ripple_ff_config_valid(cfg) ||
	    iq_ff_a == NULL ||
	    !isfinite(electrical_angle_rad)) {
		return -EINVAL;
	}

	if (!cfg->enabled || cfg->iq_ff_limit_a == 0.0f || cfg->gain == 0.0f) {
		*iq_ff_a = 0.0f;
		return 0;
	}

	float32_t wrapped = wrap_rad_2pi(electrical_angle_rad);
	float32_t scaled = wrapped * ((float32_t)cfg->table_len / (2.0f * PI_F32));
	float32_t floored = floorf(scaled);
	int32_t base = (int32_t)floored;
	float32_t frac = scaled - floored;
	uint16_t idx0 = motor_electrical_ripple_ff_wrap_index(
		base + cfg->phase_advance_bins, cfg->table_len);
	uint16_t idx1 = motor_electrical_ripple_ff_wrap_index(
		base + 1 + cfg->phase_advance_bins, cfg->table_len);
	float32_t y0 = cfg->table_iq_a[idx0];
	float32_t y1 = cfg->table_iq_a[idx1];
	float32_t out = (y0 + frac * (y1 - y0)) * cfg->gain;

	if (out > cfg->iq_ff_limit_a) {
		out = cfg->iq_ff_limit_a;
	} else if (out < -cfg->iq_ff_limit_a) {
		out = -cfg->iq_ff_limit_a;
	}

	*iq_ff_a = out;
	return 0;
}

int motor_electrical_ripple_ff_step_fast(
	const struct motor_electrical_ripple_ff_config *cfg,
	struct motor_electrical_ripple_ff_state *state,
	float32_t electrical_angle_rad,
	float32_t *iq_ff_a)
{
	if (!motor_electrical_ripple_ff_config_valid(cfg) ||
	    state == NULL ||
	    iq_ff_a == NULL ||
	    !state->initialized ||
	    !isfinite(electrical_angle_rad)) {
		return -EINVAL;
	}

	float32_t out = 0.0f;
	int ret = motor_electrical_ripple_ff_lookup(cfg, electrical_angle_rad, &out);
	if (ret != 0) {
		return ret;
	}

	float32_t wrapped = wrap_rad_2pi(electrical_angle_rad);
	float32_t scaled = wrapped * ((float32_t)cfg->table_len / (2.0f * PI_F32));
	int32_t base = (int32_t)floorf(scaled);
	uint16_t idx0 = motor_electrical_ripple_ff_wrap_index(
		base + cfg->phase_advance_bins, cfg->table_len);

	state->last_index = idx0;
	state->last_iq_ff_a = out;
	*iq_ff_a = out;

	return 0;
}
