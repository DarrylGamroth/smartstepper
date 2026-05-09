/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_chopper_map.h"

#include <errno.h>
#include <math.h>

#include "config.h"
#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"

static bool chopper_kind_valid(uint8_t kind)
{
	return kind == CHOPPER_REGION_KIND_SLOT || kind == CHOPPER_REGION_KIND_TOOTH;
}

static bool chopper_angle_in_region(float32_t angle_rad,
				    float32_t start_rad,
				    float32_t end_rad)
{
	float32_t span = wrap_rad_2pi(end_rad - start_rad);
	float32_t rel = wrap_rad_2pi(angle_rad - start_rad);

	return rel < span;
}

static int chopper_region_index_at_angle(const struct motor_chopper_cal_ctx *cal,
					 float32_t angle_rad,
					 uint16_t idx,
					 bool *match_out)
{
	if (cal == NULL || match_out == NULL || cal->edge_map_count == 0U ||
	    idx >= cal->edge_map_count) {
		return -EINVAL;
	}

	uint16_t next = (uint16_t)((idx + 1U) % cal->edge_map_count);
	*match_out = chopper_angle_in_region(angle_rad,
					     cal->blade_edges_rad[idx],
					     cal->blade_edges_rad[next]);
	return 0;
}

static int chopper_find_region_index(struct motor_chopper_cal_ctx *cal,
				     float32_t angle_rad,
				     uint16_t *idx_out)
{
	uint16_t count = cal->edge_map_count;

	if (cal->blade_state_edge_idx_valid && cal->blade_state_edge_idx < count) {
		uint16_t current = cal->blade_state_edge_idx;
		uint16_t next = (uint16_t)((current + 1U) % count);
		uint16_t prev = (current == 0U) ? (uint16_t)(count - 1U) :
						  (uint16_t)(current - 1U);
		const uint16_t candidates[] = { current, next, prev };

		for (uint16_t i = 0U; i < ARRAY_SIZE(candidates); i++) {
			bool match = false;

			if (chopper_region_index_at_angle(cal, angle_rad,
							  candidates[i], &match) == 0 &&
			    match) {
				*idx_out = candidates[i];
				cal->blade_state_edge_idx = candidates[i];
				return 0;
			}
		}
	}

	for (uint16_t i = 0U; i < count; i++) {
		bool match = false;

		if (chopper_region_index_at_angle(cal, angle_rad, i, &match) == 0 &&
		    match) {
			*idx_out = i;
			cal->blade_state_edge_idx = i;
			cal->blade_state_edge_idx_valid = true;
			return 0;
		}
	}

	cal->blade_state_edge_idx_valid = false;
	return -ENOENT;
}

int motor_chopper_map_derive_centers(struct motor_chopper_cal_ctx *cal)
{
	if (cal == NULL || cal->slots == 0U || cal->teeth == 0U ||
	    cal->edge_map_count == 0U ||
	    cal->edge_map_count > CHOPPER_CAL_MAX_SLOTS ||
	    cal->edge_map_count != (uint16_t)(cal->slots + cal->teeth)) {
		return -ERANGE;
	}

	float32_t ideal_spacing = 2.0f * PI_F32 / (float32_t)cal->edge_map_count;
	float32_t spacing_sum = 0.0f;
	float32_t spacing_min = 2.0f * PI_F32;
	float32_t spacing_max = 0.0f;
	float32_t spacing_max_error = 0.0f;

	for (uint16_t i = 0U; i < cal->edge_map_count; i++) {
		if (!isfinite(cal->blade_edges_rad[i]) ||
		    !chopper_kind_valid(cal->edge_region_after[i])) {
			cal->valid = false;
			cal->midpoint_count = 0U;
			return -ERANGE;
		}
	}

	for (uint16_t i = 0U; i < cal->edge_map_count; i++) {
		float32_t a = wrap_rad_2pi(cal->blade_edges_rad[i]);
		uint16_t next = (uint16_t)((i + 1U) % cal->edge_map_count);
		float32_t b = wrap_rad_2pi(cal->blade_edges_rad[next]);

		if (next == 0U) {
			b += 2.0f * PI_F32;
		}

		float32_t spacing = b - a;
		float32_t midpoint = a + (0.5f * spacing);

		cal->blade_edges_rad[i] = a;
		cal->blade_midpoints_rad[i] = wrap_rad_2pi(midpoint);
		cal->midpoint_kind[i] = cal->edge_region_after[i];
		spacing_sum += spacing;
		spacing_min = fminf(spacing_min, spacing);
		spacing_max = fmaxf(spacing_max, spacing);
		spacing_max_error = fmaxf(spacing_max_error, fabsf(spacing - ideal_spacing));
	}

	cal->midpoint_count = cal->edge_map_count;
	cal->spacing_min_rad = spacing_min;
	cal->spacing_max_rad = spacing_max;
	cal->spacing_mean_rad = spacing_sum / (float32_t)cal->edge_map_count;
	cal->spacing_max_error_rad = spacing_max_error;
	cal->blade_state_edge_idx_valid = false;
	cal->valid = true;
	return 0;
}

int motor_chopper_map_kind_at_angle(struct motor_chopper_cal_ctx *cal,
				    float32_t angle_rad,
				    uint8_t *kind_out)
{
	if (cal == NULL || kind_out == NULL || !cal->valid ||
	    cal->edge_map_count == 0U ||
	    cal->edge_map_count > CHOPPER_CAL_MAX_SLOTS ||
	    !isfinite(angle_rad)) {
		return -EINVAL;
	}

	uint16_t idx = 0U;
	int ret = chopper_find_region_index(cal, wrap_rad_2pi(angle_rad), &idx);
	if (ret != 0) {
		return ret;
	}

	uint8_t kind = cal->edge_region_after[idx];
	if (!chopper_kind_valid(kind)) {
		return -ERANGE;
	}

	*kind_out = kind;
	return 0;
}
