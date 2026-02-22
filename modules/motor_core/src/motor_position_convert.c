/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_position_convert.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "angle_wrap.h"
#include "math_constants.h"

#define MOTOR_POSITION_CONVERT_EPSILON 1e-9f

static float32_t motor_position_convert_clampf(float32_t value, float32_t min_value,
					       float32_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static bool motor_position_convert_is_finite_positive(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static bool motor_position_convert_is_finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

static float32_t motor_position_convert_lpf_alpha(float32_t dt_s, float32_t cutoff_hz)
{
	if (cutoff_hz <= 0.0f) {
		return 1.0f;
	}

	float32_t tau_inv = 2.0f * PI_F32 * cutoff_hz;
	float32_t alpha = 1.0f - expf(-tau_inv * dt_s);

	return motor_position_convert_clampf(alpha, 0.0f, 1.0f);
}

int motor_position_convert_validate(const struct motor_position_convert_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (!motor_position_convert_is_finite_positive(cfg->dt_s) ||
	    !motor_position_convert_is_finite_positive(cfg->velocity_lpf_hz) ||
	    !motor_position_convert_is_finite_positive(cfg->accel_lpf_hz) ||
	    !motor_position_convert_is_finite_positive(cfg->max_step_rad) ||
	    !motor_position_convert_is_finite_nonnegative(cfg->latency_samples_default) ||
	    !motor_position_convert_is_finite_nonnegative(cfg->jitter_threshold_rad) ||
	    cfg->stale_threshold_samples == 0U) {
		return -EINVAL;
	}

	if (cfg->max_step_rad > PI_F32) {
		return -EINVAL;
	}

	return 0;
}

void motor_position_convert_reset(struct motor_position_convert_state *state,
			  float32_t initial_wrapped_rad)
{
	if (state == NULL) {
		return;
	}

	float32_t wrapped = wrap_rad_2pi(initial_wrapped_rad);

	state->initialized = true;
	state->stale_latched = false;
	state->prev_meas_wrapped_rad = wrapped;
	state->position_wrapped_rad = wrapped;
	state->position_unwrapped_rad = wrapped;
	state->velocity_rad_s = 0.0f;
	state->accel_rad_s2 = 0.0f;
	state->innovation_rad = 0.0f;
	state->quality_flags = 0U;
	state->stale_count = 0U;
	state->sample_count = 0U;
	state->stale_event_count = 0U;
	state->glitch_count = 0U;
	state->jitter_count = 0U;
}

void motor_position_convert_init(struct motor_position_convert_state *state,
				 const struct motor_position_convert_config *cfg,
				 float32_t initial_wrapped_rad)
{
	if (state == NULL || cfg == NULL) {
		return;
	}

	if (motor_position_convert_validate(cfg) != 0) {
		return;
	}

	motor_position_convert_reset(state, initial_wrapped_rad);
}

int motor_position_convert_update(struct motor_position_convert_state *state,
			   const struct motor_position_convert_config *cfg,
			   const struct motor_position_convert_input *input)
{
	if (state == NULL || cfg == NULL || input == NULL) {
		return -EINVAL;
	}

	int ret = motor_position_convert_validate(cfg);
	if (ret != 0) {
		return ret;
	}

	if (!state->initialized) {
		float32_t initial_wrapped = input->sample_valid ?
			input->measurement_wrapped_rad : 0.0f;

		motor_position_convert_reset(state, initial_wrapped);
	}

	state->sample_count++;

	uint8_t quality = 0U;
	if (input->source_generated) {
		quality |= MOTOR_POSITION_CONVERT_QUALITY_GENERATED;
	}
	if (input->warning) {
		quality |= MOTOR_POSITION_CONVERT_QUALITY_WARNING;
	}
	if (input->error) {
		quality |= MOTOR_POSITION_CONVERT_QUALITY_ERROR;
	}

	float32_t dt_s = cfg->dt_s;
	float32_t alpha_v = motor_position_convert_lpf_alpha(dt_s, cfg->velocity_lpf_hz);
	float32_t alpha_a = motor_position_convert_lpf_alpha(dt_s, cfg->accel_lpf_hz);
	float32_t prev_velocity = state->velocity_rad_s;
	float32_t raw_velocity = prev_velocity;

	bool accepted_sample = false;
	bool glitch_sample = false;
	float32_t step_rad = prev_velocity * dt_s;

	if (input->sample_valid && input->sample_fresh) {
		float32_t latency_samples = input->latency_samples;
		if (!isfinite(latency_samples) || latency_samples < 0.0f) {
			latency_samples = cfg->latency_samples_default;
		}

		float32_t compensated_wrapped = wrap_rad_2pi(
			input->measurement_wrapped_rad + latency_samples * dt_s * prev_velocity);
		float32_t measured_step = wrap_rad_pi(compensated_wrapped - state->prev_meas_wrapped_rad);

		if (fabsf(measured_step) > cfg->max_step_rad) {
			glitch_sample = true;
		} else {
			accepted_sample = true;
			step_rad = measured_step;
			state->prev_meas_wrapped_rad = compensated_wrapped;
			state->position_wrapped_rad = compensated_wrapped;
			state->stale_count = 0U;
			quality |= MOTOR_POSITION_CONVERT_QUALITY_FRESH;
		}
	}

	if (!accepted_sample) {
		state->position_wrapped_rad = wrap_rad_2pi(state->position_wrapped_rad + step_rad);
		if (state->stale_count < UINT16_MAX) {
			state->stale_count++;
		}

		if (glitch_sample) {
			state->glitch_count++;
			quality |= MOTOR_POSITION_CONVERT_QUALITY_GLITCH;
		}
	}

	state->position_unwrapped_rad += step_rad;
	raw_velocity = step_rad / dt_s;

	float32_t innovation = step_rad - prev_velocity * dt_s;
	state->innovation_rad = innovation;
	if (fabsf(innovation) > cfg->jitter_threshold_rad + MOTOR_POSITION_CONVERT_EPSILON) {
		state->jitter_count++;
		quality |= MOTOR_POSITION_CONVERT_QUALITY_JITTER;
	}

	state->velocity_rad_s = prev_velocity + alpha_v * (raw_velocity - prev_velocity);
	float32_t raw_accel = (state->velocity_rad_s - prev_velocity) / dt_s;
	state->accel_rad_s2 = state->accel_rad_s2 + alpha_a * (raw_accel - state->accel_rad_s2);

	if (state->stale_count >= cfg->stale_threshold_samples) {
		quality |= MOTOR_POSITION_CONVERT_QUALITY_STALE;
		if (!state->stale_latched) {
			state->stale_latched = true;
			state->stale_event_count++;
		}
	} else {
		state->stale_latched = false;
	}

	if ((quality & (MOTOR_POSITION_CONVERT_QUALITY_ERROR |
			 MOTOR_POSITION_CONVERT_QUALITY_STALE)) == 0U) {
		quality |= MOTOR_POSITION_CONVERT_QUALITY_VALID;
	}

	state->quality_flags = quality;

	return 0;
}
