/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "motor_position_convert.h"
#include "math_constants.h"

static struct motor_position_convert_config test_cfg(void)
{
	struct motor_position_convert_config cfg = {
		.dt_s = 0.001f,
		.velocity_lpf_hz = 600.0f,
		.accel_lpf_hz = 300.0f,
		.max_step_rad = 0.95f * PI_F32,
		.latency_samples_default = 0.0f,
		.jitter_threshold_rad = 0.05f,
		.stale_threshold_samples = 3U,
	};

	return cfg;
}

static void update_with_angle(struct motor_position_convert_state *state,
			      const struct motor_position_convert_config *cfg,
			      float32_t angle_wrapped_rad,
			      bool fresh,
			      bool generated)
{
	struct motor_position_convert_input input = {
		.sample_valid = true,
		.sample_fresh = fresh,
		.source_generated = generated,
		.warning = false,
		.error = false,
		.measurement_wrapped_rad = angle_wrapped_rad,
		.latency_samples = 0.0f,
	};

	zassert_ok(motor_position_convert_update(state, cfg, &input), NULL);
}

static void update_without_sample(struct motor_position_convert_state *state,
				  const struct motor_position_convert_config *cfg)
{
	struct motor_position_convert_input input = {
		.sample_valid = false,
		.sample_fresh = false,
		.source_generated = false,
		.warning = false,
		.error = false,
		.measurement_wrapped_rad = 0.0f,
		.latency_samples = 0.0f,
	};

	zassert_ok(motor_position_convert_update(state, cfg, &input), NULL);
}

ZTEST(motor_position_convert, test_validate_rejects_invalid_config)
{
	struct motor_position_convert_config cfg = test_cfg();

	zassert_ok(motor_position_convert_validate(&cfg), NULL);
	zassert_equal(motor_position_convert_validate(NULL), -EINVAL, NULL);

	cfg.dt_s = 0.0f;
	zassert_equal(motor_position_convert_validate(&cfg), -EINVAL, NULL);
	cfg = test_cfg();
	cfg.max_step_rad = PI_F32 + 0.01f;
	zassert_equal(motor_position_convert_validate(&cfg), -EINVAL, NULL);
	cfg = test_cfg();
	cfg.stale_threshold_samples = 0U;
	zassert_equal(motor_position_convert_validate(&cfg), -EINVAL, NULL);
}

ZTEST(motor_position_convert, test_startup_behavior_initializes_from_first_sample)
{
	struct motor_position_convert_state state = {0};
	struct motor_position_convert_config cfg = test_cfg();

	update_with_angle(&state, &cfg, 1.25f, true, false);

	zassert_true(state.initialized, NULL);
	zassert_within(state.position_wrapped_rad, 1.25f, 1e-6f, NULL);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_VALID) != 0U, NULL);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_FRESH) != 0U, NULL);
	zassert_equal(state.stale_count, 0U, NULL);
}

ZTEST(motor_position_convert, test_wrap_boundary_unwraps_continuously)
{
	struct motor_position_convert_state state = {0};
	struct motor_position_convert_config cfg = test_cfg();

	update_with_angle(&state, &cfg, 6.20f, true, false);
	float32_t unwrap0 = state.position_unwrapped_rad;

	update_with_angle(&state, &cfg, 0.03f, true, false);
	float32_t unwrap1 = state.position_unwrapped_rad;
	float32_t delta = unwrap1 - unwrap0;

	zassert_true(delta > 0.0f, NULL);
	zassert_within(delta, 0.113185f, 0.01f, NULL);
}

ZTEST(motor_position_convert, test_dropouts_set_stale_and_count_once)
{
	struct motor_position_convert_state state = {0};
	struct motor_position_convert_config cfg = test_cfg();

	update_with_angle(&state, &cfg, 0.40f, true, false);
	zassert_equal(state.stale_event_count, 0U, NULL);

	update_without_sample(&state, &cfg);
	update_without_sample(&state, &cfg);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) == 0U, NULL);

	update_without_sample(&state, &cfg);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) != 0U, NULL);
	zassert_equal(state.stale_event_count, 1U, NULL);

	update_without_sample(&state, &cfg);
	zassert_equal(state.stale_event_count, 1U, NULL);

	update_with_angle(&state, &cfg, 0.45f, true, false);
	zassert_equal(state.stale_count, 0U, NULL);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) == 0U, NULL);
}

ZTEST(motor_position_convert, test_nonfresh_sample_not_marked_fresh_or_accepted)
{
	struct motor_position_convert_state state = {0};
	struct motor_position_convert_config cfg = test_cfg();
	struct motor_position_convert_input input = {
		.sample_valid = true,
		.sample_fresh = false,
		.source_generated = false,
		.warning = false,
		.error = false,
		.measurement_wrapped_rad = 0.9f,
		.latency_samples = 0.0f,
	};

	update_with_angle(&state, &cfg, 0.40f, true, false);
	float32_t unwrap_before = state.position_unwrapped_rad;

	zassert_ok(motor_position_convert_update(&state, &cfg, &input), NULL);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_FRESH) == 0U, NULL);
	zassert_equal(state.stale_count, 1U, NULL);
	zassert_within(state.position_unwrapped_rad, unwrap_before, 0.05f, NULL);
}

ZTEST(motor_position_convert, test_jitter_flag_and_counter)
{
	struct motor_position_convert_state state = {0};
	struct motor_position_convert_config cfg = test_cfg();

	update_with_angle(&state, &cfg, 0.20f, true, false);
	update_with_angle(&state, &cfg, 0.21f, true, false);
	uint32_t jitter_before = state.jitter_count;

	/* Large innovation but still below max_step_rad => should mark jitter. */
	update_with_angle(&state, &cfg, 0.80f, true, false);
	zassert_true((state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_JITTER) != 0U, NULL);
	zassert_true(state.jitter_count > jitter_before, NULL);
}

ZTEST(motor_position_convert, test_direction_reversal_tracks_velocity_sign_change)
{
	struct motor_position_convert_state state = {0};
	struct motor_position_convert_config cfg = test_cfg();

	update_with_angle(&state, &cfg, 1.00f, true, false);
	for (int i = 0; i < 10; i++) {
		update_with_angle(&state, &cfg, 1.00f + 0.01f * (float32_t)(i + 1), true, false);
	}
	zassert_true(state.velocity_rad_s > 0.0f, NULL);

	for (int i = 0; i < 16; i++) {
		update_with_angle(&state, &cfg, 1.10f - 0.015f * (float32_t)(i + 1), true, false);
	}
	zassert_true(state.velocity_rad_s < 0.0f, NULL);
}

ZTEST(motor_position_convert, test_latency_compensation_advances_measurement)
{
	struct motor_position_convert_state state_no_latency = {0};
	struct motor_position_convert_state state_latency = {0};
	struct motor_position_convert_config cfg = test_cfg();

	motor_position_convert_init(&state_no_latency, &cfg, 0.0f);
	motor_position_convert_init(&state_latency, &cfg, 0.0f);
	state_no_latency.velocity_rad_s = 20.0f;
	state_latency.velocity_rad_s = 20.0f;

	struct motor_position_convert_input no_latency = {
		.sample_valid = true,
		.sample_fresh = true,
		.source_generated = false,
		.warning = false,
		.error = false,
		.measurement_wrapped_rad = 1.0f,
		.latency_samples = 0.0f,
	};
	struct motor_position_convert_input with_latency = no_latency;
	with_latency.latency_samples = 1.0f;

	zassert_ok(motor_position_convert_update(&state_no_latency, &cfg, &no_latency), NULL);
	zassert_ok(motor_position_convert_update(&state_latency, &cfg, &with_latency), NULL);

	zassert_true(state_latency.position_wrapped_rad > state_no_latency.position_wrapped_rad,
		     NULL);
}

ZTEST_SUITE(motor_position_convert, NULL, NULL, NULL, NULL, NULL);
