/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>

#include <errno.h>

#include "motor/calibration/encoder_map_detect.h"
#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"

#define TEST_POLE_PAIRS 50.0f

static struct motor_encoder_map_detect_config default_cfg(void)
{
	return (struct motor_encoder_map_detect_config){
		.pole_pairs = TEST_POLE_PAIRS,
		.min_mech_motion_rad = 0.02f,
		.max_offset_residual_rad = 0.05f,
		.max_direction_residual_rad = 0.20f,
		.min_direction_correlation = 0.90f,
		.estimate_ratio = false,
	};
}

static void fill_sweep(struct motor_encoder_map_detect_sample *samples,
		       uint32_t count,
		       int8_t sign,
		       float32_t offset_elec_rad)
{
	for (uint32_t i = 0U; i < count; i++) {
		float32_t mech = 0.20f + (0.004f * (float32_t)i);
		float32_t generated_elec =
			((float32_t)sign * TEST_POLE_PAIRS * mech) + offset_elec_rad;

		samples[i].generated_elec_rad = wrap_rad_2pi(generated_elec);
		samples[i].encoder_mech_rad = wrap_rad_2pi(mech);
		samples[i].flags = 0U;
	}
}

ZTEST(motor_encoder_map_detect, test_positive_direction_offset)
{
	struct motor_encoder_map_detect_sample samples[64];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();
	float32_t offset_elec = 0.73f;

	fill_sweep(samples, ARRAY_SIZE(samples), 1, offset_elec);

	zassert_ok(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		   NULL);
	zassert_true(result.valid, NULL);
	zassert_equal(result.direction_sign, 1, NULL);
	zassert_within(result.offset_elec_rad, offset_elec, 2.0e-3f, NULL);
	zassert_within(result.offset_mech_rad, offset_elec / TEST_POLE_PAIRS, 2.0e-4f, NULL);
	zassert_true(result.direction_corr > 0.99f, NULL);
}

ZTEST(motor_encoder_map_detect, test_negative_direction_offset)
{
	struct motor_encoder_map_detect_sample samples[64];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();
	float32_t offset_elec = -1.10f;

	fill_sweep(samples, ARRAY_SIZE(samples), -1, offset_elec);

	zassert_ok(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		   NULL);
	zassert_true(result.valid, NULL);
	zassert_equal(result.direction_sign, -1, NULL);
	zassert_within(result.offset_elec_rad, offset_elec, 2.0e-3f, NULL);
	zassert_true(result.direction_corr < -0.99f, NULL);
}

ZTEST(motor_encoder_map_detect, test_offset_near_wrap)
{
	struct motor_encoder_map_detect_sample samples[64];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();
	float32_t offset_elec = PI_F32 - 0.04f;

	fill_sweep(samples, ARRAY_SIZE(samples), 1, offset_elec);

	zassert_ok(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		   NULL);
	zassert_within(wrap_rad_pi(result.offset_elec_rad - offset_elec), 0.0f, 2.0e-3f,
		       NULL);
}

ZTEST(motor_encoder_map_detect, test_rejects_insufficient_motion)
{
	struct motor_encoder_map_detect_sample samples[8];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();

	fill_sweep(samples, ARRAY_SIZE(samples), 1, 0.2f);
	cfg.min_mech_motion_rad = 1.0f;

	zassert_equal(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		      -ERANGE, NULL);
	zassert_false(result.valid, NULL);
}

ZTEST(motor_encoder_map_detect, test_rejects_encoder_error_samples)
{
	struct motor_encoder_map_detect_sample samples[64];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();

	fill_sweep(samples, ARRAY_SIZE(samples), 1, 0.2f);
	samples[10].flags = MOTOR_ENCODER_MAP_SAMPLE_ERROR;

	zassert_equal(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		      -ERANGE, NULL);
	zassert_false(result.valid, NULL);
	zassert_equal(result.encoder_error_count, 1U, NULL);
	zassert_equal(result.rejected_samples, 1U, NULL);
}

ZTEST(motor_encoder_map_detect, test_accepts_warning_samples_but_counts_them)
{
	struct motor_encoder_map_detect_sample samples[64];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();

	fill_sweep(samples, ARRAY_SIZE(samples), 1, -0.2f);
	samples[12].flags = MOTOR_ENCODER_MAP_SAMPLE_WARNING;

	zassert_ok(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		   NULL);
	zassert_true(result.valid, NULL);
	zassert_equal(result.encoder_warning_count, 1U, NULL);
}

ZTEST(motor_encoder_map_detect, test_rejects_noisy_offset)
{
	struct motor_encoder_map_detect_sample samples[64];
	struct motor_encoder_map_detect_result result = {0};
	struct motor_encoder_map_detect_config cfg = default_cfg();

	fill_sweep(samples, ARRAY_SIZE(samples), 1, 0.2f);
	for (uint32_t i = 0U; i < ARRAY_SIZE(samples); i += 2U) {
		samples[i].generated_elec_rad = wrap_rad_2pi(samples[i].generated_elec_rad + 0.4f);
	}

	zassert_equal(motor_encoder_map_detect_compute(&cfg, samples, ARRAY_SIZE(samples), &result),
		      -ERANGE, NULL);
	zassert_false(result.valid, NULL);
	zassert_false(result.offset_valid, NULL);
}

ZTEST_SUITE(motor_encoder_map_detect, NULL, NULL, NULL, NULL, NULL);
