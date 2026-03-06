/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "motor/calibration/offset.h"
#include "motor/calibration/timing.h"
#include "motor/calibration/window.h"

static void timer_noop_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);
}

ZTEST(motor_calibration_flow, test_offset_start_resets_filters_and_offsets)
{
	struct filter_fo_f32 ia = {0};
	struct filter_fo_f32 ib = {0};
	float32_t ia_off = 1.0f;
	float32_t ib_off = -1.0f;

	filter_fo_set_initial_conditions(&ia, 3.0f, -2.0f);
	filter_fo_set_initial_conditions(&ib, -4.0f, 5.0f);

	zassert_ok(motor_offset_measurement_start(&ia, &ib, &ia_off, &ib_off), NULL);
	zassert_within(ia_off, 0.0f, 1e-7f, NULL);
	zassert_within(ib_off, 0.0f, 1e-7f, NULL);
	zassert_within(filter_fo_get_x1(&ia), 0.0f, 1e-7f, NULL);
	zassert_within(filter_fo_get_y1(&ia), 0.0f, 1e-7f, NULL);
	zassert_within(filter_fo_get_x1(&ib), 0.0f, 1e-7f, NULL);
	zassert_within(filter_fo_get_y1(&ib), 0.0f, 1e-7f, NULL);
}

ZTEST(motor_calibration_flow, test_offset_finalize_reads_filtered_values)
{
	struct filter_fo_f32 ia = {0};
	struct filter_fo_f32 ib = {0};
	float32_t ia_off = 0.0f;
	float32_t ib_off = 0.0f;

	filter_fo_set_y1(&ia, 2.25f);
	filter_fo_set_y1(&ib, -1.75f);

	zassert_ok(motor_offset_measurement_finalize(&ia, &ib, &ia_off, &ib_off), NULL);
	zassert_within(ia_off, 2.25f, 1e-7f, NULL);
	zassert_within(ib_off, -1.75f, 1e-7f, NULL);
}

ZTEST(motor_calibration_flow, test_timer_start_validates_duration)
{
	struct k_timer timer;
	k_timer_init(&timer, timer_noop_expiry, NULL);

	zassert_equal(motor_calibration_timer_start_s(NULL, 0.1f), -EINVAL, NULL);
	zassert_equal(motor_calibration_timer_start_s(&timer, 0.0f), -EINVAL, NULL);
	zassert_equal(motor_calibration_timer_start_s(&timer, -0.1f), -EINVAL, NULL);
	zassert_ok(motor_calibration_timer_start_s(&timer, 0.001f), NULL);
}

ZTEST(motor_calibration_flow, test_timer_elapsed_detects_timeout_event_path)
{
	struct k_timer timer;
	bool stale = true;
	k_timer_init(&timer, timer_noop_expiry, NULL);

	zassert_true(motor_calibration_timer_has_elapsed(&timer, true, &stale), NULL);
	zassert_false(stale, NULL);
}

ZTEST(motor_calibration_flow, test_timer_elapsed_detects_stale_expiry_without_event)
{
	struct k_timer timer;
	bool stale = false;
	k_timer_init(&timer, timer_noop_expiry, NULL);

	zassert_ok(motor_calibration_timer_start_s(&timer, 0.002f), NULL);
	k_msleep(5);

	zassert_true(motor_calibration_timer_has_elapsed(&timer, false, &stale), NULL);
	zassert_true(stale, NULL);
}

ZTEST(motor_calibration_flow, test_sample_window_complete_when_min_reached)
{
	const struct motor_calibration_window_policy policy = {
		.min_samples = 4U,
		.max_retries = 3U,
	};
	uint8_t retries = 0U;

	zassert_equal(motor_calibration_window_evaluate(&policy, 4U, &retries),
		      MOTOR_CALIBRATION_WINDOW_COMPLETE, NULL);
	zassert_equal(retries, 0U, NULL);
}

ZTEST(motor_calibration_flow, test_sample_window_extends_and_increments_retry)
{
	const struct motor_calibration_window_policy policy = {
		.min_samples = 4U,
		.max_retries = 3U,
	};
	uint8_t retries = 1U;

	zassert_equal(motor_calibration_window_evaluate(&policy, 2U, &retries),
		      MOTOR_CALIBRATION_WINDOW_EXTEND, NULL);
	zassert_equal(retries, 2U, NULL);
}

ZTEST(motor_calibration_flow, test_sample_window_falls_back_at_retry_limit)
{
	const struct motor_calibration_window_policy policy = {
		.min_samples = 4U,
		.max_retries = 3U,
	};
	uint8_t retries = 3U;

	zassert_equal(motor_calibration_window_evaluate(&policy, 1U, &retries),
		      MOTOR_CALIBRATION_WINDOW_FALLBACK, NULL);
	zassert_equal(retries, 3U, NULL);
}

ZTEST_SUITE(motor_calibration_flow, NULL, NULL, NULL, NULL, NULL);
