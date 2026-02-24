/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/math/math_constants.h"
#include "motor/observers/angle_tracking.h"
#include "motor/observers/encoder_source.h"
#include "motor/observers/motor_encoder_feedback_core.h"

static struct motor_encoder_feedback_core_input base_input(void)
{
	struct motor_encoder_feedback_core_input in = {
		.feature_angle_gen = false,
		.sample_enabled = true,
		.sample_available = true,
		.fresh = true,
		.warning = false,
		.error = false,
		.io_fault = false,
		.status = 0x00U,
		.fault_threshold = 3U,
	};

	return in;
}

ZTEST(motor_encoder_feedback_core, test_select_source_generated_has_priority)
{
	uint8_t src = motor_encoder_feedback_select_source(true, true, true);

	zassert_equal(src, MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED, NULL);
}

ZTEST(motor_encoder_feedback_core, test_select_source_encoder_when_fresh)
{
	uint8_t src = motor_encoder_feedback_select_source(false, true, true);

	zassert_equal(src, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
}

ZTEST(motor_encoder_feedback_core, test_select_source_propagated_when_not_fresh)
{
	uint8_t src_not_fresh = motor_encoder_feedback_select_source(false, true, false);
	uint8_t src_disabled = motor_encoder_feedback_select_source(false, false, true);

	zassert_equal(src_not_fresh, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
	zassert_equal(src_disabled, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
}

ZTEST(motor_encoder_feedback_core, test_update_state_resets_when_sample_disabled)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {
		.fault_counter = 5U,
		.warning_count = 4U,
		.error_count = 3U,
		.sample_fresh = 1U,
		.sample_warning = 1U,
		.sample_error = 1U,
		.last_status = 0x12U,
	};

	in.sample_enabled = false;
	in.fresh = false;
	in.warning = true;
	in.error = true;
	in.io_fault = true;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 0U, NULL);
	zassert_equal(state.sample_fresh, 0U, NULL);
	zassert_equal(state.sample_warning, 0U, NULL);
	zassert_equal(state.sample_error, 0U, NULL);
	zassert_equal(state.warning_count, 4U, NULL);
	zassert_equal(state.error_count, 3U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_update_state_fresh_sample_clears_fault_counter)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {
		.fault_counter = 7U,
		.warning_count = 0U,
		.error_count = 0U,
		.sample_fresh = 0U,
		.sample_warning = 0U,
		.sample_error = 0U,
		.last_status = 0x33U,
	};

	in.status = 0xA5U;
	in.warning = true;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 0U, NULL);
	zassert_equal(state.warning_count, 1U, NULL);
	zassert_equal(state.error_count, 0U, NULL);
	zassert_equal(state.sample_fresh, 1U, NULL);
	zassert_equal(state.sample_warning, 1U, NULL);
	zassert_equal(state.sample_error, 0U, NULL);
	zassert_equal(state.last_status, 0xA5U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_update_state_nonfresh_accumulates_fault_warning_error)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {0};

	in.fresh = false;
	in.warning = true;
	in.error = true;
	in.io_fault = true;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 1U, NULL);
	zassert_equal(state.warning_count, 1U, NULL);
	zassert_equal(state.error_count, 1U, NULL);
	zassert_equal(state.sample_fresh, 0U, NULL);
	zassert_equal(state.sample_warning, 1U, NULL);
	zassert_equal(state.sample_error, 1U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_fault_threshold_is_strictly_greater_than)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {0};

	in.fresh = false;
	in.io_fault = true;
	in.warning = false;
	in.error = false;
	in.fault_threshold = 1U;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 1U, NULL);
	zassert_true(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 2U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_last_status_updates_only_with_valid_sample_event)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {
		.last_status = 0x44U,
	};

	in.sample_available = false;
	in.status = 0x99U;
	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.last_status, 0x44U, NULL);

	in.sample_available = true;
	in.fresh = false;
	in.warning = true;
	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.last_status, 0x99U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_fault_counter_recovers_after_invalid_frame_burst)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {0};

	in.fault_threshold = 2U;
	in.sample_available = true;
	in.fresh = false;
	in.warning = true;
	in.error = true;
	in.io_fault = true;
	in.status = 0xE1U;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_true(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 3U, NULL);
	zassert_equal(state.warning_count, 3U, NULL);
	zassert_equal(state.error_count, 3U, NULL);
	zassert_equal(state.last_status, 0xE1U, NULL);

	in.fresh = true;
	in.warning = false;
	in.error = false;
	in.io_fault = false;
	in.status = 0x10U;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 0U, NULL);
	zassert_equal(state.sample_fresh, 1U, NULL);
	zassert_equal(state.sample_warning, 0U, NULL);
	zassert_equal(state.sample_error, 0U, NULL);
	zassert_equal(state.last_status, 0x10U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_encoder_source_raw_gating_and_direction_sign)
{
	struct motor_encoder_source_sample sample = {0};

	motor_encoder_source_from_raw(true, false, true, false, false, false, 0x55U, 90.0f, false,
				      -1.0f, &sample);
	zassert_false(sample.sample_available, NULL);

	motor_encoder_source_from_raw(true, false, true, false, false, false, 0x55U, 90.0f, true,
				      -1.0f, &sample);
	zassert_true(sample.sample_available, NULL);
	zassert_false(sample.sample_enabled, NULL);
	zassert_within(sample.angle_sensor_deg, 90.0f, 1e-6f, NULL);
	zassert_within(sample.angle_control_deg, -90.0f, 1e-6f, NULL);
}

ZTEST(motor_encoder_feedback_core, test_encoder_source_resolve_angle_prefers_selected_source)
{
	struct motor_encoder_source_sample sample = {
		.angle_control_deg = 180.0f,
	};

	float32_t generated = 0.7f;
	float32_t observer = 0.9f;
	float32_t resolved = motor_encoder_source_resolve_angle_rad(
		MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED, &sample, observer, generated);
	zassert_within(resolved, generated, 1e-6f, NULL);

	resolved = motor_encoder_source_resolve_angle_rad(MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER,
							      &sample, observer, generated);
	zassert_within(resolved, PI_F32, 1e-5f, NULL);

	resolved = motor_encoder_source_resolve_angle_rad(MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED,
							      &sample, observer, generated);
	zassert_within(resolved, observer, 1e-6f, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_tracking_handoff_resets_only_when_unlocked_and_clean)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_tracking_result result = {0};

	angle_observer_init(&obs, 0.001f, 3.0f, 7U, 0.0f);
	obs.angle_est_rad = 0.2f;
	obs.speed_est_rad_s = 5.0f;

	motor_angle_tracking_update(&obs, 1.2f, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, true, false,
				    false, false, 1.0f, &result);
	zassert_equal(result.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
	zassert_within(obs.delay_samples, 1.0f, 1e-6f, NULL);
	zassert_true(obs.mech_speed_rad_s < 0.1f, NULL);
	zassert_within(result.observer_input_rad, 1.2f, 1e-6f, NULL);

	obs.angle_est_rad = 0.2f;
	obs.speed_est_rad_s = 5.0f;
	motor_angle_tracking_update(&obs, 1.2f, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, true, false,
				    false, true, 1.0f, &result);
	zassert_true(obs.mech_speed_rad_s > 4.0f, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_tracking_generated_clears_delay)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_tracking_result result = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 2.0f);
	motor_angle_tracking_update(&obs, 0.3f, MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED, true, false,
				    false, false, 1.0f, &result);

	zassert_equal(result.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED, NULL);
	zassert_within(obs.delay_samples, 0.0f, 1e-6f, NULL);
}

ZTEST_SUITE(motor_encoder_feedback_core, NULL, NULL, NULL, NULL, NULL);
