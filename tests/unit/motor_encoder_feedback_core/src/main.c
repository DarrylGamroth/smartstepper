/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <math.h>

#include "motor/math/math_constants.h"
#include "motor/math/angle_wrap.h"
#include "motor/observers/angle_path.h"
#include "motor/observers/encoder_feedback_core.h"

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
	uint8_t src = motor_encoder_feedback_select_source(true, true, true, true, true, true);

	zassert_equal(src, MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED, NULL);
}

ZTEST(motor_encoder_feedback_core, test_quality_helpers_map_trust_states)
{
	uint8_t trusted = MOTOR_FEEDBACK_QUALITY_VALID | MOTOR_FEEDBACK_QUALITY_FRESH;
	uint8_t predicted = MOTOR_FEEDBACK_QUALITY_VALID;
	uint8_t fault = MOTOR_FEEDBACK_QUALITY_ERROR;

	zassert_equal(motor_feedback_quality_trust_state(trusted),
		      MOTOR_FEEDBACK_TRUST_TRUSTED, NULL);
	zassert_equal(motor_feedback_quality_trust_state(predicted),
		      MOTOR_FEEDBACK_TRUST_PREDICTED, NULL);
	zassert_equal(motor_feedback_quality_trust_state(fault),
		      MOTOR_FEEDBACK_TRUST_FAULT, NULL);
	zassert_true(motor_feedback_quality_is_usable(trusted), NULL);
	zassert_true(motor_feedback_quality_is_usable(predicted), NULL);
	zassert_false(motor_feedback_quality_is_usable(fault), NULL);
	zassert_true(motor_feedback_quality_is_trusted(trusted), NULL);
	zassert_false(motor_feedback_quality_is_trusted(predicted), NULL);
}

ZTEST(motor_encoder_feedback_core, test_select_source_encoder_when_fresh)
{
	uint8_t src = motor_encoder_feedback_select_source(false, true, true, false, false, false);

	zassert_equal(src, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
}

ZTEST(motor_encoder_feedback_core, test_select_source_propagated_when_not_fresh)
{
	uint8_t src_not_fresh = motor_encoder_feedback_select_source(false, true, false,
								     false, false, false);
	uint8_t src_disabled = motor_encoder_feedback_select_source(false, false, true,
								   false, false, false);

	zassert_equal(src_not_fresh, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
	zassert_equal(src_disabled, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
}

ZTEST(motor_encoder_feedback_core, test_select_source_encoder_on_warning_only)
{
	uint8_t src_warning = motor_encoder_feedback_select_source(false, true, true,
								  true, false, false);

	zassert_equal(src_warning, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
}

ZTEST(motor_encoder_feedback_core, test_select_source_propagated_on_error_flags)
{
	uint8_t src_error = motor_encoder_feedback_select_source(false, true, true,
								false, true, false);
	uint8_t src_io_fault = motor_encoder_feedback_select_source(false, true, true,
								   false, false, true);

	zassert_equal(src_error, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
	zassert_equal(src_io_fault, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
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

ZTEST(motor_encoder_feedback_core, test_nonfresh_sample_counts_toward_fault_threshold)
{
	struct motor_encoder_feedback_core_input in = base_input();
	struct motor_encoder_feedback_core_state state = {0};

	in.fresh = false;
	in.warning = false;
	in.error = false;
	in.io_fault = false;
	in.fault_threshold = 1U;

	zassert_false(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 1U, NULL);
	zassert_true(motor_encoder_feedback_update_state(&in, &state), NULL);
	zassert_equal(state.fault_counter, 2U, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_encoder_fresh_updates_observer_and_quality)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 0.0f);
	in.feature_angle_gen = false;
	in.sample_enabled = true;
	in.sample_fresh = true;
	in.sample_warning = false;
	in.sample_error = false;
	in.sample_io_fault = false;
	in.sample_angle_deg = 180.0f;
	in.encoder_direction_sign = -1.0f;
	in.generated_mech_rad = 0.3f;
	in.encoder_delay_samples = 1.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
	zassert_within(obs.delay_samples, 1.0f, 1e-6f, NULL);
	zassert_within(out.angle_sensor_deg, 180.0f, 1e-6f, NULL);
	zassert_within(out.angle_control_deg, -180.0f, 1e-6f, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) != 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) != 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) == 0U, NULL);
	zassert_equal(out.control.trust_state, MOTOR_FEEDBACK_TRUST_TRUSTED, NULL);
	zassert_within(out.control.observer_delay_samples, 1.0f, 1e-6f, NULL);
	zassert_within(out.control.prediction_age_samples, 0.0f, 1e-6f, NULL);
	zassert_true(isfinite(out.control.speed_mech_rad_s), NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_generated_has_priority_and_zero_delay)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 2.0f);
	in.feature_angle_gen = true;
	in.sample_enabled = true;
	in.sample_fresh = true;
	in.sample_warning = false;
	in.sample_error = false;
	in.sample_io_fault = false;
	in.sample_angle_deg = 45.0f;
	in.encoder_direction_sign = 1.0f;
	in.generated_mech_rad = 0.8f;
	in.encoder_delay_samples = 1.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED, NULL);
	zassert_within(obs.delay_samples, 0.0f, 1e-6f, NULL);
	zassert_within(out.observer_input_rad, 0.8f, 1e-6f, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) != 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) != 0U, NULL);
	zassert_equal(out.control.trust_state, MOTOR_FEEDBACK_TRUST_TRUSTED, NULL);
	zassert_within(out.control.observer_delay_samples, 0.0f, 1e-6f, NULL);
	zassert_within(out.control.prediction_age_samples, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_error_clears_valid)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 0.0f);
	in.feature_angle_gen = false;
	in.sample_enabled = true;
	in.sample_fresh = true;
	in.sample_warning = false;
	in.sample_error = true;
	in.sample_io_fault = false;
	in.sample_angle_deg = 10.0f;
	in.encoder_direction_sign = 1.0f;
	in.generated_mech_rad = 0.1f;
	in.encoder_delay_samples = 1.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) != 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) == 0U, NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
	zassert_equal(out.control.trust_state, MOTOR_FEEDBACK_TRUST_FAULT, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_propagates_valid_for_bounded_dropout)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 0.0f);
	angle_observer_reset_tracking(&obs, 1.0f, 2.0f);
	in.feature_angle_gen = false;
	in.sample_enabled = true;
	in.sample_fresh = true;
	in.sample_warning = false;
	in.sample_error = true;
	in.sample_io_fault = false;
	in.propagated_valid = true;
	in.previous_input_source = MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER;
	in.sample_angle_deg = 250.0f;
	in.encoder_direction_sign = 1.0f;
	in.generated_mech_rad = 0.1f;
	in.encoder_delay_samples = 1.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) != 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) == 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) == 0U, NULL);
	zassert_equal(out.control.trust_state, MOTOR_FEEDBACK_TRUST_PREDICTED, NULL);
	zassert_within(out.control.prediction_age_samples, 1.0f, 1e-6f, NULL);
	zassert_within(out.control.position_mech_rad, wrap_rad_2pi(1.0f + 0.002f), 1e-6f,
		       NULL);
	zassert_within(out.control.speed_mech_rad_s, 2.0f, 1e-6f, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_propagated_threshold_exceeded_errors)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 0.0f);
	angle_observer_reset_tracking(&obs, 1.0f, 2.0f);
	in.feature_angle_gen = false;
	in.sample_enabled = true;
	in.sample_fresh = false;
	in.sample_warning = false;
	in.sample_error = true;
	in.sample_io_fault = false;
	in.propagated_valid = false;
	in.previous_input_source = MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED;
	in.sample_angle_deg = 250.0f;
	in.encoder_direction_sign = 1.0f;
	in.generated_mech_rad = 0.1f;
	in.encoder_delay_samples = 1.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) == 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) == 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) != 0U, NULL);
	zassert_equal(out.control.trust_state, MOTOR_FEEDBACK_TRUST_FAULT, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_warning_only_remains_valid)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 10.0f, 7U, 0.0f);
	in.feature_angle_gen = false;
	in.sample_enabled = true;
	in.sample_fresh = true;
	in.sample_warning = true;
	in.sample_error = false;
	in.sample_io_fault = false;
	in.sample_angle_deg = 10.0f;
	in.encoder_direction_sign = 1.0f;
	in.generated_mech_rad = 0.1f;
	in.encoder_delay_samples = 1.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) == 0U, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) != 0U, NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
	zassert_equal(out.control.trust_state, MOTOR_FEEDBACK_TRUST_TRUSTED, NULL);
}

ZTEST(motor_encoder_feedback_core, test_angle_path_encoder_handoff_reseeds_without_speed_spike)
{
	struct angle_observer_state obs = {0};
	struct motor_angle_path_input in = {0};
	struct motor_angle_path_output out = {0};

	angle_observer_init(&obs, 0.001f, 100.0f, 50U, 0.0f);
	angle_observer_reset_tracking(&obs, 0.0f, 0.0f);

	in.feature_angle_gen = false;
	in.sample_enabled = true;
	in.sample_fresh = true;
	in.sample_warning = false;
	in.sample_error = false;
	in.sample_io_fault = false;
	in.previous_input_source = MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED;
	in.sample_angle_deg = 90.0f;
	in.encoder_direction_sign = 1.0f;
	in.generated_mech_rad = 0.0f;
	in.encoder_delay_samples = 0.0f;

	zassert_ok(motor_angle_path_step(&obs, &in, &out), NULL);
	zassert_equal(out.control.input_source, MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER, NULL);
	zassert_within(out.control.position_mech_rad, PI_F32 * 0.5f, 1.0e-3f, NULL);
	zassert_within(out.control.speed_mech_rad_s, 0.0f, 1.0e-3f, NULL);
	zassert_true((out.control.quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) != 0U, NULL);
}

ZTEST_SUITE(motor_encoder_feedback_core, NULL, NULL, NULL, NULL, NULL);
