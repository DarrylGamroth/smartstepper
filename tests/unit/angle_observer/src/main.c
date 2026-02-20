/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "angle_observer.h"
#include "angle_wrap.h"
#include "math_constants.h"

static void assert_wrapped_0_2pi(float32_t angle_rad)
{
	zassert_true(angle_rad >= 0.0f && angle_rad < 2.0f * PI_F32, NULL);
}

ZTEST(angle_observer, test_init_sets_zero_state_and_gains)
{
	struct angle_observer_state obs = {0};
	angle_observer_init(&obs, 0.001f, 100.0f, 7u, 1.0f);

	zassert_within(obs.mech_angle_rad, 0.0f, 1e-6f, NULL);
	zassert_within(obs.elec_angle_rad, 0.0f, 1e-6f, NULL);
	zassert_within(obs.mech_speed_rad_s, 0.0f, 1e-6f, NULL);
	zassert_true(obs.L1Ts > 0.0f, NULL);
	zassert_true(obs.L2Ts > 0.0f, NULL);
	zassert_equal(obs.pole_pairs, 7u, NULL);
	zassert_within(obs.delay_samples, 1.0f, 1e-6f, NULL);
}

ZTEST(angle_observer, test_converges_on_constant_angle)
{
	struct angle_observer_state obs = {0};
	const float32_t target = 1.0f;
	angle_observer_init(&obs, 0.001f, 50.0f, 4u, 0.0f);

	for (int i = 0; i < 600; i++) {
		angle_observer_update(&obs, target);
	}

	zassert_within(angle_observer_get_mech_angle(&obs), target, 0.03f, NULL);
	zassert_within(angle_observer_get_mech_speed(&obs), 0.0f, 1.0f, NULL);
}

ZTEST(angle_observer, test_tracks_constant_velocity_measurement)
{
	struct angle_observer_state obs = {0};
	const float32_t Ts = 0.001f;
	const float32_t omega_mech = 6.0f;
	float32_t encoder = 0.0f;

	angle_observer_init(&obs, Ts, 80.0f, 4u, 0.0f);
	for (int i = 0; i < 2000; i++) {
		encoder = wrap_rad_2pi(encoder + omega_mech * Ts);
		angle_observer_update(&obs, encoder);
	}

	zassert_within(angle_observer_get_mech_speed(&obs), omega_mech, 0.3f, NULL);
	zassert_within(wrap_rad_pi(angle_observer_get_mech_angle(&obs) - encoder), 0.0f, 0.04f, NULL);
}

ZTEST(angle_observer, test_wrap_and_prediction_ranges)
{
	struct angle_observer_state obs = {0};
	angle_observer_init(&obs, 0.001f, 100.0f, 3u, 0.0f);

	angle_observer_update(&obs, 2.0f * PI_F32 - 0.01f);
	angle_observer_update(&obs, 0.01f);

	float32_t mech = angle_observer_get_mech_angle(&obs);
	float32_t mech_pred = angle_observer_get_mech_angle_pred(&obs);
	float32_t elec = angle_observer_get_elec_angle(&obs);
	float32_t elec_pred = angle_observer_get_elec_angle_pred(&obs);

	assert_wrapped_0_2pi(mech);
	assert_wrapped_0_2pi(mech_pred);
	assert_wrapped_0_2pi(elec);
	assert_wrapped_0_2pi(elec_pred);
}

ZTEST(angle_observer, test_offset_applies_to_electrical_angle)
{
	struct angle_observer_state obs = {0};
	const float32_t offset = 0.2f;
	angle_observer_init(&obs, 0.001f, 60.0f, 5u, 0.0f);
	angle_observer_set_offset(&obs, offset);
	angle_observer_update(&obs, 0.0f);

	float32_t expected = wrap_rad_2pi(offset * 5.0f);
	zassert_within(angle_observer_get_elec_angle(&obs), expected, 0.05f, NULL);
}

ZTEST(angle_observer, test_delay_compensation_changes_single_step_update)
{
	struct angle_observer_state no_delay = {0};
	struct angle_observer_state one_sample_delay = {0};
	const float32_t Ts = 0.001f;
	const float32_t bw_hz = 5.0f;

	angle_observer_init(&no_delay, Ts, bw_hz, 2u, 0.0f);
	angle_observer_init(&one_sample_delay, Ts, bw_hz, 2u, 1.0f);

	no_delay.angle_est_rad = 1.0f;
	no_delay.speed_est_rad_s = 2.0f;
	one_sample_delay.angle_est_rad = 1.0f;
	one_sample_delay.speed_est_rad_s = 2.0f;

	angle_observer_update(&no_delay, 1.0f);
	angle_observer_update(&one_sample_delay, 1.0f);

	zassert_within(no_delay.mech_angle_rad, 1.002f, 1e-6f, NULL);
	zassert_within(no_delay.mech_speed_rad_s, 2.0f, 1e-6f, NULL);
	zassert_true(one_sample_delay.mech_angle_rad > no_delay.mech_angle_rad, NULL);
	zassert_true(one_sample_delay.mech_speed_rad_s > no_delay.mech_speed_rad_s, NULL);
}

ZTEST(angle_observer, test_error_wrap_uses_shortest_path)
{
	struct angle_observer_state obs = {0};
	angle_observer_init(&obs, 0.001f, 20.0f, 1u, 0.0f);

	obs.angle_est_rad = 2.0f * PI_F32 - 0.01f;
	obs.speed_est_rad_s = 0.0f;

	angle_observer_update(&obs, 0.01f);
	zassert_true(obs.mech_speed_rad_s > 0.0f, NULL);
	zassert_true(obs.mech_angle_rad > (2.0f * PI_F32 - 0.02f), NULL);
}

ZTEST(angle_observer, test_prediction_matches_forward_euler_state)
{
	struct angle_observer_state obs = {0};
	const float32_t Ts = 0.0005f;
	const float32_t offset = 0.15f;

	angle_observer_init(&obs, Ts, 30.0f, 4u, 0.0f);
	angle_observer_set_offset(&obs, offset);

	obs.angle_est_rad = 1.2f;
	obs.speed_est_rad_s = 5.0f;
	angle_observer_update(&obs, 1.25f);

	const float32_t expected_mech_pred = wrap_rad_2pi(obs.mech_angle_rad + Ts * obs.mech_speed_rad_s);
	const float32_t expected_elec = wrap_rad_2pi((obs.mech_angle_rad + offset) * 4.0f);
	const float32_t expected_elec_pred = wrap_rad_2pi((expected_mech_pred + offset) * 4.0f);

	zassert_within(obs.mech_angle_pred_rad, expected_mech_pred, 1e-6f, NULL);
	zassert_within(obs.elec_angle_rad, expected_elec, 1e-6f, NULL);
	zassert_within(obs.elec_angle_pred_rad, expected_elec_pred, 1e-6f, NULL);
}

ZTEST(angle_observer, test_delay_setter_and_electrical_speed_accessor)
{
	struct angle_observer_state obs = {0};
	angle_observer_init(&obs, 0.001f, 40.0f, 7u, 0.0f);

	angle_observer_set_delay(&obs, 1.5f);
	zassert_within(obs.delay_samples, 1.5f, 1e-6f, NULL);

	obs.mech_speed_rad_s = -2.5f;
	zassert_within(angle_observer_get_elec_speed(&obs), -17.5f, 1e-6f, NULL);
}

ZTEST_SUITE(angle_observer, NULL, NULL, NULL, NULL, NULL);
