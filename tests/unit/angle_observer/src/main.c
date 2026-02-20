/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "angle_observer.h"
#include "angle_wrap.h"
#include "math_constants.h"

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

	zassert_true(mech >= 0.0f && mech < 2.0f * PI_F32, NULL);
	zassert_true(mech_pred >= 0.0f && mech_pred < 2.0f * PI_F32, NULL);
	zassert_true(elec >= 0.0f && elec < 2.0f * PI_F32, NULL);
	zassert_true(elec_pred >= 0.0f && elec_pred < 2.0f * PI_F32, NULL);
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

ZTEST_SUITE(angle_observer, NULL, NULL, NULL, NULL, NULL);
