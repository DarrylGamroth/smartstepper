/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <math.h>

#include "thermal_model.h"

ZTEST(thermal_model, test_init_sets_expected_state)
{
	struct thermal_model m = {0};
	thermal_model_init(&m, 5.0f, 20.0f, 25.0f, 10.0f);

	zassert_within(m.R_th, 5.0f, 1e-6f, NULL);
	zassert_within(m.C_th, 20.0f, 1e-6f, NULL);
	zassert_within(m.T_ambient, 25.0f, 1e-6f, NULL);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);
	zassert_within(m.P_loss, 0.0f, 1e-6f, NULL);
	zassert_within(m.dt, 0.1f, 1e-6f, NULL);
}

ZTEST(thermal_model, test_update_heats_up_with_current)
{
	struct thermal_model m = {0};
	thermal_model_init(&m, 10.0f, 1.0f, 25.0f, 100.0f);

	for (int i = 0; i < 500; i++) {
		thermal_model_update(&m, 1.0f, 0.0f, 2.0f);
	}

	zassert_true(m.T_winding > 25.0f, NULL);
	zassert_true(m.T_winding < 200.0f, NULL);
	zassert_within(m.P_loss, 2.0f, 1e-6f, NULL);
}

ZTEST(thermal_model, test_update_stays_at_ambient_with_zero_loss)
{
	struct thermal_model m = {0};
	thermal_model_init(&m, 8.0f, 2.0f, 30.0f, 50.0f);

	for (int i = 0; i < 200; i++) {
		thermal_model_update(&m, 0.0f, 0.0f, 1.0f);
	}

	zassert_within(m.T_winding, 30.0f, 1e-6f, NULL);
	zassert_within(m.P_loss, 0.0f, 1e-6f, NULL);
}

ZTEST(thermal_model, test_clamps_high_temperature)
{
	struct thermal_model m = {0};
	thermal_model_init(&m, 1.0f, 1.0f, 25.0f, 10.0f);
	m.T_winding = 250.0f;

	thermal_model_update(&m, 0.0f, 0.0f, 1.0f);
	zassert_within(m.T_winding, 200.0f, 1e-6f, NULL);
}

ZTEST(thermal_model, test_rs_to_temperature_conversion)
{
	float32_t t = thermal_Rs_to_temperature(1.1f, 1.0f, 25.0f, 0.004f);
	zassert_within(t, 50.0f, 1e-3f, NULL);
}

ZTEST(thermal_model, test_init_invalid_frequency_disables_updates)
{
	struct thermal_model m = {0};
	thermal_model_init(&m, 5.0f, 20.0f, 25.0f, 0.0f);

	zassert_within(m.dt, 0.0f, 1e-6f, NULL);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);
	zassert_within(m.P_loss, 0.0f, 1e-6f, NULL);

	thermal_model_update(&m, 2.0f, 2.0f, 1.0f);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);
	zassert_within(m.P_loss, 0.0f, 1e-6f, NULL);
}

ZTEST(thermal_model, test_init_invalid_parameters_disable_updates)
{
	struct thermal_model m = {0};

	thermal_model_init(&m, 0.0f, 20.0f, 25.0f, 100.0f);
	zassert_within(m.dt, 0.0f, 1e-6f, NULL);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);
	thermal_model_update(&m, 2.0f, 2.0f, 1.0f);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);

	thermal_model_init(&m, 5.0f, 0.0f, 25.0f, 100.0f);
	zassert_within(m.dt, 0.0f, 1e-6f, NULL);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);
	thermal_model_update(&m, 2.0f, 2.0f, 1.0f);
	zassert_within(m.T_winding, 25.0f, 1e-6f, NULL);

	thermal_model_init(&m, 5.0f, 20.0f, NAN, 100.0f);
	zassert_within(m.dt, 0.0f, 1e-6f, NULL);
	zassert_within(m.T_winding, 0.0f, 1e-6f, NULL);
	zassert_within(m.T_ambient, 0.0f, 1e-6f, NULL);
}

ZTEST(thermal_model, test_update_ignores_nonfinite_input)
{
	struct thermal_model m = {0};
	thermal_model_init(&m, 5.0f, 10.0f, 25.0f, 100.0f);
	m.T_winding = 30.0f;
	m.P_loss = 1.0f;

	thermal_model_update(&m, NAN, 1.0f, 1.0f);
	zassert_within(m.T_winding, 30.0f, 1e-6f, NULL);
	zassert_within(m.P_loss, 1.0f, 1e-6f, NULL);

	thermal_model_update(&m, 1.0f, 1.0f, NAN);
	zassert_within(m.T_winding, 30.0f, 1e-6f, NULL);
	zassert_within(m.P_loss, 1.0f, 1e-6f, NULL);
}

ZTEST_SUITE(thermal_model, NULL, NULL, NULL, NULL, NULL);
