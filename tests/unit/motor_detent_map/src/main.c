/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "motor/compensation/detent_map.h"
#include "motor/math/math_constants.h"

static float32_t table[8];

static struct motor_detent_map_config base_cfg(void)
{
	return (struct motor_detent_map_config){
		.enabled = true,
		.table_iq_a = table,
		.table_len = ARRAY_SIZE(table),
		.phase_advance_bins = 0,
		.gain = 1.0f,
		.iq_ff_limit_a = 1.0f,
	};
}

static void clear_table(void)
{
	for (uint32_t i = 0U; i < ARRAY_SIZE(table); i++) {
		table[i] = 0.0f;
	}
}

ZTEST(motor_detent_map, test_disabled_outputs_zero)
{
	clear_table();
	table[0] = 0.5f;
	struct motor_detent_map_config cfg = base_cfg();
	cfg.enabled = false;
	struct motor_detent_map_state state;
	float32_t iq = 123.0f;

	zassert_equal(motor_detent_map_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, 0.0f, &iq), 0, NULL);
	zassert_within(iq, 0.0f, 1e-6f, NULL);
	zassert_within(state.last_iq_ff_a, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_detent_map, test_exact_bin_lookup)
{
	clear_table();
	table[2] = 0.25f;
	struct motor_detent_map_config cfg = base_cfg();
	struct motor_detent_map_state state;
	float32_t iq = 0.0f;

	zassert_equal(motor_detent_map_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, 0.5f * PI_F32, &iq), 0, NULL);
	zassert_within(iq, 0.25f, 1e-6f, NULL);
	zassert_equal(state.last_index, 2U, NULL);
}

ZTEST(motor_detent_map, test_linear_interpolation)
{
	clear_table();
	table[1] = 0.20f;
	table[2] = 0.60f;
	struct motor_detent_map_config cfg = base_cfg();
	struct motor_detent_map_state state;
	float32_t iq = 0.0f;
	float32_t angle = (1.5f / 8.0f) * 2.0f * PI_F32;

	zassert_equal(motor_detent_map_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, angle, &iq), 0, NULL);
	zassert_within(iq, 0.40f, 1e-5f, NULL);
}

ZTEST(motor_detent_map, test_lookup_matches_step_without_state)
{
	clear_table();
	table[1] = 0.20f;
	table[2] = 0.60f;
	struct motor_detent_map_config cfg = base_cfg();
	struct motor_detent_map_state state;
	float32_t iq_step = 0.0f;
	float32_t iq_lookup = 0.0f;
	float32_t angle = (1.5f / 8.0f) * 2.0f * PI_F32;

	zassert_equal(motor_detent_map_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_detent_map_lookup(&cfg, angle, &iq_lookup), 0, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, angle, &iq_step), 0, NULL);
	zassert_within(iq_lookup, iq_step, 1e-6f, NULL);
	zassert_within(iq_lookup, 0.40f, 1e-5f, NULL);
}

ZTEST(motor_detent_map, test_wraparound_interpolation)
{
	clear_table();
	table[7] = -0.40f;
	table[0] = 0.40f;
	struct motor_detent_map_config cfg = base_cfg();
	struct motor_detent_map_state state;
	float32_t iq = 0.0f;
	float32_t angle = (7.5f / 8.0f) * 2.0f * PI_F32;

	zassert_equal(motor_detent_map_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, angle, &iq), 0, NULL);
	zassert_within(iq, 0.0f, 1e-5f, NULL);
}

ZTEST(motor_detent_map, test_phase_advance_and_clamp)
{
	clear_table();
	table[1] = 0.40f;
	struct motor_detent_map_config cfg = base_cfg();
	cfg.phase_advance_bins = 1;
	cfg.gain = 3.0f;
	cfg.iq_ff_limit_a = 0.50f;
	struct motor_detent_map_state state;
	float32_t iq = 0.0f;

	zassert_equal(motor_detent_map_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, 0.0f, &iq), 0, NULL);
	zassert_within(iq, 0.50f, 1e-6f, NULL);
	zassert_equal(state.last_index, 1U, NULL);
}

ZTEST(motor_detent_map, test_learning_updates_selected_bin)
{
	clear_table();
	struct motor_detent_map_config cfg = base_cfg();

	zassert_equal(motor_detent_map_learn_sample(&cfg, 0.5f * PI_F32, 0.80f, 0.25f), 0, NULL);
	zassert_within(table[2], 0.20f, 1e-6f, NULL);
	zassert_within(table[1], 0.0f, 1e-6f, NULL);
}

ZTEST(motor_detent_map, test_invalid_config_rejected)
{
	struct motor_detent_map_config cfg = base_cfg();
	struct motor_detent_map_state state;
	float32_t iq = 0.0f;

	cfg.table_len = 1U;
	zassert_equal(motor_detent_map_init(&cfg, &state), -EINVAL, NULL);
	zassert_equal(motor_detent_map_step_fast(&cfg, &state, 0.0f, &iq), -EINVAL, NULL);
}

ZTEST_SUITE(motor_detent_map, NULL, NULL, NULL, NULL, NULL);
