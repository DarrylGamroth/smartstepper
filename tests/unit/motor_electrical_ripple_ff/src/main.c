/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "motor/compensation/electrical_ripple_ff.h"
#include "motor/math/math_constants.h"

static float32_t table[8];

static struct motor_electrical_ripple_ff_config base_cfg(void)
{
	return (struct motor_electrical_ripple_ff_config){
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

ZTEST(motor_electrical_ripple_ff, test_disabled_outputs_zero)
{
	clear_table();
	table[0] = 0.5f;
	struct motor_electrical_ripple_ff_config cfg = base_cfg();
	cfg.enabled = false;
	struct motor_electrical_ripple_ff_state state;
	float32_t iq = 123.0f;

	zassert_equal(motor_electrical_ripple_ff_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_electrical_ripple_ff_step_fast(&cfg, &state, 0.0f, &iq), 0, NULL);
	zassert_within(iq, 0.0f, 1e-6f, NULL);
	zassert_within(state.last_iq_ff_a, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_electrical_ripple_ff, test_exact_bin_lookup)
{
	clear_table();
	table[2] = 0.25f;
	struct motor_electrical_ripple_ff_config cfg = base_cfg();
	struct motor_electrical_ripple_ff_state state;
	float32_t iq = 0.0f;

	zassert_equal(motor_electrical_ripple_ff_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_electrical_ripple_ff_step_fast(&cfg, &state, 0.5f * PI_F32, &iq), 0, NULL);
	zassert_within(iq, 0.25f, 1e-6f, NULL);
	zassert_equal(state.last_index, 2U, NULL);
}

ZTEST(motor_electrical_ripple_ff, test_linear_interpolation_and_wrap)
{
	clear_table();
	table[7] = -0.40f;
	table[0] = 0.40f;
	struct motor_electrical_ripple_ff_config cfg = base_cfg();
	struct motor_electrical_ripple_ff_state state;
	float32_t iq = 0.0f;
	float32_t angle = (7.5f / 8.0f) * 2.0f * PI_F32;

	zassert_equal(motor_electrical_ripple_ff_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_electrical_ripple_ff_step_fast(&cfg, &state, angle, &iq), 0, NULL);
	zassert_within(iq, 0.0f, 1e-5f, NULL);
}

ZTEST(motor_electrical_ripple_ff, test_phase_advance_and_clamp)
{
	clear_table();
	table[1] = 0.40f;
	struct motor_electrical_ripple_ff_config cfg = base_cfg();
	cfg.phase_advance_bins = 1;
	cfg.gain = 3.0f;
	cfg.iq_ff_limit_a = 0.50f;
	struct motor_electrical_ripple_ff_state state;
	float32_t iq = 0.0f;

	zassert_equal(motor_electrical_ripple_ff_init(&cfg, &state), 0, NULL);
	zassert_equal(motor_electrical_ripple_ff_step_fast(&cfg, &state, 0.0f, &iq), 0, NULL);
	zassert_within(iq, 0.50f, 1e-6f, NULL);
	zassert_equal(state.last_index, 1U, NULL);
}

ZTEST(motor_electrical_ripple_ff, test_mean_and_remove_mean)
{
	clear_table();
	table[0] = 0.30f;
	table[1] = 0.10f;
	table[2] = -0.10f;
	table[3] = -0.30f;
	table[4] = 0.50f;
	table[5] = 0.50f;
	table[6] = 0.50f;
	table[7] = 0.50f;
	struct motor_electrical_ripple_ff_config cfg = base_cfg();
	float32_t mean = 0.0f;
	float32_t removed = 0.0f;

	zassert_equal(motor_electrical_ripple_ff_mean(&cfg, &mean), 0, NULL);
	zassert_within(mean, 0.25f, 1e-6f, NULL);
	zassert_equal(motor_electrical_ripple_ff_remove_mean(&cfg, &removed), 0, NULL);
	zassert_within(removed, 0.25f, 1e-6f, NULL);
	zassert_equal(motor_electrical_ripple_ff_mean(&cfg, &mean), 0, NULL);
	zassert_within(mean, 0.0f, 1e-6f, NULL);
}

ZTEST(motor_electrical_ripple_ff, test_invalid_config_rejected)
{
	struct motor_electrical_ripple_ff_config cfg = base_cfg();
	struct motor_electrical_ripple_ff_state state;
	float32_t iq = 0.0f;

	cfg.table_len = 1U;
	zassert_equal(motor_electrical_ripple_ff_init(&cfg, &state), -EINVAL, NULL);
	zassert_equal(motor_electrical_ripple_ff_step_fast(&cfg, &state, 0.0f, &iq), -EINVAL, NULL);
	zassert_equal(motor_electrical_ripple_ff_mean(&cfg, &iq), -EINVAL, NULL);
	zassert_equal(motor_electrical_ripple_ff_remove_mean(&cfg, &iq), -EINVAL, NULL);
}

ZTEST_SUITE(motor_electrical_ripple_ff, NULL, NULL, NULL, NULL, NULL);
