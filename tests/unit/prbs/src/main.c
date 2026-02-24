/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/math/prbs.h"

ZTEST(prbs, test_init_defaults)
{
	struct prbs_gen p = {0};
	prbs_init(&p);

	zassert_equal(p.m, 12u, NULL);
	zassert_equal(p.g, PRBS_GENPOLY_M12, NULL);
	zassert_equal(p.a, 1u, NULL);
	zassert_equal(p.state, 1u, NULL);
	zassert_equal(p.n, 4095u, NULL);
}

ZTEST(prbs, test_sequence_bits_and_period)
{
	struct prbs_gen p = {0};
	prbs_init(&p);
	const uint32_t initial_state = p.state;
	uint32_t ones = 0u;
	uint32_t zeros = 0u;

	for (uint32_t i = 0u; i < p.n; i++) {
		uint32_t b = prbs_advance(&p);
		zassert_true(b <= 1u, NULL);
		if (b == 0u) {
			zeros++;
		} else {
			ones++;
		}
		zassert_not_equal(p.state, 0u, NULL);
	}

	zassert_equal(p.state, initial_state, NULL);
	zassert_equal(ones + zeros, p.n, NULL);
	zassert_true(ones > 1500u, NULL);
	zassert_true(zeros > 1500u, NULL);
}

ZTEST(prbs, test_reset_restores_seed)
{
	struct prbs_gen p = {0};
	prbs_init(&p);
	(void)prbs_advance(&p);
	(void)prbs_advance(&p);
	zassert_not_equal(p.state, p.a, NULL);

	prbs_reset(&p);
	zassert_equal(p.state, p.a, NULL);
	zassert_equal(prbs_get_output(&p), (p.state & 1u), NULL);
}

ZTEST_SUITE(prbs, NULL, NULL, NULL, NULL, NULL);
