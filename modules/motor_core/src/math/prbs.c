/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 * All rights reserved.
 */

/**
 * @file prbs.c
 * @brief Pseudo-Random Binary Sequence (PRBS) Generator Implementation
 *
 * Generates maximal-length binary sequences with excellent autocorrelation
 * properties for parameter estimation in motor control applications.
 */

#include <zephyr/toolchain.h>
#include "motor/math/prbs.h"

void prbs_init(struct prbs_gen *prbs)
{
	prbs->m = 12u;
	prbs->g = PRBS_GENPOLY_M12;
	prbs->a = 1u;
	prbs->state = prbs->a;
	prbs->n = (1u << prbs->m) - 1u;
}

uint32_t prbs_advance(struct prbs_gen *prbs)
{
	uint32_t b = POPCOUNT(prbs->state & prbs->g) & 1u;

	prbs->state <<= 1u;
	prbs->state |= b;
	prbs->state &= prbs->n;

	return b;
}
