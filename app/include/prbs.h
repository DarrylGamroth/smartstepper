/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PRBS_H_
#define PRBS_H_

#include <stdint.h>

/**
 * @file prbs.h
 * @brief Pseudo-Random Binary Sequence (PRBS) generator for RLS excitation
 *
 * Generates a maximal-length 12-bit LFSR sequence with 4095-sample period.
 * Based on liquid-dsp msequence.c implementation.
 *
 * Returns pure binary sequence (0 or 1)
 */

/**
 * @brief PRBS generator state structure
 *
 * Generates [0,1] sequence with 4095-sample period using 12-bit LFSR.
 * Pure binary sequence generator - voltage conversion done externally.
 */
struct prbs_gen {
	uint32_t m;              /**< Shift register length (12) */
	uint32_t g;              /**< Generator polynomial (0x0e08) */
	uint32_t a;              /**< Initial state (1) */
	uint32_t state;          /**< Current shift register state */
	uint32_t n;              /**< Sequence length: 4095 */
};

#define PRBS_GENPOLY_M10  0x00000240u
#define PRBS_GENPOLY_M12  0x00000e08u

/**
 * @brief Initialize PRBS generator
 *
 * Sets up the generator for a 4095-sample sequence.
 *
 * @param prbs Pointer to PRBS generator state
 */
void prbs_init(struct prbs_gen *prbs);

/**
 * @brief Advance PRBS state and return output bit
 *
 * @param prbs Pointer to PRBS generator state
 * @return Output bit (0 or 1)
 */
uint32_t prbs_advance(struct prbs_gen *prbs);

/**
 * @brief Get current output without advancing state
 *
 * @param prbs Pointer to PRBS generator state
 * @return Current output bit (0 or 1)
 */
static inline uint32_t prbs_get_output(const struct prbs_gen *prbs)
{
	/* Return current output (0 or 1) based on LSB without advancing */
	return prbs->state & 1u;
}

/**
 * @brief Reset PRBS generator to initial state
 *
 * @param prbs Pointer to PRBS generator state
 */
static inline void prbs_reset(struct prbs_gen *prbs)
{
	prbs->state = prbs->a;
}

#endif /* PRBS_H_ */
