/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHOPPER_PWMGEN_H_
#define CHOPPER_PWMGEN_H_

#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Standard SPWM (Sinusoidal PWM) for 2-phase H-bridge system
 *
 * Maps voltage commands directly to duty cycles with 50% offset.
 * Each H-bridge uses complementary PWM: duty_hb2 = 1 - duty_hb1
 * - Va_normalized = 0.5 + 0.5*Va
 * - Vb_normalized = 0.5 + 0.5*Vb
 * - Differential voltage = Vbus × (2*duty - 1)
 *
 * Linear range: ±1.0 (full ±Vbus differential voltage)
 * DC bus utilization: 100% (full Vbus available per winding)
 *
 * @param va Alpha voltage [-1.0, +1.0]
 * @param vb Beta voltage [-1.0, +1.0]
 * @param duty_a Duty cycle for phase A [0.0, 1.0]
 * @param duty_b Duty cycle for phase B [0.0, 1.0]
 */
static inline void pwmgen_spwm_2phase_f32(float32_t va, float32_t vb,
					  float32_t *duty_a, float32_t *duty_b)
{
	/* Scale from [-1, +1] to [0, +1] range: duty = 0.5 + 0.5*V */
	*duty_a = 0.5f + 0.5f * va;
	*duty_b = 0.5f + 0.5f * vb;
}

/**
 * @brief Space Vector PWM (SVPWM) for 2-phase H-bridge system
 *
 * Adds common-mode offset to center PWM waveforms for improved harmonic performance.
 * For H-bridge topology, this reduces switching but doesn't increase voltage (already have full Vbus).
 *
 * Algorithm:
 * 1. Find Vmin = min(Va, Vb) and Vmax = max(Va, Vb)
 * 2. Common-mode offset = -(Vmax + Vmin) / 2
 * 3. Add offset to center the waveform
 *
 * Linear range: ±1.0 (full ±Vbus differential voltage)
 * DC bus utilization: 100% (H-bridge provides full Vbus per winding)
 * Benefit: Reduced harmonic distortion and common-mode noise vs SPWM
 *
 * @param va Alpha voltage [-1.0, +1.0]
 * @param vb Beta voltage [-1.0, +1.0]
 * @param duty_a Duty cycle for phase A [0.0, 1.0]
 * @param duty_b Duty cycle for phase B [0.0, 1.0]
 */
static inline void pwmgen_svpwm_2phase_f32(float32_t va, float32_t vb,
					   float32_t *duty_a, float32_t *duty_b)
{
	float32_t vmin, vmax, offset;

	/* Find min and max */
	vmin = fminf(va, vb);
	vmax = fmaxf(va, vb);

	/* Common-mode offset = -(Vmax + Vmin) / 2
	 * This centers the PWM waveform for maximum DC bus utilization */
	offset = -(vmax + vmin) * 0.5f;

	/* Apply offset and convert to [0, +1] range */
	*duty_a = 0.5f + 0.5f * (va + offset);
	*duty_b = 0.5f + 0.5f * (vb + offset);
}

/**
 * @brief Discontinuous PWM (DPWM) for 2-phase H-bridge system
 *
 * Reduces switching losses by clamping one phase duty cycle to rail.
 * Lower switching losses than SVPWM but slightly higher harmonics.
 * Best for high-speed operation where switching losses dominate.
 *
 * Linear range: ±1.0 (full ±Vbus differential voltage)
 *
 * @param va Alpha voltage [-1.0, +1.0]
 * @param vb Beta voltage [-1.0, +1.0]
 * @param duty_a Duty cycle for phase A [0.0, 1.0]
 * @param duty_b Duty cycle for phase B [0.0, 1.0]
 */
static inline void pwmgen_dpwm_2phase_f32(float32_t va, float32_t vb,
					  float32_t *duty_a, float32_t *duty_b)
{
	float32_t vmin, vmax, offset;

	/* Find min and max */
	vmin = fminf(va, vb);
	vmax = fmaxf(va, vb);

	/* DPWM offset clamps the max or min phase to rail
	 * Offset = -Vmax (clamp max to top rail) or -Vmin (clamp min to bottom rail)
	 * Here we use -Vmax for DPWM1 strategy */
	offset = -vmax;

	/* Apply offset and convert to [0, +1] range */
	*duty_a = 0.5f + 0.5f * (va + offset);
	*duty_b = 0.5f + 0.5f * (vb + offset);
}

/**
 * @brief Saddle PWM for 2-phase H-bridge system
 *
 * Hybrid between SVPWM and DPWM. Provides a balance of switching losses
 * and harmonic performance. Useful for medium to high speed operation.
 *
 * Linear range: ±1.0 (full ±Vbus differential voltage)
 *
 * @param va Alpha voltage [-1.0, +1.0]
 * @param vb Beta voltage [-1.0, +1.0]
 * @param duty_a Duty cycle for phase A [0.0, 1.0]
 * @param duty_b Duty cycle for phase B [0.0, 1.0]
 */
static inline void pwmgen_saddle_pwm_2phase_f32(float32_t va, float32_t vb,
						float32_t *duty_a, float32_t *duty_b)
{
	float32_t vmin, vmax, offset;

	/* Find min and max */
	vmin = fminf(va, vb);
	vmax = fmaxf(va, vb);

	/* Saddle offset = -Vmin - Vmax (average of DPWM strategies) */
	offset = -(vmin + vmax);

	/* Apply offset and convert to [0, +1] range */
	*duty_a = 0.5f + 0.5f * (va + offset);
	*duty_b = 0.5f + 0.5f * (vb + offset);
}

#ifdef __cplusplus
}
#endif

#endif /* CHOPPER_PWMGEN_H_ */
