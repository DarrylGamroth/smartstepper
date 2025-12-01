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
 * @brief Standard SPWM (Sinusoidal PWM) for 2-phase system
 *
 * Maps voltage commands directly to duty cycles with 50% offset.
 * - Va_normalized = 0.5 + 0.5*Va
 * - Vb_normalized = 0.5 + 0.5*Vb
 *
 * Linear range: ±1.0 (±100% modulation index)
 * DC bus utilization: 50% (Vdc/2 max line-to-neutral)
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
 * @brief Space Vector PWM (SVPWM) for 2-phase system
 *
 * Optimal PWM strategy for 2-phase systems. Adds common-mode offset to maximize
 * DC bus utilization while maintaining sinusoidal line-to-line voltages.
 *
 * Algorithm:
 * 1. Find Vmin = min(Va, Vb) and Vmax = max(Va, Vb)
 * 2. Common-mode offset = -(Vmax + Vmin) / 2
 * 3. Add offset to center the waveform
 *
 * Linear range: ±1.155 (±115.5% modulation index)
 * DC bus utilization: 57.7% (1/√3 improvement over SPWM)
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
#if 0
	vmin = MIN(va, vb);
	vmax = MAX(va, vb);
#else
	vmin = fminf(va, vb);
	vmax = fmaxf(va, vb);
#endif

	/* Common-mode offset = -(Vmax + Vmin) / 2
	 * This centers the PWM waveform for maximum DC bus utilization */
	offset = -(vmax + vmin) * 0.5f;

	/* Apply offset and convert to [0, +1] range */
	*duty_a = 0.5f + 0.5f * (va + offset);
	*duty_b = 0.5f + 0.5f * (vb + offset);
}

/**
 * @brief Discontinuous PWM (DPWM) for 2-phase system
 *
 * Reduces switching losses by clamping one phase to rail for 60° sectors.
 * Lower switching losses than SVPWM but slightly higher harmonics.
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
#if 0
	vmin = MIN(va, vb);
	vmax = MAX(va, vb);
#else
	vmin = fminf(va, vb);
	vmax = fmaxf(va, vb);
#endif

	/* DPWM offset clamps the max or min phase to rail
	 * Offset = -Vmax (clamp max to top rail) or -Vmin (clamp min to bottom rail)
	 * Here we use -Vmax for DPWM1 strategy */
	offset = -vmax;

	/* Apply offset and convert to [0, +1] range */
	*duty_a = 0.5f + 0.5f * (va + offset);
	*duty_b = 0.5f + 0.5f * (vb + offset);
}

/**
 * @brief Saddle PWM for 2-phase system
 *
 * Hybrid between SVPWM and DPWM. Provides a balance of switching losses
 * and harmonic performance.
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
#if 0
	vmin = MIN(va, vb);
	vmax = MAX(va, vb);
#else
	vmin = fminf(va, vb);
	vmax = fmaxf(va, vb);
#endif

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
