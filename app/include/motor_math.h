/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Motor control fixed-point math helpers
 *
 * Optimized Q31 fixed-point operations for FOC motor control.
 * All functions are static inline for zero ISR overhead.
 */

#ifndef MOTOR_MATH_H_
#define MOTOR_MATH_H_

#include <zephyr/dsp/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Lookup table for initial reciprocal approximation (1/x)
 * Covers range [0.5, 1.0) in Q31, indexed by top 5 bits after normalization
 * Values are 1/x in Q30 format for Newton-Raphson starting point */
static const q31_t recip_initial_lut_q31[32] = {
	0x7FFFFFFF, 0x7D6F4406, 0x7AF4E38E, 0x788778E0,
	0x7626F59C, 0x73D26565, 0x71897E4F, 0x6F4BC247,
	0x6D189B95, 0x6AEF0B4F, 0x68CED9EF, 0x66B7ED44,
	0x64AA2E68, 0x62A58A3E, 0x60A9F9F3, 0x5EB66E0E,
	0x5CCAD541, 0x5AE71F5A, 0x590B3B89, 0x57370F37,
	0x556A8ADE, 0x53A59961, 0x51E826CD, 0x50321D67,
	0x4E837DB0, 0x4CDD3D94, 0x4B3E4F5E, 0x49A6A508,
	0x48162F8B, 0x468CE489, 0x450AB5FE, 0x438F9915
};

/**
 * @brief Compute reciprocal of bus voltage for normalization
 *
 * Calculates inv_vbus = (2^62) / vbus for use in voltage normalization.
 * Uses lookup table + Newton-Raphson approximation for fast computation.
 *
 * Algorithm (similar to CMSIS-DSP sqrt):
 * 1. Normalize input using CLZ
 * 2. Initial estimate from lookup table
 * 3. Two Newton-Raphson iterations: x_new = x * (2 - vbus * x)
 *
 * @param vbus Bus voltage in Q31 format [0, +1] (ADC reading)
 * @return Reciprocal of bus voltage (2^62 / vbus), or 0 if vbus is too small
 *
 * @note Returns 0 if vbus is too small (< 1% of full scale) to avoid division by zero
 */
static inline int64_t motor_compute_inv_vbus_q31(q31_t vbus)
{
	/* Protect against division by very small vbus (< 1% of full scale) */
	if (vbus < 0x020C49BA) {  /* ~1% of 2^31 */
		return 0;
	}

#if defined(__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1)
	q31_t number, var1, temp;
	int32_t signBits1;

	number = vbus;

	/* Compute the number of sign bits (leading zeros) */
	signBits1 = __CLZ(number) - 1;

	/* Normalize the input: shift left to get number in range [0.5, 1.0) in Q31 */
	number = number << signBits1;

	/* Get initial approximation from lookup table
	 * After normalization, number is in [0.5, 1.0) which means bit 30 is set
	 * Top 5 bits (26-30) give us index 16-31 when shifted right by 26
	 * We subtract 16 to get table index 0-15, but need full 0-31 range */
	var1 = recip_initial_lut_q31[((uint32_t)number >> 26) & 0x1F];

	/* Newton-Raphson iterations: x_new = x * (2 - number * x)
	 * Formula: x = x * (2 - number * x)
	 * In Q30: x = (x * (0x80000000 - ((number * x) >> 31))) >> 30
	 *
	 * First iteration */
	temp = ((q63_t)number * var1) >> 31;  /* number * x in Q31 */
	temp = 0x80000000 - temp;              /* 2.0 - number * x in Q31 */
	var1 = ((q63_t)var1 * temp) >> 30;     /* x * (2 - number * x) in Q30 */

	/* Second iteration for better accuracy */
	temp = ((q63_t)number * var1) >> 31;
	temp = 0x80000000 - temp;
	var1 = ((q63_t)var1 * temp) >> 30;

	/* Third iteration (optional, for maximum accuracy) */
	temp = ((q63_t)number * var1) >> 31;
	temp = 0x80000000 - temp;
	var1 = ((q63_t)var1 * temp) >> 30;

	/* Adjust result for original scaling
	 * We shifted input left by signBits1, so reciprocal needs to shift right
	 * Result is in Q30, we want Q62 (32 bits higher), so shift by (32 - signBits1) */
	return ((int64_t)var1 << (32 - signBits1));
#else
	/* Fallback to direct division for non-ARM platforms */
	return ((int64_t)1 << 62) / (int64_t)vbus;
#endif
}

/**
 * @brief Normalize voltage command to available bus voltage
 *
 * Calculates duty_cycle = voltage_cmd / vbus in Q31 format using pre-computed reciprocal.
 * 
 * Example:
 * - inv_vbus = reciprocal of 0x40000000 (pre-computed)
 * - voltage_cmd = 0x20000000 (25% = 12V)
 * - Result: 0x40000000 (50% duty cycle)
 *
 * Uses ARM SMMULR instruction when available for optimal performance.
 *
 * @param voltage_cmd Voltage command in Q31 format [-1, +1]
 * @param inv_vbus Pre-computed reciprocal of bus voltage (2^62 / vbus)
 * @return Normalized duty cycle in Q31 format [-1, +1]
 */
static inline q31_t motor_normalize_voltage_q31(q31_t voltage_cmd, int64_t inv_vbus)
{
#if defined(__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1)
	/* Use ARM SMMULR instruction for Q31 multiply with rounding
	 * Note: inv_vbus is 64-bit but we only use lower 32 bits for SMMULR */
	q31_t result;
	uint32_t temp = (uint32_t)inv_vbus;
	__asm__("smmulr %0, %1, %2" : "=r"(result) : "r"(voltage_cmd), "r"(temp));
	return result;
#else
	/* Fallback to 64-bit arithmetic: multiply by reciprocal and truncate */
	return (q31_t)(((int64_t)voltage_cmd * inv_vbus) >> 31);
#endif
}

/**
 * @brief Normalize voltage command to available bus voltage (2-phase)
 *
 * Optimized version that normalizes two voltages using pre-computed reciprocal.
 * Useful for alpha-beta or two-phase motor control.
 *
 * Uses ARM SMMULR instruction when available for optimal performance.
 *
 * @param voltage_a First voltage command in Q31 format
 * @param voltage_b Second voltage command in Q31 format
 * @param inv_vbus Pre-computed reciprocal of bus voltage (2^62 / vbus)
 * @param[out] duty_a Normalized duty cycle A
 * @param[out] duty_b Normalized duty cycle B
 */
static inline void motor_normalize_voltage_2phase_q31(q31_t voltage_a, q31_t voltage_b,
						      int64_t inv_vbus, q31_t *duty_a, q31_t *duty_b)
{
#if defined(__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1)
	/* Use ARM SMMULR instruction for Q31 multiply with rounding
	 * Note: inv_vbus is 64-bit but we only use lower 32 bits for SMMULR */
	uint32_t inv_vbus_32 = (uint32_t)inv_vbus;
	__asm__("smmulr %0, %1, %2" : "=r"(*duty_a) : "r"(voltage_a), "r"(inv_vbus_32));
	__asm__("smmulr %0, %1, %2" : "=r"(*duty_b) : "r"(voltage_b), "r"(inv_vbus_32));
#else
	/* Fallback to 64-bit arithmetic: multiply by reciprocal and truncate */
	*duty_a = (q31_t)(((int64_t)voltage_a * inv_vbus) >> 31);
	*duty_b = (q31_t)(((int64_t)voltage_b * inv_vbus) >> 31);
#endif
}

/**
 * @brief Clamp Q31 value to range
 *
 * Efficiently clamps a Q31 value to specified min/max range.
 *
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
static inline q31_t motor_clamp_q31(q31_t value, q31_t min, q31_t max)
{
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

/**
 * @brief Saturate Q31 value to [-1, +1] range
 *
 * @param value Value to saturate
 * @return Saturated value in range [INT32_MIN, INT32_MAX]
 */
static inline q31_t motor_saturate_q31(q31_t value)
{
	/* Q31 already represents [-1, +1] range, so this is a no-op
	 * unless we're working with intermediate calculations that could overflow */
	return value;
}

/**
 * @brief Convert bus voltage ADC reading to actual voltage
 *
 * Scales ADC reading by voltage divider ratio to get actual bus voltage.
 * Result is in Q31 format where 1.0 represents the maximum measurable voltage.
 *
 * Uses ARM SMMULR instruction when available for optimal performance.
 *
 * @param adc_reading ADC value in Q31 format [0, +1]
 * @param voltage_scale Voltage divider scaling factor in Q31 format
 * @return Actual bus voltage in Q31 format
 */
static inline q31_t motor_scale_vbus_q31(q31_t adc_reading, q31_t voltage_scale)
{
#if defined(__ARM_FEATURE_DSP) && (__ARM_FEATURE_DSP == 1)
	/* Use ARM SMMULR instruction for Q31 multiply with rounding */
	q31_t result;
	__asm__("smmulr %0, %1, %2" : "=r"(result) : "r"(adc_reading), "r"(voltage_scale));
	return result;
#else
	/* Fallback to 64-bit arithmetic: Q31 multiply with rounding
	 * SMMULR performs: ((a * b) + 0x80000000) >> 32 */
	return (q31_t)(((int64_t)adc_reading * (int64_t)voltage_scale + 0x80000000LL) >> 32);
#endif
}

/**
 * @brief Dead-time compensation for voltage commands
 *
 * Compensates for inverter dead-time effects based on current direction.
 *
 * @param voltage Voltage command in Q31
 * @param current Phase current in Q31 (sign indicates direction)
 * @param dead_time_comp Dead-time compensation value in Q31
 * @return Compensated voltage command
 */
static inline q31_t motor_deadtime_compensate_q31(q31_t voltage, q31_t current,
						   q31_t dead_time_comp)
{
	/* Add or subtract compensation based on current direction */
	if (current >= 0) {
		return voltage + dead_time_comp;
	} else {
		return voltage - dead_time_comp;
	}
}

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
 * @param Va Alpha voltage in Q31 format [-1, +1]
 * @param Vb Beta voltage in Q31 format [-1, +1]
 * @param[out] duty_a Duty cycle for phase A [0, +1]
 * @param[out] duty_b Duty cycle for phase B [0, +1]
 */
static inline void motor_spwm_2phase_q31(q31_t Va, q31_t Vb, q31_t *duty_a, q31_t *duty_b)
{
	/* Scale from [-1, +1] to [0, +1] range: duty = 0.5 + 0.5*V
	 * In Q31: multiply by 0.5 (shift right 1) and add 0.5 (0x40000000) */
	*duty_a = (Va >> 1) + 0x40000000;
	*duty_b = (Vb >> 1) + 0x40000000;
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
 * @param Va Alpha voltage in Q31 format [-1, +1]
 * @param Vb Beta voltage in Q31 format [-1, +1]
 * @param[out] duty_a Duty cycle for phase A [0, +1]
 * @param[out] duty_b Duty cycle for phase B [0, +1]
 */
static inline void motor_svpwm_2phase_q31(q31_t Va, q31_t Vb, q31_t *duty_a, q31_t *duty_b)
{
	q31_t Vmin, Vmax, offset;

	/* Find min and max */
	if (Va < Vb) {
		Vmin = Va;
		Vmax = Vb;
	} else {
		Vmin = Vb;
		Vmax = Va;
	}

	/* Common-mode offset = -(Vmax + Vmin) / 2
	 * This centers the PWM waveform for maximum DC bus utilization */
	offset = -((Vmax + Vmin) >> 1);

	/* Apply offset and convert to [0, +1] range */
	*duty_a = ((Va + offset) >> 1) + 0x40000000;
	*duty_b = ((Vb + offset) >> 1) + 0x40000000;
}

/**
 * @brief Discontinuous PWM (DPWM) for 2-phase system
 *
 * Reduces switching losses by clamping one phase to rail for 60° sectors.
 * Lower switching losses than SVPWM but slightly higher harmonics.
 *
 * @param Va Alpha voltage in Q31 format [-1, +1]
 * @param Vb Beta voltage in Q31 format [-1, +1]
 * @param[out] duty_a Duty cycle for phase A [0, +1]
 * @param[out] duty_b Duty cycle for phase B [0, +1]
 */
static inline void motor_dpwm_2phase_q31(q31_t Va, q31_t Vb, q31_t *duty_a, q31_t *duty_b)
{
	q31_t Vmin, Vmax, offset;

	/* Find min and max */
	if (Va < Vb) {
		Vmin = Va;
		Vmax = Vb;
	} else {
		Vmin = Vb;
		Vmax = Va;
	}

	/* DPWM offset clamps the max or min phase to rail
	 * Offset = -Vmax (clamp max to top rail) or -Vmin (clamp min to bottom rail)
	 * Here we use -Vmax for DPWM1 strategy */
	offset = -Vmax;

	/* Apply offset and convert to [0, +1] range */
	*duty_a = ((Va + offset) >> 1) + 0x40000000;
	*duty_b = ((Vb + offset) >> 1) + 0x40000000;
}

/**
 * @brief Saddle PWM for 2-phase system
 *
 * Hybrid between SVPWM and DPWM. Provides a balance of switching losses
 * and harmonic performance.
 *
 * @param Va Alpha voltage in Q31 format [-1, +1]
 * @param Vb Beta voltage in Q31 format [-1, +1]
 * @param[out] duty_a Duty cycle for phase A [0, +1]
 * @param[out] duty_b Duty cycle for phase B [0, +1]
 */
static inline void motor_saddle_pwm_2phase_q31(q31_t Va, q31_t Vb, q31_t *duty_a, q31_t *duty_b)
{
	q31_t Vmin, Vmax, offset;

	/* Find min and max */
	if (Va < Vb) {
		Vmin = Va;
		Vmax = Vb;
	} else {
		Vmin = Vb;
		Vmax = Va;
	}

	/* Saddle offset = -Vmin - Vmax (average of DPWM strategies) */
	offset = -(Vmin + Vmax);

	/* Apply offset and convert to [0, +1] range */
	*duty_a = ((Va + offset) >> 1) + 0x40000000;
	*duty_b = ((Vb + offset) >> 1) + 0x40000000;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MATH_H_ */
