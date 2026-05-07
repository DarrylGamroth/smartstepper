/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_SPEED_LIMIT_H_
#define MOTOR_CONTROL_SPEED_LIMIT_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "motor/math/math_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

struct motor_voltage_speed_limit_input {
	float vbus_v;
	float max_modulation_index;
	float resistance_ohm;
	float inductance_h;
	float flux_linkage_wb;
	float current_limit_a;
	uint16_t pole_pairs;
	float safety_factor;
};

struct motor_voltage_speed_limit_result {
	float max_mech_hz;
	float voltage_limit_v;
	float bemf_at_limit_v;
	bool valid;
};

static inline int motor_voltage_speed_limit_compute(
	const struct motor_voltage_speed_limit_input *in,
	struct motor_voltage_speed_limit_result *out)
{
	if (in == NULL || out == NULL) {
		return -1;
	}

	*out = (struct motor_voltage_speed_limit_result){0};

	if (!isfinite(in->vbus_v) || !isfinite(in->max_modulation_index) ||
	    !isfinite(in->resistance_ohm) || !isfinite(in->inductance_h) ||
	    !isfinite(in->flux_linkage_wb) || !isfinite(in->current_limit_a) ||
	    !isfinite(in->safety_factor) || in->vbus_v <= 0.0f ||
	    in->max_modulation_index <= 0.0f || in->max_modulation_index > 1.0f ||
	    in->resistance_ohm < 0.0f || in->inductance_h < 0.0f ||
	    in->flux_linkage_wb <= 0.0f || in->current_limit_a < 0.0f ||
	    in->pole_pairs == 0U || in->safety_factor <= 0.0f ||
	    in->safety_factor > 1.0f) {
		return -1;
	}

	const float voltage_limit_v =
		in->vbus_v * in->max_modulation_index * in->safety_factor;
	const float current_a = fabsf(in->current_limit_a);
	const float rs_i_v = in->resistance_ohm * current_a;

	if (voltage_limit_v <= rs_i_v) {
		out->voltage_limit_v = voltage_limit_v;
		out->valid = true;
		return 0;
	}

	/* Solve:
	 *   (omega_e * L * I)^2 + (R*I + omega_e*psi_f)^2 <= Vlimit^2
	 * for the positive electrical speed root.
	 */
	const float psi = in->flux_linkage_wb;
	const float li = in->inductance_h * current_a;
	const float a = (psi * psi) + (li * li);
	const float b = 2.0f * rs_i_v * psi;
	const float c = (rs_i_v * rs_i_v) - (voltage_limit_v * voltage_limit_v);
	const float disc = (b * b) - (4.0f * a * c);

	if (a <= 0.0f || disc < 0.0f || !isfinite(disc)) {
		return -1;
	}

	const float omega_e_rad_s = (-b + sqrtf(disc)) / (2.0f * a);
	if (!isfinite(omega_e_rad_s) || omega_e_rad_s <= 0.0f) {
		return -1;
	}

	out->voltage_limit_v = voltage_limit_v;
	out->bemf_at_limit_v = omega_e_rad_s * psi;
	out->max_mech_hz = omega_e_rad_s / (2.0f * PI_F32 * (float)in->pole_pairs);
	out->valid = isfinite(out->max_mech_hz) && out->max_mech_hz > 0.0f;
	return out->valid ? 0 : -1;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_SPEED_LIMIT_H_ */
