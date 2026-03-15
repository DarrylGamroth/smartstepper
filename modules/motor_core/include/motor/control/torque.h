/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CONTROL_TORQUE_H_
#define MOTOR_CONTROL_TORQUE_H_

#include <math.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

static inline float32_t motor_torque_gain_from_flux_pole_pairs(float32_t flux_linkage_wb,
							       uint16_t pole_pairs)
{
	return 1.5f * (float32_t)pole_pairs * flux_linkage_wb;
}

static inline float32_t motor_torque_gain_resolve(float32_t active_gain_nm_per_a,
						  float32_t active_flux_linkage_wb,
						  float32_t default_flux_linkage_wb,
						  uint16_t pole_pairs)
{
	float32_t kt = active_gain_nm_per_a;

	if (isfinite(kt) && kt > 0.0f) {
		return kt;
	}

	kt = motor_torque_gain_from_flux_pole_pairs(active_flux_linkage_wb, pole_pairs);
	if (isfinite(kt) && kt > 0.0f) {
		return kt;
	}

	kt = motor_torque_gain_from_flux_pole_pairs(default_flux_linkage_wb, pole_pairs);
	if (isfinite(kt) && kt > 0.0f) {
		return kt;
	}

	return 0.0f;
}

#endif /* MOTOR_CONTROL_TORQUE_H_ */
