/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_TORQUE_H_
#define MOTOR_TORQUE_H_

#include <math.h>

#include <zephyr/dsp/types.h>

#include "config.h"

static inline float32_t motor_torque_gain_from_flux(float32_t flux_linkage_wb)
{
	return 1.5f * (float32_t)MOTOR_POLE_PAIRS * flux_linkage_wb;
}

static inline float32_t motor_torque_gain_default_nm_per_a(void)
{
	return motor_torque_gain_from_flux(MOTOR_FLUX_LINKAGE_WB);
}

static inline float32_t motor_torque_gain_resolve_active(const struct motor_parameters *params)
{
	if (params == NULL) {
		return 0.0f;
	}

	float32_t kt = params->torque_gain_nm_per_a_active;
	if (isfinite(kt) && kt > 0.0f) {
		return kt;
	}

	kt = motor_torque_gain_from_flux(params->flux_linkage_wb_active);
	if (isfinite(kt) && kt > 0.0f) {
		return kt;
	}

	kt = motor_torque_gain_default_nm_per_a();
	if (isfinite(kt) && kt > 0.0f) {
		return kt;
	}

	return 0.0f;
}

#endif /* MOTOR_TORQUE_H_ */
