/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_TORQUE_H_
#define MOTOR_TORQUE_H_

#include "config.h"
#include "motor/control/torque.h"

static inline float32_t motor_torque_gain_from_flux(float32_t flux_linkage_wb)
{
	return motor_torque_gain_from_flux_pole_pairs(flux_linkage_wb, MOTOR_POLE_PAIRS);
}

static inline float32_t motor_torque_gain_default_nm_per_a(void)
{
	return motor_torque_gain_from_flux_pole_pairs(MOTOR_FLUX_LINKAGE_WB, MOTOR_POLE_PAIRS);
}

static inline float32_t motor_torque_gain_resolve_active(const struct motor_parameters *params)
{
	if (params == NULL) {
		return 0.0f;
	}

	return motor_torque_gain_resolve(params->torque_gain_nm_per_a_active,
					 params->flux_linkage_wb_active,
					 MOTOR_FLUX_LINKAGE_WB,
					 MOTOR_POLE_PAIRS);
}

#endif /* MOTOR_TORQUE_H_ */
