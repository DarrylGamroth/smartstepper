/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_TRANSFORMS_H_
#define MOTOR_TRANSFORMS_H_

#include <zephyr/dsp/types.h>

int motor_transforms_park(float32_t ia_a,
			  float32_t ib_a,
			  float32_t elec_angle_rad,
			  float32_t *id_a,
			  float32_t *iq_a);

int motor_transforms_inv_park(float32_t vd_v,
			      float32_t vq_v,
			      float32_t elec_angle_rad,
			      float32_t *va_v,
			      float32_t *vb_v);

#endif /* MOTOR_TRANSFORMS_H_ */
