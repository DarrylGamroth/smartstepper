/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CHOPPER_MAP_H_
#define MOTOR_CHOPPER_MAP_H_

#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_chopper_cal_ctx;

int motor_chopper_map_derive_centers(struct motor_chopper_cal_ctx *cal);
int motor_chopper_map_kind_at_angle(struct motor_chopper_cal_ctx *cal,
				    float32_t angle_rad,
				    uint8_t *kind_out);

#endif /* MOTOR_CHOPPER_MAP_H_ */
