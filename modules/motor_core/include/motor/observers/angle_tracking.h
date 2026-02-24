/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_ANGLE_TRACKING_H_
#define MOTOR_ANGLE_TRACKING_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

#include "motor/observers/angle_observer.h"
#include "motor/observers/motor_encoder_feedback_core.h"

struct motor_angle_tracking_result {
	float32_t observer_input_rad;
	float32_t observer_mech_rad;
	float32_t observer_elec_rad;
	uint8_t input_source;
};

void motor_angle_tracking_update(struct angle_observer_state *observer,
				 float32_t angle_raw_rad,
				 uint8_t source,
				 bool fresh,
				 bool warning,
				 bool error,
				 bool measurement_locked,
				 float32_t encoder_delay_samples,
				 struct motor_angle_tracking_result *result);

#endif /* MOTOR_ANGLE_TRACKING_H_ */
