/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/observers/angle_tracking.h"

#include <stddef.h>
#include <string.h>

#include "motor/math/angle_wrap.h"

void motor_angle_tracking_update(struct angle_observer_state *observer,
				 float32_t angle_raw_rad,
				 uint8_t source,
				 bool fresh,
				 bool warning,
				 bool error,
				 bool measurement_locked,
				 float32_t encoder_delay_samples,
				 struct motor_angle_tracking_result *result)
{
	if (observer == NULL || result == NULL) {
		return;
	}

	memset(result, 0, sizeof(*result));

	if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER) {
		angle_observer_set_delay(observer, encoder_delay_samples);
		if (fresh && !warning && !error && !measurement_locked) {
			float32_t handoff_angle_rad = wrap_rad_2pi(angle_raw_rad);
			angle_observer_reset_tracking(observer, handoff_angle_rad, 0.0f);
		}
	} else if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED) {
		angle_observer_set_delay(observer, 0.0f);
	} else {
		angle_observer_set_delay(observer, 0.0f);
	}
	result->input_source = source;

	angle_observer_update(observer, angle_raw_rad);
	result->observer_input_rad = angle_raw_rad;
	result->observer_mech_rad = angle_observer_get_mech_angle(observer);
	result->observer_elec_rad = angle_observer_get_elec_angle(observer);
}
