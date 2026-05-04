/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/observers/angle_path.h"

#include <errno.h>
#include <string.h>

#include "motor/math/math_constants.h"

int motor_angle_path_step(struct angle_observer_state *observer,
			  const struct motor_angle_path_input *in,
			  struct motor_angle_path_output *out)
{
	if (observer == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	out->angle_sensor_deg = in->sample_angle_deg;
	out->angle_control_deg = out->angle_sensor_deg * in->encoder_direction_sign;

	uint8_t source = motor_encoder_feedback_select_source(in->feature_angle_gen,
							      in->sample_enabled,
							      in->sample_fresh,
							      in->sample_warning,
							      in->sample_error,
							      in->sample_io_fault);

	float32_t observer_input_rad = 0.0f;
	bool encoder_handoff = false;
	if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED) {
		angle_observer_set_delay(observer, 0.0f);
		observer_input_rad = in->generated_mech_rad;
	} else if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER) {
		angle_observer_set_delay(observer, in->encoder_delay_samples);
		observer_input_rad = out->angle_control_deg * (PI_F32 / 180.0f);
		encoder_handoff =
			in->previous_input_source != MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER;
	} else {
		angle_observer_set_delay(observer, 0.0f);
		observer_input_rad = angle_observer_get_mech_angle(observer);
	}

	if (encoder_handoff) {
		angle_observer_reset_tracking(observer, observer_input_rad, 0.0f);
	}
	angle_observer_update(observer, observer_input_rad);

	bool has_error = in->sample_error || in->sample_io_fault;
	bool control_fresh = (source == MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED) ||
			     ((source == MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER) &&
			      in->sample_fresh);

	uint8_t quality_flags = 0U;
	if (control_fresh) {
		quality_flags |= MOTOR_FEEDBACK_QUALITY_FRESH;
	}
	if (has_error) {
		quality_flags |= MOTOR_FEEDBACK_QUALITY_ERROR;
	}
	if (control_fresh && !has_error) {
		quality_flags |= MOTOR_FEEDBACK_QUALITY_VALID;
	}

	out->observer_input_rad = observer_input_rad;
	out->observer_mech_rad = angle_observer_get_mech_angle(observer);
	out->observer_elec_rad = angle_observer_get_elec_angle(observer);
	out->observer_elec_pred_rad = angle_observer_get_elec_angle_pred(observer);
	out->observer_elec_speed_rad_s = angle_observer_get_elec_speed(observer);

	out->control.position_mech_rad = out->observer_mech_rad;
	out->control.electrical_angle_rad = out->observer_elec_rad;
	out->control.predicted_electrical_angle_rad = out->observer_elec_pred_rad;
	out->control.electrical_speed_rad_s = out->observer_elec_speed_rad_s;
	out->control.speed_mech_rad_s = angle_observer_get_mech_speed(observer);
	out->control.accel_mech_rad_s2 = 0.0f;
	out->control.speed_mech_filtered_rad_s = out->control.speed_mech_rad_s;
	out->control.input_source = source;
	out->control.quality_flags = quality_flags;

	return 0;
}
