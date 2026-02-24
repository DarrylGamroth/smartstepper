/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_encoder_feedback.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "motor_control_loop.h"
#include "config.h"
#include "motor/observers/angle_observer.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"
#include "motor/observers/motor_encoder_feedback_core.h"

int motor_encoder_feedback_update(struct motor_parameters *params,
				  const struct motor_control_encoder_sample *encoder_sample,
				  bool feature_angle_gen,
				  struct motor_encoder_feedback *feedback)
{
	if (params == NULL || feedback == NULL) {
		return -EINVAL;
	}

	memset(feedback, 0, sizeof(*feedback));
	feedback->input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;
	feedback->capture_input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;

	struct motor_encoder_feedback_core_state core_state = {
		.fault_counter = params->encoder_fault_counter,
		.warning_count = params->encoder_warning_count,
		.error_count = params->encoder_error_count,
		.sample_fresh = params->encoder_sample_fresh,
		.sample_warning = params->encoder_sample_warning,
		.sample_error = params->encoder_sample_error,
		.last_status = params->encoder_last_status,
	};

	float32_t encoder_direction_sign =
		(params->encoder_direction_sign >= 0) ? 1.0f : -1.0f;

	if (encoder_sample != NULL &&
	    (encoder_sample->enabled || params->encoder_capture_enabled)) {
		feedback->sample_enabled = encoder_sample->enabled;
		feedback->sample_available = true;
		feedback->fresh = encoder_sample->fresh;
		feedback->warning = encoder_sample->warning;
		feedback->error = encoder_sample->error;
		feedback->io_fault = encoder_sample->io_fault;
		feedback->status = encoder_sample->status;
		feedback->angle_sensor_deg = encoder_sample->angle_deg;
		feedback->angle_control_deg = feedback->angle_sensor_deg * encoder_direction_sign;

		if (feedback->fresh || feedback->warning || feedback->error) {
			params->encoder_last_status = feedback->status;
		}
	}

	struct motor_encoder_feedback_core_input core_in = {
		.feature_angle_gen = feature_angle_gen,
		.sample_enabled = (encoder_sample != NULL) ? encoder_sample->enabled : false,
		.sample_available = feedback->sample_available,
		.fresh = feedback->fresh,
		.warning = feedback->warning,
		.error = feedback->error,
		.io_fault = feedback->io_fault,
		.status = feedback->status,
		.fault_threshold = ENCODER_FAULT_THRESHOLD,
	};
	bool threshold_exceeded = motor_encoder_feedback_update_state(&core_in, &core_state);
	params->encoder_fault_counter = core_state.fault_counter;
	params->encoder_warning_count = core_state.warning_count;
	params->encoder_error_count = core_state.error_count;
	params->encoder_sample_fresh = core_state.sample_fresh;
	params->encoder_sample_warning = core_state.sample_warning;
	params->encoder_sample_error = core_state.sample_error;
	params->encoder_last_status = core_state.last_status;

	if (threshold_exceeded) {
		return -EIO;
	}

	float32_t angle_raw_rad = 0.0f;
	uint8_t source = motor_encoder_feedback_select_source(feature_angle_gen,
						      feedback->sample_enabled,
						      feedback->fresh);
	if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_GENERATED) {
		angle_raw_rad = angle_gen_get_angle(&params->angle_gen);
		angle_observer_set_delay(&params->observer, 0.0f);
		feedback->input_source = MOTOR_ANGLE_INPUT_SRC_GENERATED;
	} else if (source == MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER) {
		angle_raw_rad = feedback->angle_control_deg * (PI_F32 / 180.0f);
		angle_observer_set_delay(&params->observer, ENCODER_SPI_PIPELINE_DELAY_SAMPLES);
		feedback->input_source = MOTOR_ANGLE_INPUT_SRC_ENCODER;
		params->encoder_raw_deg = feedback->angle_sensor_deg;
		params->encoder_raw_rad = feedback->angle_sensor_deg * (PI_F32 / 180.0f);
	} else {
		angle_raw_rad = angle_observer_get_mech_angle(&params->observer);
		angle_observer_set_delay(&params->observer, 0.0f);
		feedback->input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;
	}

	if (feedback->input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER &&
	    feedback->fresh &&
	    !params->position_convert.measurement_locked) {
		float32_t handoff_angle_rad = wrap_rad_2pi(angle_raw_rad);

		/* Seed observer/position conversion on first fresh encoder sample after
		 * mode/reset handoff to avoid large residual speed spikes.
		 */
		angle_observer_reset_tracking(&params->observer, handoff_angle_rad, 0.0f);
		motor_position_convert_reset(&params->position_convert, handoff_angle_rad);
	}

	angle_observer_update(&params->observer, angle_raw_rad);
	params->encoder_observer_input_rad = angle_raw_rad;
	params->encoder_input_source = feedback->input_source;
	feedback->observer_input_rad = angle_raw_rad;
	feedback->observer_mech_rad = angle_observer_get_mech_angle(&params->observer);
	feedback->observer_elec_rad = angle_observer_get_elec_angle(&params->observer);

	feedback->capture_angle_rad = feedback->sample_available ?
					      (feedback->angle_control_deg * (PI_F32 / 180.0f)) :
					      angle_raw_rad;
	feedback->capture_angle_deg = feedback->sample_available ?
					      feedback->angle_control_deg :
					      (angle_raw_rad * (180.0f / PI_F32));
	feedback->capture_observer_mech_rad = feedback->observer_mech_rad;
	feedback->capture_observer_elec_rad = feedback->observer_elec_rad;
	feedback->capture_generated_mech_rad = wrap_rad_2pi(angle_gen_get_angle(&params->angle_gen));
	float32_t observer_mech_offset_rad = params->observer.mech_angle_offset_rad;
	feedback->capture_generated_elec_rad =
		wrap_rad_2pi((feedback->capture_generated_mech_rad + observer_mech_offset_rad) *
			     (float32_t)MOTOR_POLE_PAIRS);
	feedback->capture_input_source = feedback->sample_available ?
					 MOTOR_ANGLE_INPUT_SRC_ENCODER :
					 feedback->input_source;
	if (feedback->sample_available && feedback->fresh &&
	    !feedback->warning && !feedback->error) {
		feedback->capture_encoder_mech_rad = feedback->capture_observer_mech_rad;
		feedback->capture_encoder_elec_rad = feedback->capture_observer_elec_rad;
		feedback->capture_mech_error_rad =
			wrap_rad_pi(feedback->capture_encoder_mech_rad -
				    feedback->capture_generated_mech_rad);
		feedback->capture_elec_error_rad =
			wrap_rad_pi(feedback->capture_encoder_elec_rad -
				    feedback->capture_generated_elec_rad);
		feedback->capture_compare_valid = true;
	}

	struct motor_position_convert_input pos_input = {
		.sample_valid = false,
		.sample_fresh = false,
		.source_generated = false,
		.warning = feedback->warning,
		.error = feedback->error,
		.measurement_wrapped_rad = 0.0f,
		.latency_samples = 0.0f,
	};

	switch (feedback->input_source) {
	case MOTOR_ANGLE_INPUT_SRC_GENERATED:
		pos_input.sample_valid = true;
		pos_input.sample_fresh = true;
		pos_input.source_generated = true;
		pos_input.measurement_wrapped_rad = wrap_rad_2pi(angle_raw_rad);
		pos_input.latency_samples = 0.0f;
		break;
	case MOTOR_ANGLE_INPUT_SRC_ENCODER:
		pos_input.sample_valid = true;
		pos_input.sample_fresh = feedback->fresh;
		pos_input.source_generated = false;
		pos_input.measurement_wrapped_rad = wrap_rad_2pi(angle_raw_rad);
		pos_input.latency_samples = ENCODER_SPI_PIPELINE_DELAY_SAMPLES;
		break;
	case MOTOR_ANGLE_INPUT_SRC_PROPAGATED:
	default:
		pos_input.sample_valid = false;
		pos_input.sample_fresh = false;
		pos_input.source_generated = false;
		pos_input.measurement_wrapped_rad = 0.0f;
		pos_input.latency_samples = 0.0f;
		break;
	}

	int pos_ret = motor_position_convert_update(&params->position_convert,
						    &params->position_convert_cfg,
						    &pos_input);
	if (pos_ret != 0) {
		motor_position_convert_reset(&params->position_convert, wrap_rad_2pi(angle_raw_rad));
	}
	params->position_quality_flags = params->position_convert.quality_flags;
	params->position_stale_count = params->position_convert.stale_count;
	params->position_stale_events = params->position_convert.stale_event_count;
	params->position_glitch_count = params->position_convert.glitch_count;
	params->position_jitter_count = params->position_convert.jitter_count;

	feedback->position_mech_rad = params->position_convert.position_wrapped_rad;
	feedback->speed_mech_rad_s = params->position_convert.velocity_rad_s;
	feedback->accel_mech_rad_s2 = params->position_convert.accel_rad_s2;
	feedback->speed_mech_filtered_rad_s = feedback->speed_mech_rad_s;

	return 0;
}
