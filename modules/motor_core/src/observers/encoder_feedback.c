/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/observers/encoder_feedback.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "motor/runtime/io.h"
#include "motor/observers/angle_observer.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"
#include "motor/observers/angle_path.h"
#include "motor/observers/encoder_feedback_core.h"
#include "motor/observers/feedback_quality.h"

#define MOTOR_FEEDBACK_STALE_THRESHOLD_SAMPLES 4U

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

	struct motor_encoder_feedback_core_state core_state = {
		.fault_counter = params->encoder_fault_counter,
		.warning_count = params->encoder_warning_count,
		.error_count = params->encoder_error_count,
		.sample_fresh = params->live.encoder_sample_fresh,
		.sample_warning = params->live.encoder_sample_warning,
		.sample_error = params->live.encoder_sample_error,
		.last_status = params->live.encoder_last_status,
	};

	float32_t encoder_direction_sign =
		(params->encoder_direction_sign >= 0) ? 1.0f : -1.0f;

	bool raw_sample_present = (encoder_sample != NULL);
	bool raw_sample_enabled = raw_sample_present ? encoder_sample->enabled : false;
	bool raw_fresh = raw_sample_present ? encoder_sample->fresh : false;
	bool raw_warning = raw_sample_present ? encoder_sample->warning : false;
	bool raw_error = raw_sample_present ? encoder_sample->error : false;
	bool raw_io_fault = raw_sample_present ? encoder_sample->io_fault : false;
	uint8_t raw_status = raw_sample_present ? encoder_sample->status : 0U;
	float32_t raw_angle_deg = raw_sample_present ? encoder_sample->angle_deg : 0.0f;
	bool sample_available = raw_sample_present && raw_sample_enabled;
	feedback->sample_enabled = raw_sample_enabled;
	feedback->sample_available = sample_available;
	feedback->fresh = raw_fresh;
	feedback->warning = raw_warning;
	feedback->error = raw_error;
	feedback->io_fault = raw_io_fault;
	feedback->status = raw_status;
	feedback->angle_sensor_deg = raw_angle_deg;
	feedback->angle_control_deg = raw_angle_deg * encoder_direction_sign;

	if (feedback->sample_available && (feedback->fresh || feedback->warning || feedback->error)) {
		params->live.encoder_last_status = feedback->status;
	}

	struct motor_encoder_feedback_core_input core_in = {
		.feature_angle_gen = feature_angle_gen,
		.sample_enabled = (encoder_sample != NULL) ? encoder_sample->enabled : false,
		.sample_available = sample_available,
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
	params->live.encoder_sample_fresh = core_state.sample_fresh;
	params->live.encoder_sample_warning = core_state.sample_warning;
	params->live.encoder_sample_error = core_state.sample_error;
	params->live.encoder_last_status = core_state.last_status;

	if (threshold_exceeded) {
		return -EIO;
	}

	float32_t generated_angle_rad = angle_gen_get_angle(&params->angle_gen);
	struct motor_angle_path_input path_in = {
		.feature_angle_gen = feature_angle_gen,
		.sample_enabled = raw_sample_enabled,
		.sample_fresh = raw_fresh,
		.sample_error = raw_error,
		.sample_io_fault = raw_io_fault,
		.sample_angle_deg = raw_angle_deg,
		.encoder_direction_sign = encoder_direction_sign,
		.generated_mech_rad = generated_angle_rad,
		.encoder_delay_samples = ENCODER_SPI_PIPELINE_DELAY_SAMPLES,
	};
	struct motor_angle_path_output path_out = {0};
	int path_ret = motor_angle_path_step(&params->observer, &path_in, &path_out);
	if (path_ret != 0) {
		return path_ret;
	}

	feedback->input_source = path_out.control.input_source;
	params->live.encoder_observer_input_rad = path_out.observer_input_rad;
	params->live.encoder_input_source = feedback->input_source;
	feedback->observer_input_rad = path_out.observer_input_rad;
	feedback->observer_mech_rad = path_out.observer_mech_rad;
	feedback->observer_elec_rad = path_out.observer_elec_rad;
	feedback->control = path_out.control;

	if (feedback->input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER) {
		params->live.encoder_raw_deg = feedback->angle_sensor_deg;
		params->live.encoder_raw_rad = feedback->angle_sensor_deg * (PI_F32 / 180.0f);
	}

	uint8_t quality_flags = feedback->control.quality_flags;
	bool sample_fresh = (quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) != 0U;

	if (sample_fresh) {
		params->live.position_stale_count = 0U;
	} else if (params->live.position_stale_count < UINT16_MAX) {
		params->live.position_stale_count++;
		if (params->live.position_stale_count == MOTOR_FEEDBACK_STALE_THRESHOLD_SAMPLES) {
			params->live.position_stale_events++;
		}
	}

	params->live.position_quality_flags = quality_flags;
	params->live.position_glitch_count = 0U;
	params->live.position_jitter_count = 0U;

	feedback->position_mech_rad = feedback->control.position_mech_rad;
	feedback->speed_mech_rad_s = feedback->control.speed_mech_rad_s;
	feedback->accel_mech_rad_s2 = feedback->control.accel_mech_rad_s2;
	feedback->speed_mech_filtered_rad_s = feedback->control.speed_mech_filtered_rad_s;

	return 0;
}

int motor_encoder_feedback_prepare_capture(const struct motor_parameters *params,
					   const struct motor_encoder_feedback *feedback,
					   struct motor_capture_feedback *capture)
{
	if (params == NULL || feedback == NULL || capture == NULL) {
		return -EINVAL;
	}

	memset(capture, 0, sizeof(*capture));

	float32_t generated_mech_rad = angle_gen_get_angle(&params->angle_gen);
	float32_t observer_mech_offset_rad = params->observer.mech_angle_offset_rad;

	capture->angle_rad = feedback->sample_available ?
				     (feedback->angle_control_deg * (PI_F32 / 180.0f)) :
				     feedback->observer_input_rad;
	capture->angle_deg = feedback->sample_available ?
				     feedback->angle_control_deg :
				     (feedback->observer_input_rad * (180.0f / PI_F32));
	capture->observer_mech_rad = feedback->observer_mech_rad;
	capture->observer_elec_rad = feedback->observer_elec_rad;
	capture->generated_mech_rad = wrap_rad_2pi(generated_mech_rad);
	capture->generated_elec_rad =
		wrap_rad_2pi((capture->generated_mech_rad + observer_mech_offset_rad) *
			     (float32_t)MOTOR_POLE_PAIRS);
	capture->input_source = feedback->sample_available ?
				MOTOR_ANGLE_INPUT_SRC_ENCODER :
				feedback->input_source;

	if (feedback->sample_available && feedback->fresh &&
	    !feedback->warning && !feedback->error) {
		capture->encoder_mech_rad = capture->observer_mech_rad;
		capture->encoder_elec_rad = capture->observer_elec_rad;
		capture->mech_error_rad =
			wrap_rad_pi(capture->encoder_mech_rad - capture->generated_mech_rad);
		capture->elec_error_rad =
			wrap_rad_pi(capture->encoder_elec_rad - capture->generated_elec_rad);
		capture->compare_valid = true;
	}

	capture->sample_enabled = feedback->sample_available;
	capture->sample_fresh = feedback->fresh;
	capture->sample_warning = feedback->warning;
	capture->sample_error = feedback->error;
	capture->status = feedback->status;

	return 0;
}
