/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/observers/encoder_feedback.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "motor/runtime/io.h"
#include "motor/math/angle_wrap.h"
#include "motor/observers/angle_path.h"
#include "motor/observers/encoder_feedback_core.h"
#include "motor/observers/feedback_quality.h"

#define MOTOR_FEEDBACK_STALE_THRESHOLD_SAMPLES 4U

enum {
	MOTOR_ENCODER_INPUT_SRC_GENERATED = 0,
	MOTOR_ENCODER_INPUT_SRC_ENCODER = 1,
	MOTOR_ENCODER_INPUT_SRC_PROPAGATED = 2,
};

int motor_encoder_feedback_update(struct motor_encoder_feedback_ctx *ctx,
				  const struct motor_control_encoder_sample *encoder_sample,
				  bool feature_angle_gen,
				  struct motor_encoder_feedback *feedback)
{
	if (ctx == NULL || feedback == NULL) {
		return -EINVAL;
	}

	memset(feedback, 0, sizeof(*feedback));
	feedback->input_source = MOTOR_ENCODER_INPUT_SRC_PROPAGATED;

	struct motor_encoder_feedback_core_state core_state = {
		.fault_counter = *ctx->fault_counter,
		.warning_count = *ctx->warning_count,
		.error_count = *ctx->error_count,
		.sample_fresh = *ctx->sample_fresh,
		.sample_warning = *ctx->sample_warning,
		.sample_error = *ctx->sample_error,
		.last_status = *ctx->last_status,
	};

	float32_t encoder_direction_sign = (ctx->encoder_direction_sign >= 0) ? 1.0f : -1.0f;

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
		*ctx->last_status = feedback->status;
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
		.fault_threshold = ctx->encoder_fault_threshold,
	};
	bool threshold_exceeded = motor_encoder_feedback_update_state(&core_in, &core_state);
	*ctx->fault_counter = core_state.fault_counter;
	*ctx->warning_count = core_state.warning_count;
	*ctx->error_count = core_state.error_count;
	*ctx->sample_fresh = core_state.sample_fresh;
	*ctx->sample_warning = core_state.sample_warning;
	*ctx->sample_error = core_state.sample_error;
	*ctx->last_status = core_state.last_status;

	float32_t generated_angle_rad = angle_gen_get_angle(ctx->angle_gen);
	float32_t generated_mech_rad = wrap_rad_2pi(generated_angle_rad);
	float32_t generated_elec_rad =
		wrap_rad_2pi((generated_mech_rad + ctx->observer->mech_angle_offset_rad) *
			     (float32_t)ctx->pole_pairs);
	uint8_t previous_source = *ctx->encoder_input_source;
	bool propagated_valid =
		raw_sample_enabled && !threshold_exceeded &&
		(previous_source == MOTOR_ENCODER_FEEDBACK_SOURCE_ENCODER ||
		 previous_source == MOTOR_ENCODER_FEEDBACK_SOURCE_PROPAGATED);
	struct motor_angle_path_input path_in = {
		.feature_angle_gen = feature_angle_gen,
		.sample_enabled = raw_sample_enabled,
		.sample_fresh = raw_fresh,
		.sample_warning = raw_warning,
		.sample_error = raw_error,
		.sample_io_fault = raw_io_fault,
		.propagated_valid = propagated_valid,
		.previous_input_source = previous_source,
		.sample_angle_deg = raw_angle_deg,
		.encoder_direction_sign = encoder_direction_sign,
		.generated_mech_rad = generated_angle_rad,
		.encoder_delay_samples = ctx->encoder_delay_samples,
	};
	struct motor_angle_path_output path_out = {0};
	int path_ret = motor_angle_path_step(ctx->observer, &path_in, &path_out);
	if (path_ret != 0) {
		return path_ret;
	}

	feedback->input_source = path_out.control.input_source;
	*ctx->observer_input_rad = path_out.observer_input_rad;
	*ctx->encoder_input_source = feedback->input_source;
	feedback->generated_mech_rad = generated_mech_rad;
	feedback->generated_elec_rad = generated_elec_rad;
	feedback->observer_input_rad = path_out.observer_input_rad;
	feedback->observer_mech_rad = path_out.observer_mech_rad;
	feedback->observer_elec_rad = path_out.observer_elec_rad;
	feedback->observer_elec_pred_rad = path_out.observer_elec_pred_rad;
	feedback->observer_elec_speed_rad_s = path_out.observer_elec_speed_rad_s;
	feedback->control = path_out.control;
	feedback->trust_state = path_out.control.trust_state;

	if (feedback->input_source == MOTOR_ENCODER_INPUT_SRC_ENCODER) {
		*ctx->encoder_raw_deg = feedback->angle_sensor_deg;
		*ctx->encoder_raw_rad = feedback->angle_sensor_deg * (PI_F32 / 180.0f);
	}

	uint8_t quality_flags = feedback->control.quality_flags;
	bool sample_fresh = (quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) != 0U;

	if (sample_fresh) {
		*ctx->position_stale_count = 0U;
	} else if (raw_sample_enabled && *ctx->position_stale_count < UINT16_MAX) {
		(*ctx->position_stale_count)++;
		if (*ctx->position_stale_count == MOTOR_FEEDBACK_STALE_THRESHOLD_SAMPLES) {
			(*ctx->position_stale_events)++;
		}
	} else if (!raw_sample_enabled) {
		*ctx->position_stale_count = 0U;
	}

	*ctx->position_quality_flags = quality_flags;
	if (ctx->position_trust_state != NULL) {
		*ctx->position_trust_state = feedback->trust_state;
	}
	*ctx->position_glitch_count = 0U;
	*ctx->position_jitter_count = 0U;

	feedback->position_mech_rad = feedback->control.position_mech_rad;
	feedback->electrical_angle_rad = feedback->control.electrical_angle_rad;
	feedback->predicted_electrical_angle_rad =
		feedback->control.predicted_electrical_angle_rad;
	feedback->electrical_speed_rad_s = feedback->control.electrical_speed_rad_s;
	feedback->speed_mech_rad_s = feedback->control.speed_mech_rad_s;
	feedback->accel_mech_rad_s2 = feedback->control.accel_mech_rad_s2;
	feedback->speed_mech_filtered_rad_s = feedback->control.speed_mech_filtered_rad_s;

	return threshold_exceeded ? -EIO : 0;
}

int motor_encoder_feedback_prepare_capture(const struct motor_encoder_feedback_ctx *ctx,
					   const struct motor_encoder_feedback *feedback,
					   struct motor_capture_feedback *capture)
{
	if (ctx == NULL || feedback == NULL || capture == NULL) {
		return -EINVAL;
	}

	memset(capture, 0, sizeof(*capture));

	float32_t generated_mech_rad = angle_gen_get_angle(ctx->angle_gen);
	float32_t observer_mech_offset_rad = ctx->observer->mech_angle_offset_rad;

	bool raw_sample_valid = feedback->fresh && !feedback->error;

	capture->angle_rad = raw_sample_valid ?
				     (feedback->angle_control_deg * (PI_F32 / 180.0f)) :
				     feedback->observer_input_rad;
	capture->angle_deg = raw_sample_valid ?
				     feedback->angle_control_deg :
				     (feedback->observer_input_rad * (180.0f / PI_F32));
	capture->observer_mech_rad = feedback->observer_mech_rad;
	capture->observer_elec_rad = feedback->observer_elec_rad;
	capture->generated_mech_rad = wrap_rad_2pi(generated_mech_rad);
	capture->generated_elec_rad =
		wrap_rad_2pi((capture->generated_mech_rad + observer_mech_offset_rad) *
			     (float32_t)ctx->pole_pairs);
	capture->input_source = feedback->input_source;

	if (raw_sample_valid) {
		capture->encoder_mech_rad = wrap_rad_2pi(capture->angle_rad);
		capture->encoder_elec_rad =
			wrap_rad_2pi((capture->encoder_mech_rad + observer_mech_offset_rad) *
				     (float32_t)ctx->pole_pairs);
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
