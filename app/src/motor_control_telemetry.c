/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control_telemetry.h"

#include <string.h>

#include <zephyr/sys/util.h>

#include "config.h"
#include "motor/math/math_constants.h"

/* Phase P03 scaffold helper. Kept out of the ISR callback wiring for now. */
void motor_control_telemetry_refresh_diag(struct motor_parameters *params)
{
	if (params == NULL) {
		return;
	}

	struct motor_rt_diag_state *diag = &params->rt_diag;
	diag->state_counter = params->state_counter;
	diag->encoder_fault_counter = params->encoder_fault_counter;
	diag->encoder_warning_count = params->encoder_warning_count;
	diag->encoder_error_count = params->encoder_error_count;
	diag->max_isr_cycles = params->max_isr_cycles;
	diag->total_isr_cycles = params->total_isr_cycles;
	diag->overrun_count = params->overrun_count;
	diag->encoder_capture_overrun_count = params->encoder_capture.overrun_count;
	diag->fault_snapshot_overrun_count = params->fault_snapshot.overrun_count;
	diag->fault_snapshot_latch_loop = params->fault_snapshot.latch_loop;
	diag->fault_snapshot_latch_error_code = params->fault_snapshot.latch_error_code;
	diag->command_timeout_count = params->command_timeout_count;
	diag->profile_sequence_event_drop_count = params->profile_seq.event_drop_count;
}

/* Optional capture-path hook for extended observer/debug payloads. */
void motor_control_telemetry_consume_capture(const struct motor_capture_feedback *capture)
{
	ARG_UNUSED(capture);
}

void motor_control_telemetry_store_encoder_capture(struct motor_parameters *params,
						   const struct motor_capture_feedback *capture)
{
	if (params == NULL || capture == NULL || !params->encoder_capture.enabled) {
		return;
	}

	uint16_t decimation = MAX((uint16_t)1U, params->encoder_capture.decimation);
	if (params->encoder_capture.phase > 0U) {
		params->encoder_capture.phase--;
		return;
	}
	params->encoder_capture.phase = decimation - 1U;

	uint16_t idx = params->encoder_capture.write_idx;
	struct motor_encoder_capture_sample *sample = &params->encoder_capture.samples[idx];
	sample->control_loop_count = params->rt_fast.control_loop_count;
	sample->angle_deg = capture->angle_deg;
	sample->angle_rad = capture->angle_rad;
	sample->encoder_mech_rad = capture->encoder_mech_rad;
	sample->encoder_elec_rad = capture->encoder_elec_rad;
	sample->observer_mech_rad = capture->observer_mech_rad;
	sample->observer_elec_rad = capture->observer_elec_rad;
	sample->generated_mech_rad = capture->generated_mech_rad;
	sample->generated_elec_rad = capture->generated_elec_rad;
	sample->mech_error_rad = capture->mech_error_rad;
	sample->elec_error_rad = capture->elec_error_rad;
	sample->compare_valid = capture->compare_valid ? 1U : 0U;
	sample->input_source = capture->input_source;
	sample->sample_enabled = capture->sample_enabled ? 1U : 0U;
	sample->sample_fresh = capture->sample_fresh ? 1U : 0U;
	sample->sample_warning = capture->sample_warning ? 1U : 0U;
	sample->sample_error = capture->sample_error ? 1U : 0U;
	sample->status = capture->status;

	params->encoder_capture.write_idx =
		(uint16_t)((idx + 1U) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	if (params->encoder_capture.count < MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
		params->encoder_capture.count++;
	} else {
		params->encoder_capture.overrun_count++;
	}
}

void motor_control_telemetry_store_encoder_raw_trace(
	struct motor_parameters *params,
	const struct motor_control_encoder_sample *raw_sample,
	const struct motor_control_feedback *control_fb,
	uint8_t position_quality_flags)
{
	if (params == NULL || raw_sample == NULL || control_fb == NULL ||
	    !params->encoder_raw_trace.enabled) {
		return;
	}

	uint16_t decimation = MAX((uint16_t)1U, params->encoder_raw_trace.decimation);
	if (params->encoder_raw_trace.phase > 0U) {
		params->encoder_raw_trace.phase--;
		return;
	}
	params->encoder_raw_trace.phase = decimation - 1U;

	uint16_t idx = params->encoder_raw_trace.write_idx;
	struct motor_encoder_raw_trace_sample *sample =
		&params->encoder_raw_trace.samples[idx];
	memset(sample, 0, sizeof(*sample));

	sample->control_loop_count = params->rt_fast.control_loop_count;
	sample->raw_angle_deg = raw_sample->angle_deg;
	sample->raw_angle_rad = raw_sample->angle_deg * (PI_F32 / 180.0f);
	sample->control_angle_deg = control_fb->angle_control_deg;
	sample->control_angle_rad = control_fb->angle_control_deg * (PI_F32 / 180.0f);
	sample->observer_input_rad = control_fb->observer_input_rad;
	sample->input_source = control_fb->input_source;
	sample->quality_flags = position_quality_flags;
	sample->sample_enabled = raw_sample->enabled ? 1U : 0U;
	sample->sample_fresh = raw_sample->fresh ? 1U : 0U;
	sample->sample_warning = raw_sample->warning ? 1U : 0U;
	sample->sample_error = raw_sample->error ? 1U : 0U;
	sample->sample_io_fault = raw_sample->io_fault ? 1U : 0U;
	sample->status = raw_sample->status;

	params->encoder_raw_trace.write_idx =
		(uint16_t)((idx + 1U) % MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	if (params->encoder_raw_trace.count < MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES) {
		params->encoder_raw_trace.count++;
	} else {
		params->encoder_raw_trace.overrun_count++;
	}
}

void motor_control_telemetry_store_fault_snapshot(
	struct motor_parameters *params,
	const struct motor_control_fault_snapshot *snapshot)
{
	if (params == NULL || snapshot == NULL || !snapshot->valid ||
	    !params->fault_snapshot.enabled) {
		return;
	}

	uint16_t decimation = MAX((uint16_t)1U, params->fault_snapshot.decimation);
	uint16_t phase = (uint16_t)(params->rt_fast.control_loop_count % decimation);
	if (phase != params->fault_snapshot.phase) {
		return;
	}

	uint16_t idx = params->fault_snapshot.write_idx;
	struct motor_fault_snapshot_sample *sample = &params->fault_snapshot.samples[idx];

	sample->control_loop_count = params->rt_fast.control_loop_count;
	sample->encoder_angle_deg = snapshot->encoder_angle_deg;
	sample->observer_input_rad = snapshot->observer_input_rad;
	sample->elec_angle_rad = snapshot->elec_angle_rad;
	sample->observer_elec_speed_rad_s = snapshot->observer_elec_speed_rad_s;
	sample->Id_ref_A = snapshot->id_ref_a;
	sample->Iq_ref_A = snapshot->iq_ref_a;
	sample->Id_A = snapshot->id_a;
	sample->Iq_A = snapshot->iq_a;
	sample->Ia_A = snapshot->ia_a;
	sample->Ib_A = snapshot->ib_a;
	sample->Vd_V = snapshot->vd_v;
	sample->Vq_V = snapshot->vq_v;
	sample->input_source = snapshot->input_source;
	sample->sample_fresh = snapshot->sample_fresh;
	sample->sample_warning = snapshot->sample_warning;
	sample->sample_error = snapshot->sample_error;
	sample->status = snapshot->status;
	sample->position_quality_flags = snapshot->position_quality_flags;

	params->fault_snapshot.write_idx =
		(uint16_t)((idx + 1U) % MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	if (params->fault_snapshot.count < MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES) {
		params->fault_snapshot.count++;
	} else {
		params->fault_snapshot.overrun_count++;
	}
}
