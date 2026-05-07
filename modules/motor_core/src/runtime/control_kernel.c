/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "motor/math/math_constants.h"
#include "motor/observers/feedback.h"
#include "motor/observers/feedback_quality.h"
#include "motor/runtime/actuator_adapter.h"
#include "motor/runtime/control_kernel.h"

bool motor_control_kernel_feedback_valid(const struct motor_control_policy *policy,
						 const struct motor_feedback_ref *feedback_ref,
						 uint16_t stale_count,
						 uint32_t stale_limit)
{
	if (policy == NULL || feedback_ref == NULL) {
		return false;
	}

	if (!policy->encoder_required_for_control) {
		return true;
	}

	if (feedback_ref->source == MOTOR_FEEDBACK_ENCODER &&
	    feedback_ref->input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER &&
	    motor_feedback_quality_is_trusted(feedback_ref->quality_flags) &&
	    !feedback_ref->error) {
		return true;
	}

	/* Allow a short propagated-angle window while the asynchronous encoder
	 * transport catches up. The encoder feedback core owns the stale counter;
	 * this guard only blocks sustained stale feedback in encoder modes.
	 */
	return feedback_ref->input_source == MOTOR_ANGLE_INPUT_SRC_PROPAGATED &&
	       motor_feedback_quality_is_usable(feedback_ref->quality_flags) &&
	       stale_count <= stale_limit;
}

bool motor_control_kernel_feedback_sane(const struct motor_control_policy *policy,
					       const struct motor_feedback_ref *feedback_ref,
					       float32_t profile_max_velocity_rad_s)
{
	if (policy == NULL || feedback_ref == NULL || !policy->encoder_required_for_control) {
		return true;
	}

	if (!isfinite(feedback_ref->velocity_filtered_rad_s)) {
		return false;
	}

	if (policy->current_source == MOTOR_CURRENT_SOURCE_COMMANDED &&
	    policy->motion_source == MOTOR_MOTION_SOURCE_HOLD) {
		float32_t direct_current_limit_rad_s =
			fmaxf(profile_max_velocity_rad_s * 10.0f, 50.0f * 2.0f * PI_F32);

		return fabsf(feedback_ref->velocity_filtered_rad_s) <=
		       direct_current_limit_rad_s;
	}

	float32_t max_expected_rad_s =
		fmaxf(profile_max_velocity_rad_s * 1.10f, 2.0f * PI_F32);

	return fabsf(feedback_ref->velocity_filtered_rad_s) <= max_expected_rad_s;
}

int motor_control_kernel_step_fast(const struct motor_control_kernel_input *in,
					   struct motor_control_kernel_output *out)
{
	if (in == NULL || out == NULL || in->policy == NULL ||
	    in->motion_ref == NULL || in->feedback_ref == NULL || in->current_ref == NULL) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->feedback_valid = motor_control_kernel_feedback_valid(in->policy,
								in->feedback_ref,
								in->feedback_stale_count,
								in->feedback_stale_limit);
	out->feedback_sane = motor_control_kernel_feedback_sane(in->policy,
							      in->feedback_ref,
							      in->profile_max_velocity_rad_s);
	if (!out->feedback_valid || !out->feedback_sane) {
		motor_servo_ref_clear(&out->servo_ref);
		motor_actuator_ref_clear(&out->actuator_ref, in->policy->actuator_kind);
		out->actuator_status = -EAGAIN;
		return 0;
	}

	if (!in->current_loop_enabled) {
		motor_servo_ref_clear(&out->servo_ref);
		motor_actuator_ref_clear(&out->actuator_ref, in->policy->actuator_kind);
		out->actuator_status = 0;
		return 0;
	}

	motor_servo_ref_set_dq_current(&out->servo_ref,
					    true,
					    in->motion_ref->position_rad,
					    in->motion_ref->velocity_ref_rad_s,
					    in->motion_ref->acceleration_rad_s2,
					    in->current_ref->id_ref_a,
					    in->current_ref->iq_ref_a);
	out->actuator_status = motor_actuator_ref_from_servo(in->policy,
								     &out->servo_ref,
								     &out->actuator_ref);
	return 0;
}
