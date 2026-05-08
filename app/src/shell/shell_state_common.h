#ifndef APP_SRC_SHELL_STATE_COMMON_H_
#define APP_SRC_SHELL_STATE_COMMON_H_

/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "shell_commands_state.h"
#include "shell_commands_motion.h"
#include "motor_control_api.h"
#include "motor_current_slew.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "motor/runtime/keepalive_policy.h"
#include "motor/runtime/control_policy.h"
#include "motor_hardware.h"
#include "motor_encoder_control.h"
#include "motor_encoder_acquisition.h"
#include "motor_state_transition.h"
#include "config.h"
#include "motor/math/angle_wrap.h"
#include "shell_parse.h"
#include "shell_motor_limits.h"
#include "motor_encoder_fault_reason.h"

static inline const char *motor_shell_feedback_trust_name(uint8_t trust_state)
{
	switch (trust_state) {
	case MOTOR_FEEDBACK_TRUST_TRUSTED:
		return "trusted";
	case MOTOR_FEEDBACK_TRUST_PREDICTED:
		return "predicted";
	default:
		return "fault";
	}
}

#if DT_NODE_EXISTS(DT_ALIAS(encoder1)) && DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955_fast)
#include <drivers/encoder/aeat9955_fast.h>
#include <drivers/encoder_rt.h>
#include <drivers/rt_spi.h>
#define MOTOR_ENCODER_IS_AEAT9955 1
#define MOTOR_ENCODER_IS_AEAT9955_FAST 1
#define MOTOR_ENCODER_IS_FAST 1
#elif DT_NODE_EXISTS(DT_ALIAS(encoder1)) && DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), magntek_mt6835_fast)
#include <drivers/encoder_rt.h>
#include <drivers/rt_spi.h>
#define MOTOR_ENCODER_IS_AEAT9955 0
#define MOTOR_ENCODER_IS_AEAT9955_FAST 0
#define MOTOR_ENCODER_IS_FAST 1
#else
#define MOTOR_ENCODER_IS_AEAT9955 0
#define MOTOR_ENCODER_IS_AEAT9955_FAST 0
#define MOTOR_ENCODER_IS_FAST 0
#endif

#if MOTOR_ENCODER_IS_FAST
#define MOTOR_ENCODER_HAS_RTSPI 1
#define encoder_rtspi DEVICE_DT_GET(DT_PARENT(DT_ALIAS(encoder1)))
#else
#define MOTOR_ENCODER_HAS_RTSPI 0
#define encoder_rtspi NULL
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

static inline bool motor_state_allows_arm(int state)
{
	return state == MOTOR_STATE_IDLE ||
	       state == MOTOR_STATE_PREPARE_ONLINE ||
	       state == MOTOR_STATE_ONLINE ||
	       motor_state_is_online_submode(state);
}

static inline bool motor_state_is_align_phase(int state)
{
	return state == MOTOR_STATE_ALIGN ||
	       state == MOTOR_STATE_ALIGN_POS_INJECT ||
	       state == MOTOR_STATE_ALIGN_POS_SAMPLE;
}

static inline const char *motor_encoder_input_source_to_string(uint8_t source)
{
	switch (source) {
	case MOTOR_ANGLE_INPUT_SRC_GENERATED:
		return "generated";
	case MOTOR_ANGLE_INPUT_SRC_ENCODER:
		return "encoder";
	case MOTOR_ANGLE_INPUT_SRC_PROPAGATED:
	default:
		return "propagated";
	}
}

static inline const char *motor_calibration_mode_to_string(uint8_t mode)
{
	switch (mode) {
	case MOTOR_CALIBRATION_MODE_BOOT:
		return "boot";
	case MOTOR_CALIBRATION_MODE_COMMISSIONING:
		return "commissioning";
	default:
		return "unknown";
	}
}

static inline enum motor_control_policy_mode motor_shell_policy_mode_from_state(int state)
{
	switch (state) {
	case MOTOR_STATE_ONLINE_VELOCITY_GENERATED:
		return MOTOR_CONTROL_POLICY_MODE_VELOCITY_GENERATED;
	case MOTOR_STATE_ONLINE_POSITION_GENERATED:
		return MOTOR_CONTROL_POLICY_MODE_POSITION_GENERATED;
	case MOTOR_STATE_ONLINE_CURRENT_ENCODER:
		return MOTOR_CONTROL_POLICY_MODE_CURRENT_ENCODER;
	case MOTOR_STATE_ONLINE_VELOCITY_ENCODER:
		return MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER;
	case MOTOR_STATE_ONLINE_POSITION_ENCODER:
		return MOTOR_CONTROL_POLICY_MODE_POSITION_ENCODER;
	case MOTOR_STATE_CALIBRATION:
	case MOTOR_STATE_OFFSET_MEAS:
	case MOTOR_STATE_ROVERL_MEAS:
	case MOTOR_STATE_ALIGN:
	case MOTOR_STATE_ALIGN_POS_INJECT:
	case MOTOR_STATE_ALIGN_POS_SAMPLE:
		return MOTOR_CONTROL_POLICY_MODE_CALIBRATION;
	default:
		return MOTOR_CONTROL_POLICY_MODE_DISABLED;
	}
}

static inline int motor_shell_derive_control_policy(struct motor_parameters *params,
					     struct motor_control_policy *policy)
{
	if (params == NULL || policy == NULL) {
		return -EINVAL;
	}

	atomic_val_t flags = atomic_get(&params->feature_flags);
	struct motor_control_policy_input input = {
		.mode = motor_shell_policy_mode_from_state(motor_api_get_state()),
		.features = {
			.encoder_read_enabled =
				(flags & BIT(MOTOR_FEATURE_ENCODER_READ)) != 0,
			.angle_gen_enabled =
				(flags & BIT(MOTOR_FEATURE_ANGLE_GEN)) != 0,
			.velocity_traj_enabled =
				(flags & BIT(MOTOR_FEATURE_VELOCITY_TRAJ)) != 0,
			.commanded_currents_enabled =
				(flags & BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS)) != 0,
			.current_loop_enabled =
				(flags & BIT(MOTOR_FEATURE_PI_CONTROL)) != 0,
		},
		.profile_sequence_active = params->profile_seq.running,
	};

	return motor_control_policy_derive(&input, policy);
}

static inline void motor_shell_print_control_policy(const struct shell *sh,
					     const struct motor_control_policy *policy)
{
	struct motor_actuator_caps caps = motor_actuator_caps_for_kind(policy->actuator_kind);
	bool valid = motor_control_policy_is_valid(policy, &caps);

	shell_print(sh, "Control Policy:");
	shell_print(sh, "  Motion source:    %s",
		    motor_motion_source_to_string(policy->motion_source));
	shell_print(sh, "  Feedback source:  %s",
		    motor_feedback_source_to_string(policy->feedback_source));
	shell_print(sh, "  Angle source:     %s",
		    motor_angle_source_to_string(policy->angle_source));
	shell_print(sh, "  Current source:   %s",
		    motor_current_source_to_string(policy->current_source));
	shell_print(sh, "  Actuator/backend: %s",
		    motor_actuator_kind_to_string(policy->actuator_kind));
	shell_print(sh, "  Encoder control:  %s",
		    policy->encoder_read_enabled ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Encoder required: %s",
		    policy->encoder_required_for_control ? "YES" : "NO");
	shell_print(sh, "  Current loop:     %s",
		    policy->current_loop_enabled ? "ENABLED" : "DISABLED");
	shell_print(sh, "  Generated drive:  %s",
		    motor_generated_angle_mode_to_string(policy->generated_angle_mode));
	shell_print(sh, "  Policy valid:     %s", valid ? "YES" : "NO");
}

#define MOTOR_ENCODER_COMPARE_REF_GENERATED 0U
#define MOTOR_ENCODER_COMPARE_REF_OBSERVER 1U
#define MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS 32U

static inline bool motor_encoder_compare_ref_parse(const char *arg, uint8_t *ref_mode)
{
	if (arg == NULL || ref_mode == NULL) {
		return false;
	}

	if ((strcmp(arg, "gen") == 0) || (strcmp(arg, "generated") == 0)) {
		*ref_mode = MOTOR_ENCODER_COMPARE_REF_GENERATED;
		return true;
	}
	if ((strcmp(arg, "obs") == 0) || (strcmp(arg, "observer") == 0)) {
		*ref_mode = MOTOR_ENCODER_COMPARE_REF_OBSERVER;
		return true;
	}

	return false;
}

static inline const char *motor_encoder_compare_ref_to_string(uint8_t ref_mode)
{
	return (ref_mode == MOTOR_ENCODER_COMPARE_REF_OBSERVER) ? "obs" : "gen";
}

static inline uint32_t motor_command_age_ms(const struct motor_parameters *params)
{
	uint32_t loop_now = params->control_loop_count;
	uint32_t loop_age = loop_now - params->last_command_update_loop;

	if (loop_age > 0U) {
		uint64_t age_ms = ((uint64_t)loop_age * 1000ULL) /
				  (uint64_t)CONTROL_LOOP_FREQUENCY_HZ_U;

		return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
	}

	return k_uptime_get_32() - params->last_command_update_ms;
}

static inline float motor_encoder_normalized_from_rad(float32_t angle_rad)
{
	if (!(angle_rad == angle_rad) || fabsf(angle_rad) > 1.0e6f) {
		return 0.0f;
	}

	float32_t wrapped = wrap_rad_2pi(angle_rad);

	return wrapped / (2.0f * PI_F32);
}

static inline int32_t motor_encoder_q31_from_rad(float32_t angle_rad)
{
	if (!(angle_rad == angle_rad) || fabsf(angle_rad) > 1.0e6f) {
		return 0;
	}

	float32_t wrapped = wrap_rad_pi(angle_rad);
	float32_t scaled = wrapped * (2147483648.0f / PI_F32);

	if (scaled >= 2147483647.0f) {
		return INT32_MAX;
	}
	if (scaled <= -2147483648.0f) {
		return INT32_MIN;
	}

	return (int32_t)lrintf(scaled);
}

static inline uint32_t motor_shell_f32_bits(float32_t value)
{
	uint32_t bits = 0U;

	memcpy(&bits, &value, sizeof(bits));
	return bits;
}

static inline double motor_shell_rad_to_deg(float32_t rad)
{
	return (double)(rad * (180.0f / PI_F32));
}

static inline uint16_t motor_encoder_ring_oldest(uint16_t write_idx, uint16_t count,
					  uint16_t capacity)
{
	return (uint16_t)((write_idx + capacity - count) % capacity);
}

static inline uint16_t motor_encoder_ring_index(uint16_t oldest_idx, uint16_t offset,
					 uint16_t capacity)
{
	return (uint16_t)((oldest_idx + offset) % capacity);
}

static inline bool motor_encoder_raw_trace_sample_clean(
	const struct motor_encoder_raw_trace_sample *sample)
{
	return sample != NULL &&
	       sample->sample_fresh &&
	       !sample->sample_error &&
	       !sample->sample_io_fault &&
	       (sample->raw_angle_rad == sample->raw_angle_rad) &&
	       fabsf(sample->raw_angle_rad) <= 1.0e6f &&
	       (sample->control_angle_rad == sample->control_angle_rad) &&
	       fabsf(sample->control_angle_rad) <= 1.0e6f;
}

static inline bool motor_encoder_capture_sample_clean(
	const struct motor_encoder_capture_sample *sample)
{
	return sample != NULL &&
	       sample->sample_fresh &&
	       !sample->sample_error &&
	       (sample->angle_rad == sample->angle_rad) &&
	       fabsf(sample->angle_rad) <= 1.0e6f &&
	       (sample->encoder_mech_rad == sample->encoder_mech_rad) &&
	       fabsf(sample->encoder_mech_rad) <= 1.0e6f &&
	       (sample->generated_mech_rad == sample->generated_mech_rad) &&
	       fabsf(sample->generated_mech_rad) <= 1.0e6f;
}

static inline int motor_encoder_parse_dump_window(const struct shell *sh,
					   size_t argc,
					   char **argv,
					   uint16_t stored,
					   uint16_t capacity,
					   uint16_t write_idx,
					   uint16_t *start_idx,
					   uint16_t *count_out)
{
	if (argc < 1U || argc > 3U) {
		shell_error(sh, "Usage: dump [count] | dump <offset> <count>");
		return -EINVAL;
	}
	if (stored == 0U) {
		*start_idx = 0U;
		*count_out = 0U;
		return 0;
	}

	uint32_t offset = 0U;
	uint32_t requested = MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS;
	uint16_t oldest_idx = motor_encoder_ring_oldest(write_idx, stored, capacity);

	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &requested) || requested == 0U) {
			shell_error(sh, "count must be in [1, %u]",
				    MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
			return -EINVAL;
		}
		if (requested > MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS) {
			shell_error(sh, "count must be <= %u; use dump <offset> <count> chunks",
				    MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
			return -EINVAL;
		}

		uint16_t count = (uint16_t)MIN(requested, stored);
		offset = stored - count;
		*start_idx = motor_encoder_ring_index(oldest_idx, (uint16_t)offset, capacity);
		*count_out = count;
		return 0;
	}

	if (argc == 3U) {
		if (!shell_parse_u32(argv[1], &offset) ||
		    !shell_parse_u32(argv[2], &requested) ||
		    requested == 0U) {
			shell_error(sh, "offset/count must be unsigned integers, count > 0");
			return -EINVAL;
		}
		if (offset >= stored) {
			shell_error(sh, "offset must be < stored (%u)", stored);
			return -EINVAL;
		}
		if (requested > MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS) {
			shell_error(sh, "count must be <= %u",
				    MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
			return -EINVAL;
		}

		uint32_t available = stored - offset;
		uint16_t count = (uint16_t)MIN(requested, available);
		*start_idx = motor_encoder_ring_index(oldest_idx, (uint16_t)offset, capacity);
		*count_out = count;
		return 0;
	}

	uint16_t count = (uint16_t)MIN(MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS, stored);
	offset = stored - count;
	*start_idx = motor_encoder_ring_index(oldest_idx, (uint16_t)offset, capacity);
	*count_out = count;
	return 0;
}

static inline float32_t motor_encoder_avg_velocity_hz(float32_t delta_rad,
					       uint32_t first_loop,
					       uint32_t last_loop)
{
	if (last_loop <= first_loop) {
		return 0.0f;
	}

	float32_t dt_s = (float32_t)(last_loop - first_loop) /
			 (float32_t)CONTROL_LOOP_FREQUENCY_HZ;
	if (dt_s <= 0.0f) {
		return 0.0f;
	}

	return delta_rad / (2.0f * PI_F32 * dt_s);
}

static inline float32_t motor_encoder_trace_deg(float32_t angle_rad)
{
	return angle_rad * (180.0f / PI_F32);
}

static inline int32_t motor_encoder_deg_to_mdeg(double angle_deg)
{
	if (!isfinite(angle_deg)) {
		return 0;
	}
	if (angle_deg >= 2147483.647) {
		return INT32_MAX;
	}
	if (angle_deg <= -2147483.648) {
		return INT32_MIN;
	}
	return (int32_t)lrint(angle_deg * 1000.0);
}

static inline int32_t motor_encoder_rad_to_mdeg(float32_t angle_rad)
{
	return motor_encoder_deg_to_mdeg((double)motor_encoder_trace_deg(angle_rad));
}

static inline double motor_encoder_wrap_delta_deg(double delta_deg)
{
	while (delta_deg > 180.0) {
		delta_deg -= 360.0;
	}
	while (delta_deg <= -180.0) {
		delta_deg += 360.0;
	}

	return delta_deg;
}

static inline double motor_encoder_avg_velocity_hz_deg(double delta_deg,
						uint32_t first_loop,
						uint32_t last_loop)
{
	if (last_loop <= first_loop) {
		return 0.0;
	}

	double dt_s = (double)(last_loop - first_loop) /
		      (double)CONTROL_LOOP_FREQUENCY_HZ;
	if (dt_s <= 0.0) {
		return 0.0;
	}

	return delta_deg / (360.0 * dt_s);
}

static inline int32_t motor_encoder_avg_velocity_mhz_deg(double delta_deg,
						  uint32_t first_loop,
						  uint32_t last_loop)
{
	return motor_encoder_deg_to_mdeg(motor_encoder_avg_velocity_hz_deg(delta_deg,
									    first_loop,
									    last_loop));
}

static inline void motor_zero_control_targets(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	motor_current_slew_params_zero(params);
	params->live.velocity_target_rad_s = 0.0f;
	params->live.velocity_ref_rad_s = 0.0f;
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	motion_profile_quintic_cancel(&params->position_profile, params->live.position_rad);
	params->position_target_rad = wrap_rad_2pi(params->live.position_rad);
}

#if IS_ENABLED(CONFIG_ENCODER_MAGNET_CHECK_ON_ARM) || MOTOR_ENCODER_IS_AEAT9955_FAST
static inline int motor_encoder_read_aeat_alarm(uint8_t *status_out, bool *mhi_out, bool *mlo_out)
{
#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	ARG_UNUSED(status_out);
	ARG_UNUSED(mhi_out);
	ARG_UNUSED(mlo_out);
	return -ENOTSUP;
#else
	if (!device_is_ready(encoder1)) {
		return -ENODEV;
	}

	uint8_t status = 0U;
	int ret = aeat9955_fast_read_register(encoder1, AEAT9955_FAST_REG_ERROR_STATUS, &status);
	if (ret < 0) {
		return ret;
	}

	if (status_out) {
		*status_out = status;
	}
	if (mhi_out) {
		*mhi_out = (status & AEAT9955_FAST_ERROR_MHI_BIT) != 0U;
	}
	if (mlo_out) {
		*mlo_out = (status & AEAT9955_FAST_ERROR_MLO_BIT) != 0U;
	}

	return 0;
#endif
}
#endif

static inline void motor_encoder_capture_reset(struct motor_parameters *params, bool clear_samples)
{
	if (params == NULL) {
		return;
	}

	params->encoder_capture.phase = 0U;
	params->encoder_capture.write_idx = 0U;
	params->encoder_capture.count = 0U;
	params->encoder_capture.overrun_count = 0U;
	if (clear_samples) {
		memset(params->encoder_capture.samples, 0, sizeof(params->encoder_capture.samples));
	}
}

static inline void motor_encoder_raw_trace_reset(struct motor_parameters *params, bool clear_samples)
{
	if (params == NULL) {
		return;
	}

	params->encoder_raw_trace.phase = 0U;
	params->encoder_raw_trace.write_idx = 0U;
	params->encoder_raw_trace.count = 0U;
	params->encoder_raw_trace.overrun_count = 0U;
	if (clear_samples) {
		memset(params->encoder_raw_trace.samples, 0, sizeof(params->encoder_raw_trace.samples));
	}
}

static inline void motor_fault_snapshot_reset(struct motor_parameters *params, bool clear_samples)
{
	if (params == NULL) {
		return;
	}

	params->fault_snapshot.phase = 0U;
	params->fault_snapshot.write_idx = 0U;
	params->fault_snapshot.count = 0U;
	params->fault_snapshot.overrun_count = 0U;
	params->fault_snapshot.latched = 0U;
	params->fault_snapshot.latch_loop = 0U;
	params->fault_snapshot.latch_error_code = ERROR_NONE;
	params->fault_snapshot.latch_encoder_fault_reason = MOTOR_ENCODER_FAULT_REASON_NONE;
	if (clear_samples) {
		memset(params->fault_snapshot.samples, 0, sizeof(params->fault_snapshot.samples));
	}
}


#endif /* APP_SRC_SHELL_STATE_COMMON_H_ */
