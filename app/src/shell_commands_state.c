/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "shell_commands_state.h"
#include "shell_commands_motion.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "motor/runtime/keepalive_policy.h"
#include "motor/runtime/control_policy.h"
#include "motor_hardware.h"
#include "motor_encoder_pipeline.h"
#include "config.h"
#include "motor/math/angle_wrap.h"
#include "shell_parse.h"

#if DT_NODE_EXISTS(DT_ALIAS(encoder1)) && DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955_fast)
#include <drivers/encoder/aeat9955_fast.h>
#include <drivers/encoder_rt.h>
#define MOTOR_ENCODER_IS_AEAT9955 1
#define MOTOR_ENCODER_IS_AEAT9955_FAST 1
#elif DT_NODE_EXISTS(DT_ALIAS(encoder1)) && DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955)
#include <drivers/sensor/brcm_aeat9955.h>
#define MOTOR_ENCODER_IS_AEAT9955 1
#define MOTOR_ENCODER_IS_AEAT9955_FAST 0
#else
#define MOTOR_ENCODER_IS_AEAT9955 0
#define MOTOR_ENCODER_IS_AEAT9955_FAST 0
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

static const char *motor_encoder_input_source_to_string(uint8_t source)
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

static const char *motor_calibration_mode_to_string(uint8_t mode)
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

static enum motor_control_policy_mode motor_shell_policy_mode_from_state(int state)
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
	case MOTOR_STATE_RS_EST:
	case MOTOR_STATE_ROVERL_MEAS:
	case MOTOR_STATE_ALIGN:
	case MOTOR_STATE_ALIGN_POS_INJECT:
	case MOTOR_STATE_ALIGN_POS_SAMPLE:
		return MOTOR_CONTROL_POLICY_MODE_CALIBRATION;
	default:
		return MOTOR_CONTROL_POLICY_MODE_DISABLED;
	}
}

static int motor_shell_derive_control_policy(struct motor_parameters *params,
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

static void motor_shell_print_control_policy(const struct shell *sh,
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

static bool motor_encoder_compare_ref_parse(const char *arg, uint8_t *ref_mode)
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

static const char *motor_encoder_compare_ref_to_string(uint8_t ref_mode)
{
	return (ref_mode == MOTOR_ENCODER_COMPARE_REF_OBSERVER) ? "obs" : "gen";
}

static uint32_t motor_command_age_ms(const struct motor_parameters *params)
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

static uint16_t motor_encoder_ring_oldest(uint16_t write_idx, uint16_t count,
					  uint16_t capacity)
{
	return (uint16_t)((write_idx + capacity - count) % capacity);
}

static uint16_t motor_encoder_ring_index(uint16_t oldest_idx, uint16_t offset,
					 uint16_t capacity)
{
	return (uint16_t)((oldest_idx + offset) % capacity);
}

static bool motor_encoder_raw_trace_sample_clean(
	const struct motor_encoder_raw_trace_sample *sample)
{
	return sample != NULL &&
	       sample->sample_fresh &&
	       !sample->sample_error &&
	       !sample->sample_io_fault &&
	       (sample->raw_angle_rad == sample->raw_angle_rad) &&
	       fabsf(sample->raw_angle_rad) <= 1.0e6f;
}

static bool motor_encoder_capture_sample_clean(
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

static int motor_encoder_parse_dump_window(const struct shell *sh,
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

static float32_t motor_encoder_avg_velocity_hz(float32_t delta_rad,
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

static inline void motor_zero_control_targets(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	params->live.velocity_target_rad_s = 0.0f;
	params->live.velocity_ref_rad_s = 0.0f;
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	motion_profile_quintic_cancel(&params->position_profile, params->live.position_rad);
	params->position_target_rad = wrap_rad_2pi(params->live.position_rad);
}

#if IS_ENABLED(CONFIG_ENCODER_MAGNET_CHECK_ON_ARM) || MOTOR_ENCODER_IS_AEAT9955
static int motor_encoder_read_aeat_alarm(uint8_t *status_out, bool *mhi_out, bool *mlo_out)
{
#if !MOTOR_ENCODER_IS_AEAT9955
	ARG_UNUSED(status_out);
	ARG_UNUSED(mhi_out);
	ARG_UNUSED(mlo_out);
	return -ENOTSUP;
#else
	if (!device_is_ready(encoder1)) {
		return -ENODEV;
	}

#if MOTOR_ENCODER_IS_AEAT9955_FAST
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
#else
	struct sensor_value raw = {0};
	struct sensor_value mhi = {0};
	struct sensor_value mlo = {0};

	int ret = sensor_attr_get(encoder1, SENSOR_CHAN_ROTATION,
				  (enum sensor_attribute)AEAT9955_ATTR_ERROR_STATUS, &raw);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_attr_get(encoder1, SENSOR_CHAN_ROTATION,
			      (enum sensor_attribute)AEAT9955_ATTR_ALARM_MAGNET_HIGH, &mhi);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_attr_get(encoder1, SENSOR_CHAN_ROTATION,
			      (enum sensor_attribute)AEAT9955_ATTR_ALARM_MAGNET_LOW, &mlo);
	if (ret < 0) {
		return ret;
	}

	if (status_out) {
		*status_out = (uint8_t)(raw.val1 & 0xFF);
	}
	if (mhi_out) {
		*mhi_out = (mhi.val1 != 0);
	}
	if (mlo_out) {
		*mlo_out = (mlo.val1 != 0);
	}

	return 0;
#endif
#endif
}
#endif

static void motor_encoder_capture_reset(struct motor_parameters *params, bool clear_samples)
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

static void motor_encoder_raw_trace_reset(struct motor_parameters *params, bool clear_samples)
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

static void motor_fault_snapshot_reset(struct motor_parameters *params, bool clear_samples)
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
	if (clear_samples) {
		memset(params->fault_snapshot.samples, 0, sizeof(params->fault_snapshot.samples));
	}
}

/* motor state prepare */
int cmd_motor_state_prepare_online(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_prepare_online() == 0) {
		shell_print(sh, "PREPARE_ONLINE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request PREPARE_ONLINE state");
		return -EIO;
	}
}

/* motor state idle */
int cmd_motor_state_idle(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_idle() == 0) {
		shell_print(sh, "IDLE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request IDLE state");
		return -EIO;
	}
}

/* motor state online */
int cmd_motor_state_online(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_online() == 0) {
		shell_print(sh, "ONLINE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request ONLINE state");
		return -EIO;
	}
}

/* motor state calibrate */
int cmd_motor_state_calibrate(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (motor_api_request_calibrate() == 0) {
		shell_print(sh, "Boot calibration sequence started");
		return 0;
	} else {
		shell_error(sh, "Failed to start calibration");
		return -EIO;
	}
}

/* motor state commission */
int cmd_motor_state_commission(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (motor_api_request_commission() == 0) {
		shell_print(sh, "Commissioning sequence started");
		return 0;
	} else {
		shell_error(sh, "Failed to start commissioning");
		return -EIO;
	}
}

/* motor state clear_error */
int cmd_motor_state_clear_error(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (motor_api_clear_error() == 0) {
		shell_print(sh, "Error cleared");
		return 0;
	} else {
		shell_error(sh, "Failed to clear error");
		return -EIO;
	}
}

/* motor arm */
int cmd_motor_arm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	int state = motor_api_get_state();
	if (!motor_state_allows_arm(state)) {
		shell_error(sh, "Cannot arm while in state %s. Wait for IDLE/PREPARE_ONLINE/ONLINE.",
			    motor_state_to_string(state));
		return -EAGAIN;
	}

#if CONFIG_ENCODER_MAGNET_CHECK_ON_ARM
	{
		uint8_t status = 0U;
		bool mhi = false;
		bool mlo = false;
		int ret = motor_encoder_read_aeat_alarm(&status, &mhi, &mlo);
		if (ret == -ENOTSUP) {
			shell_error(sh, "CONFIG_ENCODER_MAGNET_CHECK_ON_ARM requires AEAT-9955 encoder1.");
			return ret;
		}
		if (ret < 0) {
			shell_error(sh, "Failed to read encoder magnet alarms (err %d)", ret);
			return ret;
		}
		if (mhi || mlo) {
			shell_error(sh,
				    "Cannot arm: encoder magnet alarm active (raw=0x%02X, MHI=%s, MLO=%s)",
				    status, mhi ? "SET" : "CLEAR", mlo ? "SET" : "CLEAR");
			return -EACCES;
		}
	}
#endif

	atomic_set(&g_motor_params->control_armed, 1);
	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Control armed (state=%s)", motor_state_to_string(state));
	return 0;
}

/* motor disarm */
int cmd_motor_disarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	atomic_set(&g_motor_params->control_armed, 0);
	motor_zero_control_targets(g_motor_params);
	motor_command_feed_watchdog(g_motor_params);

	int ret = motor_api_request_idle();
	if (ret != 0) {
		shell_error(sh, "Disarmed, but failed to request IDLE (err %d)", ret);
		return ret;
	}

	shell_print(sh, "Control disarmed and IDLE requested");
	return 0;
}

/* motor state status */
int cmd_motor_state_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	int state = motor_api_get_state();
	const char *state_str = motor_state_to_string(state);
	int error = motor_api_get_error();
	const char *error_str = motor_error_to_string(error);
	uint32_t age_ms = motor_command_age_ms(g_motor_params);
	bool control_armed = motor_control_is_armed(g_motor_params);
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_GENERATED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER);
	bool autonomous_keepalive =
		motor_keepalive_policy_should_keepalive(control_armed, autonomous_mode_active,
						 g_motor_params->profile_seq.running,
						 g_motor_params->chopper_cal.active,
						 motion_profile_quintic_is_active(
							 &g_motor_params->position_profile));
	
	shell_print(sh, "Motor Status:");
	shell_print(sh, "  State: %s (%d)", state_str, state);
	shell_print(sh, "  Error: %s (%d)", error_str, error);
	shell_print(sh, "  Armed: %s", motor_control_is_armed(g_motor_params) ? "YES" : "NO");
	shell_print(sh, "  Command timeout: %u ms", g_motor_params->command_timeout_ms);
	shell_print(sh, "  Command age: %u ms", age_ms);
	shell_print(sh, "  Timeout latch: %s", g_motor_params->command_timeout_latched ? "SET" : "CLEAR");
	shell_print(sh, "  Timeout count: %u", g_motor_params->command_timeout_count);
	shell_print(sh, "  Auto keepalive: %s", autonomous_keepalive ? "ACTIVE" : "INACTIVE");
	shell_print(sh, "  Cal complete: %s", g_motor_params->calibration.complete ? "YES" : "NO");
	shell_print(sh, "  Cal running:  %s", g_motor_params->calibration.running ? "YES" : "NO");
	shell_print(sh, "  Cal mode:     %s",
		    motor_calibration_mode_to_string(g_motor_params->calibration.mode));
	shell_print(sh, "  Commissioned: %s",
		    g_motor_params->calibration.commissioning_complete ? "YES" : "NO");
	shell_print(sh, "  Online mode:  %s",
		    motor_state_to_string(g_motor_params->calibration.requested_online_mode));
	shell_print(sh, "  Enc dir sign: %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	struct motor_control_policy policy = {0};
	if (motor_shell_derive_control_policy(g_motor_params, &policy) == 0) {
		motor_shell_print_control_policy(sh, &policy);
	} else {
		shell_print(sh, "Control Policy:");
		shell_print(sh, "  Policy valid:     NO");
	}
	if (g_motor_params->calibration.running || motor_state_is_align_phase(state)) {
		shell_print(sh, "  Align phase:  %s", state_str);
		shell_print(sh,
			    "  Align sample: samples=%u retries=%u mean=%.2f deg",
			    g_motor_params->calibration.align_sample_count,
			    g_motor_params->calibration.align_sample_retries,
			    (double)(g_motor_params->calibration.align_mech_angle_rad *
				     (180.0f / PI_F32)));
		shell_print(sh,
			    "  Align offset: %.2f deg",
			    (double)(g_motor_params->observer_alignment_offset_rad *
				     (180.0f / PI_F32)));
	}
	
	return 0;
}

/* motor state policy */
int cmd_motor_state_policy(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	int state = motor_api_get_state();
	struct motor_control_policy policy = {0};
	int ret = motor_shell_derive_control_policy(g_motor_params, &policy);
	if (ret != 0) {
		shell_error(sh, "Failed to derive control policy (err %d)", ret);
		return ret;
	}

	shell_print(sh, "State: %s (%d)", motor_state_to_string(state), state);
	motor_shell_print_control_policy(sh, &policy);
	return 0;
}

/* Helper function for mode changes */
static int motor_request_mode_change(const struct shell *sh, enum motor_state target_state, const char *mode_name)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	int current_state = motor_api_get_state();
	bool online_active = (current_state == MOTOR_STATE_ONLINE) ||
			    motor_state_is_online_submode(current_state);
	if (!online_active) {
		g_motor_params->calibration.requested_online_mode = (uint8_t)target_state;
		shell_print(sh,
			    "Online mode set to %s (will apply on next ONLINE entry)",
			    mode_name);
		return 0;
	}

	/* Post mode change event to state machine */
	struct motor_event evt = {
		.type = MOTOR_EVENT_MODE_CHANGE,
		.target_mode = target_state,
	};

	int ret = motor_api_post_event(&evt);
	if (ret != 0) {
		shell_error(sh, "Failed to post mode change event: queue full");
		return ret;
	}

	shell_print(sh, "Mode change to %s requested", mode_name);
	return 0;
}

/* motor state mode current_encoder */
int cmd_motor_state_mode_current_encoder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_CURRENT_ENCODER,
					 "current_encoder");
}

/* motor state mode velocity_generated */
int cmd_motor_state_mode_velocity_generated(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_VELOCITY_GENERATED,
					 "velocity_generated");
}

/* motor state mode position_generated */
int cmd_motor_state_mode_position_generated(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_POSITION_GENERATED,
					 "position_generated");
}

/* motor state mode velocity_encoder */
int cmd_motor_state_mode_velocity_encoder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					 "velocity_encoder");
}

/* motor state mode position_encoder */
int cmd_motor_state_mode_position_encoder(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_POSITION_ENCODER,
					 "position_encoder");
}

/* motor safety timeout <ms> */
int cmd_motor_safety_timeout(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor safety timeout <ms>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t timeout_ms = 0U;
	if (!shell_parse_u32(argv[1], &timeout_ms)) {
		shell_error(sh, "Timeout must be a non-negative integer in milliseconds.");
		return -EINVAL;
	}

	int ret = motor_api_set_param("command_timeout_ms", (float)timeout_ms);
	if (ret != 0) {
		shell_error(sh, "Failed to set command timeout (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Command timeout set to %u ms%s",
		    timeout_ms, timeout_ms == 0 ? " (disabled)" : "");
	return 0;
}

/* motor safety pet */
int cmd_motor_safety_pet(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Command watchdog fed");
	return 0;
}

/* motor safety status */
int cmd_motor_safety_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t age_ms = motor_command_age_ms(g_motor_params);
	bool timeout_enabled = g_motor_params->command_timeout_ms > 0U;
	bool control_armed = motor_control_is_armed(g_motor_params);
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_GENERATED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER);
	bool autonomous_keepalive =
		motor_keepalive_policy_should_keepalive(control_armed, autonomous_mode_active,
						 g_motor_params->profile_seq.running,
						 g_motor_params->chopper_cal.active,
						 motion_profile_quintic_is_active(
							 &g_motor_params->position_profile));
	bool timeout_expired = timeout_enabled && !autonomous_keepalive &&
			       (age_ms > g_motor_params->command_timeout_ms);

	shell_print(sh, "Safety Status:");
	shell_print(sh, "  Armed:              %s",
		    motor_control_is_armed(g_motor_params) ? "YES" : "NO");
	shell_print(sh, "  Timeout enabled:    %s", timeout_enabled ? "YES" : "NO");
	shell_print(sh, "  Timeout value:      %u ms", g_motor_params->command_timeout_ms);
	shell_print(sh, "  Command age:        %u ms", age_ms);
	shell_print(sh, "  Timeout expired:    %s", timeout_expired ? "YES" : "NO");
	shell_print(sh, "  Auto keepalive:     %s", autonomous_keepalive ? "ACTIVE" : "INACTIVE");
	shell_print(sh, "  Timeout latch:      %s",
		    g_motor_params->command_timeout_latched ? "SET" : "CLEAR");
	shell_print(sh, "  Timeout count:      %u", g_motor_params->command_timeout_count);
#if CONFIG_ENCODER_MAGNET_CHECK_ON_ARM
	shell_print(sh, "  Magnet check arm:   ENABLED (Kconfig)");
#else
	shell_print(sh, "  Magnet check arm:   DISABLED (Kconfig)");
#endif

#if MOTOR_ENCODER_IS_AEAT9955
	uint8_t mag_status = 0U;
	bool mhi = false;
	bool mlo = false;
	int mag_ret = motor_encoder_read_aeat_alarm(&mag_status, &mhi, &mlo);
	if (mag_ret == 0) {
		shell_print(sh, "  Magnet raw status:  0x%02X", mag_status);
		shell_print(sh, "  Magnet MHI:         %s", mhi ? "SET" : "CLEAR");
		shell_print(sh, "  Magnet MLO:         %s", mlo ? "SET" : "CLEAR");
	} else {
		shell_print(sh, "  Magnet status err:  %d", mag_ret);
	}
#endif

	return 0;
}

/* motor info config */
int cmd_motor_info_config(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	shell_print(sh, "Motor Configuration:");
	shell_print(sh, "  Pole pairs:     %d", MOTOR_POLE_PAIRS);
	shell_print(sh, "  Max current:    %.3f A", (double)MOTOR_MAX_CURRENT_A);
	shell_print(sh, "  Rated voltage:  %.3f V", (double)NOMINAL_VOLTAGE_V);
	shell_print(sh, "  Control freq:   %u Hz", (uint32_t)CONTROL_LOOP_FREQUENCY_HZ);
	shell_print(sh, "  PWM freq:       %u Hz", (uint32_t)PWM_FREQUENCY_HZ);
	shell_print(sh, "  Observer BW:    %.1f Hz", (double)ANGLE_OBSERVER_BANDWIDTH_HZ);
	shell_print(sh, "  PI Id BW:       %.1f Hz", (double)CURRENT_LOOP_BANDWIDTH_HZ);
	shell_print(sh, "  PI Iq BW:       %.1f Hz", (double)CURRENT_LOOP_BANDWIDTH_HZ);
	shell_print(sh, "  Overcurrent:    %.3f A", (double)OVERCURRENT_THRESHOLD_A);
	shell_print(sh, "  Overvoltage:    %.1f V", (double)VBUS_MAX_V);
	
	return 0;
}

/* motor info measured */
int cmd_motor_info_measured(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	shell_print(sh, "Measured Parameters:");
	shell_print(sh, "  Rs:             %.6f Ohm", (double)g_motor_params->Rs_measured_ohm);
	shell_print(sh, "  L:              %.9f H", (double)g_motor_params->Ls_measured_H);
	shell_print(sh, "  R/L:            %.3f rad/s", (double)g_motor_params->R_over_L_measured);
	shell_print(sh, "  Ia offset:      %.6f A", (double)g_motor_params->Ia_offset);
	shell_print(sh, "  Ib offset:      %.6f A", (double)g_motor_params->Ib_offset);
	shell_print(sh, "  Commissioned:   %s", g_motor_params->calibration.commissioning_complete ? "YES" : "NO");
	
	return 0;
}

/* motor info live */
int cmd_motor_info_live(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	int state = motor_api_get_state();
	int error = motor_api_get_error();
	float32_t live_position_rad = g_motor_params->live.position_rad;
	float32_t live_elec_angle_rad = g_motor_params->live.elec_angle_rad;
	float32_t obs_mech_rad = g_motor_params->live.observer_mech_rad;
	float32_t obs_elec_rad = g_motor_params->live.observer_elec_rad;
	float32_t obs_elec_pred_rad = g_motor_params->live.observer_elec_pred_rad;
	float32_t obs_offset_rad = g_motor_params->observer.mech_angle_offset_rad;
	float32_t align_offset_rad = g_motor_params->observer_alignment_offset_rad;
	float32_t enc_observer_input_rad = g_motor_params->live.encoder_observer_input_rad;
	float32_t enc_raw_deg = g_motor_params->live.encoder_raw_deg;
	float32_t enc_raw_rad = g_motor_params->live.encoder_raw_rad;
	
	shell_print(sh, "Live Telemetry:");
	shell_print(sh, "  State:          %s", motor_state_to_string(state));
	shell_print(sh, "  Error:          %s", motor_error_to_string(error));
	shell_print(sh, "  Angle (mech):   %.1f deg",
		    motor_shell_rad_to_deg(live_position_rad));
	shell_print(sh, "  Angle (elec):   %.1f deg",
		    motor_shell_rad_to_deg(live_elec_angle_rad));
	shell_print(sh, "  Obs angle:      mech=%.1f deg elec=%.1f deg pred=%.1f deg",
		    motor_shell_rad_to_deg(obs_mech_rad),
		    motor_shell_rad_to_deg(obs_elec_rad),
		    motor_shell_rad_to_deg(obs_elec_pred_rad));
	shell_print(sh, "  Obs raw rad:    mech=%.6f elec=%.6f pred=%.6f",
		    (double)obs_mech_rad,
		    (double)obs_elec_rad,
		    (double)obs_elec_pred_rad);
	shell_print(sh, "  Obs raw bits:   mech=0x%08X elec=0x%08X pred=0x%08X",
		    motor_shell_f32_bits(obs_mech_rad),
		    motor_shell_f32_bits(obs_elec_rad),
		    motor_shell_f32_bits(obs_elec_pred_rad));
	shell_print(sh, "  Obs offset:     %.3f deg",
		    motor_shell_rad_to_deg(obs_offset_rad));
	shell_print(sh, "  Obs off raw:    %.6f rad bits=0x%08X",
		    (double)obs_offset_rad,
		    motor_shell_f32_bits(obs_offset_rad));
	shell_print(sh, "  Align offset:   %.3f deg",
		    motor_shell_rad_to_deg(align_offset_rad));
	shell_print(sh, "  Enc raw:        %.3f deg (%.6f rad)",
		    (double)enc_raw_deg,
		    (double)enc_raw_rad);
	shell_print(sh, "  Enc dir sign:   %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	shell_print(sh, "  Enc trim:       elec=%.3f deg mech=%.4f deg",
		    (double)(g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32)),
		    (double)((g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32)) /
			     (float32_t)MOTOR_POLE_PAIRS));
	shell_print(sh, "  Enc used:       %.6f rad (%s, fresh=%s)",
		    (double)enc_observer_input_rad,
		    motor_encoder_input_source_to_string(g_motor_params->live.encoder_input_source),
		    g_motor_params->live.encoder_sample_fresh ? "yes" : "no");
	shell_print(sh, "  Enc used bits:  0x%08X", motor_shell_f32_bits(
		    enc_observer_input_rad));
	shell_print(sh, "  Enc flags:      status=0x%02X warn=%s err=%s",
		    g_motor_params->live.encoder_last_status,
		    g_motor_params->live.encoder_sample_warning ? "SET" : "CLEAR",
		    g_motor_params->live.encoder_sample_error ? "SET" : "CLEAR");
	shell_print(sh, "  Enc pipeline:   %s, %s",
		    motor_encoder_pipeline_is_enabled() ? "enabled" : "disabled",
		    motor_encoder_pipeline_is_busy() ? "busy" : "idle");
	shell_print(sh, "  Enc flag count: warn=%u err=%u",
		    g_motor_params->encoder_warning_count,
		    g_motor_params->encoder_error_count);
	shell_print(sh, "  Pos quality:    0x%02X (valid=%s fresh=%s err=%s)",
		    g_motor_params->live.position_quality_flags,
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) ? "yes" : "no",
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_FRESH) ? "yes" : "no",
		    (g_motor_params->live.position_quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) ? "yes" : "no");
	shell_print(sh, "  Pos unwrapped:  %.6f rad", (double)g_motor_params->live.position_unwrapped_rad);
	shell_print(sh, "  Pos innovation: %.6f rad", (double)g_motor_params->live.position_innovation_rad);
	shell_print(sh, "  Pos accel:      %.3f rad/s^2", (double)g_motor_params->live.acceleration_rad_s2);
	shell_print(sh, "  Pos counts:     stale=%u events=%u",
		    g_motor_params->live.position_stale_count,
		    g_motor_params->live.position_stale_events);
	shell_print(sh, "  Speed:          %.3f Hz (%.1f RPM)", 
		    (double)(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32)),
		    (double)(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32) * 60.0f));
	shell_print(sh, "  Id reference:   %.3f A", (double)g_motor_params->live.Id_ref_A);
	shell_print(sh, "  Iq reference:   %.3f A", (double)g_motor_params->live.Iq_ref_A);
	shell_print(sh, "  Id measured:    %.3f A", (double)g_motor_params->live.Id_A);
	shell_print(sh, "  Iq measured:    %.3f A", (double)g_motor_params->live.Iq_A);
	shell_print(sh, "  Ia:             %.3f A", (double)g_motor_params->live.Ia_A);
	shell_print(sh, "  Ib:             %.3f A", (double)g_motor_params->live.Ib_A);
	shell_print(sh, "  Vd:             %.3f V", (double)g_motor_params->Vd_V);
	shell_print(sh, "  Vq:             %.3f V", (double)g_motor_params->Vq_V);
	shell_print(sh, "  Va:             %.3f V", (double)g_motor_params->live.Va_V);
	shell_print(sh, "  Vb:             %.3f V", (double)g_motor_params->live.Vb_V);
	shell_print(sh, "  Vmag(max):      %.3f V", (double)g_motor_params->max_voltage_magnitude_V);
	shell_print(sh, "  Vbus:           %.1f V", (double)g_motor_params->live.dc_bus_voltage_V);
	shell_print(sh, "  Encoder OK:     %s", g_motor_params->encoder_fault_counter == 0 ? "yes" : "no");
	shell_print(sh, "  Encoder faults: %u", g_motor_params->encoder_fault_counter);
	
	return 0;
}

/* motor info stats */
int cmd_motor_info_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	
	uint32_t avg_cycles = 0;
	if (g_motor_params->control_loop_count > 0) {
		avg_cycles = g_motor_params->total_isr_cycles / g_motor_params->control_loop_count;
	}
	
	shell_print(sh, "Performance Statistics:");
	shell_print(sh, "  ISR count:              %u", g_motor_params->control_loop_count);
	shell_print(sh, "  ISR max cycles:         %u", g_motor_params->max_isr_cycles);
	shell_print(sh, "  ISR avg cycles:         %u", avg_cycles);
	shell_print(sh, "  Encoder faults:         %u", g_motor_params->encoder_fault_counter);
	shell_print(sh, "  Encoder warn count:     %u", g_motor_params->encoder_warning_count);
	shell_print(sh, "  Encoder error count:    %u", g_motor_params->encoder_error_count);
	
	return 0;
}

/* motor encoder pipeline */
int cmd_motor_encoder_pipeline(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct motor_encoder_pipeline_stats stats = {0};
	motor_encoder_pipeline_get_stats(&stats);
	enum motor_encoder_test_inject_mode inject_mode =
		motor_encoder_pipeline_get_test_inject_mode();
	const char *inject_label = "none";
	if (inject_mode == MOTOR_ENCODER_TEST_INJECT_STATUS) {
		inject_label = "status";
	} else if (inject_mode == MOTOR_ENCODER_TEST_INJECT_FRAME) {
		inject_label = "frame";
	}

	shell_print(sh, "Encoder pipeline:");
	shell_print(sh, "  State:    %s, %s",
		    motor_encoder_pipeline_is_enabled() ? "enabled" : "disabled",
		    motor_encoder_pipeline_is_busy() ? "busy" : "idle");
	shell_print(sh, "  Inject:   %s", inject_label);
	shell_print(sh, "  Request:  ok=%u busy=%u disabled=%u error=%u",
		    stats.request_ok, stats.request_busy,
		    stats.request_disabled, stats.request_error);
	shell_print(sh, "  Collect:  ok=%u pending=%u empty=%u error=%u",
		    stats.collect_ok, stats.collect_pending,
		    stats.collect_empty, stats.collect_error);
	shell_print(sh, "  Errors:   transport=%u frame=%u parity=%u status=%u",
		    stats.collect_transport_error,
		    stats.collect_frame_error,
		    stats.collect_frame_parity_error,
		    stats.collect_frame_status_error);

	return 0;
}

/* motor encoder pipeline_reset */
int cmd_motor_encoder_pipeline_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_encoder_pipeline_reset_stats();
	shell_print(sh, "Encoder pipeline counters reset");
	return 0;
}

/* motor encoder pipeline_inject [none|status|frame] */
int cmd_motor_encoder_pipeline_inject(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 2U) {
		shell_error(sh, "Usage: motor encoder pipeline_inject [none|status|frame]");
		return -EINVAL;
	}

	enum motor_encoder_test_inject_mode mode = motor_encoder_pipeline_get_test_inject_mode();

	if (argc == 1U) {
		const char *label = "none";

		if (mode == MOTOR_ENCODER_TEST_INJECT_STATUS) {
			label = "status";
		} else if (mode == MOTOR_ENCODER_TEST_INJECT_FRAME) {
			label = "frame";
		}
		shell_print(sh, "Encoder pipeline inject mode: %s", label);
		return 0;
	}

	if (strcmp(argv[1], "none") == 0) {
		mode = MOTOR_ENCODER_TEST_INJECT_NONE;
	} else if (strcmp(argv[1], "status") == 0) {
		mode = MOTOR_ENCODER_TEST_INJECT_STATUS;
	} else if (strcmp(argv[1], "frame") == 0) {
		mode = MOTOR_ENCODER_TEST_INJECT_FRAME;
	} else {
		shell_error(sh, "Invalid mode '%s' (expected none|status|frame)", argv[1]);
		return -EINVAL;
	}

	motor_encoder_pipeline_set_test_inject_mode(mode);
	shell_print(sh, "Encoder pipeline inject mode set: %s", argv[1]);
	return 0;
}

/* motor encoder direction [sign] */
int cmd_motor_encoder_direction(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder direction [<1|-1>]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (argc == 1U) {
		shell_print(sh, "Encoder direction sign: %d (devicetree default: %d)",
			    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1,
			    ENCODER_DIRECTION_SIGN);
		return 0;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing encoder direction sign");
		return -EPERM;
	}

	int state = motor_api_get_state();
	if (state == MOTOR_STATE_ONLINE || motor_state_is_online_submode(state)) {
		shell_error(sh, "Set encoder direction sign while in ONLINE state");
		return -EPERM;
	}

	float parsed = 0.0f;
	if (!shell_parse_finite_float(argv[1], &parsed)) {
		shell_error(sh, "direction sign must be numeric (-1 or 1)");
		return -EINVAL;
	}

	int sign = 0;
	if (fabsf(parsed - 1.0f) < 1.0e-3f) {
		sign = 1;
	} else if (fabsf(parsed + 1.0f) < 1.0e-3f) {
		sign = -1;
	} else {
		shell_error(sh, "direction sign must be -1 or 1");
		return -EINVAL;
	}

	int ret = motor_api_update_param("encoder_direction_sign", (float)sign);
	if (ret != 0) {
		shell_error(sh, "Failed to update encoder direction sign (err %d)", ret);
		return ret;
	}

	shell_print(sh, "Encoder direction sign update posted (%d)", sign);
	return 0;
}

/* motor encoder trim [deg] */
int cmd_motor_encoder_trim(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder trim [<-180.0..180.0>]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	double trim_deg = motor_shell_rad_to_deg(g_motor_params->observer_elec_trim_rad);
	double base_mech_offset_deg =
		motor_shell_rad_to_deg(g_motor_params->observer_alignment_offset_rad);
	double active_mech_offset_deg =
		motor_shell_rad_to_deg(g_motor_params->observer.mech_angle_offset_rad);
	if (argc == 1U) {
		shell_print(sh, "Encoder electrical trim: %.3f deg (mechanical equivalent: %.4f deg)",
			    trim_deg,
			    trim_deg / (double)MOTOR_POLE_PAIRS);
		shell_print(sh, "Observer base offset (ALIGN): %.3f deg mechanical",
			    base_mech_offset_deg);
		shell_print(sh, "Observer active offset: %.3f deg mechanical",
			    active_mech_offset_deg);
		shell_print(sh, "Observer raw: align=0x%08X active=0x%08X trim=0x%08X pos=0x%08X",
		    motor_shell_f32_bits(g_motor_params->observer_alignment_offset_rad),
		    motor_shell_f32_bits(g_motor_params->observer.mech_angle_offset_rad),
		    motor_shell_f32_bits(g_motor_params->observer_elec_trim_rad),
		    motor_shell_f32_bits(g_motor_params->calibration.align_mech_angle_rad));
		return 0;
	}

	float requested_trim_deg = 0.0f;
	if (!shell_parse_finite_float(argv[1], &requested_trim_deg)) {
		shell_error(sh, "trim must be a finite number of electrical degrees");
		return -EINVAL;
	}
	if (requested_trim_deg < -180.0f || requested_trim_deg > 180.0f) {
		shell_error(sh, "trim must be within [-180.0, 180.0] electrical degrees");
		return -EINVAL;
	}

	int ret = motor_api_update_param("observer_elec_trim_deg", requested_trim_deg);
	if (ret != 0) {
		shell_error(sh, "Failed to update observer electrical trim (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Observer electrical trim update posted: %.3f deg",
		    (double)requested_trim_deg);
	return 0;
}

/* motor encoder alarm */
int cmd_motor_encoder_alarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955
	shell_error(sh, "encoder1 is not AEAT-9955 on this build");
	return -ENOTSUP;
#else
	uint8_t status = 0U;
	bool mhi = false;
	bool mlo = false;
	int ret = motor_encoder_read_aeat_alarm(&status, &mhi, &mlo);
	if (ret < 0) {
		shell_error(sh, "Failed to read AEAT alarm status (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 alarm/error status:");
	shell_print(sh, "  Raw status: 0x%02X", status);
	shell_print(sh, "  MHI:        %s", mhi ? "SET" : "CLEAR");
	shell_print(sh, "  MLO:        %s", mlo ? "SET" : "CLEAR");

	return 0;
#endif
}

/* motor encoder fast */
int cmd_motor_encoder_fast(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "encoder1 is not a fast encoder_rt device on this build");
	return -ENOTSUP;
#else
	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}

	struct encoder_rt_stats stats = {0};
	encoder_rt_get_stats(encoder1, &stats);

	shell_print(sh, "Fast Encoder:");
	shell_print(sh, "  Device:          %s", encoder1->name);
	shell_print(sh, "  Pipeline delay:  %u samples", encoder_rt_get_pipeline_delay(encoder1));
	shell_print(sh, "  Request:         ok=%u busy=%u disabled=%u error=%u",
		    stats.request_count, stats.busy_count,
		    stats.disabled_count, stats.request_error_count);
	shell_print(sh, "  Collect:         ok=%u pending=%u empty=%u error=%u",
		    stats.collect_count, stats.pending_count,
		    stats.empty_count, stats.collect_error_count);
	shell_print(sh, "  Errors:          transport=%u frame=%u parity=%u status=%u warning=%u",
		    stats.transport_error_count,
		    stats.frame_error_count,
		    stats.frame_parity_error_count,
		    stats.frame_status_error_count,
		    stats.warning_count);

	return 0;
#endif
}

/* motor encoder reg_read <addr> */
int cmd_motor_encoder_reg_read(const struct shell *sh, size_t argc, char **argv)
{
#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "register reads require the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (argc != 2U) {
		shell_error(sh, "Usage: motor encoder reg_read <addr>");
		return -EINVAL;
	}
	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}
	if (motor_encoder_pipeline_is_enabled() || motor_encoder_pipeline_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before register reads");
		return -EBUSY;
	}

	errno = 0;
	char *endp = NULL;
	unsigned long reg = strtoul(argv[1], &endp, 0);
	if (endp == argv[1] || *endp != '\0' || errno == ERANGE || reg > UINT8_MAX) {
		shell_error(sh, "addr must be an 8-bit register address");
		return -EINVAL;
	}

	uint8_t value = 0U;
	int ret = aeat9955_fast_read_register(encoder1, (uint8_t)reg, &value);
	if (ret != 0) {
		shell_error(sh, "Failed to read AEAT register 0x%02X (err %d)",
			    (unsigned int)reg, ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 register 0x%02X = 0x%02X",
		    (unsigned int)reg, value);
	return 0;
#endif
}

/* motor encoder capture start [decimation] */
int cmd_motor_encoder_capture_start(const struct shell *sh, size_t argc, char **argv)
{
#if !IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_CAPTURE)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "Encoder capture telemetry is not compiled in");
	return -ENOTSUP;
#else
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder capture start [decimation]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t decimation = 1U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &decimation) || decimation == 0U ||
		    decimation > UINT16_MAX) {
			shell_error(sh, "decimation must be in [1, %u]", UINT16_MAX);
			return -EINVAL;
		}
	}

	g_motor_params->encoder_capture.decimation = (uint16_t)decimation;
	motor_encoder_capture_reset(g_motor_params, false);
	g_motor_params->encoder_capture.enabled = true;

	shell_print(sh,
		    "Encoder capture started: decimation=%u, capacity=%u samples",
		    g_motor_params->encoder_capture.decimation,
		    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	return 0;
#endif
}

/* motor encoder capture stop */
int cmd_motor_encoder_capture_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_capture.enabled = false;
	shell_print(sh, "Encoder capture stopped: stored=%u overrun=%u",
		    g_motor_params->encoder_capture.count,
		    g_motor_params->encoder_capture.overrun_count);
	return 0;
}

/* motor encoder capture clear */
int cmd_motor_encoder_capture_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_capture.enabled = false;
	motor_encoder_capture_reset(g_motor_params, true);
	shell_print(sh, "Encoder capture cleared");
	return 0;
}

/* motor encoder trace start [decimation] */
int cmd_motor_encoder_trace_start(const struct shell *sh, size_t argc, char **argv)
{
#if !IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_RAW_TRACE)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "Encoder raw trace telemetry is not compiled in");
	return -ENOTSUP;
#else
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder trace start [decimation]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t decimation = 1U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &decimation) || decimation == 0U ||
		    decimation > UINT16_MAX) {
			shell_error(sh, "decimation must be in [1, %u]", UINT16_MAX);
			return -EINVAL;
		}
	}

	g_motor_params->encoder_raw_trace.decimation = (uint16_t)decimation;
	motor_encoder_raw_trace_reset(g_motor_params, false);
	g_motor_params->encoder_raw_trace.enabled = true;

	shell_print(sh, "Encoder raw trace started: decimation=%u, capacity=%u samples",
		    g_motor_params->encoder_raw_trace.decimation,
		    MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	return 0;
#endif
}

/* motor encoder trace stop */
int cmd_motor_encoder_trace_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_raw_trace.enabled = false;
	shell_print(sh, "Encoder raw trace stopped: stored=%u overrun=%u",
		    g_motor_params->encoder_raw_trace.count,
		    g_motor_params->encoder_raw_trace.overrun_count);
	return 0;
}

/* motor encoder trace clear */
int cmd_motor_encoder_trace_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->encoder_raw_trace.enabled = false;
	motor_encoder_raw_trace_reset(g_motor_params, true);
	shell_print(sh, "Encoder raw trace cleared");
	return 0;
}

/* motor encoder trace status */
int cmd_motor_encoder_trace_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Encoder raw trace:");
	shell_print(sh, "  Compiled:   %s",
		    IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_RAW_TRACE) ? "YES" : "NO");
	shell_print(sh, "  Enabled:    %s", g_motor_params->encoder_raw_trace.enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->encoder_raw_trace.decimation);
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->encoder_raw_trace.count,
		    MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->encoder_raw_trace.overrun_count);

	if (g_motor_params->encoder_raw_trace.count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->encoder_raw_trace.write_idx +
						  MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES - 1U) %
						 MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
		const struct motor_encoder_raw_trace_sample *newest =
			&g_motor_params->encoder_raw_trace.samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u src=%s raw_deg=%.3f ctrl_deg=%.3f q=0x%02X fresh=%u warn=%u err=%u io=%u status=0x%02X",
			    newest->control_loop_count,
			    motor_encoder_input_source_to_string(newest->input_source),
			    (double)newest->raw_angle_deg,
			    (double)newest->control_angle_deg,
			    newest->quality_flags,
			    newest->sample_fresh,
			    newest->sample_warning,
			    newest->sample_error,
			    newest->sample_io_fault,
			    newest->status);
	}

	return 0;
}

/* motor encoder trace summary */
int cmd_motor_encoder_trace_summary(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_raw_trace.count;
	if (stored == 0U) {
		shell_print(sh, "No raw trace samples");
		return 0;
	}

	uint16_t oldest_idx = motor_encoder_ring_oldest(g_motor_params->encoder_raw_trace.write_idx,
							stored,
							MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	const struct motor_encoder_raw_trace_sample *first = NULL;
	const struct motor_encoder_raw_trace_sample *prev = NULL;
	const struct motor_encoder_raw_trace_sample *last = NULL;
	float32_t raw_delta_rad = 0.0f;
	float32_t ctrl_delta_rad = 0.0f;
	float32_t min_raw_deg = 0.0f;
	float32_t max_raw_deg = 0.0f;
	uint32_t clean_count = 0U;
	uint32_t fresh_count = 0U;
	uint32_t warn_count = 0U;
	uint32_t err_count = 0U;
	uint32_t io_count = 0U;
	uint32_t enabled_count = 0U;
	uint8_t status_or = 0U;
	uint8_t status_and = 0xFFU;

	for (uint16_t i = 0U; i < stored; i++) {
		uint16_t idx = motor_encoder_ring_index(oldest_idx, i,
							MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
		const struct motor_encoder_raw_trace_sample *sample =
			&g_motor_params->encoder_raw_trace.samples[idx];

		fresh_count += sample->sample_fresh ? 1U : 0U;
		warn_count += sample->sample_warning ? 1U : 0U;
		err_count += sample->sample_error ? 1U : 0U;
		io_count += sample->sample_io_fault ? 1U : 0U;
		enabled_count += sample->sample_enabled ? 1U : 0U;
		status_or |= sample->status;
		status_and &= sample->status;

		if (!motor_encoder_raw_trace_sample_clean(sample)) {
			continue;
		}

		if (first == NULL) {
			first = sample;
			min_raw_deg = sample->raw_angle_deg;
			max_raw_deg = sample->raw_angle_deg;
		} else {
			raw_delta_rad += wrap_rad_pi(sample->raw_angle_rad - prev->raw_angle_rad);
			ctrl_delta_rad += wrap_rad_pi(sample->control_angle_rad -
						      prev->control_angle_rad);
			min_raw_deg = MIN(min_raw_deg, sample->raw_angle_deg);
			max_raw_deg = MAX(max_raw_deg, sample->raw_angle_deg);
		}
		clean_count++;
		prev = sample;
		last = sample;
	}

	shell_print(sh, "Encoder raw trace summary:");
	shell_print(sh, "  Stored:       %u / %u", stored, MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
	shell_print(sh, "  Decimation:   %u", g_motor_params->encoder_raw_trace.decimation);
	shell_print(sh, "  Overrun:      %u", g_motor_params->encoder_raw_trace.overrun_count);
	if (clean_count > 0U) {
		shell_print(sh, "  Clean span:   %u -> %u (%u ticks, %u samples)",
			    first->control_loop_count, last->control_loop_count,
			    last->control_loop_count - first->control_loop_count,
			    clean_count);
		shell_print(sh, "  Raw first/last: %.3f -> %.3f deg",
			    (double)first->raw_angle_deg, (double)last->raw_angle_deg);
		shell_print(sh, "  Raw delta:    %.3f deg, avg %.4f Hz",
			    (double)(raw_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(raw_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Ctrl delta:   %.3f deg, avg %.4f Hz",
			    (double)(ctrl_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(ctrl_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Raw min/max:  %.3f / %.3f deg",
			    (double)min_raw_deg, (double)max_raw_deg);
	} else {
		shell_print(sh, "  Clean span:   none");
	}
	shell_print(sh, "  Counts:       clean=%u fresh=%u ctrl_en=%u warn=%u err=%u io=%u",
		    clean_count, fresh_count, enabled_count, warn_count, err_count, io_count);
	shell_print(sh, "  Status bits:  or=0x%02X and=0x%02X first=0x%02X last=0x%02X",
		    status_or, status_and,
		    g_motor_params->encoder_raw_trace.samples[oldest_idx].status,
		    g_motor_params->encoder_raw_trace.samples[
			    motor_encoder_ring_index(oldest_idx, stored - 1U,
						     MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES)].status);
	return 0;
}

/* motor encoder trace dump [count] | dump <offset> <count> */
int cmd_motor_encoder_trace_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_raw_trace.count;
	if (stored == 0U) {
		shell_print(sh, "No raw trace samples");
		return 0;
	}

	uint16_t start = 0U;
	uint16_t count = 0U;
	int ret = motor_encoder_parse_dump_window(sh, argc, argv, stored,
						  MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES,
						  g_motor_params->encoder_raw_trace.write_idx,
						  &start, &count);
	if (ret < 0) {
		return ret;
	}

	if (g_motor_params->encoder_raw_trace.enabled) {
		shell_warn(sh,
			   "Raw trace is still running; dump may include concurrently updated samples.");
	}

	shell_print(sh, "Raw trace dump: stored=%u count=%u max_chunk=%u",
		    stored, count, MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
	shell_print(sh,
		    "idx loop src raw_deg raw_rad ctrl_deg ctrl_rad obs_in_rad q fresh warn err io status ctrl_en");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES);
		const struct motor_encoder_raw_trace_sample *sample =
			&g_motor_params->encoder_raw_trace.samples[idx];
		shell_print(sh,
			    "%u %u %s %.3f %.6f %.3f %.6f %.6f 0x%02X %u %u %u %u 0x%02X %u",
			    i,
			    sample->control_loop_count,
			    motor_encoder_input_source_to_string(sample->input_source),
			    (double)sample->raw_angle_deg,
			    (double)sample->raw_angle_rad,
			    (double)sample->control_angle_deg,
			    (double)sample->control_angle_rad,
			    (double)sample->observer_input_rad,
			    sample->quality_flags,
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    sample->sample_io_fault,
			    sample->status,
			    sample->sample_enabled);
	}

	return 0;
}

/* motor encoder capture status */
int cmd_motor_encoder_capture_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Encoder capture:");
	shell_print(sh, "  Compiled:   %s",
		    IS_ENABLED(CONFIG_MOTOR_ISR_ENCODER_CAPTURE) ? "YES" : "NO");
	shell_print(sh, "  Enabled:    %s", g_motor_params->encoder_capture.enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->encoder_capture.decimation);
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->encoder_capture.count,
		    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->encoder_capture.overrun_count);

	if (g_motor_params->encoder_capture.count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->encoder_capture.write_idx +
						  MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - 1U) %
						 MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *newest =
			&g_motor_params->encoder_capture.samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u src=%s deg=%.3f fresh=%u warn=%u err=%u status=0x%02X ctrl_en=%u",
			    newest->control_loop_count,
			    motor_encoder_input_source_to_string(newest->input_source),
			    (double)newest->angle_deg,
			    newest->sample_fresh,
			    newest->sample_warning,
			    newest->sample_error,
			    newest->status,
			    newest->sample_enabled);
		if (newest->compare_valid) {
			shell_print(sh,
				    "  Compare:    gen_mech=%.3fdeg enc_mech=%.3fdeg d_mech=%.3fdeg d_elec=%.3fdeg",
				    (double)(newest->generated_mech_rad * (180.0f / PI_F32)),
				    (double)(newest->encoder_mech_rad * (180.0f / PI_F32)),
				    (double)(newest->mech_error_rad * (180.0f / PI_F32)),
				    (double)(newest->elec_error_rad * (180.0f / PI_F32)));
		} else {
			shell_print(sh, "  Compare:    unavailable (needs fresh clean encoder sample)");
		}
	}

	return 0;
}

/* motor encoder capture summary */
int cmd_motor_encoder_capture_summary(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_capture.count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t oldest_idx = motor_encoder_ring_oldest(g_motor_params->encoder_capture.write_idx,
							stored,
							MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	const struct motor_encoder_capture_sample *first = NULL;
	const struct motor_encoder_capture_sample *prev = NULL;
	const struct motor_encoder_capture_sample *last = NULL;
	float32_t angle_delta_rad = 0.0f;
	float32_t encoder_delta_rad = 0.0f;
	float32_t generated_delta_rad = 0.0f;
	float32_t min_angle_deg = 0.0f;
	float32_t max_angle_deg = 0.0f;
	uint32_t clean_count = 0U;
	uint32_t fresh_count = 0U;
	uint32_t warn_count = 0U;
	uint32_t err_count = 0U;
	uint32_t enabled_count = 0U;
	uint32_t compare_count = 0U;
	uint8_t status_or = 0U;
	uint8_t status_and = 0xFFU;

	for (uint16_t i = 0U; i < stored; i++) {
		uint16_t idx = motor_encoder_ring_index(oldest_idx, i,
							MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *sample =
			&g_motor_params->encoder_capture.samples[idx];

		fresh_count += sample->sample_fresh ? 1U : 0U;
		warn_count += sample->sample_warning ? 1U : 0U;
		err_count += sample->sample_error ? 1U : 0U;
		enabled_count += sample->sample_enabled ? 1U : 0U;
		compare_count += sample->compare_valid ? 1U : 0U;
		status_or |= sample->status;
		status_and &= sample->status;

		if (!motor_encoder_capture_sample_clean(sample)) {
			continue;
		}

		if (first == NULL) {
			first = sample;
			min_angle_deg = sample->angle_deg;
			max_angle_deg = sample->angle_deg;
		} else {
			angle_delta_rad += wrap_rad_pi(sample->angle_rad - prev->angle_rad);
			encoder_delta_rad += wrap_rad_pi(sample->encoder_mech_rad -
							 prev->encoder_mech_rad);
			generated_delta_rad += wrap_rad_pi(sample->generated_mech_rad -
							   prev->generated_mech_rad);
			min_angle_deg = MIN(min_angle_deg, sample->angle_deg);
			max_angle_deg = MAX(max_angle_deg, sample->angle_deg);
		}
		clean_count++;
		prev = sample;
		last = sample;
	}

	shell_print(sh, "Encoder capture summary:");
	shell_print(sh, "  Stored:       %u / %u", stored, MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	shell_print(sh, "  Decimation:   %u", g_motor_params->encoder_capture.decimation);
	shell_print(sh, "  Overrun:      %u", g_motor_params->encoder_capture.overrun_count);
	if (clean_count > 0U) {
		shell_print(sh, "  Clean span:   %u -> %u (%u ticks, %u samples)",
			    first->control_loop_count, last->control_loop_count,
			    last->control_loop_count - first->control_loop_count,
			    clean_count);
		shell_print(sh, "  Angle first/last: %.3f -> %.3f deg",
			    (double)first->angle_deg, (double)last->angle_deg);
		shell_print(sh, "  Angle delta:  %.3f deg, avg %.4f Hz",
			    (double)(angle_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(angle_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Enc delta:    %.3f deg, avg %.4f Hz",
			    (double)(encoder_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(encoder_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Gen delta:    %.3f deg, avg %.4f Hz",
			    (double)(generated_delta_rad * 180.0f / PI_F32),
			    (double)motor_encoder_avg_velocity_hz(generated_delta_rad,
								  first->control_loop_count,
								  last->control_loop_count));
		shell_print(sh, "  Angle min/max: %.3f / %.3f deg",
			    (double)min_angle_deg, (double)max_angle_deg);
	} else {
		shell_print(sh, "  Clean span:   none");
	}
	shell_print(sh, "  Counts:       clean=%u fresh=%u ctrl_en=%u warn=%u err=%u compare=%u",
		    clean_count, fresh_count, enabled_count, warn_count, err_count, compare_count);
	shell_print(sh, "  Status bits:  or=0x%02X and=0x%02X first=0x%02X last=0x%02X",
		    status_or, status_and,
		    g_motor_params->encoder_capture.samples[oldest_idx].status,
		    g_motor_params->encoder_capture.samples[
			    motor_encoder_ring_index(oldest_idx, stored - 1U,
						     MOTOR_ENCODER_CAPTURE_MAX_SAMPLES)].status);
	return 0;
}

/* motor encoder capture dump [count] | dump <offset> <count> */
int cmd_motor_encoder_capture_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint16_t stored = g_motor_params->encoder_capture.count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t start = 0U;
	uint16_t count = 0U;
	int ret = motor_encoder_parse_dump_window(sh, argc, argv, stored,
						  MOTOR_ENCODER_CAPTURE_MAX_SAMPLES,
						  g_motor_params->encoder_capture.write_idx,
						  &start, &count);
	if (ret < 0) {
		return ret;
	}

	if (g_motor_params->encoder_capture.enabled) {
		shell_warn(sh,
			   "Capture is still running; dump may include concurrently updated samples.");
	}

	shell_print(sh, "Capture dump: stored=%u count=%u max_chunk=%u",
		    stored, count, MOTOR_ENCODER_SHELL_DUMP_MAX_ROWS);
	shell_print(sh,
		    "idx loop source deg rad enc_rad obs_rad norm q31 fresh warn err status ctrl_en");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		struct motor_encoder_capture_sample sample =
			g_motor_params->encoder_capture.samples[idx];
		float32_t norm = motor_encoder_normalized_from_rad(sample.angle_rad);
		int32_t q31 = motor_encoder_q31_from_rad(sample.angle_rad);
		shell_print(sh,
			    "%u %u %s %.3f %.6f %.6f %.6f %.6f %d %u %u %u 0x%02X %u",
			    i,
			    sample.control_loop_count,
			    motor_encoder_input_source_to_string(sample.input_source),
			    (double)sample.angle_deg,
			    (double)sample.angle_rad,
			    (double)sample.encoder_mech_rad,
			    (double)sample.observer_mech_rad,
			    (double)norm,
			    q31,
			    sample.sample_fresh,
			    sample.sample_warning,
			    sample.sample_error,
			    sample.status,
			    sample.sample_enabled);
	}

	return 0;
}

/* motor encoder capture compare [count] [gen|obs] */
int cmd_motor_encoder_capture_compare(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 1U || argc > 3U) {
		shell_error(sh, "Usage: motor encoder capture compare [count] [gen|obs]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t requested = 32U;
	uint8_t ref_mode = MOTOR_ENCODER_COMPARE_REF_GENERATED;
	if (argc == 2U) {
		if (shell_parse_u32(argv[1], &requested)) {
			if (requested == 0U || requested > MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
				shell_error(sh, "count must be in [1, %u]",
					    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
				return -EINVAL;
			}
		} else if (!motor_encoder_compare_ref_parse(argv[1], &ref_mode)) {
			shell_error(sh, "Invalid ref '%s' (expected gen|obs)", argv[1]);
			return -EINVAL;
		}
	}
	if (argc == 3U) {
		if (!shell_parse_u32(argv[1], &requested) || requested == 0U ||
		    requested > MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
			shell_error(sh, "count must be in [1, %u]",
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
			return -EINVAL;
		}
		if (!motor_encoder_compare_ref_parse(argv[2], &ref_mode)) {
			shell_error(sh, "Invalid ref '%s' (expected gen|obs)", argv[2]);
			return -EINVAL;
		}
	}

	uint16_t stored = g_motor_params->encoder_capture.count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->encoder_capture.write_idx +
				     MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - count) %
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);

	if (g_motor_params->encoder_capture.enabled) {
		shell_warn(sh,
			   "Capture is still running; compare dump may include concurrently updated samples.");
	}

	shell_print(sh, "idx loop src fresh warn err cmp ref");
	shell_print(sh, "  mech: enc_deg ref_deg d_deg");
	shell_print(sh, "  elec: enc_deg ref_deg d_deg rel_phase_deg");
	bool rel_phase_init = false;
	float32_t rel_phase_base_rad = 0.0f;
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *sample =
			&g_motor_params->encoder_capture.samples[idx];
		float32_t enc_m_deg = sample->encoder_mech_rad * (180.0f / PI_F32);
		float32_t enc_e_deg = sample->encoder_elec_rad * (180.0f / PI_F32);
		float32_t ref_mech_rad =
			(ref_mode == MOTOR_ENCODER_COMPARE_REF_OBSERVER) ?
				sample->observer_mech_rad : sample->generated_mech_rad;
		float32_t ref_elec_rad =
			(ref_mode == MOTOR_ENCODER_COMPARE_REF_OBSERVER) ?
				sample->observer_elec_rad : sample->generated_elec_rad;
		float32_t ref_m_deg = ref_mech_rad * (180.0f / PI_F32);
		float32_t ref_e_deg = ref_elec_rad * (180.0f / PI_F32);
		float32_t d_m_deg = 0.0f;
		float32_t d_e_deg = 0.0f;
		float32_t rel_phase_deg = 0.0f;
		bool compare_valid =
			sample->compare_valid &&
			(sample->encoder_mech_rad == sample->encoder_mech_rad) &&
			fabsf(sample->encoder_mech_rad) <= 1.0e6f &&
			(ref_mech_rad == ref_mech_rad) &&
			fabsf(ref_mech_rad) <= 1.0e6f &&
			(sample->encoder_elec_rad == sample->encoder_elec_rad) &&
			fabsf(sample->encoder_elec_rad) <= 1.0e6f &&
			(ref_elec_rad == ref_elec_rad) &&
			fabsf(ref_elec_rad) <= 1.0e6f;
		if (compare_valid) {
			float32_t mech_error_rad =
				wrap_rad_pi(sample->encoder_mech_rad - ref_mech_rad);
			float32_t elec_error_rad =
				wrap_rad_pi(sample->encoder_elec_rad - ref_elec_rad);
			d_m_deg = mech_error_rad * (180.0f / PI_F32);
			d_e_deg = elec_error_rad * (180.0f / PI_F32);
			if (!rel_phase_init) {
				rel_phase_base_rad = elec_error_rad;
				rel_phase_init = true;
				rel_phase_deg = 0.0f;
			} else {
				rel_phase_deg =
					wrap_rad_pi(elec_error_rad - rel_phase_base_rad) *
					(180.0f / PI_F32);
			}
		}
		shell_print(sh,
			    "%u %u %s %u %u %u %u %s",
			    i,
			    sample->control_loop_count,
			    motor_encoder_input_source_to_string(sample->input_source),
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    compare_valid ? 1U : 0U,
			    motor_encoder_compare_ref_to_string(ref_mode));
		shell_print(sh,
			    "  mech: %.3f %.3f %.3f",
			    (double)enc_m_deg,
			    (double)ref_m_deg,
			    (double)d_m_deg);
		shell_print(sh,
			    "  elec: %.3f %.3f %.3f %.3f",
			    (double)enc_e_deg,
			    (double)ref_e_deg,
			    (double)d_e_deg,
			    (double)rel_phase_deg);
	}

	return 0;
}

/* motor fault snapshot start [decimation] */
int cmd_motor_fault_snapshot_start(const struct shell *sh, size_t argc, char **argv)
{
#if !IS_ENABLED(CONFIG_MOTOR_ISR_FAULT_SNAPSHOT)
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "Fault snapshot telemetry is not compiled in");
	return -ENOTSUP;
#else
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor fault snapshot start [decimation]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t decimation = 1U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &decimation) || decimation == 0U ||
		    decimation > UINT16_MAX) {
			shell_error(sh, "decimation must be in [1, %u]", UINT16_MAX);
			return -EINVAL;
		}
	}

	motor_fault_snapshot_reset(g_motor_params, false);
	g_motor_params->fault_snapshot.decimation = (uint16_t)decimation;
	g_motor_params->fault_snapshot.enabled = true;
	shell_print(sh,
		    "Fault snapshot started: decimation=%u, capacity=%u samples",
		    g_motor_params->fault_snapshot.decimation,
		    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	return 0;
#endif
}

/* motor fault snapshot stop */
int cmd_motor_fault_snapshot_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->fault_snapshot.enabled = false;
	shell_print(sh, "Fault snapshot stopped: stored=%u overrun=%u",
		    g_motor_params->fault_snapshot.count,
		    g_motor_params->fault_snapshot.overrun_count);
	return 0;
}

/* motor fault snapshot status */
int cmd_motor_fault_snapshot_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Fault snapshot:");
	shell_print(sh, "  Compiled:   %s",
		    IS_ENABLED(CONFIG_MOTOR_ISR_FAULT_SNAPSHOT) ? "YES" : "NO");
	shell_print(sh, "  Enabled:    %s", g_motor_params->fault_snapshot.enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->fault_snapshot.decimation);
	shell_print(sh, "  Latched:    %s", g_motor_params->fault_snapshot.latched ? "YES" : "NO");
	if (g_motor_params->fault_snapshot.latched) {
		shell_print(sh, "  Fault:      %s (%u)",
			    motor_error_to_string((int)g_motor_params->fault_snapshot.latch_error_code),
			    g_motor_params->fault_snapshot.latch_error_code);
		shell_print(sh, "  Fault loop: %u", g_motor_params->fault_snapshot.latch_loop);
	}
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->fault_snapshot.count,
		    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->fault_snapshot.overrun_count);

	if (g_motor_params->fault_snapshot.count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->fault_snapshot.write_idx +
						  MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES - 1U) %
						 MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
		const struct motor_fault_snapshot_sample *newest =
			&g_motor_params->fault_snapshot.samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u src=%s enc=%.3fdeg iq_ref=%.3fA iq=%.3fA ia=%.3fA ib=%.3fA fresh=%u warn=%u err=%u status=0x%02X",
			    newest->control_loop_count,
			    motor_encoder_input_source_to_string(newest->input_source),
			    (double)newest->encoder_angle_deg,
			    (double)newest->Iq_ref_A,
			    (double)newest->Iq_A,
			    (double)newest->Ia_A,
			    (double)newest->Ib_A,
			    newest->sample_fresh,
			    newest->sample_warning,
			    newest->sample_error,
			    newest->status);
	}

	return 0;
}

/* motor fault snapshot dump [count] */
int cmd_motor_fault_snapshot_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor fault snapshot dump [count]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t requested = 32U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &requested) || requested == 0U ||
		    requested > MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES) {
			shell_error(sh, "count must be in [1, %u]",
				    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
			return -EINVAL;
		}
	}

	uint16_t stored = g_motor_params->fault_snapshot.count;
	if (stored == 0U) {
		shell_print(sh, "No fault snapshot samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->fault_snapshot.write_idx +
				     MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES - count) %
				    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);

	if (motor_control_is_armed(g_motor_params)) {
		shell_warn(sh,
			   "Control is armed; snapshot dump may include concurrently updated samples.");
	}

	shell_print(sh,
		    "idx loop src fresh warn err status pqual enc_deg obs_in elec obs_we id_ref iq_ref id iq ia ib vd vq");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
		const struct motor_fault_snapshot_sample *sample =
			&g_motor_params->fault_snapshot.samples[idx];
		shell_print(sh,
			    "%u %u %s %u %u %u 0x%02X 0x%02X %.3f %.6f %.6f %.6f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
			    i,
			    sample->control_loop_count,
			    motor_encoder_input_source_to_string(sample->input_source),
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    sample->status,
			    sample->position_quality_flags,
			    (double)sample->encoder_angle_deg,
			    (double)sample->observer_input_rad,
			    (double)sample->elec_angle_rad,
			    (double)sample->observer_elec_speed_rad_s,
			    (double)sample->Id_ref_A,
			    (double)sample->Iq_ref_A,
			    (double)sample->Id_A,
			    (double)sample->Iq_A,
			    (double)sample->Ia_A,
			    (double)sample->Ib_A,
			    (double)sample->Vd_V,
			    (double)sample->Vq_V);
	}

	return 0;
}

/* motor fault snapshot clear */
int cmd_motor_fault_snapshot_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	g_motor_params->fault_snapshot.enabled = false;
	motor_fault_snapshot_reset(g_motor_params, true);
	shell_print(sh, "Fault snapshot cleared");
	return 0;
}
