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
#include "motor_autonomy.h"
#include "motor_hardware.h"
#include "motor_encoder_pipeline.h"
#include "config.h"
#include "angle_wrap.h"
#include "shell_parse.h"

#if DT_NODE_EXISTS(DT_ALIAS(encoder1)) && DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955)
#include <drivers/sensor/brcm_aeat9955.h>
#define MOTOR_ENCODER_IS_AEAT9955 1
#else
#define MOTOR_ENCODER_IS_AEAT9955 0
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

static inline bool motor_state_allows_arm(int state)
{
	return state == MOTOR_STATE_IDLE ||
	       state == MOTOR_STATE_OFFLINE ||
	       state == MOTOR_STATE_ONLINE ||
	       motor_state_is_online_submode(state);
}

static inline bool motor_state_is_align_phase(int state)
{
	return state == MOTOR_STATE_ALIGN ||
	       state == MOTOR_STATE_ALIGN_POS_INJECT ||
	       state == MOTOR_STATE_ALIGN_POS_SAMPLE ||
	       state == MOTOR_STATE_ALIGN_NEG_INJECT ||
	       state == MOTOR_STATE_ALIGN_NEG_SAMPLE;
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

#define MOTOR_ENCODER_COMPARE_REF_GENERATED 0U
#define MOTOR_ENCODER_COMPARE_REF_OBSERVER 1U

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

static inline float motor_encoder_normalized_from_rad(float32_t angle_rad)
{
	float32_t wrapped = wrap_rad_2pi(angle_rad);

	return wrapped / (2.0f * PI_F32);
}

static inline int32_t motor_encoder_q31_from_rad(float32_t angle_rad)
{
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

static inline void motor_zero_control_targets(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	motion_profile_quintic_cancel(&params->position_profile, params->position_rad);
	params->position_target_rad = wrap_rad_2pi(params->position_rad);
}

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
}

static void motor_encoder_capture_reset(struct motor_parameters *params, bool clear_samples)
{
	if (params == NULL) {
		return;
	}

	params->encoder_capture_phase = 0U;
	params->encoder_capture_write_idx = 0U;
	params->encoder_capture_count = 0U;
	params->encoder_capture_overrun_count = 0U;
	if (clear_samples) {
		memset(params->encoder_capture_samples, 0, sizeof(params->encoder_capture_samples));
	}
}

static void motor_fault_snapshot_reset(struct motor_parameters *params, bool clear_samples)
{
	if (params == NULL) {
		return;
	}

	params->fault_snapshot_write_idx = 0U;
	params->fault_snapshot_count = 0U;
	params->fault_snapshot_overrun_count = 0U;
	params->fault_snapshot_latched = 0U;
	params->fault_snapshot_latch_loop = 0U;
	params->fault_snapshot_latch_error_code = ERROR_NONE;
	if (clear_samples) {
		memset(params->fault_snapshot_samples, 0, sizeof(params->fault_snapshot_samples));
	}
}

/* motor state offline */
int cmd_motor_state_offline(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_api_request_offline() == 0) {
		shell_print(sh, "OFFLINE state requested");
		return 0;
	} else {
		shell_error(sh, "Failed to request OFFLINE state");
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
		shell_error(sh, "Cannot arm while in state %s. Wait for IDLE/OFFLINE/ONLINE.",
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
	uint32_t now_ms = k_uptime_get_32();
	uint32_t age_ms = now_ms - g_motor_params->last_command_update_ms;
	bool control_armed = motor_control_is_armed(g_motor_params);
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_OPEN) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_CLOSED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION);
	bool autonomous_keepalive =
		motor_autonomy_should_keepalive(control_armed, autonomous_mode_active,
						 g_motor_params->profile_sequence_running,
						 g_motor_params->chopper_cal_active,
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
	shell_print(sh, "  Cal complete: %s", g_motor_params->calibration_complete ? "YES" : "NO");
	shell_print(sh, "  Cal running:  %s", g_motor_params->calibration_running ? "YES" : "NO");
	shell_print(sh, "  Cal mode:     %s",
		    motor_calibration_mode_to_string(g_motor_params->calibration_mode));
	shell_print(sh, "  Commissioned: %s",
		    g_motor_params->commissioning_complete ? "YES" : "NO");
	shell_print(sh, "  Online mode:  %s",
		    motor_state_to_string(g_motor_params->requested_online_mode));
	shell_print(sh, "  Enc dir sign: %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	if (g_motor_params->calibration_running || motor_state_is_align_phase(state)) {
		shell_print(sh, "  Align phase:  %s", state_str);
		shell_print(sh,
			    "  Align +Id:   samples=%u retries=%u mean=%.2f deg",
			    g_motor_params->align_pos_sample_count,
			    g_motor_params->align_pos_sample_retries,
			    (double)(g_motor_params->align_pos_mech_angle_rad *
				     (180.0f / PI_F32)));
		shell_print(sh,
			    "  Align -Id:   samples=%u retries=%u mean=%.2f deg",
			    g_motor_params->align_neg_sample_count,
			    g_motor_params->align_neg_sample_retries,
			    (double)(g_motor_params->align_neg_mech_angle_rad *
				     (180.0f / PI_F32)));
		shell_print(sh,
			    "  Align offset: %.2f deg",
			    (double)(g_motor_params->observer_alignment_offset_rad *
				     (180.0f / PI_F32)));
	}
	
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
		g_motor_params->requested_online_mode = (uint8_t)target_state;
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

	extern struct k_msgq motor_event_queue;
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		shell_error(sh, "Failed to post mode change event: queue full");
		return -ENOMEM;
	}

	shell_print(sh, "Mode change to %s requested", mode_name);
	return 0;
}

/* motor state mode torque */
int cmd_motor_state_mode_torque(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_TORQUE, "torque");
}

/* motor state mode velocity_open */
int cmd_motor_state_mode_velocity_open(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_VELOCITY_OPEN, "velocity_open");
}

/* motor state mode velocity_closed */
int cmd_motor_state_mode_velocity_closed(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_VELOCITY_CLOSED, "velocity_closed");
}

/* motor state mode position */
int cmd_motor_state_mode_position(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return motor_request_mode_change(sh, MOTOR_STATE_ONLINE_POSITION, "position");
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

	uint32_t now_ms = k_uptime_get_32();
	uint32_t age_ms = now_ms - g_motor_params->last_command_update_ms;
	bool timeout_enabled = g_motor_params->command_timeout_ms > 0U;
	bool control_armed = motor_control_is_armed(g_motor_params);
	bool autonomous_mode_active =
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_OPEN) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_CLOSED) ||
		motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION);
	bool autonomous_keepalive =
		motor_autonomy_should_keepalive(control_armed, autonomous_mode_active,
						 g_motor_params->profile_sequence_running,
						 g_motor_params->chopper_cal_active,
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
	shell_print(sh, "  Commissioned:   %s", g_motor_params->commissioning_complete ? "YES" : "NO");
	
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
	
	shell_print(sh, "Live Telemetry:");
	shell_print(sh, "  State:          %s", motor_state_to_string(state));
	shell_print(sh, "  Error:          %s", motor_error_to_string(error));
	shell_print(sh, "  Angle (mech):   %.1f deg", (double)(g_motor_params->position_rad * 180.0f / PI_F32));
	shell_print(sh, "  Angle (elec):   %.1f deg", (double)(g_motor_params->elec_angle_rad * 180.0f / PI_F32));
	shell_print(sh, "  Enc raw:        %.3f deg (%.6f rad)",
		    (double)g_motor_params->encoder_raw_deg,
		    (double)g_motor_params->encoder_raw_rad);
	shell_print(sh, "  Enc dir sign:   %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	shell_print(sh, "  Enc trim:       elec=%.3f deg mech=%.4f deg",
		    (double)(g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32)),
		    (double)((g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32)) /
			     (float32_t)MOTOR_POLE_PAIRS));
	shell_print(sh, "  Enc used:       %.6f rad (%s, fresh=%s)",
		    (double)g_motor_params->encoder_observer_input_rad,
		    motor_encoder_input_source_to_string(g_motor_params->encoder_input_source),
		    g_motor_params->encoder_sample_fresh ? "yes" : "no");
	shell_print(sh, "  Enc flags:      status=0x%02X warn=%s err=%s",
		    g_motor_params->encoder_last_status,
		    g_motor_params->encoder_sample_warning ? "SET" : "CLEAR",
		    g_motor_params->encoder_sample_error ? "SET" : "CLEAR");
	shell_print(sh, "  Enc pipeline:   %s, %s",
		    motor_encoder_pipeline_is_enabled() ? "enabled" : "disabled",
		    motor_encoder_pipeline_is_busy() ? "busy" : "idle");
	shell_print(sh, "  Enc flag count: warn=%u err=%u",
		    g_motor_params->encoder_warning_count,
		    g_motor_params->encoder_error_count);
	shell_print(sh, "  Pos quality:    0x%02X (valid=%s fresh=%s stale=%s warn=%s err=%s glitch=%s jitter=%s gen=%s)",
		    g_motor_params->position_quality_flags,
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_VALID) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_FRESH) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_WARNING) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_ERROR) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_GLITCH) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_JITTER) ? "yes" : "no",
		    (g_motor_params->position_quality_flags & MOTOR_POSITION_CONVERT_QUALITY_GENERATED) ? "yes" : "no");
	shell_print(sh, "  Pos unwrapped:  %.6f rad", (double)g_motor_params->position_unwrapped_rad);
	shell_print(sh, "  Pos innovation: %.6f rad", (double)g_motor_params->position_innovation_rad);
	shell_print(sh, "  Pos accel:      %.3f rad/s^2", (double)g_motor_params->acceleration_rad_s2);
	shell_print(sh, "  Pos counts:     stale=%u events=%u glitch=%u jitter=%u",
		    g_motor_params->position_stale_count,
		    g_motor_params->position_stale_events,
		    g_motor_params->position_glitch_count,
		    g_motor_params->position_jitter_count);
	shell_print(sh, "  Speed:          %.3f Hz (%.1f RPM)", 
		    (double)(g_motor_params->velocity_rad_s / (2.0f * PI_F32)),
		    (double)(g_motor_params->velocity_rad_s / (2.0f * PI_F32) * 60.0f));
	shell_print(sh, "  Id reference:   %.3f A", (double)g_motor_params->Id_ref_A);
	shell_print(sh, "  Iq reference:   %.3f A", (double)g_motor_params->Iq_ref_A);
	shell_print(sh, "  Id measured:    %.3f A", (double)g_motor_params->Id_A);
	shell_print(sh, "  Iq measured:    %.3f A", (double)g_motor_params->Iq_A);
	shell_print(sh, "  Ia:             %.3f A", (double)g_motor_params->Ia_A);
	shell_print(sh, "  Ib:             %.3f A", (double)g_motor_params->Ib_A);
	shell_print(sh, "  Vd:             %.3f V", (double)g_motor_params->Vd_V);
	shell_print(sh, "  Vq:             %.3f V", (double)g_motor_params->Vq_V);
	shell_print(sh, "  Va:             %.3f V", (double)g_motor_params->Va_V);
	shell_print(sh, "  Vb:             %.3f V", (double)g_motor_params->Vb_V);
	shell_print(sh, "  Vmag(max):      %.3f V", (double)g_motor_params->max_voltage_magnitude_V);
	shell_print(sh, "  Vbus:           %.1f V", (double)g_motor_params->dc_bus_voltage_V);
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

	shell_print(sh, "Encoder RTIO pipeline:");
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
	shell_print(sh, "Encoder RTIO pipeline counters reset");
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

	float32_t trim_deg = g_motor_params->observer_elec_trim_rad * (180.0f / PI_F32);
	float32_t base_mech_offset_deg =
		g_motor_params->observer_alignment_offset_rad * (180.0f / PI_F32);
	if (argc == 1U) {
		shell_print(sh, "Encoder electrical trim: %.3f deg (mechanical equivalent: %.4f deg)",
			    (double)trim_deg,
			    (double)(trim_deg / (float32_t)MOTOR_POLE_PAIRS));
		shell_print(sh, "Observer base offset (ALIGN): %.3f deg mechanical",
			    (double)base_mech_offset_deg);
		return 0;
	}

	if (!shell_parse_finite_float(argv[1], &trim_deg)) {
		shell_error(sh, "trim must be a finite number of electrical degrees");
		return -EINVAL;
	}
	if (trim_deg < -180.0f || trim_deg > 180.0f) {
		shell_error(sh, "trim must be within [-180.0, 180.0] electrical degrees");
		return -EINVAL;
	}

	int ret = motor_api_update_param("observer_elec_trim_deg", trim_deg);
	if (ret != 0) {
		shell_error(sh, "Failed to update observer electrical trim (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Observer electrical trim update posted: %.3f deg", (double)trim_deg);
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

/* motor encoder capture start [decimation] */
int cmd_motor_encoder_capture_start(const struct shell *sh, size_t argc, char **argv)
{
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

	g_motor_params->encoder_capture_decimation = (uint16_t)decimation;
	motor_encoder_capture_reset(g_motor_params, false);
	g_motor_params->encoder_capture_enabled = true;

	shell_print(sh,
		    "Encoder capture started: decimation=%u, capacity=%u samples",
		    g_motor_params->encoder_capture_decimation,
		    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	return 0;
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

	g_motor_params->encoder_capture_enabled = false;
	shell_print(sh, "Encoder capture stopped: stored=%u overrun=%u",
		    g_motor_params->encoder_capture_count,
		    g_motor_params->encoder_capture_overrun_count);
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

	g_motor_params->encoder_capture_enabled = false;
	motor_encoder_capture_reset(g_motor_params, true);
	shell_print(sh, "Encoder capture cleared");
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
	shell_print(sh, "  Enabled:    %s", g_motor_params->encoder_capture_enabled ? "YES" : "NO");
	shell_print(sh, "  Decimation: %u", g_motor_params->encoder_capture_decimation);
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->encoder_capture_count,
		    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->encoder_capture_overrun_count);

	if (g_motor_params->encoder_capture_count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->encoder_capture_write_idx +
						  MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - 1U) %
						 MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *newest =
			&g_motor_params->encoder_capture_samples[newest_idx];
		shell_print(sh,
			    "  Latest:     loop=%u src=%s deg=%.3f fresh=%u warn=%u err=%u status=0x%02X enabled=%u",
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

/* motor encoder capture dump [count] */
int cmd_motor_encoder_capture_dump(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder capture dump [count]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	uint32_t requested = 32U;
	if (argc == 2U) {
		if (!shell_parse_u32(argv[1], &requested) || requested == 0U ||
		    requested > MOTOR_ENCODER_CAPTURE_MAX_SAMPLES) {
			shell_error(sh, "count must be in [1, %u]",
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
			return -EINVAL;
		}
	}

	uint16_t stored = g_motor_params->encoder_capture_count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->encoder_capture_write_idx +
				     MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - count) %
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);

	if (g_motor_params->encoder_capture_enabled) {
		shell_warn(sh,
			   "Capture is still running; dump may include concurrently updated samples.");
	}

	shell_print(sh,
		    "idx loop source deg rad norm q31 fresh warn err status enabled");
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *sample =
			&g_motor_params->encoder_capture_samples[idx];
		float32_t norm = motor_encoder_normalized_from_rad(sample->angle_rad);
		int32_t q31 = motor_encoder_q31_from_rad(sample->angle_rad);
		shell_print(sh,
			    "%u %u %s %.3f %.6f %.6f %d %u %u %u 0x%02X %u",
			    i,
			    sample->control_loop_count,
			    motor_encoder_input_source_to_string(sample->input_source),
			    (double)sample->angle_deg,
			    (double)sample->angle_rad,
			    (double)norm,
			    q31,
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    sample->status,
			    sample->sample_enabled);
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

	uint16_t stored = g_motor_params->encoder_capture_count;
	if (stored == 0U) {
		shell_print(sh, "No captured encoder samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->encoder_capture_write_idx +
				     MOTOR_ENCODER_CAPTURE_MAX_SAMPLES - count) %
				    MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);

	if (g_motor_params->encoder_capture_enabled) {
		shell_warn(sh,
			   "Capture is still running; compare dump may include concurrently updated samples.");
	}

	shell_print(sh,
		    "idx loop src fresh warn err cmp ref enc_m_deg ref_m_deg d_m_deg enc_e_deg ref_e_deg d_e_deg rel_phase_deg");
	bool rel_phase_init = false;
	float32_t rel_phase_base_rad = 0.0f;
	for (uint16_t i = 0U; i < count; i++) {
		uint16_t idx = (uint16_t)((start + i) % MOTOR_ENCODER_CAPTURE_MAX_SAMPLES);
		const struct motor_encoder_capture_sample *sample =
			&g_motor_params->encoder_capture_samples[idx];
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
		if (sample->compare_valid) {
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
			    "%u %u %s %u %u %u %u %s %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
			    i,
			    sample->control_loop_count,
			    motor_encoder_input_source_to_string(sample->input_source),
			    sample->sample_fresh,
			    sample->sample_warning,
			    sample->sample_error,
			    sample->compare_valid,
			    motor_encoder_compare_ref_to_string(ref_mode),
			    (double)enc_m_deg,
			    (double)ref_m_deg,
			    (double)d_m_deg,
			    (double)enc_e_deg,
			    (double)ref_e_deg,
			    (double)d_e_deg,
			    (double)rel_phase_deg);
	}

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
	shell_print(sh, "  Latched:    %s", g_motor_params->fault_snapshot_latched ? "YES" : "NO");
	if (g_motor_params->fault_snapshot_latched) {
		shell_print(sh, "  Fault:      %s (%u)",
			    motor_error_to_string((int)g_motor_params->fault_snapshot_latch_error_code),
			    g_motor_params->fault_snapshot_latch_error_code);
		shell_print(sh, "  Fault loop: %u", g_motor_params->fault_snapshot_latch_loop);
	}
	shell_print(sh, "  Stored:     %u / %u",
		    g_motor_params->fault_snapshot_count,
		    MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
	shell_print(sh, "  Overrun:    %u", g_motor_params->fault_snapshot_overrun_count);

	if (g_motor_params->fault_snapshot_count > 0U) {
		uint16_t newest_idx = (uint16_t)((g_motor_params->fault_snapshot_write_idx +
						  MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES - 1U) %
						 MOTOR_FAULT_SNAPSHOT_MAX_SAMPLES);
		const struct motor_fault_snapshot_sample *newest =
			&g_motor_params->fault_snapshot_samples[newest_idx];
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

	uint16_t stored = g_motor_params->fault_snapshot_count;
	if (stored == 0U) {
		shell_print(sh, "No fault snapshot samples");
		return 0;
	}

	uint16_t count = (uint16_t)MIN(requested, stored);
	uint16_t start = (uint16_t)((g_motor_params->fault_snapshot_write_idx +
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
			&g_motor_params->fault_snapshot_samples[idx];
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

	motor_fault_snapshot_reset(g_motor_params, true);
	shell_print(sh, "Fault snapshot cleared");
	return 0;
}
