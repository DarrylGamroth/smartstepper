/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control_api.h"
#include "motor_states.h"
#include "config.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/motion_planner.h"
#include "motor_state_utils.h"
#include <math.h>
#include <string.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_api, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set during init) */
static struct motor_parameters *g_motor_params = NULL;
extern struct k_msgq motor_event_queue;

/* Cached event for peek/consume pattern */
static struct motor_event cached_event;
static bool event_cached = false;

enum motor_param_id {
	PARAM_ID_ID_SETPOINT_A = 0,
	PARAM_ID_IQ_SETPOINT_A,
	PARAM_ID_VELOCITY_KP_A_PER_RAD_S,
	PARAM_ID_VELOCITY_KI_A_PER_RAD,
	PARAM_ID_VELOCITY_IQ_LIMIT_A,
	PARAM_ID_POSITION_KP_RAD_S_PER_RAD,
	PARAM_ID_POSITION_KI_RAD_S2_PER_RAD,
	PARAM_ID_VELOCITY_LOOP_DECIMATION,
	PARAM_ID_POSITION_LOOP_DECIMATION,
	PARAM_ID_OUTER_LOOP_MODE,
	PARAM_ID_VELOCITY_MPR_Q_SPEED,
	PARAM_ID_VELOCITY_MPR_R_DELTA_IQ,
	PARAM_ID_VELOCITY_MPR_HORIZON,
	PARAM_ID_VELOCITY_MPR_MAX_DELTA_IQ_A,
	PARAM_ID_VELOCITY_MPR_DISTURBANCE_KI_NM_PER_RAD_S,
	PARAM_ID_VELOCITY_DOB_ENABLE,
	PARAM_ID_VELOCITY_DOB_OBSERVER_GAIN_NM_PER_RAD_S,
	PARAM_ID_VELOCITY_DOB_TORQUE_LIMIT_NM,
	PARAM_ID_VELOCITY_DOB_IQ_FF_LIMIT_A,
	PARAM_ID_DETENT_FF_ENABLE,
	PARAM_ID_DETENT_FF_GAIN,
	PARAM_ID_DETENT_FF_IQ_LIMIT_A,
	PARAM_ID_DETENT_FF_PHASE_ADVANCE_BINS,
	PARAM_ID_POSITION_MPR_Q_POSITION,
	PARAM_ID_POSITION_MPR_Q_VELOCITY_FF,
	PARAM_ID_POSITION_MPR_R_DELTA_VELOCITY,
	PARAM_ID_POSITION_MPR_HORIZON,
	PARAM_ID_POSITION_MPR_MAX_DELTA_VELOCITY_RAD_S,
	PARAM_ID_PROFILE_MAX_VELOCITY_HZ,
	PARAM_ID_PROFILE_MAX_ACCEL_HZ_S,
	PARAM_ID_TORQUE_GAIN_NM_PER_A_ACTIVE,
	PARAM_ID_COMMAND_TIMEOUT_MS,
	PARAM_ID_ENCODER_DIRECTION_SIGN,
	PARAM_ID_OBSERVER_ELEC_TRIM_DEG,
	PARAM_ID_COUNT,
};

static const char *const motor_param_names[PARAM_ID_COUNT] = {
	[PARAM_ID_ID_SETPOINT_A] = "Id_setpoint_A",
	[PARAM_ID_IQ_SETPOINT_A] = "Iq_setpoint_A",
	[PARAM_ID_VELOCITY_KP_A_PER_RAD_S] = "velocity_cl_kp_A_per_rad_s",
	[PARAM_ID_VELOCITY_KI_A_PER_RAD] = "velocity_cl_ki_A_per_rad",
	[PARAM_ID_VELOCITY_IQ_LIMIT_A] = "velocity_cl_iq_limit_A",
	[PARAM_ID_POSITION_KP_RAD_S_PER_RAD] = "position_cl_kp_rad_s_per_rad",
	[PARAM_ID_POSITION_KI_RAD_S2_PER_RAD] = "position_cl_ki_rad_s2_per_rad",
	[PARAM_ID_VELOCITY_LOOP_DECIMATION] = "velocity_loop_decimation",
	[PARAM_ID_POSITION_LOOP_DECIMATION] = "position_loop_decimation",
	[PARAM_ID_OUTER_LOOP_MODE] = "outer_loop_mode",
	[PARAM_ID_VELOCITY_MPR_Q_SPEED] = "velocity_mpr_q_speed",
	[PARAM_ID_VELOCITY_MPR_R_DELTA_IQ] = "velocity_mpr_r_delta_iq",
	[PARAM_ID_VELOCITY_MPR_HORIZON] = "velocity_mpr_horizon",
	[PARAM_ID_VELOCITY_MPR_MAX_DELTA_IQ_A] = "velocity_mpr_max_delta_iq_a",
	[PARAM_ID_VELOCITY_MPR_DISTURBANCE_KI_NM_PER_RAD_S] =
		"velocity_mpr_disturbance_ki_nm_per_rad_s",
	[PARAM_ID_VELOCITY_DOB_ENABLE] = "velocity_dob_enable",
	[PARAM_ID_VELOCITY_DOB_OBSERVER_GAIN_NM_PER_RAD_S] =
		"velocity_dob_observer_gain_nm_per_rad_s",
	[PARAM_ID_VELOCITY_DOB_TORQUE_LIMIT_NM] = "velocity_dob_torque_limit_nm",
	[PARAM_ID_VELOCITY_DOB_IQ_FF_LIMIT_A] = "velocity_dob_iq_ff_limit_a",
	[PARAM_ID_DETENT_FF_ENABLE] = "detent_ff_enable",
	[PARAM_ID_DETENT_FF_GAIN] = "detent_ff_gain",
	[PARAM_ID_DETENT_FF_IQ_LIMIT_A] = "detent_ff_iq_limit_a",
	[PARAM_ID_DETENT_FF_PHASE_ADVANCE_BINS] = "detent_ff_phase_advance_bins",
	[PARAM_ID_POSITION_MPR_Q_POSITION] = "position_mpr_q_position",
	[PARAM_ID_POSITION_MPR_Q_VELOCITY_FF] = "position_mpr_q_velocity_ff",
	[PARAM_ID_POSITION_MPR_R_DELTA_VELOCITY] = "position_mpr_r_delta_velocity",
	[PARAM_ID_POSITION_MPR_HORIZON] = "position_mpr_horizon",
	[PARAM_ID_POSITION_MPR_MAX_DELTA_VELOCITY_RAD_S] =
		"position_mpr_max_delta_velocity_rad_s",
	[PARAM_ID_PROFILE_MAX_VELOCITY_HZ] = "profile_max_velocity_hz",
	[PARAM_ID_PROFILE_MAX_ACCEL_HZ_S] = "profile_max_accel_hz_s",
	[PARAM_ID_TORQUE_GAIN_NM_PER_A_ACTIVE] = "torque_gain_nm_per_a_active",
	[PARAM_ID_COMMAND_TIMEOUT_MS] = "command_timeout_ms",
	[PARAM_ID_ENCODER_DIRECTION_SIGN] = "encoder_direction_sign",
	[PARAM_ID_OBSERVER_ELEC_TRIM_DEG] = "observer_elec_trim_deg",
};

static bool motor_param_requires_positive(uint8_t param_id)
{
	switch (param_id) {
	case PARAM_ID_VELOCITY_KP_A_PER_RAD_S:
	case PARAM_ID_VELOCITY_KI_A_PER_RAD:
	case PARAM_ID_VELOCITY_IQ_LIMIT_A:
	case PARAM_ID_POSITION_KP_RAD_S_PER_RAD:
	case PARAM_ID_POSITION_KI_RAD_S2_PER_RAD:
	case PARAM_ID_VELOCITY_LOOP_DECIMATION:
	case PARAM_ID_POSITION_LOOP_DECIMATION:
	case PARAM_ID_VELOCITY_MPR_Q_SPEED:
	case PARAM_ID_VELOCITY_MPR_R_DELTA_IQ:
	case PARAM_ID_VELOCITY_MPR_HORIZON:
	case PARAM_ID_POSITION_MPR_R_DELTA_VELOCITY:
	case PARAM_ID_POSITION_MPR_HORIZON:
	case PARAM_ID_PROFILE_MAX_VELOCITY_HZ:
	case PARAM_ID_PROFILE_MAX_ACCEL_HZ_S:
	case PARAM_ID_TORQUE_GAIN_NM_PER_A_ACTIVE:
		return true;
	default:
		return false;
	}
}

static int motor_param_name_to_id(const char *name, uint8_t *param_id)
{
	if (name == NULL || param_id == NULL) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < PARAM_ID_COUNT; i++) {
		if (strcmp(name, motor_param_names[i]) == 0) {
			*param_id = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int motor_param_get_value(const struct motor_parameters *params, uint8_t param_id, float *value)
{
	if (params == NULL || value == NULL) {
		return -EINVAL;
	}

	switch (param_id) {
	case PARAM_ID_ID_SETPOINT_A:
		*value = params->Id_setpoint_A;
		return 0;
	case PARAM_ID_IQ_SETPOINT_A:
		*value = params->Iq_setpoint_A;
		return 0;
	case PARAM_ID_VELOCITY_KP_A_PER_RAD_S:
		*value = params->velocity_cl_kp_A_per_rad_s;
		return 0;
	case PARAM_ID_VELOCITY_KI_A_PER_RAD:
		*value = params->velocity_cl_ki_A_per_rad;
		return 0;
	case PARAM_ID_VELOCITY_IQ_LIMIT_A:
		*value = params->velocity_cl_iq_limit_A;
		return 0;
	case PARAM_ID_POSITION_KP_RAD_S_PER_RAD:
		*value = params->position_cl_kp_rad_s_per_rad;
		return 0;
	case PARAM_ID_POSITION_KI_RAD_S2_PER_RAD:
		*value = params->position_cl_ki_rad_s2_per_rad;
		return 0;
	case PARAM_ID_VELOCITY_LOOP_DECIMATION:
		*value = (float32_t)params->velocity_loop_decimation;
		return 0;
	case PARAM_ID_POSITION_LOOP_DECIMATION:
		*value = (float32_t)params->position_loop_decimation;
		return 0;
	case PARAM_ID_OUTER_LOOP_MODE:
		*value = (float32_t)params->outer_loop_mode;
		return 0;
	case PARAM_ID_VELOCITY_MPR_Q_SPEED:
		*value = params->velocity_mpr_cfg.q_speed;
		return 0;
	case PARAM_ID_VELOCITY_MPR_R_DELTA_IQ:
		*value = params->velocity_mpr_cfg.r_delta_iq;
		return 0;
	case PARAM_ID_VELOCITY_MPR_HORIZON:
		*value = (float32_t)params->velocity_mpr_cfg.horizon;
		return 0;
	case PARAM_ID_VELOCITY_MPR_MAX_DELTA_IQ_A:
		*value = params->velocity_mpr_cfg.max_delta_iq_a;
		return 0;
	case PARAM_ID_VELOCITY_MPR_DISTURBANCE_KI_NM_PER_RAD_S:
		*value = params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s;
		return 0;
	case PARAM_ID_VELOCITY_DOB_ENABLE:
		*value = params->velocity_dob_cfg.enabled ? 1.0f : 0.0f;
		return 0;
	case PARAM_ID_VELOCITY_DOB_OBSERVER_GAIN_NM_PER_RAD_S:
		*value = params->velocity_dob_cfg.observer_gain_nm_per_rad_s;
		return 0;
	case PARAM_ID_VELOCITY_DOB_TORQUE_LIMIT_NM:
		*value = params->velocity_dob_cfg.torque_limit_nm;
		return 0;
	case PARAM_ID_VELOCITY_DOB_IQ_FF_LIMIT_A:
		*value = params->velocity_dob_cfg.iq_ff_limit_a;
		return 0;
	case PARAM_ID_DETENT_FF_ENABLE:
		*value = params->detent_map_cfg.enabled ? 1.0f : 0.0f;
		return 0;
	case PARAM_ID_DETENT_FF_GAIN:
		*value = params->detent_map_cfg.gain;
		return 0;
	case PARAM_ID_DETENT_FF_IQ_LIMIT_A:
		*value = params->detent_map_cfg.iq_ff_limit_a;
		return 0;
	case PARAM_ID_DETENT_FF_PHASE_ADVANCE_BINS:
		*value = (float32_t)params->detent_map_cfg.phase_advance_bins;
		return 0;
	case PARAM_ID_POSITION_MPR_Q_POSITION:
		*value = params->position_mpr_cfg.q_position;
		return 0;
	case PARAM_ID_POSITION_MPR_Q_VELOCITY_FF:
		*value = params->position_mpr_cfg.q_velocity_ff;
		return 0;
	case PARAM_ID_POSITION_MPR_R_DELTA_VELOCITY:
		*value = params->position_mpr_cfg.r_delta_velocity;
		return 0;
	case PARAM_ID_POSITION_MPR_HORIZON:
		*value = (float32_t)params->position_mpr_cfg.horizon;
		return 0;
	case PARAM_ID_POSITION_MPR_MAX_DELTA_VELOCITY_RAD_S:
		*value = params->position_mpr_cfg.max_delta_velocity_rad_s;
		return 0;
	case PARAM_ID_PROFILE_MAX_VELOCITY_HZ:
		*value = params->profile_max_velocity_rad_s / (2.0f * PI_F32);
		return 0;
	case PARAM_ID_PROFILE_MAX_ACCEL_HZ_S:
		*value = params->profile_max_accel_rad_s2 / (2.0f * PI_F32);
		return 0;
	case PARAM_ID_TORQUE_GAIN_NM_PER_A_ACTIVE:
		*value = params->torque_gain_nm_per_a_active;
		return 0;
	case PARAM_ID_COMMAND_TIMEOUT_MS:
		*value = (float)params->command_timeout_ms;
		return 0;
	case PARAM_ID_ENCODER_DIRECTION_SIGN:
		*value = (float32_t)((params->encoder_direction_sign >= 0) ? 1.0f : -1.0f);
		return 0;
	case PARAM_ID_OBSERVER_ELEC_TRIM_DEG:
		*value = params->observer_elec_trim_rad * (180.0f / PI_F32);
		return 0;
	default:
		return -EINVAL;
	}
}

static float32_t motor_observer_base_offset_from_active(const struct motor_parameters *params)
{
	float32_t trim_mech_rad = params->observer_elec_trim_rad / (float32_t)MOTOR_POLE_PAIRS;

	return wrap_rad_pi(params->observer.mech_angle_offset_rad - trim_mech_rad);
}

static void motor_param_apply_profile_limits(struct motor_parameters *params)
{
	motor_velocity_plan_update_limits(&params->traj_velocity,
					  params->profile_max_velocity_rad_s,
					  params->profile_max_accel_rad_s2,
					  1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	params->live.velocity_target_rad_s =
		clampf(params->live.velocity_target_rad_s,
		       -params->profile_max_velocity_rad_s,
		       params->profile_max_velocity_rad_s);
	params->live.velocity_ref_rad_s =
		clampf(params->live.velocity_ref_rad_s,
		       -params->profile_max_velocity_rad_s,
		       params->profile_max_velocity_rad_s);
}

int motor_control_api_init(struct motor_parameters *params)
{
	if (params == NULL) {
		LOG_ERR("NULL motor_parameters pointer");
		return -EINVAL;
	}

	/* Initialize structure to known state */
	memset(params, 0, sizeof(*params));
	
	/* Set non-zero defaults */
	params->max_modulation_index = MAX_VS_MPU;

	g_motor_params = params;
	
	return 0;
}

static int motor_api_post_event_back(const struct motor_event *evt)
{
	if (evt == NULL) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return motor_api_enqueue_event_from_isr(evt);
	}

	int ret = k_msgq_put(&motor_event_queue, evt, K_NO_WAIT);
	return (ret == 0) ? 0 : -ENOMEM;
}

static int motor_api_post_event_front(const struct motor_event *evt)
{
	if (evt == NULL) {
		return -EINVAL;
	}

	/* Direct ISR callbacks cannot use kernel message queues. The state
	 * thread drains the ISR ring before the normal message queue, preserving
	 * priority relative to shell/protocol events.
	 */
	if (k_is_in_isr()) {
		return motor_api_enqueue_event_from_isr(evt);
	}

	int ret = k_msgq_put_front(&motor_event_queue, evt);
	return (ret == 0) ? 0 : -ENOMEM;
}

int motor_api_post_event(const struct motor_event *evt)
{
	return motor_api_post_event_back(evt);
}

int motor_api_request_prepare_online(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_PREPARE_ONLINE,
		};

		int ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post PREPARE_ONLINE request: queue full");
		return ret;
		}

		LOG_DBG("PREPARE_ONLINE request posted");
	return 0;
}

int motor_api_request_idle(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_IDLE,
	};
	
	int ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post IDLE request: queue full");
		return ret;
	}
	
	LOG_DBG("IDLE request posted");
	return 0;
}

int motor_api_request_online(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_ONLINE,
	};
	
	int ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post ONLINE request: queue full");
		return ret;
	}
	
	LOG_DBG("ONLINE request posted");
	return 0;
}

int motor_api_request_calibrate(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_CALIBRATE_REQUEST,
	};
	
	int ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post calibrate request: queue full");
		return ret;
	}
	
	LOG_DBG("Calibrate request posted");
	return 0;
}

int motor_api_request_commission(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_COMMISSION_REQUEST,
	};

	int ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post commission request: queue full");
		return ret;
	}

	LOG_DBG("Commission request posted");
	return 0;
}

int motor_api_update_param(const char *name, float value)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_PARAM_UPDATE,
	};

	uint8_t param_id;
	int ret = motor_param_name_to_id(name, &param_id);
	if (ret != 0) {
		return ret;
	}

	if (motor_param_requires_positive(param_id) && value <= 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(value)) {
		return -EINVAL;
	}

	evt.param_update.param_id = param_id;
	evt.param_update.value = value;
	
	ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post parameter update: queue full");
		return ret;
	}
	
	LOG_DBG("Parameter update posted: %s=%.6f", name, (double)value);
	return 0;
}

int motor_api_set_param(const char *name, float value)
{
	return motor_api_update_param(name, value);
}

int motor_api_get_param(const char *name, float *value)
{
	if (name == NULL || value == NULL) {
		return -EINVAL;
	}

	if (!g_motor_params) {
		return -ENODEV;
	}
	
	uint8_t param_id;
	int ret = motor_param_name_to_id(name, &param_id);
	if (ret != 0) {
		return ret;
	}

	return motor_param_get_value(g_motor_params, param_id, value);
}

size_t motor_api_get_param_count(void)
{
	return PARAM_ID_COUNT;
}

const char *motor_api_get_param_name(size_t index)
{
	if (index >= PARAM_ID_COUNT) {
		return NULL;
	}
	return motor_param_names[index];
}

int motor_api_get_param_by_index(size_t index, float *value)
{
	if (value == NULL) {
		return -EINVAL;
	}

	if (!g_motor_params) {
		return -ENODEV;
	}

	if (index >= PARAM_ID_COUNT) {
		return -EINVAL;
	}

	return motor_param_get_value(g_motor_params, (uint8_t)index, value);
}

int motor_api_set_currents(float id_A, float iq_A)
{
	int ret;
	
	/* Post Id update */
	ret = motor_api_update_param("Id_setpoint_A", id_A);
	if (ret != 0) {
		return ret;
	}
	
	/* Post Iq update */
	ret = motor_api_update_param("Iq_setpoint_A", iq_A);
	if (ret != 0) {
		return ret;
	}
	
	LOG_DBG("Current setpoints posted: Id=%.3f A, Iq=%.3f A", (double)id_A, (double)iq_A);
	return 0;
}

int motor_api_clear_error(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_CLEAR_ERROR,
	};
	
	int ret = motor_api_post_event_back(&evt);
	if (ret != 0) {
		LOG_ERR("Failed to post error clear: queue full");
		return ret;
	}
	
	LOG_DBG("Error clear posted");
	return 0;
}

int motor_api_emergency_stop(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_ERROR,
		.error_code = ERROR_EMERGENCY_STOP,
	};
	
	int ret = motor_api_post_event_front(&evt);
	if (ret != 0) {
		if (!k_is_in_isr()) {
			LOG_ERR("Failed to post error event: queue full");
		}
		return ret;
	}
	
	return ret;
}

int motor_api_post_error(uint32_t error_code)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_ERROR,
		.error_code = error_code,
	};
	
	int ret = motor_api_post_event_front(&evt);
	if (ret != 0) {
		if (!k_is_in_isr()) {
			LOG_ERR("Failed to post error event: queue full");
		}
		return ret;
	}
	
	return ret;
}

int motor_api_get_state(void)
{
	if (!g_motor_params) {
		return -1;
	}
	
	/* Use helper function from motor_states.c */
	return motor_states_get_current(g_motor_params);
}

int motor_api_get_error(void)
{
	if (!g_motor_params) {
		return ERROR_NONE;
	}
	
	/* Read-only access to last error that caused ERROR state entry */
	return g_motor_params->last_error_code;
}

void motor_api_get_telemetry(float *id_meas, float *iq_meas, 
                             float *speed_hz, float *vbus_V)
{
	if (id_meas == NULL && iq_meas == NULL && speed_hz == NULL && vbus_V == NULL) {
		return;
	}

	if (!g_motor_params) {
		if (id_meas != NULL) {
			*id_meas = 0.0f;
		}
		if (iq_meas != NULL) {
			*iq_meas = 0.0f;
		}
		if (speed_hz != NULL) {
			*speed_hz = 0.0f;
		}
		if (vbus_V != NULL) {
			*vbus_V = 0.0f;
		}
		return;
	}
	
	/* Safe read-only access: ISR updates telemetry snapshot atomically
	 * Single 32-bit float reads are atomic on Cortex-M
	 */
	if (id_meas != NULL) {
		*id_meas = g_motor_params->live.Id_A;
	}
	if (iq_meas != NULL) {
		*iq_meas = g_motor_params->live.Iq_A;
	}
	if (speed_hz != NULL) {
		*speed_hz = g_motor_params->live.velocity_rad_s / (2.0f * PI_F32);
	}
	if (vbus_V != NULL) {
		*vbus_V = g_motor_params->live.dc_bus_voltage_V;
	}
}

bool motor_api_has_event(void)
{
	if (event_cached) {
		return true;
	}
	
	/* Peek at queue without removing */
	return k_msgq_num_used_get(&motor_event_queue) > 0;
}

int motor_api_peek_event(struct motor_event *evt)
{
	if (evt == NULL) {
		return -EINVAL;
	}

	if (event_cached) {
		*evt = cached_event;
		return 0;
	}
	
	/* Get event from queue and cache it */
	int ret = k_msgq_get(&motor_event_queue, &cached_event, K_NO_WAIT);
	if (ret == 0) {
		event_cached = true;
		*evt = cached_event;
		return 0;
	}
	
	return -ENOMSG;
}

void motor_api_apply_param_update(struct motor_parameters *params)
{
	if (!params) {
		LOG_ERR("NULL motor_parameters pointer");
		return;
	}

	if (params->event.type != MOTOR_EVENT_PARAM_UPDATE) {
		LOG_WRN("Current event is not a parameter update");
		return;
	}

	float value = params->event.param_update.value;
	if (!isfinite(value)) {
		LOG_ERR("Rejected non-finite parameter value for id %u",
			params->event.param_update.param_id);
		return;
	}

	/* Direct write to setpoint fields based on param_id */
	switch (params->event.param_update.param_id) {
	case PARAM_ID_ID_SETPOINT_A:
		params->Id_setpoint_A = value;
		LOG_DBG("Updated Id_setpoint_A = %.3f A", (double)value);
		break;
	case PARAM_ID_IQ_SETPOINT_A:
		params->Iq_setpoint_A = value;
		LOG_DBG("Updated Iq_setpoint_A = %.3f A", (double)value);
		break;
	case PARAM_ID_VELOCITY_KP_A_PER_RAD_S:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_cl_kp_A_per_rad_s <= 0");
			break;
		}
		params->velocity_cl_kp_A_per_rad_s = value;
		LOG_DBG("Updated velocity_cl_kp_A_per_rad_s = %.6f",
			(double)value);
		break;
	case PARAM_ID_VELOCITY_KI_A_PER_RAD:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_cl_ki_A_per_rad <= 0");
			break;
		}
		params->velocity_cl_ki_A_per_rad = value;
		LOG_DBG("Updated velocity_cl_ki_A_per_rad = %.6f",
			(double)value);
		break;
	case PARAM_ID_VELOCITY_IQ_LIMIT_A:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_cl_iq_limit_A <= 0");
			break;
		}
		params->velocity_cl_iq_limit_A = value;
		params->velocity_mpr_cfg.iq_limit_a = value;
		if (params->velocity_dob_cfg.iq_ff_limit_a <= 0.0f ||
		    params->velocity_dob_cfg.iq_ff_limit_a > value) {
			params->velocity_dob_cfg.iq_ff_limit_a = value;
		}
		LOG_DBG("Updated velocity_cl_iq_limit_A = %.6f",
			(double)value);
		break;
	case PARAM_ID_POSITION_KP_RAD_S_PER_RAD:
		if (value <= 0.0f) {
			LOG_ERR("Rejected position_cl_kp_rad_s_per_rad <= 0");
			break;
		}
		params->position_cl_kp_rad_s_per_rad = value;
		LOG_DBG("Updated position_cl_kp_rad_s_per_rad = %.6f",
			(double)value);
		break;
	case PARAM_ID_POSITION_KI_RAD_S2_PER_RAD:
		if (value <= 0.0f) {
			LOG_ERR("Rejected position_cl_ki_rad_s2_per_rad <= 0");
			break;
		}
		params->position_cl_ki_rad_s2_per_rad = value;
		LOG_DBG("Updated position_cl_ki_rad_s2_per_rad = %.6f",
			(double)value);
		break;
	case PARAM_ID_VELOCITY_LOOP_DECIMATION: {
		uint32_t decimation = (uint32_t)(value + 0.5f);
		if (decimation < OUTER_LOOP_DECIMATION_MIN ||
		    decimation > OUTER_LOOP_DECIMATION_MAX) {
			LOG_ERR("Rejected velocity_loop_decimation outside [%u,%u]",
				OUTER_LOOP_DECIMATION_MIN, OUTER_LOOP_DECIMATION_MAX);
			break;
		}
		params->velocity_loop_decimation = decimation;
		params->velocity_loop_phase = 0U;
		params->velocity_mpr_cfg.dt_s =
			(float32_t)decimation / CONTROL_LOOP_FREQUENCY_HZ;
		params->velocity_dob_cfg.dt_s = params->velocity_mpr_cfg.dt_s;
		motor_mpr_velocity_invalidate(&params->velocity_mpr_state);
		motor_dob_invalidate(&params->velocity_dob_state);
		LOG_DBG("Updated velocity_loop_decimation = %u", decimation);
		break;
	}
	case PARAM_ID_POSITION_LOOP_DECIMATION: {
		uint32_t decimation = (uint32_t)(value + 0.5f);
		if (decimation < OUTER_LOOP_DECIMATION_MIN ||
		    decimation > OUTER_LOOP_DECIMATION_MAX) {
			LOG_ERR("Rejected position_loop_decimation outside [%u,%u]",
				OUTER_LOOP_DECIMATION_MIN, OUTER_LOOP_DECIMATION_MAX);
			break;
		}
		params->position_loop_decimation = decimation;
		params->position_loop_phase = 0U;
		params->position_mpr_cfg.dt_s =
			(float32_t)decimation / CONTROL_LOOP_FREQUENCY_HZ;
		params->position_mpr_cfg.max_delta_velocity_rad_s =
			params->profile_max_accel_rad_s2 * params->position_mpr_cfg.dt_s;
		motor_mpr_position_invalidate(&params->position_mpr_state);
		LOG_DBG("Updated position_loop_decimation = %u", decimation);
		break;
	}
	case PARAM_ID_OUTER_LOOP_MODE:
		if (value < 0.0f || value > 1.0f) {
			LOG_ERR("Rejected outer_loop_mode outside [0,1]");
			break;
		}
		params->outer_loop_mode = (uint8_t)(value + 0.5f);
		motor_mpr_velocity_reset(&params->velocity_mpr_state, params->live.velocity_rad_s,
					 params->live.Iq_ref_A);
		motor_mpr_position_reset(&params->position_mpr_state, params->live.velocity_ref_rad_s);
		motor_dob_reset(&params->velocity_dob_state, params->live.velocity_rad_s);
		motor_mpr_velocity_invalidate(&params->velocity_mpr_state);
		motor_mpr_position_invalidate(&params->position_mpr_state);
		motor_dob_invalidate(&params->velocity_dob_state);
		LOG_DBG("Updated outer_loop_mode = %u", params->outer_loop_mode);
		break;
	case PARAM_ID_VELOCITY_MPR_Q_SPEED:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_mpr_q_speed <= 0");
			break;
		}
		params->velocity_mpr_cfg.q_speed = value;
		LOG_DBG("Updated velocity_mpr_q_speed = %.6f", (double)value);
		break;
	case PARAM_ID_VELOCITY_MPR_R_DELTA_IQ:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_mpr_r_delta_iq <= 0");
			break;
		}
		params->velocity_mpr_cfg.r_delta_iq = value;
		LOG_DBG("Updated velocity_mpr_r_delta_iq = %.6f", (double)value);
		break;
	case PARAM_ID_VELOCITY_MPR_HORIZON:
		if (value <= 0.0f || value > (float32_t)MOTOR_MPR_HORIZON_MAX) {
			LOG_ERR("Rejected velocity_mpr_horizon outside (0,%u]",
				MOTOR_MPR_HORIZON_MAX);
			break;
		}
		params->velocity_mpr_cfg.horizon = (uint16_t)(value + 0.5f);
		LOG_DBG("Updated velocity_mpr_horizon = %u", params->velocity_mpr_cfg.horizon);
		break;
	case PARAM_ID_VELOCITY_MPR_MAX_DELTA_IQ_A:
		if (value < 0.0f) {
			LOG_ERR("Rejected velocity_mpr_max_delta_iq_a < 0");
			break;
		}
		params->velocity_mpr_cfg.max_delta_iq_a = value;
		LOG_DBG("Updated velocity_mpr_max_delta_iq_a = %.6f", (double)value);
		break;
	case PARAM_ID_VELOCITY_MPR_DISTURBANCE_KI_NM_PER_RAD_S:
		if (value < 0.0f) {
			LOG_ERR("Rejected velocity_mpr_disturbance_ki_nm_per_rad_s < 0");
			break;
		}
		params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s = value;
		LOG_DBG("Updated velocity_mpr_disturbance_ki_nm_per_rad_s = %.6f",
			(double)value);
		break;
	case PARAM_ID_VELOCITY_DOB_ENABLE:
		if (value < 0.0f || value > 1.0f) {
			LOG_ERR("Rejected velocity_dob_enable outside [0,1]");
			break;
		}
		params->velocity_dob_cfg.enabled = (value >= 0.5f);
		motor_dob_reset(&params->velocity_dob_state, params->live.velocity_rad_s);
		motor_dob_invalidate(&params->velocity_dob_state);
		LOG_DBG("Updated velocity_dob_enable = %u", params->velocity_dob_cfg.enabled ? 1U : 0U);
		break;
	case PARAM_ID_VELOCITY_DOB_OBSERVER_GAIN_NM_PER_RAD_S:
		if (value < 0.0f) {
			LOG_ERR("Rejected velocity_dob_observer_gain_nm_per_rad_s < 0");
			break;
		}
		params->velocity_dob_cfg.observer_gain_nm_per_rad_s = value;
		LOG_DBG("Updated velocity_dob_observer_gain_nm_per_rad_s = %.6f", (double)value);
		break;
	case PARAM_ID_VELOCITY_DOB_TORQUE_LIMIT_NM:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_dob_torque_limit_nm <= 0");
			break;
		}
		params->velocity_dob_cfg.torque_limit_nm = value;
		LOG_DBG("Updated velocity_dob_torque_limit_nm = %.6f", (double)value);
		break;
	case PARAM_ID_VELOCITY_DOB_IQ_FF_LIMIT_A:
		if (value < 0.0f) {
			LOG_ERR("Rejected velocity_dob_iq_ff_limit_a < 0");
			break;
		}
		params->velocity_dob_cfg.iq_ff_limit_a = value;
		LOG_DBG("Updated velocity_dob_iq_ff_limit_a = %.6f", (double)value);
		break;
	case PARAM_ID_DETENT_FF_ENABLE:
		if (value < 0.0f || value > 1.0f) {
			LOG_ERR("Rejected detent_ff_enable outside [0,1]");
			break;
		}
		params->detent_map_cfg.enabled = (value >= 0.5f);
		motor_detent_map_reset(&params->detent_map_state);
		params->live.detent_iq_ff_a = 0.0f;
		LOG_DBG("Updated detent_ff_enable = %u",
			params->detent_map_cfg.enabled ? 1U : 0U);
		break;
	case PARAM_ID_DETENT_FF_GAIN:
		if (value < 0.0f) {
			LOG_ERR("Rejected detent_ff_gain < 0");
			break;
		}
		params->detent_map_cfg.gain = value;
		LOG_DBG("Updated detent_ff_gain = %.6f", (double)value);
		break;
	case PARAM_ID_DETENT_FF_IQ_LIMIT_A:
		if (value < 0.0f || value > params->velocity_cl_iq_limit_A) {
			LOG_ERR("Rejected detent_ff_iq_limit_a outside [0, velocity limit]");
			break;
		}
		params->detent_map_cfg.iq_ff_limit_a = value;
		LOG_DBG("Updated detent_ff_iq_limit_a = %.6f", (double)value);
		break;
	case PARAM_ID_DETENT_FF_PHASE_ADVANCE_BINS:
		if (value < -(float32_t)MOTOR_DETENT_MAP_BINS ||
		    value > (float32_t)MOTOR_DETENT_MAP_BINS) {
			LOG_ERR("Rejected detent_ff_phase_advance_bins outside +/- table length");
			break;
		}
		params->detent_map_cfg.phase_advance_bins = (int16_t)value;
		LOG_DBG("Updated detent_ff_phase_advance_bins = %d",
			params->detent_map_cfg.phase_advance_bins);
		break;
	case PARAM_ID_POSITION_MPR_Q_POSITION:
		if (value < 0.0f ||
		    (value == 0.0f && params->position_mpr_cfg.q_velocity_ff <= 0.0f)) {
			LOG_ERR("Rejected position_mpr_q_position; q_position + q_velocity_ff must stay > 0");
			break;
		}
		params->position_mpr_cfg.q_position = value;
		LOG_DBG("Updated position_mpr_q_position = %.6f", (double)value);
		break;
	case PARAM_ID_POSITION_MPR_Q_VELOCITY_FF:
		if (value < 0.0f ||
		    (value == 0.0f && params->position_mpr_cfg.q_position <= 0.0f)) {
			LOG_ERR("Rejected position_mpr_q_velocity_ff; q_position + q_velocity_ff must stay > 0");
			break;
		}
		params->position_mpr_cfg.q_velocity_ff = value;
		LOG_DBG("Updated position_mpr_q_velocity_ff = %.6f", (double)value);
		break;
	case PARAM_ID_POSITION_MPR_R_DELTA_VELOCITY:
		if (value <= 0.0f) {
			LOG_ERR("Rejected position_mpr_r_delta_velocity <= 0");
			break;
		}
		params->position_mpr_cfg.r_delta_velocity = value;
		LOG_DBG("Updated position_mpr_r_delta_velocity = %.6f", (double)value);
		break;
	case PARAM_ID_POSITION_MPR_HORIZON:
		if (value <= 0.0f || value > (float32_t)MOTOR_MPR_HORIZON_MAX) {
			LOG_ERR("Rejected position_mpr_horizon outside (0,%u]",
				MOTOR_MPR_HORIZON_MAX);
			break;
		}
		params->position_mpr_cfg.horizon = (uint16_t)(value + 0.5f);
		motor_mpr_position_invalidate(&params->position_mpr_state);
		LOG_DBG("Updated position_mpr_horizon = %u", params->position_mpr_cfg.horizon);
		break;
	case PARAM_ID_POSITION_MPR_MAX_DELTA_VELOCITY_RAD_S:
		if (value < 0.0f) {
			LOG_ERR("Rejected position_mpr_max_delta_velocity_rad_s < 0");
			break;
		}
		params->position_mpr_cfg.max_delta_velocity_rad_s = value;
		LOG_DBG("Updated position_mpr_max_delta_velocity_rad_s = %.6f", (double)value);
		break;
	case PARAM_ID_PROFILE_MAX_VELOCITY_HZ:
		if (value <= 0.0f) {
			LOG_ERR("Rejected profile_max_velocity_hz <= 0");
			break;
		}
		params->profile_max_velocity_rad_s = value * 2.0f * PI_F32;
		params->position_mpr_cfg.velocity_limit_rad_s = params->profile_max_velocity_rad_s;
		motor_param_apply_profile_limits(params);
		LOG_DBG("Updated profile_max_velocity_hz = %.6f",
			(double)value);
		break;
	case PARAM_ID_PROFILE_MAX_ACCEL_HZ_S:
		if (value <= 0.0f) {
			LOG_ERR("Rejected profile_max_accel_hz_s <= 0");
			break;
		}
		params->profile_max_accel_rad_s2 = value * 2.0f * PI_F32;
		params->position_mpr_cfg.max_delta_velocity_rad_s =
			params->profile_max_accel_rad_s2 * params->position_mpr_cfg.dt_s;
		motor_param_apply_profile_limits(params);
		LOG_DBG("Updated profile_max_accel_hz_s = %.6f",
			(double)value);
		break;
	case PARAM_ID_TORQUE_GAIN_NM_PER_A_ACTIVE:
		if (value <= 0.0f) {
			LOG_ERR("Rejected torque_gain_nm_per_a_active <= 0");
			break;
		}
		params->torque_gain_nm_per_a_active = value;
		motor_mpr_velocity_invalidate(&params->velocity_mpr_state);
		motor_dob_invalidate(&params->velocity_dob_state);
		if (!isfinite(params->velocity_dob_cfg.torque_limit_nm) ||
		    params->velocity_dob_cfg.torque_limit_nm <= 0.0f) {
			params->velocity_dob_cfg.torque_limit_nm =
				value * params->velocity_cl_iq_limit_A;
		}
		LOG_DBG("Updated torque_gain_nm_per_a_active = %.6f", (double)value);
		break;
	case PARAM_ID_COMMAND_TIMEOUT_MS:
		if (value < 0.0f) {
			LOG_ERR("Rejected command_timeout_ms < 0");
			break;
		}
		if (value > (float)UINT32_MAX) {
			LOG_ERR("Rejected command_timeout_ms > UINT32_MAX");
			break;
		}
		params->command_timeout_ms = (uint32_t)(value + 0.5f);
		if (params->command_timeout_ms == 0U) {
			params->command_timeout_latched = false;
		}
		LOG_DBG("Updated command_timeout_ms = %u", params->command_timeout_ms);
		break;
	case PARAM_ID_ENCODER_DIRECTION_SIGN: {
		int8_t sign = 0;
		if (fabsf(value - 1.0f) < 1.0e-3f) {
			sign = 1;
		} else if (fabsf(value + 1.0f) < 1.0e-3f) {
			sign = -1;
		} else {
			LOG_ERR("Rejected encoder_direction_sign (expected -1 or 1)");
			break;
		}
		if (motor_state_ptr_is_online_control_state(params->state_for_isr)) {
			LOG_ERR("Rejected encoder_direction_sign update in ONLINE state");
			break;
		}
		if (atomic_get(&params->control_armed) != 0) {
			LOG_ERR("Rejected encoder_direction_sign update while armed");
			break;
		}
		params->encoder_direction_sign = sign;
		params->live.position_quality_flags = 0U;
		params->live.position_stale_count = 0U;
		params->live.position_stale_events = 0U;
		params->live.position_glitch_count = 0U;
		params->live.position_jitter_count = 0U;
		LOG_DBG("Updated encoder_direction_sign = %d", sign);
		break;
	}
	case PARAM_ID_OBSERVER_ELEC_TRIM_DEG: {
		if (value < -180.0f || value > 180.0f) {
			LOG_ERR("Rejected observer_elec_trim_deg outside [-180,180]");
			break;
		}
		params->observer_alignment_offset_rad =
			motor_observer_base_offset_from_active(params);
		params->observer_elec_trim_rad = value * (PI_F32 / 180.0f);
		float32_t mech_trim_rad =
			params->observer_elec_trim_rad / (float32_t)MOTOR_POLE_PAIRS;
		angle_observer_set_offset(&params->observer,
					  params->observer_alignment_offset_rad + mech_trim_rad);
		LOG_DBG("Updated observer_elec_trim_deg = %.3f", (double)value);
		break;
	}
	default:
		LOG_ERR("Unknown parameter ID: %u", params->event.param_update.param_id);
		break;
	}
}

void motor_api_consume_event(void)
{
	if (!event_cached) {
		LOG_WRN("No cached event to consume");
		return;
	}

	/* Just clear the cached event - parameter updates must be applied explicitly */
	event_cached = false;
}
