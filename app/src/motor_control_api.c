/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control_api.h"
#include "motor_states.h"
#include "config.h"
#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_api, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set during init) */
static struct motor_parameters *g_motor_params = NULL;

/* Cached event for peek/consume pattern */
static struct motor_event cached_event;
static bool event_cached = false;

enum motor_param_id {
	PARAM_ID_ID_SETPOINT_A = 0,
	PARAM_ID_IQ_SETPOINT_A,
	PARAM_ID_VELOCITY_KP_A_PER_RAD_S,
	PARAM_ID_VELOCITY_IQ_LIMIT_A,
	PARAM_ID_POSITION_KP_RAD_S_PER_RAD,
	PARAM_ID_PROFILE_MAX_VELOCITY_HZ,
	PARAM_ID_PROFILE_MAX_ACCEL_HZ_S,
	PARAM_ID_COMMAND_TIMEOUT_MS,
	PARAM_ID_COUNT,
};

static const char *const motor_param_names[PARAM_ID_COUNT] = {
	[PARAM_ID_ID_SETPOINT_A] = "Id_setpoint_A",
	[PARAM_ID_IQ_SETPOINT_A] = "Iq_setpoint_A",
	[PARAM_ID_VELOCITY_KP_A_PER_RAD_S] = "velocity_cl_kp_A_per_rad_s",
	[PARAM_ID_VELOCITY_IQ_LIMIT_A] = "velocity_cl_iq_limit_A",
	[PARAM_ID_POSITION_KP_RAD_S_PER_RAD] = "position_cl_kp_rad_s_per_rad",
	[PARAM_ID_PROFILE_MAX_VELOCITY_HZ] = "profile_max_velocity_hz",
	[PARAM_ID_PROFILE_MAX_ACCEL_HZ_S] = "profile_max_accel_hz_s",
	[PARAM_ID_COMMAND_TIMEOUT_MS] = "command_timeout_ms",
};

static bool motor_param_requires_positive(uint8_t param_id)
{
	switch (param_id) {
	case PARAM_ID_VELOCITY_KP_A_PER_RAD_S:
	case PARAM_ID_VELOCITY_IQ_LIMIT_A:
	case PARAM_ID_POSITION_KP_RAD_S_PER_RAD:
	case PARAM_ID_PROFILE_MAX_VELOCITY_HZ:
	case PARAM_ID_PROFILE_MAX_ACCEL_HZ_S:
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
	case PARAM_ID_VELOCITY_IQ_LIMIT_A:
		*value = params->velocity_cl_iq_limit_A;
		return 0;
	case PARAM_ID_POSITION_KP_RAD_S_PER_RAD:
		*value = params->position_cl_kp_rad_s_per_rad;
		return 0;
	case PARAM_ID_PROFILE_MAX_VELOCITY_HZ:
		*value = params->profile_max_velocity_rad_s / (2.0f * PI_F32);
		return 0;
	case PARAM_ID_PROFILE_MAX_ACCEL_HZ_S:
		*value = params->profile_max_accel_rad_s2 / (2.0f * PI_F32);
		return 0;
	case PARAM_ID_COMMAND_TIMEOUT_MS:
		*value = (float)params->command_timeout_ms;
		return 0;
	default:
		return -EINVAL;
	}
}

static void motor_param_apply_profile_limits(struct motor_parameters *params)
{
	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity,
			      clampf(traj_get_target_value(&params->traj_velocity),
				     -params->profile_max_velocity_rad_s,
				     params->profile_max_velocity_rad_s));
	traj_set_int_value(&params->traj_velocity,
			   clampf(traj_get_int_value(&params->traj_velocity),
				  -params->profile_max_velocity_rad_s,
				  params->profile_max_velocity_rad_s));
	params->velocity_target_rad_s =
		clampf(params->velocity_target_rad_s,
		       -params->profile_max_velocity_rad_s,
		       params->profile_max_velocity_rad_s);
	params->velocity_ref_rad_s =
		clampf(params->velocity_ref_rad_s,
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

int motor_api_request_offline(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_OFFLINE,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post OFFLINE request: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("OFFLINE request posted");
	return 0;
}

int motor_api_request_idle(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_IDLE,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post IDLE request: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("IDLE request posted");
	return 0;
}

int motor_api_request_online(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_ONLINE,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post ONLINE request: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("ONLINE request posted");
	return 0;
}

int motor_api_request_calibrate(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_CALIBRATE_REQUEST,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post calibrate request: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("Calibrate request posted");
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
	
	/* Non-blocking post to queue */
	ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post parameter update: queue full");
		return -ENOMEM;
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
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post error clear: queue full");
		return -ENOMEM;
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
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put_front(&motor_event_queue, &evt);
	if (ret != 0) {
		LOG_ERR("Failed to post error event: queue full");
		return -ENOMEM;
	}
	
	return ret;
}

int motor_api_post_error(uint32_t error_code)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_ERROR,
		.error_code = error_code,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put_front(&motor_event_queue, &evt);
	if (ret != 0) {
		LOG_ERR("Failed to post error event: queue full");
		return -ENOMEM;
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
		*id_meas = g_motor_params->Id_A;
	}
	if (iq_meas != NULL) {
		*iq_meas = g_motor_params->Iq_A;
	}
	if (speed_hz != NULL) {
		*speed_hz = g_motor_params->velocity_rad_s / (2.0f * PI_F32);
	}
	if (vbus_V != NULL) {
		*vbus_V = g_motor_params->dc_bus_voltage_V;
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
	case PARAM_ID_VELOCITY_IQ_LIMIT_A:
		if (value <= 0.0f) {
			LOG_ERR("Rejected velocity_cl_iq_limit_A <= 0");
			break;
		}
		params->velocity_cl_iq_limit_A = value;
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
	case PARAM_ID_PROFILE_MAX_VELOCITY_HZ:
		if (value <= 0.0f) {
			LOG_ERR("Rejected profile_max_velocity_hz <= 0");
			break;
		}
		params->profile_max_velocity_rad_s = value * 2.0f * PI_F32;
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
		motor_param_apply_profile_limits(params);
		LOG_DBG("Updated profile_max_accel_hz_s = %.6f",
			(double)value);
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
