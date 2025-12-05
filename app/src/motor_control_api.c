/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_control_api.h"
#include "motor_states.h"
#include "config.h"
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_api, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set during init) */
static struct motor_parameters *g_motor_params = NULL;

/* Cached event for peek/consume pattern */
static struct motor_event cached_event;
static bool event_cached = false;

/* Parameter descriptor for name-based access */
struct param_descr {
	const char *name;
	uint16_t offset;
};

/* Parameter table - double-buffered control parameters only */
#define PARAM_DESCR(field_name_) \
	{ \
		.name = #field_name_, \
		.offset = offsetof(struct motor_control_params, field_name_), \
	}

static const struct param_descr param_table[] = {
	PARAM_DESCR(Id_setpoint_A),
	PARAM_DESCR(Iq_setpoint_A),
};

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

int motor_api_request_start(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_START_REQUEST,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post start request: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("Start request posted");
	return 0;
}

int motor_api_request_stop(void)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_STOP_REQUEST,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post stop request: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("Stop request posted");
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

int motor_api_update_param(uint16_t param_offset, float value)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_PARAM_UPDATE,
		.param_update.param_offset = param_offset,
		.param_update.value = value,
	};
	
	/* Non-blocking post to queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post parameter update: queue full");
		return -ENOMEM;
	}
	
	LOG_DBG("Parameter update posted: offset=%u, value=%.6f", param_offset, (double)value);
	return 0;
}

int motor_api_set_param(const char *name, float value)
{
	/* Linear search through parameter table */
	for (size_t i = 0; i < ARRAY_SIZE(param_table); i++) {
		if (strcmp(name, param_table[i].name) == 0) {
			return motor_api_update_param(param_table[i].offset, value);
		}
	}
	return -EINVAL;  /* Parameter not found */
}

int motor_api_get_param(const char *name, float *value)
{
	if (!g_motor_params) {
		return -ENODEV;
	}
	
	/* Linear search through parameter table */
	for (size_t i = 0; i < ARRAY_SIZE(param_table); i++) {
		if (strcmp(name, param_table[i].name) == 0) {
			/* Read from write buffer (protocol's view of pending parameters) */
			uint8_t write_index = g_motor_params->ctrl_index ^ 1;
			*value = *(float *)((uint8_t *)&g_motor_params->ctrl_buf[write_index] + 
			                   param_table[i].offset);
			return 0;
		}
	}
	return -EINVAL;  /* Parameter not found */
}

size_t motor_api_get_param_count(void)
{
	return ARRAY_SIZE(param_table);
}

const char *motor_api_get_param_name(size_t index)
{
	if (index >= ARRAY_SIZE(param_table)) {
		return NULL;
	}
	return param_table[index].name;
}

int motor_api_get_param_by_index(size_t index, float *value)
{
	if (!g_motor_params || index >= ARRAY_SIZE(param_table)) {
		return -EINVAL;
	}
	
	/* Read from write buffer */
	uint8_t write_index = g_motor_params->ctrl_index ^ 1;
	*value = *(float *)((uint8_t *)&g_motor_params->ctrl_buf[write_index] + 
	                   param_table[index].offset);
	return 0;
}

int motor_api_set_currents(float id_A, float iq_A)
{
	int ret;
	
	/* Post Id update */
	ret = motor_api_update_param(
		offsetof(struct motor_control_params, Id_setpoint_A), id_A);
	if (ret != 0) {
		return ret;
	}
	
	/* Post Iq update */
	ret = motor_api_update_param(
		offsetof(struct motor_control_params, Iq_setpoint_A), iq_A);
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
	if (!g_motor_params) {
		*id_meas = 0.0f;
		*iq_meas = 0.0f;
		*speed_hz = 0.0f;
		*vbus_V = 0.0f;
		return;
	}
	
	/* Safe read-only access: ISR updates telemetry snapshot atomically
	 * Single 32-bit float reads are atomic on Cortex-M
	 */
	*id_meas = g_motor_params->Id_A;
	*iq_meas = g_motor_params->Iq_A;
	*speed_hz = g_motor_params->velocity_rad_s / (2.0f * PI_F32);
	*vbus_V = g_motor_params->dc_bus_voltage_V;
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

	/* Write to shadow buffer (write index = ctrl_index XOR 1) */
	uint8_t write_idx = params->ctrl_index ^ 1;
	uint8_t *shadow_base = (uint8_t *)&params->ctrl_buf[write_idx];

	/* Direct offset write */
	*(float *)(shadow_base + params->event.param_update.param_offset) = 
		params->event.param_update.value;

	/* Signal ISR to swap buffers on next control cycle */
	params->ctrl_swap_pending = true;

	LOG_DBG("Parameter updated: offset=%u, value=%.6f", 
	        params->event.param_update.param_offset, 
	        (double)params->event.param_update.value);
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