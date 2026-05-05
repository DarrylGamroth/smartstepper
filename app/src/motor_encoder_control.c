/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_encoder_control.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "config.h"
#include "motor_hardware.h"

#if DT_NODE_EXISTS(DT_ALIAS(encoder1)) && DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955_fast)
#include <drivers/encoder/aeat9955_fast.h>
#define MOTOR_ENCODER_CONTROL_AEAT_FAST 1
#else
#define MOTOR_ENCODER_CONTROL_AEAT_FAST 0
#endif

#if MOTOR_ENCODER_CONTROL_AEAT_FAST
static bool motor_encoder_control_aeat_regs_match_spi4_8_crc16(
	const struct motor_encoder_control_status *status)
{
	const uint8_t expected_config0 =
		AEAT9955_FAST_CONFIG0_SAFETY_BIT |
		AEAT9955_FAST_CONFIG0_CRC_SELECT |
		AEAT9955_FAST_CONFIG0_CRC_INIT_FFFF;

	return (status->config0 & expected_config0) == expected_config0 &&
	       ((status->config7 & AEAT9955_FAST_CONFIG0_SPI4_MODE_MASK) ==
		AEAT9955_FAST_CONFIG0_SPI4_MODE_8) &&
	       ((status->config9 & AEAT9955_FAST_CONFIG1_PSEL_BIT) == 0U);
}

static int motor_encoder_control_read_aeat_protocol_regs(
	struct motor_encoder_control_status *status)
{
	int ret = aeat9955_fast_read_register(encoder1, AEAT9955_FAST_REG_CONFIG0,
					      &status->config0);
	if (ret == 0) {
		ret = aeat9955_fast_read_register(encoder1,
						  AEAT9955_FAST_REG_CONFIG0_SPI4,
						  &status->config7);
	}
	if (ret == 0) {
		ret = aeat9955_fast_read_register(encoder1,
						  AEAT9955_FAST_REG_CONFIG1_PSEL,
						  &status->config9);
	}

	return ret;
}
#endif

bool motor_encoder_control_mode_requires_encoder(enum motor_state state)
{
	return state == MOTOR_STATE_ONLINE_CURRENT_ENCODER ||
	       state == MOTOR_STATE_ONLINE_VELOCITY_ENCODER ||
	       state == MOTOR_STATE_ONLINE_POSITION_ENCODER;
}

static void motor_encoder_control_set_reason(char *reason, size_t reason_len,
					     const char *text)
{
	if (reason == NULL || reason_len == 0U) {
		return;
	}

	(void)snprintf(reason, reason_len, "%s", text);
}

static bool motor_encoder_control_acquisition_has_stale_errors(
	const struct motor_encoder_acquisition_stats *stats)
{
	return stats->request_error != 0U ||
	       stats->collect_transport_error != 0U ||
	       stats->collect_frame_parity_error != 0U ||
	       stats->collect_frame_crc_error != 0U ||
	       stats->collect_frame_glitch_error != 0U;
}

static int motor_encoder_control_get_status_internal(
	const struct motor_parameters *params,
	bool check_registers,
	bool allow_active_acquisition,
	struct motor_encoder_control_status *status,
	char *reason,
	size_t reason_len)
{
	if (params == NULL || status == NULL) {
		motor_encoder_control_set_reason(reason, reason_len, "motor not initialized");
		return -EINVAL;
	}

	memset(status, 0, sizeof(*status));
	motor_encoder_acquisition_get_stats(&status->acquisition_stats);

	status->device_ready = device_is_ready(encoder1);
	status->mapping_complete = params->calibration.encoder_mapping_complete;
	status->acquisition_idle = !motor_encoder_acquisition_is_enabled() &&
				!motor_encoder_acquisition_is_busy();
	status->injection_disabled =
		motor_encoder_acquisition_get_test_inject_mode() == MOTOR_ENCODER_TEST_INJECT_NONE;
	status->protocol_ok = true;

	if (!status->device_ready) {
		motor_encoder_control_set_reason(reason, reason_len, "encoder1 is not ready");
		return 0;
	}
	if (!status->mapping_complete) {
		motor_encoder_control_set_reason(reason, reason_len,
						 "encoder mapping has not been applied");
		return 0;
	}
	if (!status->acquisition_idle && !allow_active_acquisition) {
		motor_encoder_control_set_reason(reason, reason_len,
						 "encoder acquisition is active or busy");
		return 0;
	}
	if (!status->injection_disabled) {
		motor_encoder_control_set_reason(reason, reason_len,
						 "encoder fault injection is enabled");
		return 0;
	}
	if (!allow_active_acquisition &&
	    motor_encoder_control_acquisition_has_stale_errors(&status->acquisition_stats)) {
		motor_encoder_control_set_reason(reason, reason_len,
						 "encoder acquisition has uncleared hard errors");
		return 0;
	}

#if MOTOR_ENCODER_CONTROL_AEAT_FAST
	enum aeat9955_fast_spi4_mode mode = AEAT9955_FAST_SPI4_16_PARITY;
	int ret = aeat9955_fast_get_spi4_mode(encoder1, &mode);
	if (ret != 0) {
		status->protocol_ok = false;
		status->protocol_error = ret;
		motor_encoder_control_set_reason(reason, reason_len,
						 "failed to read AEAT driver protocol mode");
		return 0;
	}

	if (mode != AEAT9955_FAST_SPI4_8_CRC16) {
		status->protocol_ok = false;
		motor_encoder_control_set_reason(reason, reason_len,
						 "AEAT driver is not in SPI4-8 CRC16 mode");
		return 0;
	}

	if (check_registers) {
		status->protocol_checked = true;
		ret = motor_encoder_control_read_aeat_protocol_regs(status);
		if (ret != 0) {
			status->protocol_ok = false;
			status->protocol_error = ret;
			motor_encoder_control_set_reason(reason, reason_len,
							 "failed to read AEAT protocol registers");
			return 0;
		}

		if (!motor_encoder_control_aeat_regs_match_spi4_8_crc16(status)) {
			/* The AEAT register path can return a stale frame immediately
			 * after realtime position sampling stops. Retry once before
			 * rejecting an otherwise clean encoder-control entry.
			 */
			k_busy_wait(1000);
			ret = motor_encoder_control_read_aeat_protocol_regs(status);
		}

		if (ret != 0 ||
		    !motor_encoder_control_aeat_regs_match_spi4_8_crc16(status)) {
			status->protocol_ok = false;
			status->protocol_error = ret;
			motor_encoder_control_set_reason(reason, reason_len,
							 "AEAT protocol registers are not SPI4-8 CRC16");
			return 0;
		}
	}
#else
	ARG_UNUSED(check_registers);
#endif

	status->ready = status->device_ready &&
			status->mapping_complete &&
			(status->acquisition_idle || allow_active_acquisition) &&
			status->injection_disabled &&
			status->protocol_ok &&
			(allow_active_acquisition ||
			 !motor_encoder_control_acquisition_has_stale_errors(
				 &status->acquisition_stats));
	if (status->ready) {
		motor_encoder_control_set_reason(reason, reason_len, "ready");
	}
	return 0;
}

int motor_encoder_control_get_status(const struct motor_parameters *params,
				     bool check_registers,
				     struct motor_encoder_control_status *status,
				     char *reason,
				     size_t reason_len)
{
	return motor_encoder_control_get_status_internal(params, check_registers, false,
							 status, reason, reason_len);
}

bool motor_encoder_control_ready_for_mode(const struct motor_parameters *params,
					  enum motor_state state,
					  bool check_registers,
					  char *reason,
					  size_t reason_len)
{
	if (!motor_encoder_control_mode_requires_encoder(state)) {
		motor_encoder_control_set_reason(reason, reason_len, "not an encoder mode");
		return true;
	}

	struct motor_encoder_control_status status = {0};
	int ret = motor_encoder_control_get_status(params, check_registers, &status,
						   reason, reason_len);
	return ret == 0 && status.ready;
}

bool motor_encoder_control_ready_for_transition(const struct motor_parameters *params,
						enum motor_state current_state,
						enum motor_state target_state,
						bool check_registers,
						char *reason,
						size_t reason_len)
{
	if (!motor_encoder_control_mode_requires_encoder(target_state)) {
		motor_encoder_control_set_reason(reason, reason_len, "not an encoder mode");
		return true;
	}

	bool allow_active_acquisition =
		motor_encoder_control_mode_requires_encoder(current_state);
	struct motor_encoder_control_status status = {0};
	int ret = motor_encoder_control_get_status_internal(params,
							    check_registers &&
								    !allow_active_acquisition,
							    allow_active_acquisition,
							    &status, reason,
							    reason_len);
	return ret == 0 && status.ready;
}
