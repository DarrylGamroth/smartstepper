/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_STATE_H_
#define MOTOR_RUNTIME_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/utils.h>
#include <zephyr/sys/atomic.h>

/**
 * @brief Runtime state intended for ISR-rate updates.
 *
 * Update cadence:
 * 1. Written/read at control-loop ISR rate.
 * 2. Keep fields small and hot-path relevant.
 */
struct motor_rt_fast_state {
	uint32_t control_loop_count;
	uint32_t rls_d_prev_cycle;
	uint32_t rls_q_prev_cycle;
	uint8_t rls_d_prev_valid;
	uint8_t rls_q_prev_valid;
	float32_t Id_setpoint_A;
	float32_t Iq_setpoint_A;
	float32_t Vd_V;
	float32_t Vq_V;
	atomic_val_t feature_flags_shadow;
	bool control_armed_shadow;
};

#endif /* MOTOR_RUNTIME_STATE_H_ */
