/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_RUNTIME_IO_H_
#define MOTOR_RUNTIME_IO_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dsp/types.h>

struct motor_control_encoder_sample {
	bool enabled;
	bool fresh;
	bool warning;
	bool error;
	bool io_fault;
	uint8_t status;
	float32_t angle_deg;
};

struct motor_control_pwm_output {
	bool update_pwm;
	float32_t da_hb1_pu;
	float32_t da_hb2_pu;
	float32_t db_hb1_pu;
	float32_t db_hb2_pu;
};

#endif /* MOTOR_RUNTIME_IO_H_ */
