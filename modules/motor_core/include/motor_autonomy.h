/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_AUTONOMY_H_
#define MOTOR_AUTONOMY_H_

#include <stdbool.h>

/**
 * @brief Return true when timeout keepalive should be auto-petted.
 *
 * Keepalive is active only when control is armed and at least one autonomous
 * motion source is active.
 */
bool motor_autonomy_should_keepalive(bool control_armed, bool autonomous_mode_active,
				     bool profile_sequence_running, bool chopper_cal_active,
				     bool quintic_profile_active);

#endif /* MOTOR_AUTONOMY_H_ */
