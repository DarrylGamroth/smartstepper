/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_AUTONOMY_H_
#define MOTOR_AUTONOMY_H_

#include <stdbool.h>
#include <zephyr/smf.h>

struct motor_parameters;

/**
 * @brief Return true when timeout keepalive should be auto-petted.
 *
 * This is used by the ISR timeout gate and shell status reporting so both
 * paths use identical autonomy criteria.
 */
bool motor_autonomous_keepalive_active(const struct motor_parameters *params,
				       const struct smf_state *state,
				       bool control_armed);

#endif /* MOTOR_AUTONOMY_H_ */
