/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/motion/motor_autonomy.h"

bool motor_autonomy_should_keepalive(bool control_armed, bool autonomous_mode_active,
				     bool profile_sequence_running, bool chopper_cal_active,
				     bool quintic_profile_active)
{
	return control_armed && (autonomous_mode_active || profile_sequence_running ||
				 chopper_cal_active || quintic_profile_active);
}
