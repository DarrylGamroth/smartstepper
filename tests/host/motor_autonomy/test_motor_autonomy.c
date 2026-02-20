/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <stdbool.h>

#include "motor_autonomy.h"

int main(void)
{
	assert(motor_autonomy_should_keepalive(false, true, true, true, true) == false);
	assert(motor_autonomy_should_keepalive(true, false, false, false, false) == false);
	assert(motor_autonomy_should_keepalive(true, true, false, false, false) == true);
	assert(motor_autonomy_should_keepalive(true, false, true, false, false) == true);
	assert(motor_autonomy_should_keepalive(true, false, false, true, false) == true);
	assert(motor_autonomy_should_keepalive(true, false, false, false, true) == true);
	return 0;
}
