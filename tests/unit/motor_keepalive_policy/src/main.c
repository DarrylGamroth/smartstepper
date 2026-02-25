/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "motor/runtime/keepalive_policy.h"

ZTEST(motor_keepalive_policy, test_keepalive_false_when_disarmed)
{
	zassert_false(motor_keepalive_policy_should_keepalive(false, true, true, true, true), NULL);
}

ZTEST(motor_keepalive_policy, test_keepalive_false_when_no_autonomous_sources)
{
	zassert_false(motor_keepalive_policy_should_keepalive(true, false, false, false, false), NULL);
}

ZTEST(motor_keepalive_policy, test_keepalive_true_for_autonomous_mode)
{
	zassert_true(motor_keepalive_policy_should_keepalive(true, true, false, false, false), NULL);
}

ZTEST(motor_keepalive_policy, test_keepalive_true_for_sequence_run)
{
	zassert_true(motor_keepalive_policy_should_keepalive(true, false, true, false, false), NULL);
}

ZTEST(motor_keepalive_policy, test_keepalive_true_for_chopper_calibration)
{
	zassert_true(motor_keepalive_policy_should_keepalive(true, false, false, true, false), NULL);
}

ZTEST(motor_keepalive_policy, test_keepalive_true_for_quintic_profile)
{
	zassert_true(motor_keepalive_policy_should_keepalive(true, false, false, false, true), NULL);
}

ZTEST_SUITE(motor_keepalive_policy, NULL, NULL, NULL, NULL, NULL);
