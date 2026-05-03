/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/calibration/timing.h"

#include <errno.h>
#include <limits.h>
#include <math.h>

int motor_calibration_timer_start_s(struct k_timer *timer, float32_t duration_s)
{
	if (timer == NULL || !isfinite(duration_s) || duration_s <= 0.0f) {
		return -EINVAL;
	}

	const double ms_d = ceil((double)duration_s * 1000.0);
	if (!isfinite(ms_d) || ms_d > (double)UINT32_MAX) {
		return -ERANGE;
	}

	uint32_t duration_ms = (uint32_t)ms_d;
	if (duration_ms == 0U) {
		duration_ms = 1U;
	}

	k_timer_start(timer, K_MSEC(duration_ms), K_NO_WAIT);
	return 0;
}

bool motor_calibration_timer_has_elapsed(struct k_timer *timer,
					 bool timeout_event_seen,
					 bool *stale_expiry_out)
{
	if (stale_expiry_out != NULL) {
		*stale_expiry_out = false;
	}
	if (timer == NULL) {
		return false;
	}

	if (k_timer_status_get(timer) > 0U) {
		if (stale_expiry_out != NULL) {
			*stale_expiry_out = !timeout_event_seen;
		}
		return true;
	}

	return false;
}
