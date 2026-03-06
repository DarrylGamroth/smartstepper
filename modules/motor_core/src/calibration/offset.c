/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/calibration/offset.h"

#include <errno.h>
#include <math.h>

int motor_offset_measurement_start(struct filter_fo_f32 *filter_ia,
				   struct filter_fo_f32 *filter_ib,
				   float32_t *ia_offset_a,
				   float32_t *ib_offset_a)
{
	if (filter_ia == NULL || filter_ib == NULL ||
	    ia_offset_a == NULL || ib_offset_a == NULL) {
		return -EINVAL;
	}

	*ia_offset_a = 0.0f;
	*ib_offset_a = 0.0f;

	filter_fo_set_initial_conditions(filter_ia, 0.0f, 0.0f);
	filter_fo_set_initial_conditions(filter_ib, 0.0f, 0.0f);
	return 0;
}

int motor_offset_measurement_finalize(const struct filter_fo_f32 *filter_ia,
				      const struct filter_fo_f32 *filter_ib,
				      float32_t *ia_offset_a,
				      float32_t *ib_offset_a)
{
	if (filter_ia == NULL || filter_ib == NULL ||
	    ia_offset_a == NULL || ib_offset_a == NULL) {
		return -EINVAL;
	}

	const float32_t ia = filter_fo_get_y1(filter_ia);
	const float32_t ib = filter_fo_get_y1(filter_ib);
	if (!isfinite(ia) || !isfinite(ib)) {
		return -ERANGE;
	}

	*ia_offset_a = ia;
	*ib_offset_a = ib;
	return 0;
}
