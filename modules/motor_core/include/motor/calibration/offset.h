/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_CALIBRATION_OFFSET_H_
#define MOTOR_CALIBRATION_OFFSET_H_

#include <zephyr/dsp/types.h>

#include "motor/filters/filter_fo.h"

int motor_offset_measurement_start(struct filter_fo_f32 *filter_ia,
				   struct filter_fo_f32 *filter_ib,
				   float32_t *ia_offset_a,
				   float32_t *ib_offset_a);

int motor_offset_measurement_finalize(const struct filter_fo_f32 *filter_ia,
				      const struct filter_fo_f32 *filter_ib,
				      float32_t *ia_offset_a,
				      float32_t *ib_offset_a);

#endif /* MOTOR_CALIBRATION_OFFSET_H_ */
