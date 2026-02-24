/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_FOC_VOLTAGE_PWM_H_
#define MOTOR_FOC_VOLTAGE_PWM_H_

#include <stdbool.h>
#include <zephyr/dsp/types.h>

#include "motor/filters/pi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inputs for one current-loop PI to PWM synthesis step.
 */
struct motor_foc_voltage_pwm_inputs {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	float32_t id_a;
	float32_t iq_a;
	float32_t vbus_v;
	float32_t max_modulation_index;
	float32_t inv_park_angle_rad;
	bool decoupling_enabled;
	float32_t electrical_speed_rad_s;
	float32_t ld_h;
	float32_t lq_h;
	float32_t flux_linkage_wb;

	bool braking_enabled;
	float32_t braking_iq_ref_a;
	float32_t braking_speed_rad_s;
	float32_t braking_vbus_limit_v;
	float32_t braking_vbus_margin_inv;
};

/**
 * @brief Outputs from one current-loop PI to PWM synthesis step.
 */
struct motor_foc_voltage_pwm_outputs {
	float32_t vd_v;
	float32_t vq_v;
	float32_t va_v;
	float32_t vb_v;
	float32_t ua_pu;
	float32_t ub_pu;
	float32_t da_pu;
	float32_t db_pu;
	float32_t da_hb1_pu;
	float32_t da_hb2_pu;
	float32_t db_hb1_pu;
	float32_t db_hb2_pu;
	float32_t vd_ff_v;
	float32_t vq_ff_v;
	float32_t max_voltage_magnitude_v;
	float32_t vq_limit_v;
};

/**
 * @brief Run PI current control and synthesize complementary PWM duties.
 *
 * This function is side-effect free except for updating PI controller states.
 *
 * @param pi_id d-axis PI controller
 * @param pi_iq q-axis PI controller
 * @param in Input data for this control step
 * @param out Output data populated on success
 * @return 0 on success, negative errno on invalid inputs
 */
int motor_foc_voltage_pwm_step(struct pi_f32 *pi_id, struct pi_f32 *pi_iq,
			       const struct motor_foc_voltage_pwm_inputs *in,
			       struct motor_foc_voltage_pwm_outputs *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_FOC_VOLTAGE_PWM_H_ */
