/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/control/foc_voltage_pwm.h"

#include <errno.h>
#include <math.h>

#include "motor/control/current_loop.h"
#include "motor/control/dq_decoupling.h"
#include "motor/control/pwm_synthesis.h"
#include "motor/control/transforms.h"

int motor_foc_voltage_pwm_step(struct pi_f32 *pi_id, struct pi_f32 *pi_iq,
			       const struct motor_foc_voltage_pwm_inputs *in,
			       struct motor_foc_voltage_pwm_outputs *out)
{
	if (pi_id == NULL || pi_iq == NULL || in == NULL || out == NULL) {
		return -EINVAL;
	}
	if (!isfinite(in->vbus_v) || in->vbus_v <= 0.0f) {
		return -EINVAL;
	}
	if (!isfinite(in->max_modulation_index) || in->max_modulation_index <= 0.0f ||
	    in->max_modulation_index > 1.0f) {
		return -EINVAL;
	}
	if (!isfinite(in->inv_park_angle_rad) || !isfinite(in->id_ref_a) || !isfinite(in->iq_ref_a) ||
	    !isfinite(in->id_a) || !isfinite(in->iq_a)) {
		return -EINVAL;
	}
	if (in->braking_enabled &&
	    (!isfinite(in->braking_iq_ref_a) || !isfinite(in->braking_speed_rad_s) ||
	     !isfinite(in->braking_vbus_limit_v) || !isfinite(in->braking_vbus_margin_inv) ||
	     in->braking_vbus_limit_v < 0.0f || in->braking_vbus_margin_inv <= 0.0f)) {
		return -EINVAL;
	}

	float32_t max_voltage_magnitude_v = in->max_modulation_index * in->vbus_v;
	if (!isfinite(max_voltage_magnitude_v) || max_voltage_magnitude_v <= 0.0f) {
		return -ERANGE;
	}

	struct motor_dq_decoupling_feedforward_input decoupling_in = {
		.enabled = in->dq_decoupling_enabled,
		.electrical_speed_rad_s = in->electrical_speed_rad_s,
		.ld_h = in->ld_h,
		.lq_h = in->lq_h,
		.flux_linkage_wb = in->flux_linkage_wb,
		.id_a = in->id_a,
		.iq_a = in->iq_a,
		.max_voltage_magnitude_v = max_voltage_magnitude_v,
		.flux_headroom_ratio = in->dq_decoupling_flux_headroom_ratio,
		.ff_limit_ratio = in->dq_decoupling_ff_limit_ratio,
	};
	struct motor_dq_decoupling_feedforward_output decoupling_out = {0};
	int decoupling_ret = motor_dq_decoupling_feedforward_step(&decoupling_in, &decoupling_out);
	if (decoupling_ret != 0) {
		return decoupling_ret;
	}

	out->max_voltage_magnitude_v = max_voltage_magnitude_v;
	out->vd_ff_v = decoupling_out.vd_ff_v;
	out->vq_ff_v = decoupling_out.vq_ff_v;

	struct motor_current_loop_input current_loop_in = {
		.id_ref_a = in->id_ref_a,
		.iq_ref_a = in->iq_ref_a,
		.id_a = in->id_a,
		.iq_a = in->iq_a,
		.max_voltage_magnitude_v = max_voltage_magnitude_v,
		.vd_ff_v = out->vd_ff_v,
		.vq_ff_v = out->vq_ff_v,
	};
	struct motor_current_loop_output current_loop_out = {0};
	int current_loop_ret =
		motor_current_loop_step(pi_id, pi_iq, &current_loop_in, &current_loop_out);
	if (current_loop_ret != 0) {
		return current_loop_ret;
	}
	out->vd_v = current_loop_out.vd_v;
	out->vq_v = current_loop_out.vq_v;
	out->vq_limit_v = current_loop_out.vq_limit_v;

	int xform_ret = motor_transforms_inv_park(out->vd_v, out->vq_v, in->inv_park_angle_rad,
						  &out->va_v, &out->vb_v);
	if (xform_ret != 0) {
		return xform_ret;
	}

	struct motor_pwm_synthesis_input pwm_in = {
		.va_v = out->va_v,
		.vb_v = out->vb_v,
		.vbus_v = in->vbus_v,
		.braking_enabled = in->braking_enabled,
		.braking_iq_ref_a = in->braking_iq_ref_a,
		.braking_speed_rad_s = in->braking_speed_rad_s,
		.braking_vbus_limit_v = in->braking_vbus_limit_v,
		.braking_vbus_margin_inv = in->braking_vbus_margin_inv,
	};
	struct motor_pwm_synthesis_output pwm_out = {0};
	int pwm_ret = motor_pwm_synthesis_step(&pwm_in, &pwm_out);
	if (pwm_ret != 0) {
		return pwm_ret;
	}
	out->ua_pu = pwm_out.ua_pu;
	out->ub_pu = pwm_out.ub_pu;
	out->da_pu = pwm_out.da_pu;
	out->db_pu = pwm_out.db_pu;
	out->da_hb1_pu = pwm_out.da_hb1_pu;
	out->da_hb2_pu = pwm_out.da_hb2_pu;
	out->db_hb1_pu = pwm_out.db_hb1_pu;
	out->db_hb2_pu = pwm_out.db_hb2_pu;

	return 0;
}
