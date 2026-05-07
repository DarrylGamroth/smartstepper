/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "config.h"
#include "motor/control/current_slew.h"
#include "motor/estimation/electrical_id.h"
#include "motor/estimation/saliency_id.h"
#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"
#include "motor/motion/angle_gen.h"
#include "motor/observers/angle_observer.h"
#include "motor_current_slew.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "shell_commands_commission.h"
#include "shell_commands_motion.h"
#include "shell_commands_state.h"
#include "shell_commission_internal.h"
#include "shell_parse.h"

struct electrical_id_staged_state {
	struct motor_electrical_id_rs_result rs;
	struct motor_electrical_id_l_result ld;
	struct motor_electrical_id_l_result lq;
	struct motor_electrical_id_demod_result ld_demod;
	struct motor_electrical_id_demod_result lq_demod;
	struct motor_saliency_id_result saliency;
	struct motor_electrical_id_result result;
	struct motor_electrical_id_pi_recommendation pi;
	bool rs_valid;
	bool ld_valid;
	bool lq_valid;
	bool result_valid;
	bool pi_valid;
	bool inductance_fallback;
	bool inductance_roverl;
	bool inductance_demod;
	bool inductance_saliency;
};

static struct electrical_id_staged_state staged;

static void electrical_id_stop_direct_voltage(void);

#define COMMISSION_ELECTRICAL_SALIENCY_VECTORS_DEFAULT 32U
#define COMMISSION_ELECTRICAL_SALIENCY_PAIRS_DEFAULT 4U
#define COMMISSION_ELECTRICAL_SALIENCY_REVS_DEFAULT 2U
#define COMMISSION_ELECTRICAL_SALIENCY_MAX_VECTORS 128U
#define COMMISSION_ELECTRICAL_SALIENCY_MAX_PAIRS 32U
#define COMMISSION_ELECTRICAL_SALIENCY_MAX_REVS 8U
#define COMMISSION_ELECTRICAL_SALIENCY_MIN_SETTLE_TICKS 20U
#define COMMISSION_ELECTRICAL_SALIENCY_MAX_SETTLE_TICKS 1000U

static struct motor_saliency_id_bin saliency_bins[COMMISSION_ELECTRICAL_SALIENCY_MAX_VECTORS];

struct electrical_id_l_segment_sample {
	float32_t flux_vs;
	float32_t delta_current_a;
	uint32_t samples;
};

static struct motor_electrical_id_limits electrical_id_limits(void)
{
	float32_t rs = MOTOR_RESISTANCE_OHM;
	float32_t ld = MOTOR_INDUCTANCE_D_H;
	float32_t lq = MOTOR_INDUCTANCE_Q_H;
	float32_t l_nom = fmaxf(fminf(ld, lq), 1.0e-6f);

	return (struct motor_electrical_id_limits){
		.rs_min_ohm = fmaxf(rs * 0.25f, 0.01f),
		.rs_max_ohm = fmaxf(rs * 4.0f, 0.05f),
		.l_min_h = fmaxf(l_nom * 0.20f, 1.0e-6f),
		.l_max_h = fmaxf(fmaxf(ld, lq) * 5.0f, 5.0e-6f),
		.max_axis_mismatch_ratio = 0.90f,
		.min_confidence = 0.10f,
	};
}

static float32_t electrical_id_current_limit(void)
{
	return fminf(COMMISSION_ELECTRICAL_CURRENT_LIMIT_A, MOTOR_MAX_CURRENT_A);
}

static float32_t electrical_id_voltage_pulse_limit(void)
{
	float32_t current_based_v = fmaxf(staged.rs.rs_ohm, MOTOR_RESISTANCE_OHM) *
				    electrical_id_current_limit() * 1.25f;

	return clampf(current_based_v, COMMISSION_ELECTRICAL_MIN_PULSE_V, COMMISSION_ELECTRICAL_MAX_PULSE_V);
}

static int parse_optional_float(char **argv, size_t argc, size_t idx,
				float32_t fallback, float32_t *out)
{
	if (idx >= argc) {
		*out = fallback;
		return 0;
	}
	if (!shell_parse_finite_float(argv[idx], out)) {
		return -EINVAL;
	}
	return 0;
}

static int parse_optional_u32(char **argv, size_t argc, size_t idx,
			      uint32_t fallback, uint32_t *out)
{
	if (idx >= argc) {
		*out = fallback;
		return 0;
	}
	if (!shell_parse_u32(argv[idx], out)) {
		return -EINVAL;
	}
	return 0;
}

static int electrical_id_enter_generated_current_mode(const struct shell *sh)
{
	int ret = motor_commission_prepare_idle_zero_current(COMMISSION_ELECTRICAL_SETTLE_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to prepare idle zero-current state (err %d)", ret);
		return ret;
	}

	if (!g_motor_params->calibration.complete) {
		shell_print(sh, "Current offset calibration required before production electrical ID.");
		ret = motor_api_request_calibrate();
		if (ret != 0) {
			shell_error(sh, "Failed to request current offset calibration (err %d)", ret);
			return ret;
		}
		int64_t deadline = k_uptime_get() + 5000;
		while (k_uptime_get() < deadline) {
			int state = motor_api_get_state();
			if (state == MOTOR_STATE_ERROR) {
				shell_error(sh, "Current offset calibration entered ERROR state");
				return -EIO;
			}
			if (g_motor_params->calibration.complete && state == MOTOR_STATE_IDLE) {
				shell_print(sh, "Current offsets ready: Ia=%.4f Ib=%.4f",
					    (double)g_motor_params->Ia_offset,
					    (double)g_motor_params->Ib_offset);
				break;
			}
			motor_command_feed_watchdog(g_motor_params);
			k_msleep(10);
		}
		if (!g_motor_params->calibration.complete) {
			shell_error(sh, "Current offset calibration timed out");
			return -ETIMEDOUT;
		}
		ret = motor_commission_prepare_idle_zero_current(COMMISSION_ELECTRICAL_SETTLE_MS);
		if (ret != 0) {
			shell_error(sh, "Failed to re-enter idle after current offsets (err %d)", ret);
			return ret;
		}
	}

	ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED);
	if (ret != 0) {
		shell_error(sh, "Failed to request velocity_generated mode (err %d)", ret);
		return ret;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_GENERATED, 3000U);
	if (ret != 0) {
		shell_error(sh, "Timed out entering velocity_generated mode (err %d)", ret);
		return ret;
	}

	motor_commission_set_velocity_target_hz(0.0f);
	ret = cmd_motor_arm(sh, 0, NULL);
	if (ret != 0) {
		shell_error(sh, "Failed to arm control (err %d)", ret);
		return ret;
	}

	return 0;
}

static void electrical_id_stop(void)
{
	if (g_motor_params == NULL) {
		return;
	}

	electrical_id_stop_direct_voltage();
	motor_commission_set_velocity_target_hz(0.0f);
	motor_current_slew_params_set_target_ramp(g_motor_params, 0.0f, 0.0f,
						    COMMISSION_ELECTRICAL_CURRENT_RAMP_MS / 1000.0f);
	motor_command_feed_watchdog(g_motor_params);
	(void)motor_commission_wait_ms_or_fault(COMMISSION_ELECTRICAL_CURRENT_RAMP_MS + 50U);
	atomic_set(&g_motor_params->control_armed, 0);
	(void)motor_commission_request_idle_disarmed();
}

static void electrical_id_force_current(float32_t id_a, float32_t iq_a)
{
	g_motor_params->Id_setpoint_A = id_a;
	g_motor_params->Iq_setpoint_A = iq_a;
	struct motor_current_slew_pair slew = motor_current_slew_from_params(g_motor_params);

	motor_current_slew_force(&slew, id_a, iq_a);
	motor_command_feed_watchdog(g_motor_params);
}

static void electrical_id_capture_reset_common(uint8_t mode, uint32_t target_samples)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	cap->active = false;
	cap->done = false;
	cap->valid = false;
	cap->mode = mode;
	cap->target_samples = target_samples;
	cap->sample_count = 0U;
	cap->rejected_samples = 0U;
	cap->update_count = 0U;
	cap->prev_valid = false;
	cap->direct_voltage_enabled = false;
	cap->prev_current_a = 0.0f;
	cap->validation_target_a = 0.0f;
	cap->vd_cmd_v = 0.0f;
	cap->vq_cmd_v = 0.0f;
	cap->voltage_limit_v = 0.0f;
	cap->l_segment_active = false;
	cap->l_segment_latch_pending = false;
	cap->l_segment_start_current_a = 0.0f;
	cap->l_segment_last_current_a = 0.0f;
	cap->l_segment_flux_vs = 0.0f;
	cap->l_segment_samples = 0U;
	cap->l_rejected_segments = 0U;
	cap->validation_sum_abs_error_a = 0.0f;
	cap->validation_max_abs_error_a = 0.0f;
}

static void electrical_id_start_rs_capture(float32_t current_a, uint32_t samples, bool reset_accum)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	electrical_id_capture_reset_common(MOTOR_ELECTRICAL_ID_CAPTURE_RS, samples);
	cap->rs_cfg = (struct motor_electrical_id_rs_config){
		.min_abs_current_a = fmaxf(fabsf(current_a) * 0.25f, 0.002f),
		.min_samples = (uint16_t)MAX(COMMISSION_ELECTRICAL_MIN_SAMPLES, samples / 2U),
		.max_residual_ratio = 0.25f,
	};
	if (reset_accum) {
		motor_electrical_id_rs_reset(&cap->rs_accum);
	}
	cap->active = true;
}

static void electrical_id_start_l_capture(uint8_t mode, float32_t rs_ohm,
					  float32_t voltage_limit_v, uint32_t samples)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;
	float32_t dt_s = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;

	electrical_id_capture_reset_common(mode, samples);
	cap->rs_ohm = rs_ohm;
	cap->l_cfg = (struct motor_electrical_id_l_config){
		.dt_s = dt_s,
		.min_abs_di_dt_a_per_s = 2.0f,
		.min_abs_delta_current_a = 0.005f,
		.min_samples = (uint16_t)MAX(COMMISSION_ELECTRICAL_MIN_SAMPLES, samples / 2U),
		.max_residual_ratio = 0.35f,
	};
	motor_electrical_id_l_reset(&cap->l_accum);
	cap->direct_voltage_enabled = true;
	cap->voltage_limit_v = voltage_limit_v;
	cap->active = true;
}

static void electrical_id_set_direct_voltage(float32_t vd_v, float32_t vq_v)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	cap->vd_cmd_v = vd_v;
	cap->vq_cmd_v = vq_v;
	motor_command_feed_watchdog(g_motor_params);
}

static void electrical_id_begin_l_segment(void)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	cap->l_segment_active = true;
	cap->l_segment_latch_pending = true;
	cap->l_segment_start_current_a = 0.0f;
	cap->l_segment_last_current_a = 0.0f;
	cap->l_segment_flux_vs = 0.0f;
	cap->l_segment_samples = 0U;
}

static int electrical_id_end_l_segment_sample(struct electrical_id_l_segment_sample *sample)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	if (sample != NULL) {
		memset(sample, 0, sizeof(*sample));
	}

	cap->l_segment_active = false;
	if (cap->l_segment_latch_pending || cap->l_segment_samples == 0U) {
		cap->l_rejected_segments++;
		return 1;
	}

	float32_t delta_current_a = cap->l_segment_last_current_a -
				    cap->l_segment_start_current_a;

	if (sample != NULL) {
		sample->flux_vs = cap->l_segment_flux_vs;
		sample->delta_current_a = delta_current_a;
		sample->samples = cap->l_segment_samples;
	}

	return 0;
}

static int electrical_id_end_l_segment(void)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;
	struct electrical_id_l_segment_sample sample;

	int ret = electrical_id_end_l_segment_sample(&sample);
	if (ret != 0) {
		return ret;
	}

	ret = motor_electrical_id_l_add_integral(&cap->l_accum,
						 &cap->l_cfg,
						 sample.flux_vs,
						 sample.delta_current_a);
	if (ret == 0) {
		cap->sample_count++;
		if (cap->sample_count >= cap->target_samples) {
			cap->active = false;
			cap->done = true;
			cap->valid = true;
		}
	} else if (ret > 0) {
		cap->rejected_samples++;
	} else {
		cap->rejected_samples++;
	}

	return ret;
}

static void electrical_id_stop_direct_voltage(void)
{
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	cap->vd_cmd_v = 0.0f;
	cap->vq_cmd_v = 0.0f;
	cap->direct_voltage_enabled = false;
	cap->l_segment_active = false;
	cap->l_segment_latch_pending = false;
	motor_command_feed_watchdog(g_motor_params);
}

static void electrical_id_start_validate_capture(float32_t target_a, uint32_t hold_ms)
{
	uint32_t samples = MAX(1U,
		(uint32_t)((CONTROL_LOOP_FREQUENCY_HZ * (float32_t)hold_ms) / 1000.0f));
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	electrical_id_capture_reset_common(MOTOR_ELECTRICAL_ID_CAPTURE_VALIDATE_ID, samples);
	cap->validation_target_a = target_a;
	cap->active = true;
}

static int electrical_id_wait_capture_done(uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	while (k_uptime_get() < deadline) {
		if (cap->done) {
			return cap->valid ? 0 : -EIO;
		}
		motor_command_feed_watchdog(g_motor_params);
		int ret = motor_commission_wait_ms_or_fault(1U);
		if (ret != 0) {
			cap->active = false;
			return ret;
		}
	}

	cap->active = false;
	return -ETIMEDOUT;
}

static int electrical_id_wait_control_ticks_or_fault(uint32_t ticks)
{
	if (ticks == 0U) {
		return 0;
	}

	uint32_t start_loop = g_motor_params->control_loop_count;

	while ((g_motor_params->control_loop_count - start_loop) < ticks) {
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_yield();
	}

	return 0;
}

static int electrical_id_collect_rs(float32_t current_a, uint32_t samples,
					   uint32_t settle_ms,
					   struct motor_electrical_id_rs_result *out)
{
	struct motor_electrical_id_limits limits = electrical_id_limits();
	uint32_t capture_settle_ms = MAX(settle_ms, COMMISSION_ELECTRICAL_CURRENT_RAMP_MS + 50U);

	motor_current_slew_params_set_target_ramp(g_motor_params, current_a, 0.0f,
						    COMMISSION_ELECTRICAL_CURRENT_RAMP_MS / 1000.0f);
	motor_command_feed_watchdog(g_motor_params);
	int ret = motor_commission_wait_ms_or_fault(capture_settle_ms);
	if (ret != 0) {
		return ret;
	}

	electrical_id_start_rs_capture(current_a, samples, true);
	ret = electrical_id_wait_capture_done(1000U);
	if (ret != 0) {
		return ret;
	}

	motor_current_slew_params_set_target_ramp(g_motor_params, -current_a, 0.0f,
						    COMMISSION_ELECTRICAL_CURRENT_RAMP_MS / 1000.0f);
	motor_command_feed_watchdog(g_motor_params);
	ret = motor_commission_wait_ms_or_fault(capture_settle_ms);
	if (ret != 0) {
		return ret;
	}

	electrical_id_start_rs_capture(-current_a, samples, false);
	ret = electrical_id_wait_capture_done(1000U);
	if (ret != 0) {
		return ret;
	}

	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	return motor_electrical_id_rs_finalize(&cap->rs_accum, &cap->rs_cfg, &limits, out);
}

static int electrical_id_collect_l_axis(bool q_axis, float32_t pulse_v,
					uint32_t pulse_ms,
					uint32_t repeats,
					struct motor_electrical_id_l_result *out)
{
	struct motor_electrical_id_limits limits = electrical_id_limits();
	uint8_t mode = q_axis ? MOTOR_ELECTRICAL_ID_CAPTURE_LQ : MOTOR_ELECTRICAL_ID_CAPTURE_LD;
	uint32_t max_pulses = MAX(16U, repeats * 2U);
	float32_t voltage_limit_v = electrical_id_voltage_pulse_limit();
	float32_t limited_pulse_v = clampf(pulse_v, -voltage_limit_v, voltage_limit_v);

	electrical_id_force_current(0.0f, 0.0f);
	int ret = motor_commission_wait_ms_or_fault(COMMISSION_ELECTRICAL_SETTLE_MS);
	if (ret != 0) {
		return ret;
	}

	electrical_id_start_l_capture(mode, staged.rs.rs_ohm, voltage_limit_v, repeats);
	for (uint32_t i = 0U; i < max_pulses; ++i) {
		float32_t sign = (i & 1U) ? -1.0f : 1.0f;
		float32_t vd_v = q_axis ? 0.0f : sign * limited_pulse_v;
		float32_t vq_v = q_axis ? sign * limited_pulse_v : 0.0f;

		electrical_id_set_direct_voltage(vd_v, vq_v);
		electrical_id_begin_l_segment();
		int ret = motor_commission_wait_ms_or_fault(pulse_ms);
		if (ret != 0) {
			electrical_id_stop_direct_voltage();
			g_motor_params->electrical_id_capture.active = false;
			return ret;
		}
		(void)electrical_id_end_l_segment();
		electrical_id_set_direct_voltage(0.0f, 0.0f);
		ret = motor_commission_wait_ms_or_fault(1U);
		if (ret != 0) {
			electrical_id_stop_direct_voltage();
			g_motor_params->electrical_id_capture.active = false;
			return ret;
		}
		if (g_motor_params->electrical_id_capture.done) {
			break;
		}
	}
	electrical_id_stop_direct_voltage();
	if (!g_motor_params->electrical_id_capture.done) {
		g_motor_params->electrical_id_capture.active = false;
		return -ETIMEDOUT;
	}

	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	return motor_electrical_id_l_finalize(&cap->l_accum, &cap->l_cfg, &limits, out);
}

static int electrical_id_collect_l_axis_demod_cycles(
	bool q_axis,
	float32_t pulse_v,
	uint32_t half_cycles,
	uint32_t repeats,
	struct motor_electrical_id_demod_result *out)
{
	struct motor_electrical_id_limits limits = electrical_id_limits();
	uint8_t mode = q_axis ? MOTOR_ELECTRICAL_ID_CAPTURE_LQ : MOTOR_ELECTRICAL_ID_CAPTURE_LD;
	uint32_t max_pairs = MAX(16U, repeats + 2U);
	float32_t voltage_limit_v = electrical_id_voltage_pulse_limit();
	float32_t limited_pulse_v = clampf(pulse_v, -voltage_limit_v, voltage_limit_v);
	float32_t half_period_s = (float32_t)half_cycles / CONTROL_LOOP_FREQUENCY_HZ;
	struct motor_electrical_id_demod_config cfg = {
		.min_abs_flux_vs = fmaxf(fabsf(limited_pulse_v) * half_period_s * 0.02f,
					 1.0e-7f),
		.max_spread_ratio = COMMISSION_ELECTRICAL_DEMOD_MAX_SPREAD_RATIO,
		.scale_factor = COMMISSION_ELECTRICAL_DEMOD_SCALE_FACTOR,
		.min_samples = (uint16_t)MAX(COMMISSION_ELECTRICAL_MIN_SAMPLES, repeats / 2U),
	};
	struct motor_electrical_id_demod_accum accum;

	motor_electrical_id_demod_reset(&accum);
	electrical_id_force_current(0.0f, 0.0f);
	int ret = motor_commission_wait_ms_or_fault(COMMISSION_ELECTRICAL_SETTLE_MS);
	if (ret != 0) {
		return ret;
	}

	electrical_id_start_l_capture(mode, staged.rs.rs_ohm, voltage_limit_v, repeats);
	for (uint32_t i = 0U; i < max_pairs; ++i) {
		float32_t vd_pos_v = q_axis ? 0.0f : limited_pulse_v;
		float32_t vq_pos_v = q_axis ? limited_pulse_v : 0.0f;
		float32_t vd_neg_v = q_axis ? 0.0f : -limited_pulse_v;
		float32_t vq_neg_v = q_axis ? -limited_pulse_v : 0.0f;
		struct electrical_id_l_segment_sample pos;
		struct electrical_id_l_segment_sample neg;

		electrical_id_set_direct_voltage(vd_pos_v, vq_pos_v);
		electrical_id_begin_l_segment();
		ret = electrical_id_wait_control_ticks_or_fault(half_cycles);
		if (ret != 0) {
			electrical_id_stop_direct_voltage();
			g_motor_params->electrical_id_capture.active = false;
			return ret;
		}
		ret = electrical_id_end_l_segment_sample(&pos);
		if (ret < 0) {
			electrical_id_stop_direct_voltage();
			g_motor_params->electrical_id_capture.active = false;
			return ret;
		}

		electrical_id_set_direct_voltage(vd_neg_v, vq_neg_v);
		electrical_id_begin_l_segment();
		ret = electrical_id_wait_control_ticks_or_fault(half_cycles);
		if (ret != 0) {
			electrical_id_stop_direct_voltage();
			g_motor_params->electrical_id_capture.active = false;
			return ret;
		}
		ret = electrical_id_end_l_segment_sample(&neg);
		if (ret < 0) {
			electrical_id_stop_direct_voltage();
			g_motor_params->electrical_id_capture.active = false;
			return ret;
		}

		if (i == 0U) {
			continue;
		}

		ret = motor_electrical_id_demod_add_pair(&accum, &cfg,
							 pos.flux_vs,
							 pos.delta_current_a,
							 neg.flux_vs,
							 neg.delta_current_a);
		if (ret == 0) {
			g_motor_params->electrical_id_capture.sample_count++;
		} else if (ret > 0) {
			g_motor_params->electrical_id_capture.rejected_samples++;
		} else {
			g_motor_params->electrical_id_capture.rejected_samples++;
		}
		if (accum.samples >= repeats) {
			break;
		}
	}
	electrical_id_stop_direct_voltage();
	g_motor_params->electrical_id_capture.active = false;

	return motor_electrical_id_demod_finalize(&accum, &cfg, &limits, out);
}

static float32_t electrical_id_set_generated_electrical_angle(float32_t theta_elec_rad)
{
	float32_t mech_rad = wrap_rad_pi(theta_elec_rad / (float32_t)MOTOR_POLE_PAIRS);

	angle_gen_set_velocity(&g_motor_params->angle_gen, 0.0f);
	angle_gen_set_angle(&g_motor_params->angle_gen, mech_rad);
	angle_observer_reset_tracking(&g_motor_params->observer, mech_rad, 0.0f);

	return angle_observer_get_elec_angle_pred(&g_motor_params->observer);
}

static uint32_t electrical_id_default_saliency_settle_ticks(void)
{
	float32_t l_h = MOTOR_INDUCTANCE_D_H;

	if (g_motor_params != NULL && g_motor_params->Ls_measured_H > 0.0f) {
		l_h = fmaxf(l_h, g_motor_params->Ls_measured_H);
	}
	if (staged.ld_valid && staged.ld.inductance_h > 0.0f) {
		l_h = fmaxf(l_h, staged.ld.inductance_h);
	}
	if (staged.lq_valid && staged.lq.inductance_h > 0.0f) {
		l_h = fmaxf(l_h, staged.lq.inductance_h);
	}

	float32_t rs_ohm = fmaxf(staged.rs.rs_ohm, MOTOR_RESISTANCE_OHM);
	float32_t settle_s = 5.0f * l_h / rs_ohm;
	uint32_t ticks = (uint32_t)(settle_s * CONTROL_LOOP_FREQUENCY_HZ) + 1U;

	return CLAMP(ticks,
		     COMMISSION_ELECTRICAL_SALIENCY_MIN_SETTLE_TICKS,
		     COMMISSION_ELECTRICAL_SALIENCY_MAX_SETTLE_TICKS);
}

static int electrical_id_collect_saliency_sweep(float32_t pulse_v,
						uint32_t vectors,
						uint32_t pairs,
						uint32_t revs,
						uint32_t half_cycles,
						uint32_t settle_ticks,
						struct motor_saliency_id_result *out)
{
	if (out == NULL || vectors == 0U || pairs == 0U || revs == 0U ||
	    half_cycles == 0U || settle_ticks == 0U) {
		return -EINVAL;
	}

	uint32_t total_pairs = vectors * pairs * revs;
	struct motor_electrical_id_limits limits = electrical_id_limits();
	float32_t voltage_limit_v = electrical_id_voltage_pulse_limit();
	float32_t limited_pulse_v = clampf(pulse_v, -voltage_limit_v, voltage_limit_v);
	float32_t half_period_s = (float32_t)half_cycles / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t min_abs_flux_vs = fmaxf(fabsf(limited_pulse_v) * half_period_s * 0.02f,
					  1.0e-7f);
	struct motor_saliency_id_config cfg = {
		.min_samples = (uint16_t)MAX(COMMISSION_ELECTRICAL_MIN_SAMPLES,
					     total_pairs / 2U),
		.fit_first_harmonic = true,
		.fit_fourth_harmonic = true,
		.scale_factor = COMMISSION_ELECTRICAL_DEMOD_SCALE_FACTOR,
		.min_inductance_h = limits.l_min_h,
		.max_inductance_h = limits.l_max_h,
		.max_residual_ratio = COMMISSION_ELECTRICAL_DEMOD_MAX_SPREAD_RATIO,
		.max_saliency_ratio = limits.max_axis_mismatch_ratio,
		.min_confidence = limits.min_confidence,
	};
	struct motor_saliency_id_state accum;
	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;

	motor_saliency_id_init(&accum, &cfg);
	if (!accum.config_valid) {
		return -EINVAL;
	}
	memset(saliency_bins, 0, sizeof(saliency_bins));

	electrical_id_force_current(0.0f, 0.0f);
	int ret = motor_commission_wait_ms_or_fault(COMMISSION_ELECTRICAL_SETTLE_MS);
	if (ret != 0) {
		return ret;
	}

	electrical_id_start_l_capture(MOTOR_ELECTRICAL_ID_CAPTURE_LD,
				      staged.rs.rs_ohm,
				      voltage_limit_v,
				      total_pairs);
	for (uint32_t rev = 0U; rev < revs && accum.sample_count < total_pairs; ++rev) {
		for (uint32_t vector = 0U; vector < vectors &&
				      accum.sample_count < total_pairs; ++vector) {
			float32_t theta_elec_rad =
				(2.0f * PI_F32 * (float32_t)vector) / (float32_t)vectors;
			float32_t fit_theta_elec_rad;

			electrical_id_set_direct_voltage(0.0f, 0.0f);
			fit_theta_elec_rad =
				electrical_id_set_generated_electrical_angle(theta_elec_rad);
			ret = electrical_id_wait_control_ticks_or_fault(settle_ticks);
			if (ret != 0) {
				goto fail;
			}

			/* Discard the first pair after each vector change so the segment
			 * starts after the generated frame has reached the ISR path.
			 */
			for (uint32_t pair = 0U; pair <= pairs; ++pair) {
				struct electrical_id_l_segment_sample pos;
				struct electrical_id_l_segment_sample neg;

				electrical_id_set_direct_voltage(limited_pulse_v, 0.0f);
				electrical_id_begin_l_segment();
				ret = electrical_id_wait_control_ticks_or_fault(half_cycles);
				if (ret != 0) {
					goto fail;
				}
				ret = electrical_id_end_l_segment_sample(&pos);
				if (ret != 0) {
					goto fail;
				}

				electrical_id_set_direct_voltage(-limited_pulse_v, 0.0f);
				electrical_id_begin_l_segment();
				ret = electrical_id_wait_control_ticks_or_fault(half_cycles);
				if (ret != 0) {
					goto fail;
				}
				ret = electrical_id_end_l_segment_sample(&neg);
				if (ret != 0) {
					goto fail;
				}

				if (pair == 0U) {
					continue;
				}

				float32_t flux_vs = pos.flux_vs - neg.flux_vs;
				float32_t delta_current_a =
					pos.delta_current_a - neg.delta_current_a;
				float32_t inv_l_h_inv = 0.0f;
				bool accepted = false;

				if (fabsf(flux_vs) >= min_abs_flux_vs) {
					inv_l_h_inv = delta_current_a / flux_vs;
					accepted = motor_saliency_id_add(&accum,
									 fit_theta_elec_rad,
									 inv_l_h_inv);
				} else {
					(void)motor_saliency_id_add(&accum,
								    fit_theta_elec_rad,
								    -1.0f);
				}
				if (accepted) {
					saliency_bins[vector].theta_elec_rad = fit_theta_elec_rad;
					saliency_bins[vector].sum_inv_l += inv_l_h_inv;
					saliency_bins[vector].sum_inv_l2 +=
						inv_l_h_inv * inv_l_h_inv;
					saliency_bins[vector].samples++;
					cap->sample_count++;
				} else {
					cap->rejected_samples++;
				}
				if (accum.sample_count >= total_pairs) {
					break;
				}
			}
		}
	}

	electrical_id_stop_direct_voltage();
	cap->active = false;
	return motor_saliency_id_finalize_bins(saliency_bins, vectors,
					       accum.rejected_samples, &cfg, out);

fail:
	electrical_id_stop_direct_voltage();
	cap->active = false;
	return ret;
}

static void electrical_id_update_combined(void)
{
	struct motor_electrical_id_limits limits = electrical_id_limits();

	staged.result_valid = false;
	staged.pi_valid = false;
	if (!staged.rs_valid || !staged.ld_valid || !staged.lq_valid) {
		return;
	}

	int ret = motor_electrical_id_combine(&staged.rs, &staged.ld, &staged.lq,
					       &limits, &staged.result);
	staged.result_valid = (ret == 0 && staged.result.valid);
	if (!staged.result_valid) {
		return;
	}

	ret = motor_electrical_id_recommend_current_pi(staged.result.rs_ohm,
						     staged.result.ld_h,
						     staged.result.lq_h,
						     CURRENT_LOOP_BANDWIDTH_HZ,
						     1.0f / CONTROL_LOOP_FREQUENCY_HZ,
						     &staged.pi);
	staged.pi_valid = (ret == 0 && staged.pi.valid);
	if (staged.pi_valid) {
		staged.result.flags |= MOTOR_ELECTRICAL_ID_FLAG_PI_VALID;
	}
}

static int electrical_id_stage_fallback_inductance(void)
{
	float32_t ld_h = (g_motor_params != NULL && g_motor_params->Ls_measured_H > 0.0f) ?
		g_motor_params->Ls_measured_H : MOTOR_INDUCTANCE_D_H;
	float32_t lq_h = (g_motor_params != NULL && g_motor_params->Ls_measured_H > 0.0f) ?
		g_motor_params->Ls_measured_H : MOTOR_INDUCTANCE_Q_H;

	staged.ld = (struct motor_electrical_id_l_result){
		.inductance_h = ld_h,
		.confidence = 0.25f,
		.valid = true,
	};
	staged.lq = (struct motor_electrical_id_l_result){
		.inductance_h = lq_h,
		.confidence = 0.25f,
		.valid = true,
	};
	staged.ld_valid = true;
	staged.lq_valid = true;
	staged.inductance_fallback = true;
	staged.inductance_roverl = false;
	staged.inductance_demod = false;
	staged.inductance_saliency = false;
	electrical_id_update_combined();

	return staged.result_valid && staged.pi_valid ? 0 : -ERANGE;
}

static int electrical_id_stage_roverl_inductance(void)
{
	if (g_motor_params == NULL || !staged.rs_valid ||
	    g_motor_params->R_over_L_measured <= 0.0f) {
		return -EINVAL;
	}

	float32_t l_h = staged.rs.rs_ohm / g_motor_params->R_over_L_measured;
	struct motor_electrical_id_limits limits = electrical_id_limits();
	if (l_h < limits.l_min_h || l_h > limits.l_max_h) {
		return -ERANGE;
	}

	staged.ld = (struct motor_electrical_id_l_result){
		.inductance_h = l_h,
		.confidence = 0.50f,
		.valid = true,
	};
	staged.lq = staged.ld;
	staged.ld_valid = true;
	staged.lq_valid = true;
	staged.inductance_fallback = true;
	staged.inductance_roverl = true;
	staged.inductance_demod = false;
	staged.inductance_saliency = false;
	electrical_id_update_combined();

	return staged.result_valid && staged.pi_valid ? 0 : -ERANGE;
}

static void electrical_id_stage_scalar_inductance(float32_t l_h,
						  uint32_t samples,
						  float32_t confidence,
						  float32_t residual_ratio)
{
	staged.ld = (struct motor_electrical_id_l_result){
		.inductance_h = l_h,
		.residual_ratio = residual_ratio,
		.confidence = confidence,
		.samples = samples,
		.valid = true,
	};
	staged.lq = staged.ld;
	staged.ld_valid = true;
	staged.lq_valid = true;
	staged.inductance_fallback = false;
	staged.inductance_roverl = false;
	staged.inductance_demod = false;
	staged.inductance_saliency = false;
	electrical_id_update_combined();
}

static void electrical_id_stage_demod_inductance(void)
{
	staged.ld = (struct motor_electrical_id_l_result){
		.inductance_h = staged.ld_demod.inductance_h,
		.residual_ratio = staged.ld_demod.spread_ratio,
		.confidence = staged.ld_demod.confidence,
		.samples = staged.ld_demod.samples,
		.rejected_low_slew = staged.ld_demod.rejected_low_signal +
				      staged.ld_demod.rejected_non_positive,
		.valid = staged.ld_demod.valid,
	};
	staged.lq = (struct motor_electrical_id_l_result){
		.inductance_h = staged.lq_demod.inductance_h,
		.residual_ratio = staged.lq_demod.spread_ratio,
		.confidence = staged.lq_demod.confidence,
		.samples = staged.lq_demod.samples,
		.rejected_low_slew = staged.lq_demod.rejected_low_signal +
				      staged.lq_demod.rejected_non_positive,
		.valid = staged.lq_demod.valid,
	};
	staged.ld_valid = staged.ld.valid;
	staged.lq_valid = staged.lq.valid;
	staged.inductance_fallback = false;
	staged.inductance_roverl = false;
	staged.inductance_demod = true;
	staged.inductance_saliency = false;
	electrical_id_update_combined();
}

static void electrical_id_stage_demod_scalar_inductance(void)
{
	staged.lq_demod = staged.ld_demod;
	electrical_id_stage_demod_inductance();
}

static void electrical_id_stage_saliency_inductance(void)
{
	staged.ld = (struct motor_electrical_id_l_result){
		.inductance_h = staged.saliency.ld_h,
		.residual_ratio = staged.saliency.residual_ratio,
		.confidence = staged.saliency.confidence,
		.samples = staged.saliency.sample_count,
		.rejected_low_slew = staged.saliency.rejected_samples,
		.valid = staged.saliency.valid,
	};
	staged.lq = (struct motor_electrical_id_l_result){
		.inductance_h = staged.saliency.lq_h,
		.residual_ratio = staged.saliency.residual_ratio,
		.confidence = staged.saliency.confidence,
		.samples = staged.saliency.sample_count,
		.rejected_low_slew = staged.saliency.rejected_samples,
		.valid = staged.saliency.valid,
	};
	staged.ld_valid = staged.ld.valid;
	staged.lq_valid = staged.lq.valid;
	staged.inductance_fallback = false;
	staged.inductance_roverl = false;
	staged.inductance_demod = false;
	staged.inductance_saliency = true;
	electrical_id_update_combined();
}

int cmd_motor_commission_electrical_plan(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct motor_electrical_id_limits lim = electrical_id_limits();
	shell_print(sh, "Production electrical ID plan:");
	shell_print(sh, "  Existing Rs/R-over-L remains fallback/bootstrap.");
	shell_print(sh, "  Default Rs current: %.4f A", (double)COMMISSION_ELECTRICAL_RS_CURRENT_A);
	shell_print(sh, "  Safe current limit: %.4f A (profile max %.4f A)",
		    (double)electrical_id_current_limit(),
		    (double)MOTOR_MAX_CURRENT_A);
	shell_print(sh, "  Default pulses: integral %.3f V/%u ms, demod %.3f V/%u ms",
		    (double)COMMISSION_ELECTRICAL_L_PULSE_V,
		    COMMISSION_ELECTRICAL_L_PULSE_MS,
		    (double)COMMISSION_ELECTRICAL_DEMOD_PULSE_V,
		    COMMISSION_ELECTRICAL_DEMOD_PULSE_MS);
	shell_print(sh, "  Default demod half-period: %u control ticks (%.1f Hz)",
		    COMMISSION_ELECTRICAL_DEMOD_HALF_CYCLES,
		    (double)(CONTROL_LOOP_FREQUENCY_HZ /
			     (2.0f * (float32_t)COMMISSION_ELECTRICAL_DEMOD_HALF_CYCLES)));
	shell_print(sh, "  Samples default/range: %u / %u..%u",
		    COMMISSION_ELECTRICAL_SAMPLES,
		    COMMISSION_ELECTRICAL_MIN_SAMPLES,
		    COMMISSION_ELECTRICAL_MAX_SAMPLES);
	shell_print(sh, "  Plausibility: Rs %.4f..%.4f ohm, L %.7f..%.7f H",
		    (double)lim.rs_min_ohm, (double)lim.rs_max_ohm,
		    (double)lim.l_min_h, (double)lim.l_max_h);
	shell_print(sh, "  Commands:");
	shell_print(sh, "    motor commission electrical run [rs_current_a] [demod_pulse_v] [samples]");
	shell_print(sh, "    motor commission electrical measure inductance [pulse_v] [samples] [pulse_ms]");
	shell_print(sh, "    motor commission electrical measure demod [pulse_v] [samples] [half_cycles]");
	shell_print(sh, "    motor commission electrical demod_sweep [pulse_v] [samples]");
	shell_print(sh, "    motor commission electrical saliency_sweep [pulse_v] [vectors] [pairs] [revs] [half_cycles] [settle_ticks]");
	shell_print(sh, "    motor commission electrical saliency_apply");
	shell_print(sh, "    motor commission electrical sweep [samples]");
	shell_print(sh, "    motor commission electrical status");
	shell_print(sh, "    motor commission electrical apply");
	shell_print(sh, "    motor commission electrical validate [current_a] [hold_ms] [max_error_a]");
	return 0;
}

int cmd_motor_commission_electrical_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	memset(&staged, 0, sizeof(staged));
	shell_print(sh, "Production electrical ID staged results cleared");
	return 0;
}

int cmd_motor_commission_electrical_measure_rs(const struct shell *sh, size_t argc, char **argv)
{
	float32_t current_a;
	uint32_t samples;
	uint32_t settle_ms;
	int ret = parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_RS_CURRENT_A, &current_a);
	if (ret != 0 || parse_optional_u32(argv, argc, 2U, COMMISSION_ELECTRICAL_SAMPLES, &samples) != 0 ||
	    parse_optional_u32(argv, argc, 3U, COMMISSION_ELECTRICAL_SETTLE_MS, &settle_ms) != 0) {
		shell_error(sh, "Usage: motor commission electrical measure rs [current_a] [samples] [settle_ms]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (current_a <= 0.0f || current_a > electrical_id_current_limit() ||
	    samples < COMMISSION_ELECTRICAL_MIN_SAMPLES || samples > COMMISSION_ELECTRICAL_MAX_SAMPLES) {
		shell_error(sh, "Invalid current/samples; current in (0, %.4f], samples %u..%u",
			    (double)electrical_id_current_limit(), COMMISSION_ELECTRICAL_MIN_SAMPLES,
			    COMMISSION_ELECTRICAL_MAX_SAMPLES);
		return -EINVAL;
	}

	ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}
	ret = electrical_id_collect_rs(current_a, samples, settle_ms, &staged.rs);
	electrical_id_stop();
	staged.rs_valid = (ret == 0 && staged.rs.valid);
	if (!staged.rs_valid) {
		shell_error(sh, "Production Rs measurement rejected (err %d)", ret);
		return ret;
	}

	g_motor_params->Rs_measured_ohm = staged.rs.rs_ohm;
	electrical_id_update_combined();
	shell_print(sh, "Production Rs staged: Rs=%.6f ohm samples=%u confidence=%.3f residual=%.4f",
		    (double)staged.rs.rs_ohm, staged.rs.samples,
		    (double)staged.rs.confidence, (double)staged.rs.residual_ratio);
	return 0;
}

int cmd_motor_commission_electrical_measure_inductance(const struct shell *sh, size_t argc,
							char **argv)
{
	float32_t pulse_v;
	uint32_t repeats;
	uint32_t pulse_ms;
	int ret = parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_L_PULSE_V, &pulse_v);
	if (ret != 0 ||
	    parse_optional_u32(argv, argc, 2U, COMMISSION_ELECTRICAL_SAMPLES, &repeats) != 0 ||
	    parse_optional_u32(argv, argc, 3U, COMMISSION_ELECTRICAL_L_PULSE_MS, &pulse_ms) != 0) {
		shell_error(sh, "Usage: motor commission electrical measure inductance [pulse_v] [samples] [pulse_ms]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!staged.rs_valid) {
		shell_error(sh, "Measure/stage Rs before inductance.");
		return -EACCES;
	}
	if (pulse_v <= 0.0f || pulse_v > electrical_id_voltage_pulse_limit() ||
	    repeats < COMMISSION_ELECTRICAL_MIN_SAMPLES || repeats > COMMISSION_ELECTRICAL_MAX_SAMPLES ||
	    pulse_ms < COMMISSION_ELECTRICAL_MIN_PULSE_MS) {
		shell_error(sh,
			    "Invalid pulse/samples; pulse in (0, %.4f] V, samples %u..%u, pulse_ms >= %u",
			    (double)electrical_id_voltage_pulse_limit(),
			    COMMISSION_ELECTRICAL_MIN_SAMPLES, COMMISSION_ELECTRICAL_MAX_SAMPLES,
			    COMMISSION_ELECTRICAL_MIN_PULSE_MS);
		return -EINVAL;
	}

	ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}
	staged.ld_valid = false;
	staged.lq_valid = false;
	staged.result_valid = false;
	staged.pi_valid = false;
	staged.inductance_fallback = false;
	staged.inductance_roverl = false;
	staged.inductance_demod = false;
	staged.inductance_saliency = false;
	ret = electrical_id_collect_l_axis(false, pulse_v, pulse_ms, repeats, &staged.ld);
	if (ret == 0) {
		staged.ld_valid = staged.ld.valid;
		ret = electrical_id_collect_l_axis(true, pulse_v, pulse_ms, repeats, &staged.lq);
		staged.lq_valid = (ret == 0 && staged.lq.valid);
		staged.inductance_fallback = false;
		if (!staged.lq_valid && staged.ld_valid) {
			shell_warn(sh,
				   "Q-axis inductance pulse rejected (err %d); using D-axis scalar L for Q",
				   ret);
			electrical_id_stage_scalar_inductance(staged.ld.inductance_h,
							      staged.ld.samples,
							      staged.ld.confidence,
							      staged.ld.residual_ratio);
			ret = 0;
		}
	}
	electrical_id_stop();
	if (ret != 0 || !staged.ld_valid || !staged.lq_valid) {
		shell_error(sh, "Production inductance measurement rejected (err %d)", ret);
		return ret;
	}

	electrical_id_update_combined();
	shell_print(sh, "Production inductance staged: Ld=%.9f H Lq=%.9f H Lavg=%.9f H diff=%.9f H",
		    (double)staged.ld.inductance_h, (double)staged.lq.inductance_h,
		    (double)staged.result.l_avg_h, (double)staged.result.lq_minus_ld_h);
	return 0;
}

int cmd_motor_commission_electrical_measure_demod(const struct shell *sh, size_t argc,
						  char **argv)
{
	float32_t pulse_v;
	uint32_t repeats;
	uint32_t half_cycles;
	int ret = parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_DEMOD_PULSE_V, &pulse_v);
	if (ret != 0 ||
	    parse_optional_u32(argv, argc, 2U, COMMISSION_ELECTRICAL_SAMPLES, &repeats) != 0 ||
	    parse_optional_u32(argv, argc, 3U, COMMISSION_ELECTRICAL_DEMOD_HALF_CYCLES,
			       &half_cycles) != 0) {
		shell_error(sh,
			    "Usage: motor commission electrical measure demod [pulse_v] [samples] [half_cycles]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!staged.rs_valid) {
		shell_error(sh, "Measure/stage Rs before demodulated inductance.");
		return -EACCES;
	}
	if (pulse_v <= 0.0f || pulse_v > electrical_id_voltage_pulse_limit() ||
	    repeats < COMMISSION_ELECTRICAL_MIN_SAMPLES || repeats > COMMISSION_ELECTRICAL_MAX_SAMPLES ||
	    half_cycles == 0U) {
		shell_error(sh,
			    "Invalid pulse/samples; pulse in (0, %.4f] V, samples %u..%u, half_cycles > 0",
			    (double)electrical_id_voltage_pulse_limit(),
			    COMMISSION_ELECTRICAL_MIN_SAMPLES, COMMISSION_ELECTRICAL_MAX_SAMPLES);
		return -EINVAL;
	}

	ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}
	staged.ld_valid = false;
	staged.lq_valid = false;
	staged.result_valid = false;
	staged.pi_valid = false;
	staged.inductance_fallback = false;
	staged.inductance_roverl = false;
	staged.inductance_demod = false;
	staged.inductance_saliency = false;
	memset(&staged.ld_demod, 0, sizeof(staged.ld_demod));
	memset(&staged.lq_demod, 0, sizeof(staged.lq_demod));

	float32_t demod_hz = CONTROL_LOOP_FREQUENCY_HZ / (2.0f * (float32_t)half_cycles);

	shell_print(sh, "Demodulated inductance: pulse=%.3f V half_cycles=%u freq=%.1f Hz samples=%u",
		    (double)pulse_v, half_cycles, (double)demod_hz, repeats);
	int ret_d = electrical_id_collect_l_axis_demod_cycles(false, pulse_v, half_cycles,
							      repeats, &staged.ld_demod);
	int ret_q = electrical_id_collect_l_axis_demod_cycles(true, pulse_v, half_cycles,
							      repeats, &staged.lq_demod);
	electrical_id_stop();
	ret = (ret_d != 0) ? ret_d : ret_q;
	if (ret != 0 || !staged.ld_demod.valid || !staged.lq_demod.valid) {
		if (staged.ld_demod.valid) {
			struct motor_electrical_id_demod_result rejected_q = staged.lq_demod;

			shell_warn(sh,
				   "Q-axis demodulated inductance rejected (D err %d Q err %d); using D-axis scalar L for Q",
				   ret_d, ret_q);
			electrical_id_stage_demod_scalar_inductance();
			shell_print(sh,
				    "Demodulated scalar inductance staged: Ld=Lq=%.9f H scale=%.2f",
				    (double)staged.result.l_avg_h,
				    (double)COMMISSION_ELECTRICAL_DEMOD_SCALE_FACTOR);
			shell_print(sh,
				    "  D: inv=%.3f 1/H spread=%.3f conf=%.3f samples=%u",
				    (double)staged.ld_demod.inv_l_mean,
				    (double)staged.ld_demod.spread_ratio,
				    (double)staged.ld_demod.confidence,
				    staged.ld_demod.samples);
			shell_print(sh,
				    "  Rejected Q: L=%.9f H inv=%.3f spread=%.3f conf=%.3f samples=%u",
				    (double)rejected_q.inductance_h,
				    (double)rejected_q.inv_l_mean,
				    (double)rejected_q.spread_ratio,
				    (double)rejected_q.confidence,
				    rejected_q.samples);
			return staged.result_valid ? 0 : -ERANGE;
		}
		shell_error(sh, "Demodulated inductance measurement rejected (D err %d Q err %d)",
			    ret_d, ret_q);
		shell_print(sh,
			    "  D: L=%.9f H inv=%.3f spread=%.3f conf=%.3f samples=%u reject_low=%u reject_sign=%u",
			    (double)staged.ld_demod.inductance_h,
			    (double)staged.ld_demod.inv_l_mean,
			    (double)staged.ld_demod.spread_ratio,
			    (double)staged.ld_demod.confidence,
			    staged.ld_demod.samples,
			    staged.ld_demod.rejected_low_signal,
			    staged.ld_demod.rejected_non_positive);
		shell_print(sh,
			    "  Q: L=%.9f H inv=%.3f spread=%.3f conf=%.3f samples=%u reject_low=%u reject_sign=%u",
			    (double)staged.lq_demod.inductance_h,
			    (double)staged.lq_demod.inv_l_mean,
			    (double)staged.lq_demod.spread_ratio,
			    (double)staged.lq_demod.confidence,
			    staged.lq_demod.samples,
			    staged.lq_demod.rejected_low_signal,
			    staged.lq_demod.rejected_non_positive);
		return ret != 0 ? ret : -ERANGE;
	}

	electrical_id_stage_demod_inductance();
	shell_print(sh,
		    "Demodulated inductance staged: Ld=%.9f H Lq=%.9f H Lavg=%.9f H diff=%.9f H scale=%.2f",
		    (double)staged.result.ld_h,
		    (double)staged.result.lq_h,
		    (double)staged.result.l_avg_h,
		    (double)staged.result.lq_minus_ld_h,
		    (double)COMMISSION_ELECTRICAL_DEMOD_SCALE_FACTOR);
	shell_print(sh,
		    "  D: inv=%.3f 1/H spread=%.3f conf=%.3f samples=%u",
		    (double)staged.ld_demod.inv_l_mean,
		    (double)staged.ld_demod.spread_ratio,
		    (double)staged.ld_demod.confidence,
		    staged.ld_demod.samples);
	shell_print(sh,
		    "  Q: inv=%.3f 1/H spread=%.3f conf=%.3f samples=%u",
		    (double)staged.lq_demod.inv_l_mean,
		    (double)staged.lq_demod.spread_ratio,
		    (double)staged.lq_demod.confidence,
		    staged.lq_demod.samples);
	return staged.result_valid ? 0 : -ERANGE;
}

int cmd_motor_commission_electrical_demod_sweep(const struct shell *sh, size_t argc,
						char **argv)
{
	float32_t pulse_v;
	uint32_t repeats;
	int ret = parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_DEMOD_PULSE_V,
				       &pulse_v);
	if (ret != 0 ||
	    parse_optional_u32(argv, argc, 2U, COMMISSION_ELECTRICAL_SAMPLES, &repeats) != 0) {
		shell_error(sh, "Usage: motor commission electrical demod_sweep [pulse_v] [samples]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!staged.rs_valid) {
		shell_error(sh, "Measure/stage Rs before demodulated inductance sweep.");
		return -EACCES;
	}
	if (pulse_v <= 0.0f || pulse_v > electrical_id_voltage_pulse_limit() ||
	    repeats < COMMISSION_ELECTRICAL_MIN_SAMPLES ||
	    repeats > COMMISSION_ELECTRICAL_MAX_SAMPLES) {
		shell_error(sh, "Invalid pulse/samples; pulse in (0, %.4f] V, samples %u..%u",
			    (double)electrical_id_voltage_pulse_limit(),
			    COMMISSION_ELECTRICAL_MIN_SAMPLES,
			    COMMISSION_ELECTRICAL_MAX_SAMPLES);
		return -EINVAL;
	}

	const uint32_t half_cycles[] = { 100U, 67U, 50U, 40U, 25U, 20U, 10U };
	struct motor_electrical_id_demod_result best = {0};
	float32_t best_score = INFINITY;
	uint32_t best_half_cycles = 0U;
	uint32_t valid_count = 0U;

	staged.ld_valid = false;
	staged.lq_valid = false;
	staged.result_valid = false;
	staged.pi_valid = false;
	staged.inductance_fallback = false;
	staged.inductance_roverl = false;
	staged.inductance_demod = false;
	staged.inductance_saliency = false;

	ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}

	shell_print(sh, "D-axis demod frequency sweep:");
	shell_print(sh, "  pulse=%.3f V samples/point=%u Rs=%.6f ohm",
		    (double)pulse_v, repeats, (double)staged.rs.rs_ohm);
	for (size_t i = 0U; i < ARRAY_SIZE(half_cycles); ++i) {
		struct motor_electrical_id_demod_result result = {0};
		float32_t freq_hz = CONTROL_LOOP_FREQUENCY_HZ /
				     (2.0f * (float32_t)half_cycles[i]);

		ret = electrical_id_collect_l_axis_demod_cycles(false, pulse_v,
								half_cycles[i],
								repeats, &result);
		if (ret == 0 && result.valid) {
			float32_t score = result.spread_ratio;

			shell_print(sh,
				    "  PASS half=%u freq=%.1f Hz L=%.9f H spread=%.3f conf=%.3f samples=%u reject_low=%u reject_sign=%u",
				    half_cycles[i], (double)freq_hz,
				    (double)result.inductance_h,
				    (double)result.spread_ratio,
				    (double)result.confidence,
				    result.samples,
				    result.rejected_low_signal,
				    result.rejected_non_positive);
			valid_count++;
			if (score < best_score) {
				best = result;
				best_score = score;
				best_half_cycles = half_cycles[i];
			}
		} else {
			shell_print(sh,
				    "  FAIL half=%u freq=%.1f Hz err=%d L=%.9f H spread=%.3f conf=%.3f samples=%u reject_low=%u reject_sign=%u",
				    half_cycles[i], (double)freq_hz, ret,
				    (double)result.inductance_h,
				    (double)result.spread_ratio,
				    (double)result.confidence,
				    result.samples,
				    result.rejected_low_signal,
				    result.rejected_non_positive);
		}
	}
	electrical_id_stop();

	if (valid_count == 0U) {
		shell_error(sh, "Demod frequency sweep rejected: no valid D-axis points");
		return -ERANGE;
	}

	staged.ld_demod = best;
	electrical_id_stage_demod_scalar_inductance();
	shell_print(sh,
		    "Demod scalar inductance staged from best point: half=%u freq=%.1f Hz Ld=Lq=%.9f H spread=%.3f confidence=%.3f",
		    best_half_cycles,
		    (double)(CONTROL_LOOP_FREQUENCY_HZ /
			     (2.0f * (float32_t)best_half_cycles)),
		    (double)staged.result.l_avg_h,
		    (double)best.spread_ratio,
		    (double)best.confidence);
	return staged.result_valid ? 0 : -ERANGE;
}

int cmd_motor_commission_electrical_saliency_sweep(const struct shell *sh, size_t argc,
						   char **argv)
{
	float32_t pulse_v;
	uint32_t vectors;
	uint32_t pairs;
	uint32_t revs;
	uint32_t half_cycles;
	uint32_t settle_ticks;
	int ret = parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_DEMOD_PULSE_V,
				       &pulse_v);

	if (ret != 0 ||
	    parse_optional_u32(argv, argc, 2U,
			       COMMISSION_ELECTRICAL_SALIENCY_VECTORS_DEFAULT,
			       &vectors) != 0 ||
	    parse_optional_u32(argv, argc, 3U,
			       COMMISSION_ELECTRICAL_SALIENCY_PAIRS_DEFAULT,
			       &pairs) != 0 ||
	    parse_optional_u32(argv, argc, 4U,
			       COMMISSION_ELECTRICAL_SALIENCY_REVS_DEFAULT,
			       &revs) != 0 ||
	    parse_optional_u32(argv, argc, 5U,
			       COMMISSION_ELECTRICAL_DEMOD_HALF_CYCLES,
			       &half_cycles) != 0 ||
	    parse_optional_u32(argv, argc, 6U,
			       electrical_id_default_saliency_settle_ticks(),
			       &settle_ticks) != 0) {
		shell_error(sh,
			    "Usage: motor commission electrical saliency_sweep [pulse_v] [vectors] [pairs] [revs] [half_cycles] [settle_ticks]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!staged.rs_valid) {
		shell_error(sh, "Measure/stage Rs before saliency sweep.");
		return -EACCES;
	}
	if (pulse_v <= 0.0f || pulse_v > electrical_id_voltage_pulse_limit() ||
	    vectors < 6U || vectors > COMMISSION_ELECTRICAL_SALIENCY_MAX_VECTORS ||
	    pairs == 0U || pairs > COMMISSION_ELECTRICAL_SALIENCY_MAX_PAIRS ||
	    revs == 0U || revs > COMMISSION_ELECTRICAL_SALIENCY_MAX_REVS ||
	    half_cycles == 0U ||
	    settle_ticks < COMMISSION_ELECTRICAL_SALIENCY_MIN_SETTLE_TICKS ||
	    settle_ticks > COMMISSION_ELECTRICAL_SALIENCY_MAX_SETTLE_TICKS) {
		shell_error(sh,
			    "Invalid saliency sweep; pulse in (0, %.4f] V, vectors 6..%u, pairs 1..%u, revs 1..%u, half_cycles > 0, settle_ticks %u..%u",
			    (double)electrical_id_voltage_pulse_limit(),
			    COMMISSION_ELECTRICAL_SALIENCY_MAX_VECTORS,
			    COMMISSION_ELECTRICAL_SALIENCY_MAX_PAIRS,
			    COMMISSION_ELECTRICAL_SALIENCY_MAX_REVS,
			    COMMISSION_ELECTRICAL_SALIENCY_MIN_SETTLE_TICKS,
			    COMMISSION_ELECTRICAL_SALIENCY_MAX_SETTLE_TICKS);
		return -EINVAL;
	}

	uint32_t total_pairs = vectors * pairs * revs;

	if (total_pairs > 4096U) {
		shell_error(sh, "Invalid saliency sweep; total accepted pairs must be <= 4096");
		return -EINVAL;
	}

	ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}
	memset(&staged.saliency, 0, sizeof(staged.saliency));

	float32_t demod_hz = CONTROL_LOOP_FREQUENCY_HZ / (2.0f * (float32_t)half_cycles);

	shell_print(sh,
		    "Saliency sweep: pulse=%.3f V vectors=%u pairs/vector=%u revs=%u half_cycles=%u freq=%.1f Hz settle_ticks=%u",
		    (double)pulse_v, vectors, pairs, revs, half_cycles, (double)demod_hz,
		    settle_ticks);
	ret = electrical_id_collect_saliency_sweep(pulse_v, vectors, pairs, revs,
						   half_cycles, settle_ticks,
						   &staged.saliency);
	electrical_id_stop();
	if (ret != 0 || !staged.saliency.valid) {
		shell_error(sh,
			    "Saliency sweep rejected (err %d): Ld=%.9f H Lq=%.9f H ratio=%.3f residual=%.3f conf=%.3f samples=%u rejected=%u",
			    ret,
			    (double)staged.saliency.ld_h,
			    (double)staged.saliency.lq_h,
			    (double)staged.saliency.saliency_ratio,
			    (double)staged.saliency.residual_ratio,
			    (double)staged.saliency.confidence,
			    staged.saliency.sample_count,
			    staged.saliency.rejected_samples);
		shell_print(sh,
			    "  Fit reject detail: inv_offset=%.3f inv_amp1=%.3f inv_amp2=%.3f inv_amp4=%.3f phase=%.3f rad scale=%.2f",
			    (double)staged.saliency.inv_l_offset,
			    (double)staged.saliency.inv_l_amplitude1,
			    (double)staged.saliency.inv_l_amplitude,
			    (double)staged.saliency.inv_l_amplitude4,
			    (double)staged.saliency.phase_rad,
			    (double)COMMISSION_ELECTRICAL_DEMOD_SCALE_FACTOR);
		return ret != 0 ? ret : -ERANGE;
	}

	shell_print(sh,
		    "Saliency diagnostic: Ld=%.9f H Lq=%.9f H Lavg=%.9f H diff=%.9f H ratio=%.3f phase=%.3f rad",
		    (double)staged.saliency.ld_h,
		    (double)staged.saliency.lq_h,
		    (double)staged.saliency.l_avg_h,
		    (double)staged.saliency.lq_minus_ld_h,
		    (double)staged.saliency.saliency_ratio,
		    (double)staged.saliency.phase_rad);
	shell_print(sh,
		    "  Fit: inv_offset=%.3f inv_amp1=%.3f inv_amp2=%.3f inv_amp4=%.3f residual=%.3f conf=%.3f samples=%u rejected=%u scale=%.2f",
		    (double)staged.saliency.inv_l_offset,
		    (double)staged.saliency.inv_l_amplitude1,
		    (double)staged.saliency.inv_l_amplitude,
		    (double)staged.saliency.inv_l_amplitude4,
		    (double)staged.saliency.residual_ratio,
		    (double)staged.saliency.confidence,
		    staged.saliency.sample_count,
		    staged.saliency.rejected_samples,
		    (double)COMMISSION_ELECTRICAL_DEMOD_SCALE_FACTOR);
	shell_print(sh,
		    "  Production L is unchanged; use 'motor commission electrical saliency_apply' to explicitly stage this diagnostic result.");
	return 0;
}

int cmd_motor_commission_electrical_saliency_apply(const struct shell *sh, size_t argc,
						   char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!staged.saliency.valid) {
		shell_error(sh, "No valid saliency diagnostic result to apply.");
		return -EACCES;
	}

	electrical_id_stage_saliency_inductance();
	if (!staged.result_valid || !staged.pi_valid) {
		shell_error(sh, "Saliency result failed production staging.");
		return -ERANGE;
	}

	shell_warn(sh,
		   "Saliency Ld/Lq explicitly staged; validate current response before applying to runtime.");
	shell_print(sh,
		    "Saliency inductance staged: Ld=%.9f H Lq=%.9f H Lavg=%.9f H diff=%.9f H ratio=%.3f phase=%.3f rad",
		    (double)staged.result.ld_h,
		    (double)staged.result.lq_h,
		    (double)staged.result.l_avg_h,
		    (double)staged.result.lq_minus_ld_h,
		    (double)staged.saliency.saliency_ratio,
		    (double)staged.saliency.phase_rad);
	return 0;
}

int cmd_motor_commission_electrical_sweep(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t samples;
	if (parse_optional_u32(argv, argc, 1U, COMMISSION_ELECTRICAL_SAMPLES, &samples) != 0) {
		shell_error(sh, "Usage: motor commission electrical sweep [samples]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!staged.rs_valid) {
		shell_error(sh, "Measure/stage Rs before inductance sweep.");
		return -EACCES;
	}
	if (samples < COMMISSION_ELECTRICAL_MIN_SAMPLES || samples > COMMISSION_ELECTRICAL_MAX_SAMPLES) {
		shell_error(sh, "Invalid samples; samples %u..%u",
			    COMMISSION_ELECTRICAL_MIN_SAMPLES, COMMISSION_ELECTRICAL_MAX_SAMPLES);
		return -EINVAL;
	}

	float32_t pulse_limit_v = electrical_id_voltage_pulse_limit();
	float32_t pulse_v[] = {
		fminf(0.150f, pulse_limit_v * 0.45f),
		fminf(0.250f, pulse_limit_v * 0.65f),
		fminf(0.350f, pulse_limit_v * 0.85f),
		fminf(0.440f, pulse_limit_v * 0.98f),
	};
	uint32_t pulse_ms[] = { 5U, 10U, 20U };
	float32_t sum_l_h = 0.0f;
	float32_t min_l_h = INFINITY;
	float32_t max_l_h = 0.0f;
	uint32_t accepted = 0U;
	float32_t qualified_sum_l_h = 0.0f;
	float32_t qualified_min_l_h = INFINITY;
	float32_t qualified_max_l_h = 0.0f;
	float32_t qualified_min_confidence = 1.0f;
	float32_t qualified_max_residual = 0.0f;
	uint32_t qualified = 0U;
	uint32_t qualified_samples = 0U;
	float32_t qualified_min_v = fmaxf(COMMISSION_ELECTRICAL_SWEEP_MIN_QUALIFIED_V,
					  pulse_limit_v * 0.55f);

	int ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}

	shell_print(sh, "Production D-axis inductance sweep:");
	shell_print(sh, "  Rs=%.6f ohm pulse_limit=%.4f V samples/point=%u",
		    (double)staged.rs.rs_ohm, (double)pulse_limit_v, samples);
	shell_print(sh, "  Qualified staging requires V >= %.3f V", (double)qualified_min_v);
	for (size_t t = 0U; t < ARRAY_SIZE(pulse_ms); ++t) {
		for (size_t p = 0U; p < ARRAY_SIZE(pulse_v); ++p) {
			if (p > 0U && fabsf(pulse_v[p] - pulse_v[p - 1U]) < 0.005f) {
				continue;
			}
			struct motor_electrical_id_l_result result = {0};

			ret = electrical_id_collect_l_axis(false, pulse_v[p], pulse_ms[t],
							   samples, &result);
			if (ret == 0 && result.valid) {
				shell_print(sh,
					    "  PASS V=%.3f ms=%u L=%.9f H conf=%.3f residual=%.4f samples=%u",
					    (double)pulse_v[p], pulse_ms[t],
					    (double)result.inductance_h,
					    (double)result.confidence,
					    (double)result.residual_ratio,
					    result.samples);
				sum_l_h += result.inductance_h;
				min_l_h = fminf(min_l_h, result.inductance_h);
				max_l_h = fmaxf(max_l_h, result.inductance_h);
				accepted++;
				if (pulse_v[p] >= qualified_min_v) {
					qualified_sum_l_h += result.inductance_h;
					qualified_min_l_h = fminf(qualified_min_l_h,
								  result.inductance_h);
					qualified_max_l_h = fmaxf(qualified_max_l_h,
								  result.inductance_h);
					qualified_min_confidence =
						fminf(qualified_min_confidence,
						      result.confidence);
					qualified_max_residual =
						fmaxf(qualified_max_residual,
						      result.residual_ratio);
					qualified_samples += result.samples;
					qualified++;
				}
			} else {
				shell_print(sh,
					    "  FAIL V=%.3f ms=%u err=%d L=%.9f H conf=%.3f residual=%.4f samples=%u",
					    (double)pulse_v[p], pulse_ms[t], ret,
					    (double)result.inductance_h,
					    (double)result.confidence,
					    (double)result.residual_ratio,
					    result.samples);
			}
		}
	}
	electrical_id_stop();

	if (accepted == 0U) {
		shell_error(sh, "Production inductance sweep rejected: no valid points");
		return -ERANGE;
	}

	float32_t avg_l_h = sum_l_h / (float32_t)accepted;
	float32_t spread_ratio = (avg_l_h > 0.0f) ? (max_l_h - min_l_h) / avg_l_h : INFINITY;
	shell_print(sh,
		    "  Summary: accepted=%u avg=%.9f H min=%.9f H max=%.9f H spread=%.3f",
		    accepted, (double)avg_l_h, (double)min_l_h, (double)max_l_h,
		    (double)spread_ratio);
	if (g_motor_params->R_over_L_measured > 0.0f) {
		float32_t implied_l_h = staged.rs.rs_ohm / g_motor_params->R_over_L_measured;

		shell_print(sh, "  R/L implied L: %.9f H from Rs/R_over_L", (double)implied_l_h);
	}

	if (qualified == 0U) {
		shell_error(sh, "Production inductance sweep rejected: no qualified high-SNR points");
		ret = electrical_id_stage_roverl_inductance();
		if (ret == 0) {
			shell_warn(sh, "Staged R/L-implied scalar L after sweep rejection");
			return 0;
		}
		return -ERANGE;
	}

	float32_t qualified_avg_l_h = qualified_sum_l_h / (float32_t)qualified;
	float32_t qualified_spread_ratio = (qualified_avg_l_h > 0.0f) ?
		(qualified_max_l_h - qualified_min_l_h) / qualified_avg_l_h : INFINITY;
	shell_print(sh,
		    "  Qualified: accepted=%u avg=%.9f H min=%.9f H max=%.9f H spread=%.3f",
		    qualified, (double)qualified_avg_l_h, (double)qualified_min_l_h,
		    (double)qualified_max_l_h, (double)qualified_spread_ratio);
	if (qualified_spread_ratio > COMMISSION_ELECTRICAL_SWEEP_MAX_SPREAD_RATIO) {
		shell_error(sh,
			    "Production inductance sweep unstable: qualified spread %.3f > %.3f",
			    (double)qualified_spread_ratio,
			    (double)COMMISSION_ELECTRICAL_SWEEP_MAX_SPREAD_RATIO);
		ret = electrical_id_stage_roverl_inductance();
		if (ret == 0) {
			shell_warn(sh, "Staged R/L-implied scalar L after sweep instability");
			return 0;
		}
		return -ERANGE;
	}

	electrical_id_stage_scalar_inductance(qualified_avg_l_h, qualified_samples,
					      qualified_min_confidence,
					      qualified_max_residual);
	shell_print(sh,
		    "Production scalar inductance staged from sweep: Ld=Lq=%.9f H points=%u confidence=%.3f residual=%.4f",
		    (double)qualified_avg_l_h, qualified,
		    (double)qualified_min_confidence,
		    (double)qualified_max_residual);
	return staged.result_valid ? 0 : -ERANGE;
}

int cmd_motor_commission_electrical_run(const struct shell *sh, size_t argc, char **argv)
{
	float32_t current_a;
	float32_t pulse_v;
	uint32_t samples;
	if (parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_RS_CURRENT_A, &current_a) != 0 ||
	    parse_optional_float(argv, argc, 2U, COMMISSION_ELECTRICAL_DEMOD_PULSE_V, &pulse_v) != 0 ||
	    parse_optional_u32(argv, argc, 3U, COMMISSION_ELECTRICAL_SAMPLES, &samples) != 0) {
		shell_error(sh, "Usage: motor commission electrical run [rs_current_a] [demod_pulse_v] [samples]");
		return -EINVAL;
	}

	char *rs_args[] = { "rs", NULL, NULL, NULL };
	char current_buf[16];
	char sample_buf[16];
	snprintk(current_buf, sizeof(current_buf), "%.6f", (double)current_a);
	snprintk(sample_buf, sizeof(sample_buf), "%u", samples);
	rs_args[1] = current_buf;
	rs_args[2] = sample_buf;
	int ret = cmd_motor_commission_electrical_measure_rs(sh, 3, rs_args);
	if (ret != 0) {
		return ret;
	}

	char *l_args[] = { "demod", NULL, NULL };
	char pulse_buf[16];
	snprintk(pulse_buf, sizeof(pulse_buf), "%.6f", (double)pulse_v);
	l_args[1] = pulse_buf;
	l_args[2] = sample_buf;
	ret = cmd_motor_commission_electrical_measure_inductance(sh, 3, l_args);
	if (ret == 0) {
		return 0;
	}

	shell_warn(sh,
		   "Production inductance pulse rejected (err %d); staging fallback L for PI recommendation",
		   ret);
	ret = electrical_id_stage_fallback_inductance();
	if (ret != 0) {
		shell_error(sh, "Fallback inductance staging failed (err %d)", ret);
		return ret;
	}
	shell_print(sh,
		    "Production inductance staged from fallback: Ld=%.9f H Lq=%.9f H Lavg=%.9f H",
		    (double)staged.ld.inductance_h,
		    (double)staged.lq.inductance_h,
		    (double)staged.result.l_avg_h);
	return 0;
}

int cmd_motor_commission_electrical_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Production Electrical ID:");
	shell_print(sh, "  Rs:     %s %.6f ohm samples=%u confidence=%.3f residual=%.4f",
		    staged.rs_valid ? "valid" : "staged?",
		    (double)staged.rs.rs_ohm, staged.rs.samples,
		    (double)staged.rs.confidence, (double)staged.rs.residual_ratio);
	shell_print(sh, "  Ld:     %s %.9f H samples=%u confidence=%.3f residual=%.4f",
		    staged.ld_valid ? "valid" : "staged?",
		    (double)staged.ld.inductance_h, staged.ld.samples,
		    (double)staged.ld.confidence, (double)staged.ld.residual_ratio);
	shell_print(sh, "  Lq:     %s %.9f H samples=%u confidence=%.3f residual=%.4f",
		    staged.lq_valid ? "valid" : "staged?",
		    (double)staged.lq.inductance_h, staged.lq.samples,
		    (double)staged.lq.confidence, (double)staged.lq.residual_ratio);
	shell_print(sh, "  Result: %s Lavg=%.9f H Lq-Ld=%.9f H mismatch=%.3f flags=0x%02x",
		    staged.result_valid ? "valid" : "not-valid",
		    (double)staged.result.l_avg_h, (double)staged.result.lq_minus_ld_h,
		    (double)staged.result.axis_mismatch_ratio, staged.result.flags);
	shell_print(sh, "  L source: %s",
		    staged.inductance_roverl ? "roverL" :
		    (staged.inductance_fallback ? "fallback" :
		     (staged.inductance_demod ? "demod" :
		      (staged.inductance_saliency ? "saliency" : "pulse"))));
	if (staged.inductance_demod) {
		shell_print(sh, "  Demod:  D inv=%.3f spread=%.3f, Q inv=%.3f spread=%.3f",
			    (double)staged.ld_demod.inv_l_mean,
			    (double)staged.ld_demod.spread_ratio,
			    (double)staged.lq_demod.inv_l_mean,
			    (double)staged.lq_demod.spread_ratio);
	}
	if (staged.saliency.sample_count > 0U) {
		shell_print(sh,
			    "  Saliency diagnostic: %s ratio=%.3f phase=%.3f rad inv_offset=%.3f inv_amp1=%.3f inv_amp2=%.3f inv_amp4=%.3f rejected=%u",
			    staged.inductance_saliency ? "production" : "not-staged",
			    (double)staged.saliency.saliency_ratio,
			    (double)staged.saliency.phase_rad,
			    (double)staged.saliency.inv_l_offset,
			    (double)staged.saliency.inv_l_amplitude1,
			    (double)staged.saliency.inv_l_amplitude,
			    (double)staged.saliency.inv_l_amplitude4,
			    staged.saliency.rejected_samples);
	}
	shell_print(sh, "  PI:     %s bw=%.1f Hz Id(Kp=%.6f Ki=%.6f) Iq(Kp=%.6f Ki=%.6f)",
		    staged.pi_valid ? "valid" : "not-valid",
		    (double)staged.pi.bandwidth_hz,
		    (double)staged.pi.kp_d, (double)staged.pi.ki_d,
		    (double)staged.pi.kp_q, (double)staged.pi.ki_q);
	if (g_motor_params != NULL) {
		shell_print(sh, "  Active fallback/current: Rs=%.6f ohm Ls=%.9f H R/L=%.3f rad/s",
			    (double)g_motor_params->Rs_measured_ohm,
			    (double)g_motor_params->Ls_measured_H,
			    (double)g_motor_params->R_over_L_measured);
		if (g_motor_params->R_over_L_measured > 0.0f && staged.rs_valid) {
			shell_print(sh, "  R/L implied L from staged Rs: %.9f H",
				    (double)(staged.rs.rs_ohm /
					     g_motor_params->R_over_L_measured));
		}
	}
	return 0;
}

int cmd_motor_commission_electrical_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before applying production electrical ID.");
		return -EACCES;
	}
	if (!staged.result_valid || !staged.pi_valid) {
		shell_error(sh, "No valid production electrical ID result staged.");
		return -EACCES;
	}

	g_motor_params->Rs_measured_ohm = staged.result.rs_ohm;
	g_motor_params->Ls_measured_H = staged.result.l_avg_h;
	g_motor_params->R_over_L_measured = staged.result.rs_ohm / staged.result.l_avg_h;
	pi_set_gains(&g_motor_params->pi_Id, staged.pi.kp_d, staged.pi.ki_d);
	pi_set_gains(&g_motor_params->pi_Iq, staged.pi.kp_q, staged.pi.ki_q);
	pi_set_ui(&g_motor_params->pi_Id, 0.0f);
	pi_set_ui(&g_motor_params->pi_Iq, 0.0f);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Production electrical ID applied: Rs=%.6f Ld=%.9f Lq=%.9f R/L=%.3f",
		    (double)staged.result.rs_ohm,
		    (double)staged.result.ld_h,
		    (double)staged.result.lq_h,
		    (double)g_motor_params->R_over_L_measured);
	shell_print(sh, "Current PI applied: Id Kp=%.6f Ki=%.6f, Iq Kp=%.6f Ki=%.6f",
		    (double)staged.pi.kp_d, (double)staged.pi.ki_d,
		    (double)staged.pi.kp_q, (double)staged.pi.ki_q);
	return 0;
}

int cmd_motor_commission_electrical_validate(const struct shell *sh, size_t argc, char **argv)
{
	float32_t current_a;
	uint32_t hold_ms;
	float32_t max_error_a;
	if (parse_optional_float(argv, argc, 1U, COMMISSION_ELECTRICAL_RS_CURRENT_A, &current_a) != 0 ||
	    parse_optional_u32(argv, argc, 2U, COMMISSION_ELECTRICAL_VALIDATE_MS, &hold_ms) != 0 ||
	    parse_optional_float(argv, argc, 3U, COMMISSION_ELECTRICAL_VALIDATE_ERROR_A,
				 &max_error_a) != 0) {
		shell_error(sh,
			    "Usage: motor commission electrical validate [current_a] [hold_ms] [max_error_a]");
		return -EINVAL;
	}
	if (g_motor_params == NULL) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (current_a <= 0.0f || current_a > electrical_id_current_limit() ||
	    hold_ms < COMMISSION_ELECTRICAL_MIN_PULSE_MS || max_error_a <= 0.0f) {
		shell_error(sh, "Invalid current/hold/error; current in (0, %.4f]",
			    (double)electrical_id_current_limit());
		return -EINVAL;
	}

	int ret = electrical_id_enter_generated_current_mode(sh);
	if (ret != 0) {
		return ret;
	}

	motor_current_slew_params_set_target_ramp(g_motor_params, current_a, 0.0f,
						  COMMISSION_ELECTRICAL_CURRENT_RAMP_MS / 1000.0f);
	motor_command_feed_watchdog(g_motor_params);
	ret = motor_commission_wait_ms_or_fault(COMMISSION_ELECTRICAL_CURRENT_RAMP_MS + 50U);
	if (ret != 0) {
		electrical_id_stop();
		return ret;
	}

	electrical_id_start_validate_capture(current_a, hold_ms);
	ret = electrical_id_wait_capture_done(hold_ms + 1000U);
	if (ret != 0) {
		electrical_id_stop();
		return ret;
	}

	struct motor_electrical_id_capture_ctx *cap = &g_motor_params->electrical_id_capture;
	electrical_id_stop();
	uint32_t samples = cap->sample_count;
	float32_t avg_abs_error = (samples > 0U) ?
		cap->validation_sum_abs_error_a / (float32_t)samples : INFINITY;
	float32_t max_abs_error = cap->validation_max_abs_error_a;
	bool pass = samples > 0U && avg_abs_error <= max_error_a;
	shell_print(sh,
		    "Production electrical current-step validation: %s target=%.4f A avg_err=%.5f A max_err=%.5f A samples=%u limit=%.5f A",
		    pass ? "PASS" : "FAIL",
		    (double)current_a, (double)avg_abs_error, (double)max_abs_error,
		    samples, (double)max_error_a);
	return pass ? 0 : -ERANGE;
}
