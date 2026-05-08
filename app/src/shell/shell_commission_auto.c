/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "shell_commands_commission.h"
#include "shell_commands_motion.h"
#include "shell_commission_internal.h"
#include "motor/runtime/commission_runtime.h"
#include "motor_control_api.h"
#include "motor_current_slew.h"
#include "motor_torque.h"
#include "shell_control_common.h"
#include "shell_parse.h"
#include "motor/math/math_constants.h"

#define MOTOR_COMMISSION_AUTO_POLL_MS 10U
#define MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_MAX_HZ 5.0f
#define MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HZ_CAP 20.0f
#define MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_HOLD_MS 2000U

static const float32_t motor_commission_mech_step_pattern[] = {
	-1.0f, 1.0f, -0.5f, 0.5f, 0.0f, -1.0f,
};

static const float32_t motor_commission_validate_step_scale[] = {
	0.2f, 0.6f, 1.0f, -0.2f, -0.6f, -1.0f, 0.0f,
};

struct motor_commission_velocity_gain_restore {
	float32_t kp_a_per_rad_s;
	float32_t ki_a_per_rad;
	float32_t iq_limit_a;
	float32_t i_term_a;
	bool valid;
};



static const char *motor_commission_mech_reject_to_string(uint8_t reason)
{
	switch (reason) {
	case MOTOR_COMMISSION_MECH_REJECT_NONE:
		return "none";
	case MOTOR_COMMISSION_MECH_REJECT_KT_INVALID:
		return "kt_invalid";
	case MOTOR_COMMISSION_MECH_REJECT_SAMPLES:
		return "samples";
	case MOTOR_COMMISSION_MECH_REJECT_SOLVER:
		return "solver";
	case MOTOR_COMMISSION_MECH_REJECT_FINALIZE:
		return "finalize";
	case MOTOR_COMMISSION_MECH_REJECT_VISCOUS_NEGATIVE:
		return "viscous_negative";
	case MOTOR_COMMISSION_MECH_REJECT_FIT_INVALID:
		return "fit_invalid";
	case MOTOR_COMMISSION_MECH_REJECT_FRICTION_INVALID:
		return "friction_invalid";
	case MOTOR_COMMISSION_MECH_REJECT_INERTIA_INVALID:
		return "inertia_invalid";
	case MOTOR_COMMISSION_MECH_REJECT_IMPLAUSIBLE:
		return "implausible";
	case MOTOR_COMMISSION_MECH_REJECT_CONFIDENCE:
		return "confidence";
	default:
		return "unknown";
	}
}

static void motor_commission_velocity_gains_save(
	struct motor_commission_velocity_gain_restore *restore)
{
	if (restore == NULL || g_motor_params == NULL) {
		return;
	}

	*restore = (struct motor_commission_velocity_gain_restore){
		.kp_a_per_rad_s = g_motor_params->velocity_cl_kp_A_per_rad_s,
		.ki_a_per_rad = g_motor_params->velocity_cl_ki_A_per_rad,
		.iq_limit_a = g_motor_params->velocity_cl_iq_limit_A,
		.i_term_a = g_motor_params->velocity_cl_i_term_A,
		.valid = true,
	};
}

static void motor_commission_velocity_gains_restore(
	const struct motor_commission_velocity_gain_restore *restore)
{
	if (restore == NULL || !restore->valid || g_motor_params == NULL) {
		return;
	}

	(void)motor_apply_velocity_gains(restore->kp_a_per_rad_s,
					 restore->ki_a_per_rad,
					 restore->iq_limit_a);
	g_motor_params->velocity_cl_i_term_A = restore->i_term_a;
}

static int motor_commission_stage_velocity_capture_gains(
	const struct shell *sh,
	float32_t iq_limit_a,
	struct motor_commission_velocity_gain_restore *restore)
{
	if (g_motor_params == NULL || restore == NULL) {
		return -ENODEV;
	}

	motor_commission_velocity_gains_save(restore);

	float32_t kp = 0.0f;
	float32_t ki = 0.0f;
	float32_t kt = 0.0f;
	float32_t iq_limit = clampf(iq_limit_a, 0.04f, MOTOR_MAX_CURRENT_A);
	bool measured_mech_model =
		g_motor_params->mech_model_source == MOTOR_MODEL_SOURCE_MEASURED &&
		isfinite(g_motor_params->inertia_kgm2_active) &&
		g_motor_params->inertia_kgm2_active > 0.0f;
	int ret = measured_mech_model ?
			  motor_compute_velocity_bandwidth_gains(g_motor_params,
								 COMMISSION_AUTO_VELOCITY_BANDWIDTH_HZ,
								 0.8f,
								 iq_limit,
								 &kp,
								 &ki,
								 &kt) :
			  -ENOENT;
	const char *source = "model";
	if (ret != 0) {
		float32_t gain_speed_rad_s = 2.0f * PI_F32 * VELOCITY_DEFAULT_GAIN_SPEED_HZ;
		kp = 0.75f * iq_limit / gain_speed_rad_s;
		ki = 2.0f * kp;
		source = "authority";
	}

	ret = motor_apply_velocity_gains(kp, ki, iq_limit);
	if (ret != 0) {
		motor_commission_velocity_gains_restore(restore);
		return ret;
	}

	shell_print(sh,
		    "  Commission velocity PI: bw=%.2f Hz source=%s Kp=%.5f Ki=%.5f iq=%.3f A",
		    (double)COMMISSION_AUTO_VELOCITY_BANDWIDTH_HZ,
		    source,
		    (double)kp,
		    (double)ki,
		    (double)iq_limit);
	return 0;
}

static const char *motor_commission_tune_error_to_string(int err)
{
	switch (err) {
	case 0:
		return "none";
	case -ERANGE:
		return "quality_rejected";
	case -ENOENT:
		return "not_staged";
	case -ETIMEDOUT:
		return "timeout";
	case -EFAULT:
		return "state_error";
	case -ECANCELED:
		return "aborted";
	default:
		return "error";
	}
}

static void motor_commission_print_tune_reject_flags(const struct shell *sh, uint32_t flags)
{
	if (flags == MOTOR_COMMISSION_TUNE_REJECT_NONE) {
		shell_print(sh, "  Tune reject:    none");
		return;
	}

	shell_print(sh, "  Tune reject:    0x%08X", flags);
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_INVALID) != 0U) {
		shell_print(sh, "    - psi_f estimate invalid");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_SAMPLES) != 0U) {
		shell_print(sh, "    - psi_f sample count too low");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_R2) != 0U) {
		shell_print(sh, "    - psi_f R2 below threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_RMS) != 0U) {
		shell_print(sh, "    - psi_f residual RMS above threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_PSI_SIGN) != 0U) {
		shell_print(sh, "    - psi_f sign/finite check failed");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_INVALID) != 0U) {
		shell_print(sh, "    - mechanical estimate invalid");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_SAMPLES) != 0U) {
		shell_print(sh, "    - mechanical sample count too low");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_R2) != 0U) {
		shell_print(sh, "    - mechanical R2 below threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_MECH_RMS) != 0U) {
		shell_print(sh, "    - mechanical residual RMS above threshold");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_INERTIA_SIGN) != 0U) {
		shell_print(sh, "    - inertia sign/finite check failed");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_VISCOUS_SIGN) != 0U) {
		shell_print(sh, "    - viscous friction sign/finite check failed");
	}
	if ((flags & MOTOR_COMMISSION_TUNE_REJECT_KT_INVALID) != 0U) {
		shell_print(sh, "    - torque constant check failed");
	}
}

static void motor_commission_print_mech_fit_summary(const struct shell *sh,
						    const char *prefix,
						    const struct motor_commission_results *res)
{
	float32_t confidence_margin =
		res->mech_confidence - COMMISSION_AUTO_MECH_MIN_CONFIDENCE;

	shell_print(sh,
		    "%s valid=%s reason=%s err=%d tq_sign=%d J=%.8f B=%.8f Tc=%.8f T0=%.8f R2=%.4f rms=%.6f N=%u",
		    prefix,
		    res->mech_valid ? "YES" : "NO",
		    motor_commission_mech_reject_to_string(res->mech_reject_reason),
		    (int)res->mech_finalize_error,
		    res->mech_fit_torque_sign,
		    (double)res->inertia_kgm2,
		    (double)res->viscous_friction_nm_per_rad_s,
		    (double)res->coulomb_friction_nm,
		    (double)res->offset_friction_nm,
		    (double)res->mech_r2,
		    (double)res->mech_residual_rms_nm,
		    res->mech_sample_count);
	shell_print(sh,
		    "%s v2=%s friction=%s inertia=%s conf=%.3f min=%.3f margin=%+.3f J/fallback=%.3f detent=%s accelN=%u",
		    prefix,
		    res->mech_v2_valid ? "YES" : "NO",
		    res->mech_friction_valid ? "YES" : "NO",
		    res->mech_inertia_valid ? "YES" : "NO",
		    (double)res->mech_confidence,
		    (double)COMMISSION_AUTO_MECH_MIN_CONFIDENCE,
		    (double)confidence_margin,
		    (double)res->mech_inertia_plausibility_ratio,
		    res->mech_detent_corrected ? "active_map" : "none",
		    res->mech_accel_window_valid_count);
	shell_print(sh,
		    "%s components: friction_rms=%.6f Nm friction_N=%u inertia_rms=%.6f Nm inertia_N=%u",
		    prefix,
		    (double)res->mech_friction_residual_rms_nm,
		    res->mech_friction_sample_count,
		    (double)res->mech_inertia_residual_rms_nm,
		    res->mech_inertia_sample_count);
}
static int motor_commission_wait_for_capture_stop(uint32_t timeout_ms)
{
	uint32_t start_ms = k_uptime_get_32();

	while ((k_uptime_get_32() - start_ms) < timeout_ms) {
		const struct motor_commission_ctx *ctx = &g_motor_params->commission;
		if (!ctx->active) {
			if (ctx->stage == MOTOR_COMMISSION_STAGE_ABORTED) {
				return -ECANCELED;
			}
			if (ctx->stage == MOTOR_COMMISSION_STAGE_COMPLETED) {
				return 0;
			}
			return -EIO;
		}
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return -ETIMEDOUT;
}

void motor_commission_set_velocity_target_hz(float32_t target_hz)
{
	float32_t target_rad_s = target_hz * 2.0f * PI_F32;
	float32_t limited = clampf(target_rad_s,
				   -g_motor_params->profile_max_velocity_rad_s,
				   g_motor_params->profile_max_velocity_rad_s);

	traj_set_target_value(&g_motor_params->traj_velocity, limited);
}


int motor_commission_wait_ms_or_fault(uint32_t hold_ms)
{
	uint32_t start_ms = k_uptime_get_32();

	while ((k_uptime_get_32() - start_ms) < hold_ms) {
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			return -EFAULT;
		}
		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

	return 0;
}

void motor_commission_print_velocity_validation_sample(const struct shell *sh,
							     float32_t target_hz)
{
	float32_t ref_hz = g_motor_params->live.velocity_ref_rad_s / (2.0f * PI_F32);
	float32_t meas_hz = g_motor_params->live.velocity_filtered_rad_s / (2.0f * PI_F32);
	float32_t err_hz = target_hz - meas_hz;

	shell_print(sh,
		    "  target=%7.3f Hz ref=%7.3f Hz meas=%7.3f Hz err=%7.3f Hz Iq_ref=%.4f A Iq=%.4f A Id=%.4f A warn=%u err=%u",
		    (double)target_hz,
		    (double)ref_hz,
		    (double)meas_hz,
		    (double)err_hz,
		    (double)g_motor_params->live.Iq_ref_A,
		    (double)g_motor_params->live.Iq_A,
		    (double)g_motor_params->live.Id_A,
		    g_motor_params->live.encoder_sample_warning,
		    g_motor_params->live.encoder_sample_error);
}


void motor_commission_encoder_trace_force_on(
	struct motor_commission_encoder_trace_guard *guard)
{
	guard->raw_trace_enabled = g_motor_params->encoder_raw_trace.enabled;
	guard->raw_trace_decimation = g_motor_params->encoder_raw_trace.decimation;
	guard->raw_trace_phase = g_motor_params->encoder_raw_trace.phase;

	/* Generated/open-loop modes do not normally request encoder samples.
	 * Raw-trace enable is the existing ISR-safe telemetry gate that asks the
	 * encoder acquisition to sample without changing the commutation policy.
	 */
	g_motor_params->encoder_raw_trace.write_idx = 0U;
	g_motor_params->encoder_raw_trace.count = 0U;
	g_motor_params->encoder_raw_trace.overrun_count = 0U;
	g_motor_params->encoder_raw_trace.enabled = true;
	g_motor_params->encoder_raw_trace.decimation = 1U;
	g_motor_params->encoder_raw_trace.phase = 0U;
}

void motor_commission_encoder_trace_force_on_decimated(
	struct motor_commission_encoder_trace_guard *guard,
	uint16_t decimation)
{
	motor_commission_encoder_trace_force_on(guard);
	g_motor_params->encoder_raw_trace.decimation = MAX(decimation, (uint16_t)1U);
}

void motor_commission_encoder_trace_restore(
	const struct motor_commission_encoder_trace_guard *guard)
{
	g_motor_params->encoder_raw_trace.enabled = guard->raw_trace_enabled;
	g_motor_params->encoder_raw_trace.decimation = guard->raw_trace_decimation;
	g_motor_params->encoder_raw_trace.phase = guard->raw_trace_phase;
}

bool motor_commission_encoder_latest_raw_trace_after(
	uint32_t min_loop,
	struct motor_encoder_raw_trace_sample *out)
{
	if (out == NULL || g_motor_params == NULL ||
	    g_motor_params->encoder_raw_trace.count == 0U) {
		return false;
	}

	uint16_t write_idx = g_motor_params->encoder_raw_trace.write_idx;
	uint16_t idx = (write_idx == 0U) ?
			       (uint16_t)(MOTOR_ENCODER_RAW_TRACE_MAX_SAMPLES - 1U) :
			       (uint16_t)(write_idx - 1U);
	struct motor_encoder_raw_trace_sample sample =
		g_motor_params->encoder_raw_trace.samples[idx];
	if (sample.control_loop_count <= min_loop) {
		return false;
	}

	*out = sample;
	return true;
}

struct motor_commission_motion_threshold_result {
	float32_t threshold_a;
	float32_t best_net_motion_rad;
	float32_t best_abs_motion_rad;
	uint16_t sample_count;
	uint16_t warning_count;
	uint16_t error_count;
	bool valid;
};
static int motor_commission_auto_run_flux(const struct shell *sh,
					  const struct motor_commission_flux_config *cfg)
{
	int ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		return ret;
	}

	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		return ret;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	ret = motor_commission_start_flux(&commission_ctx, cfg);
	if (ret != 0) {
		return ret;
	}

	(void)motor_api_set_param("velocity_cl_iq_limit_A", cfg->iq_limit_a);

	float32_t delta_hz = (cfg->steps > 1U) ?
				    ((cfg->max_speed_hz - cfg->min_speed_hz) /
				     (float32_t)(cfg->steps - 1U)) :
				    0.0f;
	uint32_t dwell_ms = cfg->settle_ms + cfg->sample_ms;

	for (uint32_t pass = 0U; pass < 2U; pass++) {
		float32_t sign = (pass == 0U) ? 1.0f : -1.0f;
		for (uint32_t i = 0U; i < cfg->steps; i++) {
			float32_t speed_hz = cfg->min_speed_hz + (delta_hz * (float32_t)i);
			float32_t target_hz = sign * speed_hz;
			motor_commission_set_velocity_target_hz(target_hz);
			motor_command_feed_watchdog(g_motor_params);

			shell_print(sh, "  Flux sweep step %u/%u: target=%.3f Hz",
				    (unsigned int)(pass * cfg->steps + i + 1U),
				    (unsigned int)(2U * cfg->steps),
				    (double)target_hz);

			uint32_t start_ms = k_uptime_get_32();
			while ((k_uptime_get_32() - start_ms) < dwell_ms) {
				if (g_motor_params->commission.stage == MOTOR_COMMISSION_STAGE_ABORTED) {
					return -ECANCELED;
				}
				if (motor_api_get_state() == MOTOR_STATE_ERROR) {
					return -EFAULT;
				}
				motor_command_feed_watchdog(g_motor_params);
				k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
			}
		}
	}

	ret = motor_commission_wait_for_capture_stop(COMMISSION_AUTO_POST_WAIT_MS);
	if (ret != 0) {
		return ret;
	}
	if (!g_motor_params->commission.results.psi_f_valid) {
		return -ERANGE;
	}

	return 0;
}

static int motor_commission_auto_run_mech(const struct shell *sh,
					  const struct motor_commission_mech_config *cfg,
					  bool require_fit)
{
	int ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		return ret;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		return ret;
	}

	motor_commission_set_velocity_target_hz(cfg->base_speed_hz);

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	ret = motor_commission_start_mech(&commission_ctx, cfg);
	if (ret != 0) {
		return ret;
	}

	const uint32_t pattern_len = ARRAY_SIZE(motor_commission_mech_step_pattern);
	const uint32_t one_direction_steps = pattern_len;
	const uint32_t full_cycle_steps = 2U * one_direction_steps;
	uint32_t run_start_ms = k_uptime_get_32();
	uint32_t last_step = UINT32_MAX;
	while (g_motor_params->commission.active) {
		uint32_t now_ms = k_uptime_get_32();
		uint32_t elapsed_ms = now_ms - run_start_ms;
		uint32_t step = elapsed_ms / cfg->dither_period_ms;
		if (step != last_step) {
			uint32_t cycle_step = step % full_cycle_steps;
			float32_t direction = (cycle_step < one_direction_steps) ? 1.0f : -1.0f;
			uint32_t pattern_index = cycle_step % pattern_len;
			float32_t dither =
				motor_commission_mech_step_pattern[pattern_index] *
				cfg->dither_speed_hz;
			float32_t target_hz = direction * (cfg->base_speed_hz + dither);
			motor_commission_set_velocity_target_hz(target_hz);
			last_step = step;
		}

		if (g_motor_params->commission.stage == MOTOR_COMMISSION_STAGE_ABORTED) {
			ret = -ECANCELED;
			goto stop_current;
		}
		if (motor_api_get_state() == MOTOR_STATE_ERROR) {
			ret = -EFAULT;
			goto stop_current;
		}

		motor_command_feed_watchdog(g_motor_params);
		k_msleep(MOTOR_COMMISSION_AUTO_POLL_MS);
	}

stop_current:
	motor_current_slew_params_zero(g_motor_params);
	motor_commission_set_velocity_target_hz(0.0f);
	if (ret != 0) {
		return ret;
	}

	ret = motor_commission_wait_for_capture_stop(COMMISSION_AUTO_POST_WAIT_MS);
	if (ret != 0) {
		return ret;
	}
	if (require_fit && !g_motor_params->commission.results.mech_valid) {
		return -ERANGE;
	}

	shell_print(sh, "  Mech excitation complete");
	return 0;
}

struct motor_commission_mech_aggregate {
	uint8_t count;
	uint32_t sample_count;
	float32_t sum_j;
	float32_t sum_j2;
	float32_t sum_b;
	float32_t sum_b2;
	float32_t sum_tc;
	float32_t sum_tc2;
	float32_t sum_t0;
	float32_t sum_rms;
	float32_t sum_r2;
	float32_t sum_confidence;
	float32_t min_r2;
	float32_t max_rms;
	float32_t min_confidence;
	float32_t max_confidence;
	int8_t fit_torque_sign;
};

static float32_t motor_commission_stddev(uint8_t count, float32_t sum, float32_t sum2)
{
	if (count < 2U) {
		return 0.0f;
	}

	float32_t n = (float32_t)count;
	float32_t mean = sum / n;
	float32_t variance = (sum2 / n) - (mean * mean);
	return sqrtf(fmaxf(variance, 0.0f));
}

static void motor_commission_mech_aggregate_add(
	struct motor_commission_mech_aggregate *agg,
	const struct motor_commission_results *res)
{
	if (agg == NULL || res == NULL || !res->mech_valid) {
		return;
	}

	if (agg->count == 0U) {
		agg->min_r2 = res->mech_r2;
		agg->max_rms = res->mech_residual_rms_nm;
		agg->min_confidence = res->mech_confidence;
		agg->max_confidence = res->mech_confidence;
		agg->fit_torque_sign = res->mech_fit_torque_sign;
	} else {
		agg->min_r2 = fminf(agg->min_r2, res->mech_r2);
		agg->max_rms = fmaxf(agg->max_rms, res->mech_residual_rms_nm);
		agg->min_confidence = fminf(agg->min_confidence, res->mech_confidence);
		agg->max_confidence = fmaxf(agg->max_confidence, res->mech_confidence);
		if (agg->fit_torque_sign != res->mech_fit_torque_sign) {
			agg->fit_torque_sign = 0;
		}
	}

	agg->count++;
	agg->sample_count += res->mech_sample_count;
	agg->sum_j += res->inertia_kgm2;
	agg->sum_j2 += res->inertia_kgm2 * res->inertia_kgm2;
	agg->sum_b += res->viscous_friction_nm_per_rad_s;
	agg->sum_b2 += res->viscous_friction_nm_per_rad_s *
		       res->viscous_friction_nm_per_rad_s;
	agg->sum_tc += res->coulomb_friction_nm;
	agg->sum_tc2 += res->coulomb_friction_nm * res->coulomb_friction_nm;
	agg->sum_t0 += res->offset_friction_nm;
	agg->sum_rms += res->mech_residual_rms_nm;
	agg->sum_r2 += res->mech_r2;
	agg->sum_confidence += res->mech_confidence;
}

static int motor_commission_mech_aggregate_finalize(
	const struct motor_commission_mech_aggregate *agg,
	struct motor_commission_results *res)
{
	if (agg == NULL || res == NULL || agg->count == 0U) {
		return -ENODATA;
	}

	float32_t n = (float32_t)agg->count;
	res->mech_capture_count = agg->count;
	res->inertia_kgm2 = agg->sum_j / n;
	res->viscous_friction_nm_per_rad_s = agg->sum_b / n;
	res->coulomb_friction_nm = agg->sum_tc / n;
	res->offset_friction_nm = agg->sum_t0 / n;
	res->mech_residual_rms_nm = agg->sum_rms / n;
	res->mech_r2 = agg->sum_r2 / n;
	res->mech_sample_count = (uint16_t)MIN(agg->sample_count / agg->count, UINT16_MAX);
	res->mech_finalize_error = 0;
	res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;
	res->mech_fit_torque_sign = agg->fit_torque_sign;
	res->mapping_direction_valid = false;
	res->mapping_direction_pass = false;
	res->mapping_direction_corr = 0.0f;
	res->mapping_valid = false;
	res->mapping_pass = false;
	res->mapping_confidence = 0.0f;
	res->inertia_stddev_kgm2 =
		motor_commission_stddev(agg->count, agg->sum_j, agg->sum_j2);
	res->viscous_friction_stddev_nm_per_rad_s =
		motor_commission_stddev(agg->count, agg->sum_b, agg->sum_b2);
	res->coulomb_friction_stddev_nm =
		motor_commission_stddev(agg->count, agg->sum_tc, agg->sum_tc2);

	float32_t j_cv = res->inertia_stddev_kgm2 /
			 fmaxf(fabsf(res->inertia_kgm2), 1.0e-9f);
	/*
	 * Per-capture confidence has already gated each accepted run. Keep
	 * aggregate confidence tied to the weakest accepted capture instead of
	 * rejecting on B spread: viscous friction is weakly observable on small
	 * hybrid steppers and may collapse to zero while J/Tc remain useful.
	 */
	res->mech_confidence = agg->min_confidence;
	res->mech_valid = isfinite(res->inertia_kgm2) && res->inertia_kgm2 > 0.0f &&
			  isfinite(res->viscous_friction_nm_per_rad_s) &&
			  res->viscous_friction_nm_per_rad_s >= 0.0f &&
			  isfinite(res->coulomb_friction_nm) &&
			  res->coulomb_friction_nm >= 0.0f &&
			  isfinite(res->mech_r2) && res->mech_r2 >= 0.20f &&
			  isfinite(res->mech_confidence) &&
			  res->mech_confidence >= COMMISSION_AUTO_MECH_MIN_CONFIDENCE;
	if (res->mech_valid && j_cv > 1.0f) {
		res->mech_valid = false;
	}
	res->mech_v2_valid = res->mech_valid;
	res->mech_friction_valid = res->mech_valid;
	res->mech_inertia_valid = res->mech_valid;
	if (!res->mech_valid) {
		res->mech_reject_reason =
			(res->mech_confidence < COMMISSION_AUTO_MECH_MIN_CONFIDENCE) ?
				MOTOR_COMMISSION_MECH_REJECT_CONFIDENCE :
				MOTOR_COMMISSION_MECH_REJECT_FIT_INVALID;
	}
	return res->mech_valid ? 0 : -ERANGE;
}

static int motor_commission_mech_validate_fit(
	const struct motor_commission_results *fit,
	float32_t *rms_nm,
	uint16_t *sample_count,
	int8_t *selected_torque_sign)
{
	if (fit == NULL || rms_nm == NULL || sample_count == NULL ||
	    !fit->psi_f_valid || !fit->mech_valid) {
		return -EINVAL;
	}

	float32_t kt = motor_torque_gain_from_flux_pole_pairs(fit->psi_f_wb,
							      MOTOR_POLE_PAIRS);
	if (!isfinite(kt) || kt <= 0.0f) {
		return -EINVAL;
	}

	float32_t sum_sq = 0.0f;
	float32_t sum_sq_inverted = 0.0f;
	uint32_t count = 0U;
	for (uint32_t i = 0U; i < g_motor_params->commission.sample_count; i++) {
		const struct motor_commission_sample *s = &g_motor_params->commission.samples[i];
		if (!isfinite(s->mech_speed_rad_s) || !isfinite(s->mech_accel_rad_s2) ||
		    !isfinite(s->iq_a) || fabsf(s->mech_speed_rad_s) < 0.5f) {
			continue;
		}

		float32_t sign_term = (s->mech_speed_rad_s >= 0.0f) ? 1.0f : -1.0f;
		float32_t predicted_nm =
			(fit->inertia_kgm2 * s->mech_accel_rad_s2) +
			(fit->viscous_friction_nm_per_rad_s * s->mech_speed_rad_s) +
			(fit->coulomb_friction_nm * sign_term) +
			fit->offset_friction_nm;
		float32_t measured_pos_nm = kt * s->iq_a;
		float32_t measured_neg_nm = -measured_pos_nm;
		float32_t err_pos = measured_pos_nm - predicted_nm;
		float32_t err_neg = measured_neg_nm - predicted_nm;
		sum_sq += err_pos * err_pos;
		sum_sq_inverted += err_neg * err_neg;
		count++;
	}

	if (count < 32U) {
		return -ENODATA;
	}

	float32_t rms_pos = sqrtf(sum_sq / (float32_t)count);
	float32_t rms_neg = sqrtf(sum_sq_inverted / (float32_t)count);
	if (!isfinite(rms_pos) || !isfinite(rms_neg)) {
		return -ERANGE;
	}

	if (rms_neg < rms_pos) {
		*rms_nm = rms_neg;
		if (selected_torque_sign != NULL) {
			*selected_torque_sign = -1;
		}
	} else {
		*rms_nm = rms_pos;
		if (selected_torque_sign != NULL) {
			*selected_torque_sign = 1;
		}
	}
	*sample_count = (uint16_t)MIN(count, UINT16_MAX);
	return isfinite(*rms_nm) ? 0 : -ERANGE;
}

int cmd_motor_commission_auto_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct motor_commission_ctx *ctx = &g_motor_params->commission;
	shell_print(sh, "Commission Auto Status:");
	shell_print(sh, "  Staged:        %s", ctx->auto_tune_valid ? "YES" : "NO");
	shell_print(sh, "  Applied:       %s", ctx->auto_tune_applied ? "YES" : "NO");
	shell_print(sh, "  Last error:    %d (%s)",
		    ctx->auto_tune_last_error,
		    motor_commission_tune_error_to_string(ctx->auto_tune_last_error));
	shell_print(sh, "  Tuned BW:      vel=%.2f Hz pos=%.2f Hz",
		    (double)ctx->auto_tune_staged.velocity_bw_hz,
		    (double)ctx->auto_tune_staged.position_bw_hz);
	shell_print(sh, "  Tuned PI:      vel(kp=%.5f ki=%.5f iq=%.3f) pos(kp=%.5f ki=%.5f)",
		    (double)ctx->auto_tune_staged.velocity_kp_a_per_rad_s,
		    (double)ctx->auto_tune_staged.velocity_ki_a_per_rad,
		    (double)ctx->auto_tune_staged.velocity_iq_limit_a,
		    (double)ctx->auto_tune_staged.position_kp_rad_s_per_rad,
		    (double)ctx->auto_tune_staged.position_ki_rad_s2_per_rad);
	shell_print(sh, "  Tuned DOB:     en=%s gain=%.5f tq_lim=%.5f iq_ff_lim=%.5f",
		    ctx->auto_tune_staged.velocity_dob_enable ? "YES" : "NO",
		    (double)ctx->auto_tune_staged.velocity_dob_observer_gain_nm_per_rad_s,
		    (double)ctx->auto_tune_staged.velocity_dob_torque_limit_nm,
		    (double)ctx->auto_tune_staged.velocity_dob_iq_ff_limit_a);
	shell_print(sh, "  Model source:  flux=%s mech=%s",
		    motor_shell_flux_source_name(g_motor_params),
		    g_motor_params->mech_model_source == MOTOR_MODEL_SOURCE_MEASURED ?
			    "MEASURED" : "FALLBACK");
	motor_commission_print_tune_reject_flags(sh, ctx->auto_tune_staged.reject_flags);
	return 0;
}

int cmd_motor_commission_auto_apply(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_apply_staged_auto_tune(&commission_ctx);
	if (ret == -ENOENT) {
		shell_error(sh, "No staged auto-tune result to apply");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "Failed to apply staged auto-tune (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Auto-tuned parameters applied to active runtime configuration");
	return 0;
}

int cmd_motor_commission_auto_validate(const struct shell *sh, size_t argc, char **argv)
{
	float32_t max_hz = MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_MAX_HZ;
	uint32_t hold_ms = MOTOR_COMMISSION_AUTO_VALIDATE_DEFAULT_HOLD_MS;

	if (argc > 3) {
		shell_error(sh, "Usage: motor commission auto validate [max_hz] [hold_ms]");
		return -EINVAL;
	}
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Calibration is not complete; run calibration first");
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before validation");
		return -EACCES;
	}
	if (argc >= 2 && !shell_parse_finite_float(argv[1], &max_hz)) {
		shell_error(sh, "max_hz must be a finite number");
		return -EINVAL;
	}
	if (argc >= 3 && !shell_parse_u32(argv[2], &hold_ms)) {
		shell_error(sh, "hold_ms must be an integer");
		return -EINVAL;
	}
	if (!isfinite(max_hz) || max_hz <= 0.0f) {
		shell_error(sh, "max_hz must be positive");
		return -EINVAL;
	}
	hold_ms = CLAMP(hold_ms,
			MOTOR_COMMISSION_AUTO_VALIDATE_MIN_HOLD_MS,
			MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HOLD_MS);

	float32_t profile_max_hz =
		g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	float32_t limited_max_hz =
		clampf(max_hz, 0.1f, fminf(profile_max_hz,
					    MOTOR_COMMISSION_AUTO_VALIDATE_MAX_HZ_CAP));

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	int ret = motor_commission_apply_staged_auto_tune(&commission_ctx);
	if (ret == -ENOENT) {
		shell_error(sh, "No staged auto-tune result to validate");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "Failed to apply staged auto-tune before validation (err %d)",
			    ret);
		return ret;
	}

	/* Validate the conservative PI path first. DOB/MPR can be enabled after this passes. */
	(void)motor_api_set_param("outer_loop_mode", (float32_t)MOTOR_OUTER_LOOP_MODE_PI);
	(void)motor_api_set_param("velocity_dob_enable", 0.0f);

	ret = motor_commission_request_online_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER);
	if (ret != 0) {
		shell_error(sh, "Failed to request velocity_encoder mode (err %d)", ret);
		return ret;
	}
	ret = motor_commission_wait_for_mode(MOTOR_STATE_ONLINE_VELOCITY_ENCODER,
					     MOTOR_COMMISSION_AUTO_MODE_TIMEOUT_MS);
	if (ret != 0) {
		shell_error(sh, "Failed to enter velocity_encoder mode (err %d)", ret);
		return ret;
	}

	shell_print(sh,
		    "Auto validation: applied staged tune, PI outer loop, DOB disabled, max=%.3f Hz hold=%u ms",
		    (double)limited_max_hz, hold_ms);
	shell_print(sh,
		    "  active: psi_f=%.8f Wb Kt=%.8f Nm/A J=%.8f kgm2 B=%.8f Tc=%.8f",
		    (double)g_motor_params->flux_linkage_wb_active,
		    (double)motor_torque_gain_resolve_active(g_motor_params),
		    (double)g_motor_params->inertia_kgm2_active,
		    (double)g_motor_params->viscous_friction_nm_per_rad_s_active,
		    (double)g_motor_params->coulomb_friction_nm_active);

	for (uint32_t i = 0U; i < ARRAY_SIZE(motor_commission_validate_step_scale); i++) {
		float32_t target_hz = limited_max_hz * motor_commission_validate_step_scale[i];
		motor_commission_set_velocity_target_hz(target_hz);
		motor_command_feed_watchdog(g_motor_params);
		ret = motor_commission_wait_ms_or_fault(hold_ms);
		motor_commission_print_velocity_validation_sample(sh, target_hz);
		if (ret != 0) {
			shell_error(sh, "Validation stopped by motor fault (err %d)", ret);
			goto stop_velocity;
		}
	}

stop_velocity:
	motor_commission_set_velocity_target_hz(0.0f);
	motor_command_feed_watchdog(g_motor_params);
	if (ret == 0) {
		shell_print(sh, "Auto validation complete; velocity target returned to 0 Hz");
	}
	return ret;
}

static void motor_commission_auto_print_usage(const struct shell *sh)
{
	shell_error(sh,
		    "Usage: motor commission auto run [slow|confirm] [apply]");
}

int cmd_motor_commission_auto_run(const struct shell *sh, size_t argc, char **argv)
{
	bool apply_on_success = false;
	bool execute_motion = false;
	bool slow_profile = false;

	if (argc > 3) {
		motor_commission_auto_print_usage(sh);
		return -EINVAL;
	}
	for (size_t i = 1U; i < argc; i++) {
		if (strcmp(argv[i], "slow") == 0) {
			execute_motion = true;
			slow_profile = true;
		} else if (strcmp(argv[i], "confirm") == 0) {
			execute_motion = true;
			slow_profile = false;
		} else if (strcmp(argv[i], "apply") == 0 || strcmp(argv[i], "1") == 0 ||
			   strcmp(argv[i], "true") == 0) {
			apply_on_success = true;
		} else {
			motor_commission_auto_print_usage(sh);
			return -EINVAL;
		}
	}
	if (apply_on_success && !execute_motion) {
		shell_error(sh, "Refusing implicit motion. Use 'run slow apply' or 'run confirm apply'.");
		return -EACCES;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	float32_t max_velocity_hz = g_motor_params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	if (!isfinite(max_velocity_hz) || max_velocity_hz < 0.10f) {
		max_velocity_hz = MOTOR_MAX_SPEED_HZ;
	}
	float32_t flux_cap_hz = slow_profile ?
					COMMISSION_AUTO_SLOW_FLUX_MAX_HZ :
					COMMISSION_AUTO_NORMAL_FLUX_MAX_HZ;
	float32_t flux_min_req_hz = slow_profile ?
					    COMMISSION_AUTO_SLOW_FLUX_MIN_HZ :
					    COMMISSION_AUTO_NORMAL_FLUX_MIN_HZ;
	float32_t mech_cap_hz = slow_profile ?
					COMMISSION_AUTO_SLOW_MECH_MAX_HZ :
					COMMISSION_AUTO_NORMAL_MECH_MAX_HZ;
	float32_t mech_base_req_hz = slow_profile ?
					     COMMISSION_AUTO_SLOW_MECH_BASE_HZ :
					     COMMISSION_AUTO_NORMAL_MECH_BASE_HZ;
	float32_t mech_dither_req_hz = slow_profile ?
					       COMMISSION_AUTO_SLOW_MECH_DITHER_HZ :
					       COMMISSION_AUTO_NORMAL_MECH_DITHER_HZ;
	float32_t planned_flux_max_hz = fminf(max_velocity_hz, flux_cap_hz);
	float32_t planned_flux_min_hz =
		fminf(flux_min_req_hz, fmaxf(0.05f, 0.50f * planned_flux_max_hz));
	float32_t planned_mech_upper_hz = fminf(max_velocity_hz, mech_cap_hz);
	float32_t planned_mech_base_hz =
		fminf(mech_base_req_hz, fmaxf(0.05f, 0.75f * planned_mech_upper_hz));
	float32_t planned_mech_dither_hz =
		fminf(mech_dither_req_hz,
		       fmaxf(0.0f, planned_mech_upper_hz - planned_mech_base_hz));
	float32_t planned_iq_limit_a = clampf(0.60f * g_motor_params->velocity_cl_iq_limit_A,
					      0.10f, MOTOR_MAX_CURRENT_A);

	if (planned_flux_max_hz < 0.10f || planned_mech_upper_hz < 0.10f) {
		shell_error(sh, "Profile max velocity is too low for auto commissioning");
		return -ERANGE;
	}
	if (!execute_motion) {
		shell_print(sh, "Auto commission plan only; no motion started.");
		shell_print(sh, "  Slow profile:  flux %.3f..%.3f Hz, mech base %.3f Hz dither %.3f Hz",
			    (double)fminf(COMMISSION_AUTO_SLOW_FLUX_MIN_HZ,
					  0.50f * COMMISSION_AUTO_SLOW_FLUX_MAX_HZ),
			    (double)COMMISSION_AUTO_SLOW_FLUX_MAX_HZ,
			    (double)COMMISSION_AUTO_SLOW_MECH_BASE_HZ,
			    (double)COMMISSION_AUTO_SLOW_MECH_DITHER_HZ);
		shell_print(sh, "  Confirm profile: flux %.3f..%.3f Hz, mech base %.3f Hz dither %.3f Hz",
			    (double)COMMISSION_AUTO_NORMAL_FLUX_MIN_HZ,
			    (double)COMMISSION_AUTO_NORMAL_FLUX_MAX_HZ,
			    (double)COMMISSION_AUTO_NORMAL_MECH_BASE_HZ,
			    (double)COMMISSION_AUTO_NORMAL_MECH_DITHER_HZ);
		shell_print(sh,
			    "Run 'motor commission auto run slow' for bounded bring-up, or 'motor commission auto run confirm' for the higher-speed profile.");
		return 0;
	}

	struct motor_commission_runtime_ctx commission_ctx;
	motor_commission_ctx_from_global(&commission_ctx);
	if (motor_commission_is_active(&commission_ctx)) {
		shell_error(sh, "Commission capture is already active");
		return -EBUSY;
	}
	if (!g_motor_params->calibration.complete) {
		shell_error(sh, "Calibration is not complete; run calibration before auto commission");
		return -EACCES;
	}
	if (!motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before auto commission");
		return -EACCES;
	}
	if (motor_api_get_state() == MOTOR_STATE_ERROR) {
		shell_error(sh, "Motor is in ERROR state; clear error first");
		return -EFAULT;
	}

	motor_commission_reset(&commission_ctx);

	struct motor_commission_flux_config flux_cfg = {0};
	struct motor_commission_mech_config mech_cfg = {0};
	struct motor_commission_tune_config tune_cfg = {0};
	struct motor_commission_velocity_gain_restore velocity_restore = {0};
	float32_t iq_limit_default = planned_iq_limit_a;
	float32_t threshold_start_a = clampf(0.10f * MOTOR_MAX_CURRENT_A,
					     0.02f,
					     iq_limit_default);
	float32_t threshold_stop_a = iq_limit_default;
	float32_t threshold_step_a =
		fmaxf(0.01f, (threshold_stop_a - threshold_start_a) / 8.0f);

	shell_print(sh,
		    "Auto commission threshold sweep: start=%.3f A stop=%.3f A step=%.3f A",
		    (double)threshold_start_a,
		    (double)threshold_stop_a,
		    (double)threshold_step_a);
	int ret = motor_commission_run_motion_threshold(sh, threshold_start_a,
							threshold_stop_a,
							threshold_step_a,
							250U,
							2.0f * (PI_F32 / 180.0f));
	if (ret != 0 || !g_motor_params->commission.results.iq_move_valid) {
		g_motor_params->commission.auto_tune_last_error = (ret != 0) ? ret : -ENODATA;
		shell_error(sh, "Auto commission failed during motion threshold stage (err %d)",
			    g_motor_params->commission.auto_tune_last_error);
		return g_motor_params->commission.auto_tune_last_error;
	}

	float32_t iq_move_recommended = g_motor_params->commission.results.iq_move_recommended_a;
	iq_limit_default = clampf(fmaxf(iq_limit_default, 1.50f * iq_move_recommended),
				  0.10f,
				  MOTOR_MAX_CURRENT_A);

	flux_cfg.max_speed_hz = planned_flux_max_hz;
	flux_cfg.min_speed_hz = planned_flux_min_hz;
	flux_cfg.steps = slow_profile ? 4U : 5U;
	flux_cfg.settle_ms = slow_profile ? 500U : 400U;
	flux_cfg.sample_ms = slow_profile ? 500U : 400U;
	flux_cfg.iq_limit_a = iq_limit_default;

	ret = motor_commission_stage_velocity_capture_gains(sh,
							    flux_cfg.iq_limit_a,
							    &velocity_restore);
	if (ret != 0) {
		g_motor_params->commission.auto_tune_last_error = ret;
		shell_error(sh, "Failed to stage commissioning velocity PI gains (err %d)", ret);
		return ret;
	}

	float32_t mech_speed_upper_hz = planned_mech_upper_hz;
	mech_cfg.base_speed_hz = planned_mech_base_hz;
	mech_cfg.dither_speed_hz = planned_mech_dither_hz;
	mech_cfg.min_confidence = COMMISSION_AUTO_MECH_MIN_CONFIDENCE;

	float32_t max_accel_hz_s =
		fmaxf(g_motor_params->profile_max_accel_rad_s2 / (2.0f * PI_F32), 1.0f);
	float32_t half_cycle_ms =
		(COMMISSION_AUTO_MECH_ACCEL_MARGIN * 2.0f * mech_speed_upper_hz *
		 1000.0f) /
		max_accel_hz_s;
	mech_cfg.dither_period_ms =
		(uint32_t)ceilf(half_cycle_ms /
				(float32_t)ARRAY_SIZE(motor_commission_mech_step_pattern));
	mech_cfg.dither_period_ms =
		MAX(mech_cfg.dither_period_ms, slow_profile ? 1000U : 500U);
	mech_cfg.duration_ms = COMMISSION_AUTO_MECH_CYCLES_PER_CAPTURE *
			       2U * ARRAY_SIZE(motor_commission_mech_step_pattern) *
			       mech_cfg.dither_period_ms;

	(void)motor_commission_tune_config_default(&tune_cfg,
						   (float32_t)MOTOR_POLE_PAIRS,
						   1.0f / CONTROL_LOOP_FREQUENCY_HZ,
						   fmaxf(MOTOR_MAX_CURRENT_A, 0.1f),
						   fmaxf(g_motor_params->profile_max_velocity_rad_s,
							  1.0f),
						   fmaxf(g_motor_params->profile_max_accel_rad_s2,
							  1.0f));
	tune_cfg.iq_limit_a = flux_cfg.iq_limit_a;
	g_motor_params->commission.auto_tune_cfg = tune_cfg;

	shell_print(sh, "Auto commission start (%s profile):",
		    slow_profile ? "slow" : "confirmed");
	shell_print(sh, "  Motion threshold: pos=%.3f A neg=%.3f A rec=%.3f A",
		    (double)g_motor_params->commission.results.iq_move_min_pos_a,
		    (double)g_motor_params->commission.results.iq_move_min_neg_a,
		    (double)g_motor_params->commission.results.iq_move_recommended_a);
	shell_print(sh, "  Iq->mech sign: %d",
		    g_motor_params->commission.results.iq_to_mech_sign);
	shell_print(sh, "  Flux cfg: min=%.3f Hz max=%.3f Hz steps=%u settle=%u sample=%u iq=%.3f A",
		    (double)flux_cfg.min_speed_hz, (double)flux_cfg.max_speed_hz, flux_cfg.steps,
		    flux_cfg.settle_ms, flux_cfg.sample_ms, (double)flux_cfg.iq_limit_a);
	shell_print(sh,
		    "  Mech cfg: base=%.3f Hz dither=%.3f Hz accel=%.3f Hz/s dither_period=%u ms duration=%u ms pattern=%u cycles=%u runs=%u/%u min_conf=%.3f",
		    (double)mech_cfg.base_speed_hz,
		    (double)mech_cfg.dither_speed_hz,
		    (double)max_accel_hz_s,
		    mech_cfg.dither_period_ms,
		    mech_cfg.duration_ms,
		    (unsigned int)ARRAY_SIZE(motor_commission_mech_step_pattern),
		    (unsigned int)COMMISSION_AUTO_MECH_CYCLES_PER_CAPTURE,
		    (unsigned int)COMMISSION_AUTO_MECH_RUNS,
		    (unsigned int)COMMISSION_AUTO_MECH_MAX_ATTEMPTS,
		    (double)COMMISSION_AUTO_MECH_MIN_CONFIDENCE);

	ret = motor_commission_auto_run_flux(sh, &flux_cfg);
	if (ret != 0) {
		motor_current_slew_params_zero(g_motor_params);
		motor_commission_velocity_gains_restore(&velocity_restore);
		if (g_motor_params->commission.active) {
			struct motor_commission_runtime_ctx commission_ctx;
			motor_commission_ctx_from_global(&commission_ctx);
			motor_commission_abort(&commission_ctx, "auto flux failed");
		}
		g_motor_params->commission.auto_tune_last_error = ret;
		shell_error(sh, "Auto commission failed during flux stage (err %d)", ret);
		return ret;
	}

	shell_print(sh, "  Flux result: psi_f=%.8f Wb R2=%.4f rms=%.4fV N=%u",
		    (double)g_motor_params->commission.results.psi_f_wb,
		    (double)g_motor_params->commission.results.psi_f_r2,
		    (double)g_motor_params->commission.results.psi_f_residual_rms_v,
		    g_motor_params->commission.results.psi_f_sample_count);
	if (g_motor_params->commission.results.mapping_offset_valid ||
	    g_motor_params->commission.results.mapping_pole_pairs_valid) {
		shell_print(sh, "  Flux mapping: offset=%s pole_pairs=%s",
			    g_motor_params->commission.results.mapping_offset_pass ?
				    "PASS" :
				    "FAIL",
			    g_motor_params->commission.results.mapping_pole_pairs_pass ?
				    "PASS" :
				    "FAIL");
	}

	struct motor_commission_mech_aggregate mech_agg = {0};
	for (uint32_t attempt = 0U;
	     attempt < COMMISSION_AUTO_MECH_MAX_ATTEMPTS &&
	     mech_agg.count < COMMISSION_AUTO_MECH_RUNS;
	     attempt++) {
		ret = motor_commission_auto_run_mech(sh, &mech_cfg, true);
		if (ret != 0) {
			if (ret == -ERANGE) {
				shell_print(sh,
					    "  Mech attempt %u/%u rejected by fit quality",
					    (unsigned int)(attempt + 1U),
					    (unsigned int)COMMISSION_AUTO_MECH_MAX_ATTEMPTS);
				motor_commission_print_mech_fit_summary(
					sh, "    Fit",
					&g_motor_params->commission.results);
					shell_print(sh,
						    "    Capture: accepted=%u rejected=%u stored=%u/%u rejects(mode=%u disarmed=%u encoder=%u fault=%u sat=%u invalid=%u track=%u)",
						    g_motor_params->commission.accepted_samples,
						    g_motor_params->commission.rejected_samples,
						    g_motor_params->commission.sample_count,
						    MOTOR_COMMISSION_MAX_SAMPLES,
						    g_motor_params->commission.reject_mode_mismatch,
						    g_motor_params->commission.reject_disarmed,
						    g_motor_params->commission.reject_encoder,
						    g_motor_params->commission.reject_fault,
						    g_motor_params->commission.reject_saturation,
						    g_motor_params->commission.reject_data_invalid,
						    g_motor_params->commission.reject_velocity_tracking);
				continue;
			}

			motor_current_slew_params_zero(g_motor_params);
			motor_commission_velocity_gains_restore(&velocity_restore);
			if (g_motor_params->commission.active) {
				struct motor_commission_runtime_ctx commission_ctx;
				motor_commission_ctx_from_global(&commission_ctx);
				motor_commission_abort(&commission_ctx, "auto mech failed");
			}
			g_motor_params->commission.auto_tune_last_error = ret;
			shell_error(sh, "Auto commission failed during mechanical stage (err %d)",
				    ret);
			return ret;
		}

		const struct motor_commission_results *mech_res =
			&g_motor_params->commission.results;
		motor_commission_mech_aggregate_add(&mech_agg, mech_res);
		char prefix[40];
		snprintk(prefix, sizeof(prefix), "  Mech run %u/%u attempt %u:",
			 (unsigned int)mech_agg.count,
			 (unsigned int)COMMISSION_AUTO_MECH_RUNS,
			 (unsigned int)(attempt + 1U));
		motor_commission_print_mech_fit_summary(sh, prefix, mech_res);
	}

	if (mech_agg.count < COMMISSION_AUTO_MECH_RUNS) {
		g_motor_params->commission.auto_tune_last_error = -ERANGE;
		motor_commission_velocity_gains_restore(&velocity_restore);
		shell_error(sh, "Auto commission only accepted %u/%u mechanical runs",
			    mech_agg.count,
			    COMMISSION_AUTO_MECH_RUNS);
		return -ERANGE;
	}

	ret = motor_commission_mech_aggregate_finalize(&mech_agg,
						       &g_motor_params->commission.results);
	if (ret != 0 || !g_motor_params->commission.results.mech_valid) {
		g_motor_params->commission.auto_tune_last_error = (ret != 0) ? ret : -ERANGE;
		motor_commission_velocity_gains_restore(&velocity_restore);
		shell_error(sh, "Auto commission failed to aggregate mechanical runs (err %d)",
			    g_motor_params->commission.auto_tune_last_error);
		return g_motor_params->commission.auto_tune_last_error;
	}

	struct motor_commission_results aggregate_results = g_motor_params->commission.results;
	ret = motor_commission_auto_run_mech(sh, &mech_cfg, false);
	if (ret != 0) {
		g_motor_params->commission.results = aggregate_results;
		g_motor_params->commission.auto_tune_last_error = ret;
		motor_commission_velocity_gains_restore(&velocity_restore);
		shell_error(sh, "Auto commission failed during mechanical validation (err %d)",
			    ret);
		return ret;
	}

	float32_t validation_rms_nm = 0.0f;
	uint16_t validation_samples = 0U;
	int8_t validation_torque_sign = 0;
	ret = motor_commission_mech_validate_fit(&aggregate_results,
						 &validation_rms_nm,
						 &validation_samples,
						 &validation_torque_sign);
	float32_t validation_limit_nm =
		fmaxf(COMMISSION_AUTO_MECH_VALIDATE_RMS_NM,
		      COMMISSION_AUTO_MECH_VALIDATE_RMS_GAIN *
			      aggregate_results.mech_residual_rms_nm);
	if (ret == -ENODATA) {
		/*
		 * The aggregate model already consists of multiple accepted captures.
		 * Treat a later validation capture with no usable motion samples as an
		 * advisory miss instead of throwing away a consistent aggregate fit.
		 */
		validation_rms_nm = aggregate_results.mech_residual_rms_nm;
		validation_samples = aggregate_results.mech_sample_count;
		validation_torque_sign = aggregate_results.mech_fit_torque_sign;
		aggregate_results.mech_validation_valid = false;
		aggregate_results.mech_validation_residual_rms_nm = validation_rms_nm;
		aggregate_results.mech_validation_pass =
			aggregate_results.mech_confidence >=
			COMMISSION_AUTO_MECH_MIN_CONFIDENCE;
	} else {
		aggregate_results.mech_validation_valid = (ret == 0);
		aggregate_results.mech_validation_residual_rms_nm =
			(ret == 0) ? validation_rms_nm : 0.0f;
		aggregate_results.mech_validation_pass =
			(ret == 0) &&
			(validation_rms_nm <= validation_limit_nm);
	}
	g_motor_params->commission.results = aggregate_results;
	if (!g_motor_params->commission.results.mech_validation_pass) {
		g_motor_params->commission.auto_tune_last_error = (ret != 0) ? ret : -ERANGE;
		motor_commission_velocity_gains_restore(&velocity_restore);
		shell_error(sh,
			    "Auto commission mechanical validation failed (err %d rms=%.6f Nm limit=%.6f Nm conf=%.2f min_conf=%.2f sign=%d N=%u)",
			    g_motor_params->commission.auto_tune_last_error,
			    (double)validation_rms_nm,
			    (double)validation_limit_nm,
			    (double)aggregate_results.mech_confidence,
			    (double)COMMISSION_AUTO_MECH_MIN_CONFIDENCE,
			    validation_torque_sign,
			    validation_samples);
		return g_motor_params->commission.auto_tune_last_error;
	}

	shell_print(sh, "  Mech aggregate: runs=%u J=%.8f+/-%.8f B=%.8f+/-%.8f Tc=%.8f+/-%.8f R2=%.4f conf=%.2f",
		    g_motor_params->commission.results.mech_capture_count,
		    (double)g_motor_params->commission.results.inertia_kgm2,
		    (double)g_motor_params->commission.results.inertia_stddev_kgm2,
		    (double)g_motor_params->commission.results.viscous_friction_nm_per_rad_s,
		    (double)g_motor_params->commission.results.viscous_friction_stddev_nm_per_rad_s,
		    (double)g_motor_params->commission.results.coulomb_friction_nm,
		    (double)g_motor_params->commission.results.coulomb_friction_stddev_nm,
		    (double)g_motor_params->commission.results.mech_r2,
		    (double)g_motor_params->commission.results.mech_confidence);
	if (ret == -ENODATA) {
		shell_warn(sh,
			   "  Mech validation: independent capture had no usable samples; accepted aggregate confidence %.2f >= %.2f",
			   (double)g_motor_params->commission.results.mech_confidence,
			   (double)COMMISSION_AUTO_MECH_MIN_CONFIDENCE);
	} else {
		shell_print(sh, "  Mech validation: rms=%.6f Nm limit=%.6f Nm sign=%d N=%u -> PASS",
			    (double)g_motor_params->commission.results.mech_validation_residual_rms_nm,
			    (double)validation_limit_nm,
			    validation_torque_sign,
			    validation_samples);
	}
	if (g_motor_params->commission.results.mapping_direction_valid) {
		shell_print(sh, "  Mech mapping: direction=%s corr=%.4f",
			    g_motor_params->commission.results.mapping_direction_pass ?
				    "PASS" :
				    "FAIL",
			    (double)g_motor_params->commission.results.mapping_direction_corr);
	}
	if (g_motor_params->commission.results.mapping_valid) {
		shell_print(sh, "  Mapping summary: PASS=%s confidence=%.2f",
			    g_motor_params->commission.results.mapping_pass ?
				    "YES" :
				    "NO",
			    (double)g_motor_params->commission.results.mapping_confidence);
	}

	ret = motor_commission_stage_auto_tune(&commission_ctx, &tune_cfg);
	if (ret != 0) {
		motor_commission_velocity_gains_restore(&velocity_restore);
		shell_error(sh, "Auto tune staging failed (err %d: %s)", ret,
			    motor_commission_tune_error_to_string(ret));
		motor_commission_print_tune_reject_flags(
			sh, g_motor_params->commission.auto_tune_staged.reject_flags);
		return ret;
	}

	shell_print(sh,
		    "  Tuned defaults staged: vel(kp=%.5f ki=%.5f iq=%.3f) pos(kp=%.5f ki=%.5f) dob(gain=%.5f tq=%.5f iqff=%.5f)",
		    (double)g_motor_params->commission.auto_tune_staged.velocity_kp_a_per_rad_s,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_ki_a_per_rad,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_iq_limit_a,
		    (double)g_motor_params->commission.auto_tune_staged.position_kp_rad_s_per_rad,
		    (double)g_motor_params->commission.auto_tune_staged.position_ki_rad_s2_per_rad,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_dob_observer_gain_nm_per_rad_s,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_dob_torque_limit_nm,
		    (double)g_motor_params->commission.auto_tune_staged.velocity_dob_iq_ff_limit_a);

	if (apply_on_success) {
		ret = motor_commission_apply_staged_auto_tune(&commission_ctx);
		if (ret != 0) {
			motor_commission_velocity_gains_restore(&velocity_restore);
			shell_error(sh, "Auto commission completed but apply failed (err %d)", ret);
			return ret;
		}
		shell_print(sh, "Auto commission complete and applied");
	} else {
		shell_print(sh, "Auto commission complete. Run 'motor commission auto apply' to apply.");
	}

	return 0;
}
