/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/sys/util.h>

#include "motor/runtime/commission_runtime.h"
#include "motor/math/math_constants.h"
#include "motor/math/matrix_solve.h"
#include "motor/estimation/commission_estimators.h"
#include "motor/control/torque.h"

#define MOTOR_COMMISSION_DERIV_ALPHA 0.2f
#define MOTOR_COMMISSION_FLAG_SATURATED BIT(0)
#define MOTOR_COMMISSION_MIN_FLUX_SAMPLES 16U
#define MOTOR_COMMISSION_MIN_MECH_SAMPLES 32U
#define MOTOR_COMMISSION_MIN_SPEED_RAD_S (2.0f * PI_F32)
#define MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S 0.5f
#define MOTOR_COMMISSION_MIN_KT_NM_PER_A 1.0e-6f
#define MOTOR_COMMISSION_VISCOUS_NEG_TOL_NM_PER_RAD_S 1.0e-6f
#define MOTOR_COMMISSION_MAPPING_MIN_SPEED_RAD_S (2.0f * PI_F32)
#define MOTOR_COMMISSION_MAPPING_MIN_IQ_A 0.03f
#define MOTOR_COMMISSION_MAPPING_MIN_ACCEL_RAD_S2 0.5f
#define MOTOR_COMMISSION_MAPPING_MIN_FLUX_SAMPLES 24U
#define MOTOR_COMMISSION_MAPPING_MIN_MECH_SAMPLES 48U
#define MOTOR_COMMISSION_MAPPING_DIRECTION_CORR_MIN 0.10f
#define MOTOR_COMMISSION_MAPPING_OFFSET_RATIO_MAX 0.50f
#define MOTOR_COMMISSION_MAPPING_POLE_REL_ERR_MAX 0.15f
#define MOTOR_COMMISSION_DEFAULT_SAMPLE_RATE_HZ 80U
#define MOTOR_COMMISSION_CAPTURE_TARGET_MAX_SAMPLES 480U
#define MOTOR_COMMISSION_TRACKING_MIN_TARGET_RAD_S (2.0f * PI_F32 * 0.05f)
#define MOTOR_COMMISSION_TRACKING_ABS_TOL_RAD_S (2.0f * PI_F32 * 0.05f)
#define MOTOR_COMMISSION_TRACKING_REL_TOL 0.75f
#define MOTOR_COMMISSION_MECH_FRICTION_MAX_ACCEL_RAD_S2 (2.0f * PI_F32 * 1.0f)
#define MOTOR_COMMISSION_MECH_INERTIA_MIN_ACCEL_RAD_S2 (2.0f * PI_F32 * 0.5f)
#define MOTOR_COMMISSION_MECH_ACCEL_HALF_WINDOW 2U
#define MOTOR_COMMISSION_MECH_MIN_CONFIDENCE 0.50f
#define MOTOR_COMMISSION_MECH_PLAUSIBILITY_MIN 0.10f
#define MOTOR_COMMISSION_MECH_PLAUSIBILITY_MAX 10.0f
#define MOTOR_COMMISSION_MECH_WARN_PLAUSIBILITY_MIN 0.25f
#define MOTOR_COMMISSION_MECH_WARN_PLAUSIBILITY_MAX 4.0f
#define MOTOR_COMMISSION_MECH_MAX_RESIDUAL_NM 0.20f

static inline uint32_t motor_commission_default_decimation_hz(float32_t control_loop_frequency_hz)
{
	/* Keep default auto-commission windows (~5-6 s) within capture capacity. */
	uint32_t decim = ((uint32_t)control_loop_frequency_hz +
			  (MOTOR_COMMISSION_DEFAULT_SAMPLE_RATE_HZ - 1U)) /
			 MOTOR_COMMISSION_DEFAULT_SAMPLE_RATE_HZ;

	return MAX(decim, 1U);
}

static inline uint32_t motor_commission_decimation_for_capture(float32_t control_loop_frequency_hz,
							      uint32_t duration_ms)
{
	uint32_t total_loops =
		(uint32_t)fmaxf(1.0f,
				(control_loop_frequency_hz * (float32_t)duration_ms) / 1000.0f);
	uint32_t capacity_decim =
		(total_loops + (MOTOR_COMMISSION_CAPTURE_TARGET_MAX_SAMPLES - 1U)) /
		MOTOR_COMMISSION_CAPTURE_TARGET_MAX_SAMPLES;

	return MAX(motor_commission_default_decimation_hz(control_loop_frequency_hz),
		   MAX(capacity_decim, 1U));
}

static inline bool motor_commission_obs_matches_expected(
	const struct motor_commission_observation *obs,
	uint8_t expected_mode)
{
	if (obs == NULL) {
		return false;
	}

	switch (expected_mode) {
	case MOTOR_COMMISSION_EXPECT_ANY:
		return true;
	case MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED:
		return obs->mode_velocity_encoder;
	case MOTOR_COMMISSION_EXPECT_TORQUE:
		return obs->mode_current_encoder;
	default:
		return false;
	}
}

static bool motor_commission_obs_velocity_tracking_ok(
	const struct motor_commission_ctx *commission,
	const struct motor_commission_observation *obs)
{
	if (commission == NULL || obs == NULL) {
		return false;
	}
	if (commission->mode != MOTOR_COMMISSION_MODE_FLUX &&
	    commission->mode != MOTOR_COMMISSION_MODE_MECH) {
		return true;
	}
	if (commission->expected_mode != MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED) {
		return true;
	}
	if (!isfinite(obs->velocity_ref_rad_s) ||
	    !isfinite(obs->mech_speed_rad_s)) {
		return false;
	}

	float32_t target_abs = fabsf(obs->velocity_ref_rad_s);
	if (target_abs < MOTOR_COMMISSION_TRACKING_MIN_TARGET_RAD_S) {
		return true;
	}

	float32_t err = fabsf(obs->mech_speed_rad_s - obs->velocity_ref_rad_s);
	float32_t limit = fmaxf(MOTOR_COMMISSION_TRACKING_ABS_TOL_RAD_S,
				MOTOR_COMMISSION_TRACKING_REL_TOL * target_abs);

	return err <= limit;
}

const char *motor_commission_mode_to_string(uint8_t mode)
{
	switch (mode) {
	case MOTOR_COMMISSION_MODE_FLUX:
		return "FLUX";
	case MOTOR_COMMISSION_MODE_MECH:
		return "MECH";
	case MOTOR_COMMISSION_MODE_NONE:
	default:
		return "NONE";
	}
}

const char *motor_commission_stage_to_string(uint8_t stage)
{
	switch (stage) {
	case MOTOR_COMMISSION_STAGE_RUNNING:
		return "RUNNING";
	case MOTOR_COMMISSION_STAGE_COMPLETED:
		return "COMPLETED";
	case MOTOR_COMMISSION_STAGE_ABORTED:
		return "ABORTED";
	case MOTOR_COMMISSION_STAGE_IDLE:
	default:
		return "IDLE";
	}
}

bool motor_commission_is_active(const struct motor_commission_runtime_ctx *ctx)
{
	return ctx != NULL && ctx->commission != NULL && ctx->commission->active;
}

static void motor_commission_reset_capture_state(struct motor_commission_ctx *ctx)
{
	ctx->sample_count = 0U;
	ctx->accepted_samples = 0U;
	ctx->rejected_samples = 0U;
	ctx->reject_mode_mismatch = 0U;
	ctx->reject_disarmed = 0U;
	ctx->reject_encoder = 0U;
	ctx->reject_fault = 0U;
	ctx->reject_saturation = 0U;
	ctx->reject_data_invalid = 0U;
	ctx->reject_velocity_tracking = 0U;
	ctx->sample_decimation_counter = 0U;
	ctx->prev_valid = false;
	ctx->prev_loop_count = 0U;
	ctx->prev_id_a = 0.0f;
	ctx->prev_iq_a = 0.0f;
	ctx->prev_speed_rad_s = 0.0f;
	ctx->did_dt_filt_a_s = 0.0f;
	ctx->diq_dt_filt_a_s = 0.0f;
	ctx->domega_dt_filt_rad_s2 = 0.0f;
	memset(ctx->samples, 0, sizeof(ctx->samples));
}

static void motor_commission_update_mapping_summary(const struct motor_commission_runtime_ctx *ctx,
						    struct motor_commission_results *res)
{
	if (res == NULL) {
		return;
	}

	res->mapping_valid = res->mapping_direction_valid &&
			     res->mapping_offset_valid &&
			     res->mapping_pole_pairs_valid;
	res->mapping_pass = res->mapping_valid &&
			    res->mapping_direction_pass &&
			    res->mapping_offset_pass &&
			    res->mapping_pole_pairs_pass;

	float32_t conf_sum = 0.0f;
	uint32_t conf_count = 0U;

	if (res->mapping_direction_valid) {
		float32_t conf_dir = clampf(res->mapping_direction_corr /
						    MOTOR_COMMISSION_MAPPING_DIRECTION_CORR_MIN,
					    0.0f, 1.0f);
		conf_sum += conf_dir;
		conf_count++;
	}
	if (res->mapping_offset_valid) {
		float32_t conf_off = clampf(1.0f -
						    (res->mapping_offset_ratio /
						     MOTOR_COMMISSION_MAPPING_OFFSET_RATIO_MAX),
					    0.0f, 1.0f);
		conf_sum += conf_off;
		conf_count++;
	}
	if (res->mapping_pole_pairs_valid) {
		float32_t pole_pairs = (float32_t)MAX(ctx->pole_pairs, 1U);
		float32_t rel_err = fabsf(res->mapping_pole_pairs_est -
					  (float32_t)ctx->pole_pairs) /
				    pole_pairs;
		float32_t conf_pp = clampf(1.0f -
						   (rel_err / MOTOR_COMMISSION_MAPPING_POLE_REL_ERR_MAX),
					   0.0f, 1.0f);
		conf_sum += conf_pp;
		conf_count++;
	}

	res->mapping_confidence = (conf_count > 0U) ? (conf_sum / (float32_t)conf_count) : 0.0f;
}

static void motor_commission_estimate_flux(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;
	struct motor_commission_results *res = &commission->results;
	struct motor_flux_id_state estimator;
	struct motor_flux_id_result estimate;
	const struct motor_flux_id_config cfg = {
		.rs_ohm = *ctx->rs_measured_ohm,
		.ld_h = *ctx->rls_ld_est_h,
		.lq_h = *ctx->rls_lq_est_h,
		.min_abs_speed_rad_s = MOTOR_COMMISSION_MIN_SPEED_RAD_S,
		.min_speed_span_rad_s = 2.0f * MOTOR_COMMISSION_MIN_SPEED_RAD_S,
		.min_samples = MOTOR_COMMISSION_MIN_FLUX_SAMPLES,
		.min_r2 = 0.2f,
		.require_positive_psi = true,
	};

	res->psi_f_valid = false;
	res->psi_f_sample_count = 0U;
	res->psi_f_wb = 0.0f;
	res->psi_f_bias_v = 0.0f;
	res->psi_f_residual_rms_v = 0.0f;
	res->psi_f_r2 = 0.0f;

	motor_flux_id_init(&estimator, &cfg);
	for (uint32_t i = 0U; i < commission->sample_count; i++) {
		const struct motor_commission_sample *s = &commission->samples[i];
		(void)motor_flux_id_accumulate(&estimator, s->elec_speed_rad_s, s->id_a, s->iq_a,
					       s->diq_dt_a_s, s->vq_v);
	}

	if (motor_flux_id_finalize(&estimator, &estimate) < 0) {
		return;
	}

	res->psi_f_sample_count = estimate.sample_count;
	res->psi_f_wb = estimate.psi_f_wb;
	res->psi_f_bias_v = estimate.bias_v;
	res->psi_f_residual_rms_v = estimate.residual_rms_v;
	res->psi_f_r2 = estimate.r2;
	res->psi_f_valid = estimate.valid;
}

static void motor_commission_validate_mapping_flux(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;
	struct motor_commission_results *res = &commission->results;
	float32_t id_sq = 0.0f;
	float32_t iq_sq = 0.0f;
	float32_t pole_weighted = 0.0f;
	float32_t pole_weight_sum = 0.0f;
	uint32_t axis_count = 0U;
	uint32_t pole_count = 0U;

	res->mapping_offset_valid = false;
	res->mapping_offset_pass = false;
	res->mapping_offset_ratio = 0.0f;
	res->mapping_pole_pairs_valid = false;
	res->mapping_pole_pairs_pass = false;
	res->mapping_pole_pairs_est = 0.0f;

	for (uint32_t i = 0U; i < commission->sample_count; i++) {
		const struct motor_commission_sample *s = &commission->samples[i];
		if (!isfinite(s->mech_speed_rad_s) || !isfinite(s->elec_speed_rad_s) ||
		    !isfinite(s->id_a) || !isfinite(s->iq_a)) {
			continue;
		}
		if (fabsf(s->mech_speed_rad_s) < MOTOR_COMMISSION_MAPPING_MIN_SPEED_RAD_S) {
			continue;
		}

		id_sq += s->id_a * s->id_a;
		iq_sq += s->iq_a * s->iq_a;
		axis_count++;

		float32_t pole_est = s->elec_speed_rad_s / s->mech_speed_rad_s;
		float32_t weight = fabsf(s->mech_speed_rad_s);
		if (isfinite(pole_est) && isfinite(weight) && weight > 0.0f) {
			pole_weighted += weight * pole_est;
			pole_weight_sum += weight;
			pole_count++;
		}
	}

	if (axis_count >= MOTOR_COMMISSION_MAPPING_MIN_FLUX_SAMPLES) {
		float32_t id_rms = sqrtf(id_sq / (float32_t)axis_count);
		float32_t iq_rms = sqrtf(iq_sq / (float32_t)axis_count);
		float32_t ratio = id_rms /
				   fmaxf(iq_rms, MOTOR_COMMISSION_MAPPING_MIN_IQ_A);

		res->mapping_offset_valid = isfinite(ratio);
		if (res->mapping_offset_valid) {
			res->mapping_offset_ratio = ratio;
			res->mapping_offset_pass = ratio <= MOTOR_COMMISSION_MAPPING_OFFSET_RATIO_MAX;
		}
	}

	if (pole_count >= MOTOR_COMMISSION_MAPPING_MIN_FLUX_SAMPLES &&
	    pole_weight_sum > 0.0f) {
		float32_t pole_est = pole_weighted / pole_weight_sum;
		float32_t pole_pairs = (float32_t)MAX(ctx->pole_pairs, 1U);
		float32_t rel_err = fabsf(pole_est - (float32_t)ctx->pole_pairs) /
				    pole_pairs;
		res->mapping_pole_pairs_valid = isfinite(pole_est) && isfinite(rel_err);
		if (res->mapping_pole_pairs_valid) {
			res->mapping_pole_pairs_est = pole_est;
			res->mapping_pole_pairs_pass =
				rel_err <= MOTOR_COMMISSION_MAPPING_POLE_REL_ERR_MAX;
		}
	}

	motor_commission_update_mapping_summary(ctx, res);
}

static int motor_commission_estimate_mech_zero_viscous(
	const struct motor_commission_ctx *commission,
	float32_t kt_nm_per_a,
	struct motor_mech_id_result *estimate)
{
	if (commission == NULL || estimate == NULL ||
	    !isfinite(kt_nm_per_a) ||
	    fabsf(kt_nm_per_a) < MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		return -EINVAL;
	}

	float32_t A[3][3] = {0};
	float32_t b[3] = {0};
	float32_t theta[3] = {0};
	float32_t sum_z = 0.0f;
	float32_t sum_z2 = 0.0f;
	uint32_t count = 0U;

	memset(estimate, 0, sizeof(*estimate));

	for (uint32_t i = 0U; i < commission->sample_count; i++) {
		const struct motor_commission_sample *s = &commission->samples[i];
		if (!isfinite(s->mech_speed_rad_s) ||
		    !isfinite(s->mech_accel_rad_s2) ||
		    !isfinite(s->iq_a) ||
		    fabsf(s->mech_speed_rad_s) < MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S) {
			continue;
		}

		const float32_t sign_term = (s->mech_speed_rad_s > 0.0f) ? 1.0f : -1.0f;
		const float32_t phi[3] = {
			s->mech_accel_rad_s2,
			sign_term,
			1.0f,
		};
		const float32_t z = kt_nm_per_a * s->iq_a;
		if (!isfinite(z)) {
			continue;
		}

		for (uint32_t r = 0U; r < 3U; r++) {
			b[r] += phi[r] * z;
			for (uint32_t c = 0U; c < 3U; c++) {
				A[r][c] += phi[r] * phi[c];
			}
		}
		sum_z += z;
		sum_z2 += z * z;
		count++;
	}

	estimate->sample_count = (uint16_t)MIN(count, UINT16_MAX);
	if (count < MOTOR_COMMISSION_MIN_MECH_SAMPLES) {
		return -ENODATA;
	}
	if (!motor_math_solve_linear_3x3(A, b, theta)) {
		return -ERANGE;
	}

	const float32_t n = (float32_t)count;
	const float32_t theta_dot_b = theta[0] * b[0] + theta[1] * b[1] + theta[2] * b[2];
	float32_t sse = sum_z2 - theta_dot_b;
	if (sse < 0.0f) {
		sse = 0.0f;
	}
	const float32_t mean_z = sum_z / n;
	float32_t sst = sum_z2 - n * mean_z * mean_z;
	if (sst < 0.0f) {
		sst = 0.0f;
	}

	const float32_t residual_dof = (count > 3U) ? (n - 3.0f) : n;
	const float32_t residual_rms_nm = sqrtf(sse / residual_dof);
	const float32_t r2 = (sst > 1.0e-8f) ? (1.0f - sse / sst) : 0.0f;

	estimate->inertia_kgm2 = theta[0];
	estimate->viscous_friction_nm_per_rad_s = 0.0f;
	estimate->coulomb_friction_nm = fabsf(theta[1]);
	estimate->offset_friction_nm = theta[2];
	estimate->residual_rms_nm = residual_rms_nm;
	estimate->r2 = r2;
	estimate->valid = isfinite(theta[0]) &&
			  isfinite(theta[1]) &&
			  isfinite(theta[2]) &&
			  isfinite(residual_rms_nm) &&
			  isfinite(r2) &&
			  theta[0] > 0.0f &&
			  r2 >= 0.0f;

	return 0;
}

static void motor_commission_estimate_mech_legacy(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;
	struct motor_commission_results *res = &commission->results;
	float32_t kt = motor_torque_gain_resolve(*ctx->torque_gain_nm_per_a_active,
						 *ctx->flux_linkage_wb_active,
						 ctx->default_flux_linkage_wb,
						 ctx->pole_pairs);
	if (res->psi_f_valid) {
		float32_t derived_kt =
			motor_torque_gain_from_flux_pole_pairs(res->psi_f_wb, ctx->pole_pairs);
		if (isfinite(derived_kt) && derived_kt > 0.0f) {
			kt = derived_kt;
		}
	}

	res->mech_valid = false;
	res->mech_sample_count = 0U;
	res->inertia_kgm2 = 0.0f;
	res->viscous_friction_nm_per_rad_s = 0.0f;
	res->coulomb_friction_nm = 0.0f;
	res->offset_friction_nm = 0.0f;
	res->mech_residual_rms_nm = 0.0f;
	res->mech_r2 = 0.0f;
	res->inertia_stddev_kgm2 = 0.0f;
	res->viscous_friction_stddev_nm_per_rad_s = 0.0f;
	res->coulomb_friction_stddev_nm = 0.0f;
	res->mech_validation_residual_rms_nm = 0.0f;
	res->mech_confidence = 0.0f;
	res->mech_finalize_error = 0;
	res->mech_capture_count = 0U;
	res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;
	res->mech_fit_torque_sign = 0;
	res->mech_validation_valid = false;
	res->mech_validation_pass = false;

	if (!isfinite(kt) || fabsf(kt) < MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_KT_INVALID;
		return;
	}

	const int8_t preferred_sign =
		(commission->expected_mode == MOTOR_COMMISSION_EXPECT_TORQUE &&
		 res->iq_to_mech_sign < 0) ? -1 : 1;
	const int8_t signs[2] = {preferred_sign, (int8_t)-preferred_sign};
	struct motor_mech_id_result best_estimate = {0};
	int32_t best_finalize_error = 0;
	int8_t best_sign = 0;
	uint8_t best_reject_reason = MOTOR_COMMISSION_MECH_REJECT_FINALIZE;
	bool have_best = false;

	for (uint32_t sign_idx = 0U; sign_idx < ARRAY_SIZE(signs); sign_idx++) {
		const int8_t fit_sign = signs[sign_idx];
		const struct motor_mech_id_config cfg = {
			.kt_nm_per_a = kt * (float32_t)fit_sign,
			.sign_deadband_rad_s = MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S,
			.min_samples = MOTOR_COMMISSION_MIN_MECH_SAMPLES,
			.min_r2 = 0.0f,
			.require_positive_inertia = true,
			.require_nonnegative_viscous = false,
		};
		struct motor_mech_id_state estimator;
		struct motor_mech_id_result estimate;
		uint8_t reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;
		int ret;

		motor_mech_id_init(&estimator, &cfg);
		for (uint32_t i = 0U; i < commission->sample_count; i++) {
			const struct motor_commission_sample *s = &commission->samples[i];
			if (fabsf(s->mech_speed_rad_s) < MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S) {
				continue;
			}
			(void)motor_mech_id_accumulate(&estimator, s->mech_speed_rad_s,
						       s->mech_accel_rad_s2, s->iq_a);
		}

		ret = motor_mech_id_finalize(&estimator, &estimate);
		if (ret < 0) {
			if (!have_best || estimate.sample_count > best_estimate.sample_count) {
				best_estimate = estimate;
				best_finalize_error = ret;
				best_sign = fit_sign;
				have_best = true;
			}
			continue;
		}

		if (estimate.viscous_friction_nm_per_rad_s <
		    -MOTOR_COMMISSION_VISCOUS_NEG_TOL_NM_PER_RAD_S) {
			reject_reason = MOTOR_COMMISSION_MECH_REJECT_VISCOUS_NEGATIVE;
		} else if (!estimate.valid) {
			reject_reason = MOTOR_COMMISSION_MECH_REJECT_FIT_INVALID;
		}

		if (reject_reason == MOTOR_COMMISSION_MECH_REJECT_VISCOUS_NEGATIVE) {
			struct motor_mech_id_result zero_b_estimate = {0};
			ret = motor_commission_estimate_mech_zero_viscous(commission,
									  cfg.kt_nm_per_a,
									  &zero_b_estimate);
			if (ret == 0 && zero_b_estimate.valid) {
				estimate = zero_b_estimate;
				reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;
			}
		}

		if (!have_best ||
		    (reject_reason == MOTOR_COMMISSION_MECH_REJECT_NONE &&
		     best_reject_reason != MOTOR_COMMISSION_MECH_REJECT_NONE) ||
		    (reject_reason == best_reject_reason &&
		     estimate.residual_rms_nm < best_estimate.residual_rms_nm)) {
			best_estimate = estimate;
			best_finalize_error = 0;
			best_sign = fit_sign;
			best_reject_reason = reject_reason;
			have_best = true;
		}
	}

	if (!have_best) {
		return;
	}

	res->mech_finalize_error = best_finalize_error;
	res->mech_fit_torque_sign = best_sign;
	res->mech_sample_count = best_estimate.sample_count;

	if (best_finalize_error < 0) {
		if (best_finalize_error == -ENODATA) {
			res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_SAMPLES;
		} else if (best_finalize_error == -ERANGE) {
			res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_SOLVER;
		} else {
			res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_FINALIZE;
		}
		return;
	}

	if (best_reject_reason != MOTOR_COMMISSION_MECH_REJECT_NONE) {
		res->mech_reject_reason = best_reject_reason;
		return;
	}

	if (best_estimate.viscous_friction_nm_per_rad_s <
	    -MOTOR_COMMISSION_VISCOUS_NEG_TOL_NM_PER_RAD_S) {
		res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_VISCOUS_NEGATIVE;
		return;
	}

	res->inertia_kgm2 = best_estimate.inertia_kgm2;
	res->viscous_friction_nm_per_rad_s =
		(best_estimate.viscous_friction_nm_per_rad_s > 0.0f) ?
			best_estimate.viscous_friction_nm_per_rad_s : 0.0f;
	res->coulomb_friction_nm = best_estimate.coulomb_friction_nm;
	res->offset_friction_nm = best_estimate.offset_friction_nm;
	res->mech_residual_rms_nm = best_estimate.residual_rms_nm;
	res->mech_r2 = best_estimate.r2;
	res->mech_valid = best_estimate.valid;
}

static bool motor_commission_sample_window_accel(const struct motor_commission_ctx *commission,
						 uint16_t index,
						 float32_t control_loop_frequency_hz,
						 float32_t *accel_rad_s2)
{
	if (commission == NULL || accel_rad_s2 == NULL) {
		return false;
	}

	const uint16_t half_window = MOTOR_COMMISSION_MECH_ACCEL_HALF_WINDOW;
	if (index < half_window ||
	    (uint32_t)index + (uint32_t)half_window >= commission->sample_count ||
	    !isfinite(control_loop_frequency_hz) ||
	    control_loop_frequency_hz <= 0.0f) {
		return false;
	}

	const uint16_t first = index - half_window;
	const uint16_t last = index + half_window;
	const uint32_t t0_count = commission->samples[index].loop_count;
	const float32_t inv_fs = 1.0f / control_loop_frequency_hz;
	float32_t sum_t = 0.0f;
	float32_t sum_w = 0.0f;
	float32_t sum_tt = 0.0f;
	float32_t sum_tw = 0.0f;
	uint16_t n = 0U;

	for (uint16_t i = first; i <= last; i++) {
		const struct motor_commission_sample *s = &commission->samples[i];
		if (!isfinite(s->mech_speed_rad_s)) {
			return false;
		}
		const int32_t dt_count = (int32_t)(s->loop_count - t0_count);
		const float32_t t = (float32_t)dt_count * inv_fs;
		sum_t += t;
		sum_w += s->mech_speed_rad_s;
		sum_tt += t * t;
		sum_tw += t * s->mech_speed_rad_s;
		n++;
	}

	const float32_t nf = (float32_t)n;
	const float32_t denom = nf * sum_tt - sum_t * sum_t;
	if (fabsf(denom) < 1.0e-8f) {
		return false;
	}

	const float32_t slope = (nf * sum_tw - sum_t * sum_w) / denom;
	if (!isfinite(slope)) {
		return false;
	}

	*accel_rad_s2 = slope;
	return true;
}

static uint16_t motor_commission_refresh_window_accel(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;
	uint16_t valid_count = 0U;

	for (uint16_t i = 0U; i < commission->sample_count; i++) {
		float32_t accel = 0.0f;
		if (motor_commission_sample_window_accel(commission, i,
							 ctx->control_loop_frequency_hz,
							 &accel)) {
			commission->samples[i].mech_accel_window_rad_s2 = accel;
			valid_count++;
		} else {
			commission->samples[i].mech_accel_window_rad_s2 = NAN;
		}
	}

	return valid_count;
}

static float32_t motor_commission_sample_detent_torque_nm(
	const struct motor_commission_runtime_ctx *ctx,
	const struct motor_commission_sample *sample,
	float32_t kt_nm_per_a,
	bool *corrected)
{
	if (corrected != NULL) {
		*corrected = false;
	}
	if (ctx == NULL || sample == NULL || ctx->detent_map_cfg == NULL ||
	    !ctx->detent_map_cfg->enabled ||
	    !isfinite(kt_nm_per_a) ||
	    fabsf(kt_nm_per_a) < MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		return 0.0f;
	}

	float32_t iq_ff_a = 0.0f;
	if (motor_detent_map_lookup(ctx->detent_map_cfg, sample->mech_position_rad, &iq_ff_a) != 0 ||
	    !isfinite(iq_ff_a)) {
		return 0.0f;
	}

	if (corrected != NULL) {
		*corrected = true;
	}
	return kt_nm_per_a * iq_ff_a;
}

static float32_t motor_commission_resolve_torque_gain(const struct motor_commission_runtime_ctx *ctx)
{
	const struct motor_commission_results *res = &ctx->commission->results;
	float32_t kt = motor_torque_gain_resolve(*ctx->torque_gain_nm_per_a_active,
						 *ctx->flux_linkage_wb_active,
						 ctx->default_flux_linkage_wb,
						 ctx->pole_pairs);
	if (res->psi_f_valid) {
		float32_t derived_kt =
			motor_torque_gain_from_flux_pole_pairs(res->psi_f_wb, ctx->pole_pairs);
		if (isfinite(derived_kt) && derived_kt > 0.0f) {
			kt = derived_kt;
		}
	}

	return kt;
}

static void motor_commission_estimate_mech(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;
	struct motor_commission_results *res = &commission->results;

	/*
	 * Keep the legacy coupled fit visible as a diagnostic seed, then
	 * overwrite validity with the staged physically constrained result.
	 */
	motor_commission_estimate_mech_legacy(ctx);

	const float32_t kt_base = motor_commission_resolve_torque_gain(ctx);
	const int8_t preferred_sign =
		(commission->expected_mode == MOTOR_COMMISSION_EXPECT_TORQUE &&
		 res->iq_to_mech_sign < 0) ? -1 : 1;
	const int8_t signs[2] = {preferred_sign, (int8_t)-preferred_sign};
	const uint16_t accel_window_count = motor_commission_refresh_window_accel(ctx);

	res->mech_valid = false;
	res->mech_v2_valid = false;
	res->mech_friction_valid = false;
	res->mech_inertia_valid = false;
	res->mech_detent_corrected = false;
	res->mech_accel_window_valid_count = accel_window_count;
	res->mech_friction_sample_count = 0U;
	res->mech_inertia_sample_count = 0U;
	res->mech_friction_residual_rms_nm = 0.0f;
	res->mech_inertia_residual_rms_nm = 0.0f;
	res->mech_inertia_plausibility_ratio = 0.0f;
	res->mech_detent_correction_source = MOTOR_COMMISSION_DETENT_CORRECTION_NONE;

	if (!isfinite(kt_base) || fabsf(kt_base) < MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_KT_INVALID;
		return;
	}

	struct motor_mech_friction_id_result best_friction = {0};
	struct motor_mech_inertia_id_result best_inertia = {0};
	int8_t best_sign = 0;
	float32_t best_confidence = -1.0f;
	uint8_t best_reject = MOTOR_COMMISSION_MECH_REJECT_FIT_INVALID;
	bool best_detent_corrected = false;

	for (uint32_t sign_idx = 0U; sign_idx < ARRAY_SIZE(signs); sign_idx++) {
		const int8_t fit_sign = signs[sign_idx];
		const float32_t kt = kt_base * (float32_t)fit_sign;
		const struct motor_mech_friction_id_config friction_cfg = {
			.kt_nm_per_a = kt,
			.sign_deadband_rad_s = MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S,
			.max_abs_accel_rad_s2 = MOTOR_COMMISSION_MECH_FRICTION_MAX_ACCEL_RAD_S2,
			.min_samples = MOTOR_COMMISSION_MIN_MECH_SAMPLES,
			.min_samples_per_direction = 8U,
			.min_r2 = 0.0f,
			.require_nonnegative_viscous = true,
			.require_nonnegative_coulomb = true,
		};
		struct motor_mech_friction_id_state friction_state;
		struct motor_mech_friction_id_result friction;
		bool detent_corrected = false;

		motor_mech_friction_id_init(&friction_state, &friction_cfg);
		for (uint32_t i = 0U; i < commission->sample_count; i++) {
			const struct motor_commission_sample *s = &commission->samples[i];
			const float32_t accel = isfinite(s->mech_accel_window_rad_s2) ?
				s->mech_accel_window_rad_s2 : s->mech_accel_rad_s2;
			bool corrected = false;
			float32_t detent_nm =
				motor_commission_sample_detent_torque_nm(ctx, s, kt, &corrected);
			detent_corrected = detent_corrected || corrected;
			(void)motor_mech_friction_id_accumulate(&friction_state,
								s->mech_speed_rad_s,
								accel,
								s->iq_a,
								detent_nm);
		}

		int ret = motor_mech_friction_id_finalize(&friction_state, &friction);
		if (ret < 0 || !friction.valid) {
			best_reject = MOTOR_COMMISSION_MECH_REJECT_FRICTION_INVALID;
			continue;
		}

		const struct motor_mech_inertia_id_config inertia_cfg = {
			.kt_nm_per_a = kt,
			.sign_deadband_rad_s = MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S,
			.min_abs_accel_rad_s2 = MOTOR_COMMISSION_MECH_INERTIA_MIN_ACCEL_RAD_S2,
			.viscous_friction_nm_per_rad_s =
				friction.viscous_friction_nm_per_rad_s,
			.coulomb_friction_nm = friction.coulomb_friction_nm,
			.offset_friction_nm = friction.offset_friction_nm,
			.fallback_inertia_kgm2 = *ctx->inertia_kgm2_active,
			.min_plausibility_ratio = MOTOR_COMMISSION_MECH_PLAUSIBILITY_MIN,
			.max_plausibility_ratio = MOTOR_COMMISSION_MECH_PLAUSIBILITY_MAX,
			.min_samples = 16U,
			.min_samples_per_accel_direction = 4U,
			.max_residual_rms_nm = MOTOR_COMMISSION_MECH_MAX_RESIDUAL_NM,
			.require_plausible = true,
		};
		struct motor_mech_inertia_id_state inertia_state;
		struct motor_mech_inertia_id_result inertia;

		motor_mech_inertia_id_init(&inertia_state, &inertia_cfg);
		for (uint32_t i = 0U; i < commission->sample_count; i++) {
			const struct motor_commission_sample *s = &commission->samples[i];
			if (!isfinite(s->mech_accel_window_rad_s2)) {
				continue;
			}
			bool corrected = false;
			float32_t detent_nm =
				motor_commission_sample_detent_torque_nm(ctx, s, kt, &corrected);
			detent_corrected = detent_corrected || corrected;
			(void)motor_mech_inertia_id_accumulate(&inertia_state,
							       s->mech_speed_rad_s,
							       s->mech_accel_window_rad_s2,
							       s->iq_a,
							       detent_nm);
		}

		ret = motor_mech_inertia_id_finalize(&inertia_state, &inertia);
		if (ret < 0 || !inertia.valid) {
			best_reject = MOTOR_COMMISSION_MECH_REJECT_INERTIA_INVALID;
			continue;
		}

		float32_t confidence = 0.5f * clampf(friction.r2, 0.0f, 1.0f) +
				       0.5f * clampf(inertia.r2, 0.0f, 1.0f);
		if (inertia.plausibility_ratio < MOTOR_COMMISSION_MECH_WARN_PLAUSIBILITY_MIN ||
		    inertia.plausibility_ratio > MOTOR_COMMISSION_MECH_WARN_PLAUSIBILITY_MAX) {
			confidence *= 0.75f;
		}
		confidence = clampf(confidence, 0.0f, 1.0f);

		if (confidence > best_confidence) {
			best_confidence = confidence;
			best_friction = friction;
			best_inertia = inertia;
			best_sign = fit_sign;
			best_detent_corrected = detent_corrected;
			best_reject = MOTOR_COMMISSION_MECH_REJECT_NONE;
		}
	}

	if (best_reject != MOTOR_COMMISSION_MECH_REJECT_NONE || best_confidence < 0.0f) {
		res->mech_reject_reason = best_reject;
		return;
	}

	res->mech_fit_torque_sign = best_sign;
	res->inertia_kgm2 = best_inertia.inertia_kgm2;
	res->viscous_friction_nm_per_rad_s = best_friction.viscous_friction_nm_per_rad_s;
	res->coulomb_friction_nm = best_friction.coulomb_friction_nm;
	res->offset_friction_nm = best_friction.offset_friction_nm;
	res->mech_friction_residual_rms_nm = best_friction.residual_rms_nm;
	res->mech_inertia_residual_rms_nm = best_inertia.residual_rms_nm;
	res->mech_residual_rms_nm =
		0.5f * (best_friction.residual_rms_nm + best_inertia.residual_rms_nm);
	res->mech_r2 = 0.5f * (best_friction.r2 + best_inertia.r2);
	res->mech_inertia_plausibility_ratio = best_inertia.plausibility_ratio;
	res->mech_sample_count = (uint16_t)MIN(best_friction.sample_count +
					       best_inertia.sample_count,
					       UINT16_MAX);
	res->mech_friction_sample_count = best_friction.sample_count;
	res->mech_inertia_sample_count = best_inertia.sample_count;
	res->mech_confidence = best_confidence;
	res->mech_friction_valid = best_friction.valid;
	res->mech_inertia_valid = best_inertia.valid;
	res->mech_detent_corrected = best_detent_corrected;
	res->mech_detent_correction_source = best_detent_corrected ?
		MOTOR_COMMISSION_DETENT_CORRECTION_ACTIVE_MAP :
		MOTOR_COMMISSION_DETENT_CORRECTION_NONE;
	res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;

	if (best_inertia.plausibility_ratio < MOTOR_COMMISSION_MECH_PLAUSIBILITY_MIN ||
	    best_inertia.plausibility_ratio > MOTOR_COMMISSION_MECH_PLAUSIBILITY_MAX) {
		res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_IMPLAUSIBLE;
		return;
	}
	if (best_confidence < MOTOR_COMMISSION_MECH_MIN_CONFIDENCE) {
		res->mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_CONFIDENCE;
		return;
	}

	res->mech_v2_valid = true;
	res->mech_valid = true;
}

static void motor_commission_validate_mapping_mech(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;
	struct motor_commission_results *res = &commission->results;
	float32_t sum_iq2 = 0.0f;
	float32_t sum_acc2 = 0.0f;
	float32_t sum_cross = 0.0f;
	uint32_t count = 0U;

	res->mapping_direction_valid = false;
	res->mapping_direction_pass = false;
	res->mapping_direction_corr = 0.0f;

	for (uint32_t i = 0U; i < commission->sample_count; i++) {
		const struct motor_commission_sample *s = &commission->samples[i];
		if (!isfinite(s->iq_a) || !isfinite(s->mech_accel_rad_s2)) {
			continue;
		}
		if (fabsf(s->iq_a) < MOTOR_COMMISSION_MAPPING_MIN_IQ_A ||
		    fabsf(s->mech_accel_rad_s2) < MOTOR_COMMISSION_MAPPING_MIN_ACCEL_RAD_S2) {
			continue;
		}

		sum_iq2 += s->iq_a * s->iq_a;
		sum_acc2 += s->mech_accel_rad_s2 * s->mech_accel_rad_s2;
		float32_t signed_iq = s->iq_a;
		if (commission->expected_mode == MOTOR_COMMISSION_EXPECT_TORQUE &&
		    res->iq_to_mech_sign < 0) {
			signed_iq = -signed_iq;
		}
		sum_cross += signed_iq * s->mech_accel_rad_s2;
		count++;
	}

	if (count >= MOTOR_COMMISSION_MAPPING_MIN_MECH_SAMPLES &&
	    sum_iq2 > 0.0f && sum_acc2 > 0.0f) {
		float32_t corr = sum_cross / sqrtf(sum_iq2 * sum_acc2);
		res->mapping_direction_valid = isfinite(corr);
		if (res->mapping_direction_valid) {
			res->mapping_direction_corr = corr;
			res->mapping_direction_pass =
				corr >= MOTOR_COMMISSION_MAPPING_DIRECTION_CORR_MIN;
		}
	}

	motor_commission_update_mapping_summary(ctx, res);
}

static void motor_commission_finalize(struct motor_commission_runtime_ctx *ctx)
{
	struct motor_commission_ctx *commission = ctx->commission;

	switch (commission->mode) {
	case MOTOR_COMMISSION_MODE_FLUX:
		motor_commission_estimate_flux(ctx);
		motor_commission_validate_mapping_flux(ctx);
		break;
	case MOTOR_COMMISSION_MODE_MECH:
		motor_commission_estimate_mech(ctx);
		motor_commission_validate_mapping_mech(ctx);
		break;
	default:
		break;
	}
}

void motor_commission_reset(struct motor_commission_runtime_ctx *ctx)
{
	if (ctx == NULL || ctx->commission == NULL || ctx->control_loop_count == NULL) {
		return;
	}

	struct motor_commission_ctx *commission = ctx->commission;
	commission->active = false;
	commission->mode = MOTOR_COMMISSION_MODE_NONE;
	commission->stage = MOTOR_COMMISSION_STAGE_IDLE;
	commission->expected_mode = MOTOR_COMMISSION_EXPECT_ANY;
	commission->start_loop_count = 0U;
	commission->stop_loop_count = 0U;
	commission->sample_decimation =
		motor_commission_default_decimation_hz(ctx->control_loop_frequency_hz);
	commission->flux_cfg.min_speed_hz = 0.0f;
	commission->flux_cfg.max_speed_hz = 0.0f;
	commission->flux_cfg.steps = 0U;
	commission->flux_cfg.settle_ms = 0U;
	commission->flux_cfg.sample_ms = 0U;
	commission->flux_cfg.iq_limit_a = 0.0f;
	commission->mech_cfg.base_speed_hz = 0.0f;
	commission->mech_cfg.dither_speed_hz = 0.0f;
	commission->mech_cfg.dither_period_ms = 0U;
	commission->mech_cfg.duration_ms = 0U;
	commission->results.iq_move_min_pos_a = 0.0f;
	commission->results.iq_move_min_neg_a = 0.0f;
	commission->results.iq_move_recommended_a = 0.0f;
	commission->results.psi_f_wb = 0.0f;
	commission->results.psi_f_bias_v = 0.0f;
	commission->results.psi_f_residual_rms_v = 0.0f;
	commission->results.psi_f_r2 = 0.0f;
	commission->results.psi_f_sample_count = 0U;
	commission->results.inertia_kgm2 = 0.0f;
	commission->results.viscous_friction_nm_per_rad_s = 0.0f;
	commission->results.coulomb_friction_nm = 0.0f;
	commission->results.offset_friction_nm = 0.0f;
	commission->results.mech_residual_rms_nm = 0.0f;
	commission->results.mech_r2 = 0.0f;
	commission->results.inertia_stddev_kgm2 = 0.0f;
	commission->results.viscous_friction_stddev_nm_per_rad_s = 0.0f;
	commission->results.coulomb_friction_stddev_nm = 0.0f;
	commission->results.mech_validation_residual_rms_nm = 0.0f;
	commission->results.mech_confidence = 0.0f;
	commission->results.mech_friction_residual_rms_nm = 0.0f;
	commission->results.mech_inertia_residual_rms_nm = 0.0f;
	commission->results.mech_inertia_plausibility_ratio = 0.0f;
	commission->results.mech_finalize_error = 0;
	commission->results.mech_sample_count = 0U;
	commission->results.mech_friction_sample_count = 0U;
	commission->results.mech_inertia_sample_count = 0U;
	commission->results.mech_accel_window_valid_count = 0U;
	commission->results.mech_capture_count = 0U;
	commission->results.mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;
	commission->results.mech_fit_torque_sign = 0;
	commission->results.mech_detent_correction_source =
		MOTOR_COMMISSION_DETENT_CORRECTION_NONE;
	commission->results.mapping_direction_corr = 0.0f;
	commission->results.mapping_offset_ratio = 0.0f;
	commission->results.mapping_pole_pairs_est = 0.0f;
	commission->results.mapping_confidence = 0.0f;
	commission->results.iq_move_pos_sample_count = 0U;
	commission->results.iq_move_neg_sample_count = 0U;
	commission->results.iq_move_warning_count = 0U;
	commission->results.iq_move_error_count = 0U;
	commission->results.iq_to_mech_sign = 0;
	commission->results.iq_move_pos_valid = false;
	commission->results.iq_move_neg_valid = false;
	commission->results.iq_move_valid = false;
	commission->results.psi_f_valid = false;
	commission->results.mech_valid = false;
	commission->results.mech_v2_valid = false;
	commission->results.mech_friction_valid = false;
	commission->results.mech_inertia_valid = false;
	commission->results.mech_detent_corrected = false;
	commission->results.mech_validation_valid = false;
	commission->results.mech_validation_pass = false;
	commission->results.mapping_direction_valid = false;
	commission->results.mapping_direction_pass = false;
	commission->results.mapping_offset_valid = false;
	commission->results.mapping_offset_pass = false;
	commission->results.mapping_pole_pairs_valid = false;
	commission->results.mapping_pole_pairs_pass = false;
	commission->results.mapping_valid = false;
	commission->results.mapping_pass = false;
	commission->auto_tune_staged = (struct motor_commission_tune_output){0};
	commission->auto_tune_valid = false;
	commission->auto_tune_applied = false;
	commission->auto_tune_last_error = 0;
	(void)motor_commission_tune_config_default(&commission->auto_tune_cfg,
						   (float32_t)ctx->pole_pairs,
						   1.0f / ctx->control_loop_frequency_hz,
						   ctx->motor_max_current_a,
						   fmaxf(ctx->profile_max_velocity_rad_s, 1.0f),
						   fmaxf(ctx->profile_max_accel_rad_s2, 1.0f));
	strncpy(commission->last_abort_reason, "none", sizeof(commission->last_abort_reason) - 1U);
	commission->last_abort_reason[sizeof(commission->last_abort_reason) - 1U] = '\0';
	motor_commission_reset_capture_state(commission);
}

void motor_commission_init(struct motor_commission_runtime_ctx *ctx)
{
	motor_commission_reset(ctx);
}

void motor_commission_abort(struct motor_commission_runtime_ctx *ctx, const char *reason)
{
	if (ctx == NULL || ctx->commission == NULL || ctx->control_loop_count == NULL) {
		return;
	}

	struct motor_commission_ctx *commission = ctx->commission;
	commission->active = false;
	commission->stage = MOTOR_COMMISSION_STAGE_ABORTED;
	commission->stop_loop_count = *ctx->control_loop_count;

	if (reason == NULL || reason[0] == '\0') {
		reason = "aborted";
	}

	strncpy(commission->last_abort_reason, reason, sizeof(commission->last_abort_reason) - 1U);
	commission->last_abort_reason[sizeof(commission->last_abort_reason) - 1U] = '\0';
}

static int motor_commission_start_common(struct motor_commission_runtime_ctx *ctx, uint8_t mode,
					 uint8_t expected_mode, uint32_t duration_ms)
{
	if (ctx == NULL || ctx->commission == NULL || ctx->control_loop_count == NULL) {
		return -EINVAL;
	}

	struct motor_commission_ctx *commission = ctx->commission;
	motor_commission_reset_capture_state(commission);
	commission->active = true;
	commission->mode = mode;
	commission->stage = MOTOR_COMMISSION_STAGE_RUNNING;
	commission->expected_mode = expected_mode;
	commission->start_loop_count = *ctx->control_loop_count;
	commission->sample_decimation =
		motor_commission_decimation_for_capture(ctx->control_loop_frequency_hz,
							duration_ms);
	commission->stop_loop_count = commission->start_loop_count +
			       (uint32_t)fmaxf(1.0f, (ctx->control_loop_frequency_hz *
						      (float32_t)duration_ms) /
							     1000.0f);
	strncpy(commission->last_abort_reason, "none", sizeof(commission->last_abort_reason) - 1U);
	commission->last_abort_reason[sizeof(commission->last_abort_reason) - 1U] = '\0';

	return 0;
}

int motor_commission_start_flux(struct motor_commission_runtime_ctx *ctx,
				const struct motor_commission_flux_config *cfg)
{
	if (ctx == NULL || ctx->commission == NULL || cfg == NULL) {
		return -EINVAL;
	}
	if (!isfinite(cfg->min_speed_hz) || !isfinite(cfg->max_speed_hz) ||
	    !isfinite(cfg->iq_limit_a)) {
		return -EINVAL;
	}
	if (cfg->steps == 0U || cfg->sample_ms == 0U || cfg->settle_ms == 0U ||
	    cfg->max_speed_hz <= cfg->min_speed_hz || cfg->iq_limit_a <= 0.0f) {
		return -EINVAL;
	}

	uint64_t point_ms = (uint64_t)cfg->settle_ms + (uint64_t)cfg->sample_ms;
	uint64_t total_ms = point_ms * (uint64_t)cfg->steps * 2ULL;
	if (total_ms > UINT32_MAX) {
		return -ERANGE;
	}

	int ret = motor_commission_start_common(ctx, MOTOR_COMMISSION_MODE_FLUX,
						MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED,
						(uint32_t)total_ms);
	if (ret < 0) {
		return ret;
	}

	ctx->commission->flux_cfg = *cfg;
	ctx->commission->results.psi_f_valid = false;
	ctx->commission->results.psi_f_sample_count = 0U;
	ctx->commission->results.psi_f_wb = 0.0f;
	ctx->commission->results.psi_f_bias_v = 0.0f;
	ctx->commission->results.psi_f_residual_rms_v = 0.0f;
	ctx->commission->results.psi_f_r2 = 0.0f;
	ctx->commission->results.mapping_offset_valid = false;
	ctx->commission->results.mapping_offset_pass = false;
	ctx->commission->results.mapping_offset_ratio = 0.0f;
	ctx->commission->results.mapping_pole_pairs_valid = false;
	ctx->commission->results.mapping_pole_pairs_pass = false;
	ctx->commission->results.mapping_pole_pairs_est = 0.0f;
	motor_commission_update_mapping_summary(ctx, &ctx->commission->results);
	return 0;
}

int motor_commission_start_mech(struct motor_commission_runtime_ctx *ctx,
				const struct motor_commission_mech_config *cfg)
{
	if (ctx == NULL || ctx->commission == NULL || cfg == NULL) {
		return -EINVAL;
	}
	if (!isfinite(cfg->base_speed_hz) || !isfinite(cfg->dither_speed_hz)) {
		return -EINVAL;
	}
	if (cfg->duration_ms == 0U || cfg->dither_period_ms == 0U ||
	    cfg->base_speed_hz <= 0.0f || cfg->dither_speed_hz <= 0.0f ||
	    cfg->dither_speed_hz >= cfg->base_speed_hz) {
		return -EINVAL;
	}

	int ret = motor_commission_start_common(ctx, MOTOR_COMMISSION_MODE_MECH,
						MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED,
						cfg->duration_ms);
	if (ret < 0) {
		return ret;
	}

	ctx->commission->mech_cfg = *cfg;
	ctx->commission->results.mech_valid = false;
	ctx->commission->results.mech_sample_count = 0U;
	ctx->commission->results.inertia_kgm2 = 0.0f;
	ctx->commission->results.viscous_friction_nm_per_rad_s = 0.0f;
	ctx->commission->results.coulomb_friction_nm = 0.0f;
	ctx->commission->results.offset_friction_nm = 0.0f;
	ctx->commission->results.mech_residual_rms_nm = 0.0f;
	ctx->commission->results.mech_r2 = 0.0f;
	ctx->commission->results.inertia_stddev_kgm2 = 0.0f;
	ctx->commission->results.viscous_friction_stddev_nm_per_rad_s = 0.0f;
	ctx->commission->results.coulomb_friction_stddev_nm = 0.0f;
	ctx->commission->results.mech_validation_residual_rms_nm = 0.0f;
	ctx->commission->results.mech_confidence = 0.0f;
	ctx->commission->results.mech_friction_residual_rms_nm = 0.0f;
	ctx->commission->results.mech_inertia_residual_rms_nm = 0.0f;
	ctx->commission->results.mech_inertia_plausibility_ratio = 0.0f;
	ctx->commission->results.mech_finalize_error = 0;
	ctx->commission->results.mech_friction_sample_count = 0U;
	ctx->commission->results.mech_inertia_sample_count = 0U;
	ctx->commission->results.mech_accel_window_valid_count = 0U;
	ctx->commission->results.mech_capture_count = 0U;
	ctx->commission->results.mech_reject_reason = MOTOR_COMMISSION_MECH_REJECT_NONE;
	ctx->commission->results.mech_fit_torque_sign = 0;
	ctx->commission->results.mech_detent_correction_source =
		MOTOR_COMMISSION_DETENT_CORRECTION_NONE;
	ctx->commission->results.mech_validation_valid = false;
	ctx->commission->results.mech_validation_pass = false;
	ctx->commission->results.mech_v2_valid = false;
	ctx->commission->results.mech_friction_valid = false;
	ctx->commission->results.mech_inertia_valid = false;
	ctx->commission->results.mech_detent_corrected = false;
	ctx->commission->results.mapping_direction_valid = false;
	ctx->commission->results.mapping_direction_pass = false;
	ctx->commission->results.mapping_direction_corr = 0.0f;
	motor_commission_update_mapping_summary(ctx, &ctx->commission->results);
	return 0;
}

int motor_commission_apply_results(struct motor_commission_runtime_ctx *ctx)
{
	if (ctx == NULL || ctx->commission == NULL) {
		return -EINVAL;
	}

	const struct motor_commission_results *results = &ctx->commission->results;
	if (!results->psi_f_valid && !results->mech_valid) {
		return -ENOENT;
	}

	if (results->psi_f_valid) {
		*ctx->flux_linkage_wb_active = results->psi_f_wb;
		float32_t derived_kt = motor_torque_gain_from_flux_pole_pairs(
			results->psi_f_wb, ctx->pole_pairs);
		if (isfinite(derived_kt) && derived_kt > MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
			*ctx->torque_gain_nm_per_a_active = derived_kt;
		}
		if (ctx->flux_model_source != NULL) {
			*ctx->flux_model_source = MOTOR_MODEL_SOURCE_MEASURED;
		}
	}
	if (results->mech_valid) {
		*ctx->inertia_kgm2_active = results->inertia_kgm2;
		*ctx->viscous_friction_nm_per_rad_s_active =
			results->viscous_friction_nm_per_rad_s;
		*ctx->coulomb_friction_nm_active = results->coulomb_friction_nm;
		if (ctx->mech_model_source != NULL) {
			*ctx->mech_model_source = MOTOR_MODEL_SOURCE_MEASURED;
		}
	}

	if (!isfinite(*ctx->torque_gain_nm_per_a_active) ||
	    *ctx->torque_gain_nm_per_a_active <= MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		*ctx->torque_gain_nm_per_a_active = motor_torque_gain_resolve(
			*ctx->torque_gain_nm_per_a_active,
			*ctx->flux_linkage_wb_active,
			ctx->default_flux_linkage_wb,
			ctx->pole_pairs);
	}

	return 0;
}

int motor_commission_stage_auto_tune(struct motor_commission_runtime_ctx *ctx,
				     const struct motor_commission_tune_config *cfg)
{
	if (ctx == NULL || ctx->commission == NULL) {
		return -EINVAL;
	}

	struct motor_commission_ctx *commission = ctx->commission;
	const struct motor_commission_tune_config *cfg_sel = cfg;
	if (cfg_sel == NULL) {
		cfg_sel = &commission->auto_tune_cfg;
	}

	if (cfg != NULL) {
		commission->auto_tune_cfg = *cfg;
	}

	const struct motor_commission_results *results = &commission->results;
	struct motor_commission_fit_summary fit = {
		.psi_f_valid = results->psi_f_valid,
		.psi_f_wb = results->psi_f_wb,
		.psi_f_r2 = results->psi_f_r2,
		.psi_f_residual_rms_v = results->psi_f_residual_rms_v,
		.psi_f_sample_count = results->psi_f_sample_count,
		.mech_valid = results->mech_valid,
		.inertia_kgm2 = results->inertia_kgm2,
		.viscous_friction_nm_per_rad_s = results->viscous_friction_nm_per_rad_s,
		.mech_r2 = results->mech_r2,
		.mech_residual_rms_nm = results->mech_residual_rms_nm,
		.mech_sample_count = results->mech_sample_count,
	};

	commission->auto_tune_staged = (struct motor_commission_tune_output){0};
	int ret = motor_commission_tune_compute(&fit, cfg_sel, &commission->auto_tune_staged);
	commission->auto_tune_valid = (ret == 0) && commission->auto_tune_staged.accepted;
	commission->auto_tune_applied = false;
	commission->auto_tune_last_error = ret;
	return ret;
}

int motor_commission_apply_staged_auto_tune(struct motor_commission_runtime_ctx *ctx)
{
	if (ctx == NULL || ctx->commission == NULL) {
		return -EINVAL;
	}

	struct motor_commission_ctx *commission = ctx->commission;
	const struct motor_commission_tune_output *staged = &commission->auto_tune_staged;
	if (!commission->auto_tune_valid || !staged->accepted) {
		return -ENOENT;
	}

	int ret = motor_commission_apply_results(ctx);
	if (ret < 0) {
		commission->auto_tune_last_error = ret;
		return ret;
	}

	*ctx->velocity_cl_kp_a_per_rad_s = staged->velocity_kp_a_per_rad_s;
	*ctx->velocity_cl_ki_a_per_rad = staged->velocity_ki_a_per_rad;
	*ctx->velocity_cl_iq_limit_a = staged->velocity_iq_limit_a;
	*ctx->velocity_cl_i_term_a = 0.0f;
	if (isfinite(staged->kt_nm_per_a) && staged->kt_nm_per_a > MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		*ctx->torque_gain_nm_per_a_active = staged->kt_nm_per_a;
	}

	*ctx->position_cl_kp_rad_s_per_rad = staged->position_kp_rad_s_per_rad;
	*ctx->position_cl_ki_rad_s2_per_rad = staged->position_ki_rad_s2_per_rad;
	*ctx->position_cl_i_term_rad_s = 0.0f;

	ctx->velocity_mpr_cfg->horizon = staged->velocity_mpr_horizon;
	ctx->velocity_mpr_cfg->q_speed = staged->velocity_mpr_q_speed;
	ctx->velocity_mpr_cfg->r_delta_iq = staged->velocity_mpr_r_delta_iq;
	ctx->velocity_mpr_cfg->iq_limit_a = staged->velocity_iq_limit_a;
	ctx->velocity_mpr_cfg->max_delta_iq_a = staged->velocity_mpr_max_delta_iq_a;
	ctx->velocity_mpr_cfg->disturbance_ki_nm_per_rad_s =
		staged->velocity_mpr_disturbance_ki_nm_per_rad_s;

	ctx->position_mpr_cfg->horizon = staged->position_mpr_horizon;
	ctx->position_mpr_cfg->q_position = staged->position_mpr_q_position;
	ctx->position_mpr_cfg->q_velocity_ff = staged->position_mpr_q_velocity_ff;
	ctx->position_mpr_cfg->r_delta_velocity = staged->position_mpr_r_delta_velocity;
	ctx->position_mpr_cfg->max_delta_velocity_rad_s =
		staged->position_mpr_max_delta_velocity_rad_s;
	ctx->position_mpr_cfg->velocity_limit_rad_s = ctx->profile_max_velocity_rad_s;

	ctx->velocity_dob_cfg->enabled = staged->velocity_dob_enable;
	ctx->velocity_dob_cfg->observer_gain_nm_per_rad_s =
		staged->velocity_dob_observer_gain_nm_per_rad_s;
	ctx->velocity_dob_cfg->torque_limit_nm = staged->velocity_dob_torque_limit_nm;
	ctx->velocity_dob_cfg->iq_ff_limit_a = staged->velocity_dob_iq_ff_limit_a;

	motor_mpr_velocity_reset(ctx->velocity_mpr_state, *ctx->live_velocity_rad_s, 0.0f);
	motor_mpr_position_reset(ctx->position_mpr_state, *ctx->live_position_rad);
	motor_dob_reset(ctx->velocity_dob_state, *ctx->live_velocity_rad_s);
	*ctx->live_velocity_dob_iq_ff_a = 0.0f;
	*ctx->live_velocity_dob_disturbance_nm = 0.0f;
	*ctx->live_velocity_dob_residual_rad_s = 0.0f;

	commission->auto_tune_applied = true;
	commission->auto_tune_last_error = 0;
	return 0;
}

void motor_commission_update(struct motor_commission_runtime_ctx *ctx,
			     const struct motor_commission_observation *obs)
{
	if (ctx == NULL || ctx->commission == NULL || obs == NULL) {
		return;
	}

	struct motor_commission_ctx *commission = ctx->commission;
	if (!commission->active || commission->stage != MOTOR_COMMISSION_STAGE_RUNNING) {
		return;
	}

	if ((int32_t)(obs->control_loop_count - commission->stop_loop_count) >= 0) {
		commission->active = false;
		commission->stage = MOTOR_COMMISSION_STAGE_COMPLETED;
		motor_commission_finalize(ctx);
		return;
	}

	commission->sample_decimation_counter++;
	if (commission->sample_decimation_counter < commission->sample_decimation) {
		return;
	}
	commission->sample_decimation_counter = 0U;

	if (!obs->data_valid) {
		commission->rejected_samples++;
		commission->reject_data_invalid++;
		return;
	}
	if (!motor_commission_obs_matches_expected(obs, commission->expected_mode)) {
		commission->rejected_samples++;
		commission->reject_mode_mismatch++;
		return;
	}
	if (!obs->control_armed) {
		commission->rejected_samples++;
		commission->reject_disarmed++;
		return;
	}
	if (obs->fault_active) {
		commission->rejected_samples++;
		commission->reject_fault++;
		return;
	}
	if (!obs->encoder_fresh || obs->encoder_error) {
		commission->rejected_samples++;
		commission->reject_encoder++;
		return;
	}
	if (obs->saturation) {
		commission->rejected_samples++;
		commission->reject_saturation++;
		return;
	}
	if (!motor_commission_obs_velocity_tracking_ok(commission, obs)) {
		commission->rejected_samples++;
		commission->reject_velocity_tracking++;
		return;
	}
	if (commission->sample_count >= MOTOR_COMMISSION_MAX_SAMPLES) {
		motor_commission_abort(ctx, "capture buffer full");
		return;
	}

	float32_t did_dt = 0.0f;
	float32_t diq_dt = 0.0f;
	float32_t domega_dt = 0.0f;

	if (commission->prev_valid) {
		uint32_t delta_cycles = obs->control_loop_count - commission->prev_loop_count;
		if (delta_cycles == 0U) {
			delta_cycles = 1U;
		}
		float32_t dt_s = (float32_t)delta_cycles / ctx->control_loop_frequency_hz;
		did_dt = (obs->id_a - commission->prev_id_a) / dt_s;
		diq_dt = (obs->iq_a - commission->prev_iq_a) / dt_s;
		domega_dt = (obs->mech_speed_rad_s - commission->prev_speed_rad_s) / dt_s;
	}

	commission->did_dt_filt_a_s +=
		MOTOR_COMMISSION_DERIV_ALPHA * (did_dt - commission->did_dt_filt_a_s);
	commission->diq_dt_filt_a_s +=
		MOTOR_COMMISSION_DERIV_ALPHA * (diq_dt - commission->diq_dt_filt_a_s);
	commission->domega_dt_filt_rad_s2 +=
		MOTOR_COMMISSION_DERIV_ALPHA * (domega_dt - commission->domega_dt_filt_rad_s2);

	struct motor_commission_sample *sample =
		&commission->samples[commission->sample_count++];
	sample->loop_count = obs->control_loop_count;
	sample->mech_position_rad = obs->mech_position_rad;
	sample->mech_speed_rad_s = obs->mech_speed_rad_s;
	sample->elec_speed_rad_s = obs->elec_speed_rad_s;
	sample->mech_accel_rad_s2 = commission->domega_dt_filt_rad_s2;
	sample->mech_accel_window_rad_s2 = commission->domega_dt_filt_rad_s2;
	sample->id_a = obs->id_a;
	sample->iq_a = obs->iq_a;
	sample->did_dt_a_s = commission->did_dt_filt_a_s;
	sample->diq_dt_a_s = commission->diq_dt_filt_a_s;
	sample->vd_v = obs->vd_v;
	sample->vq_v = obs->vq_v;
	sample->vbus_v = obs->vbus_v;
	sample->encoder_status = obs->encoder_status;
	sample->flags = obs->saturation ? MOTOR_COMMISSION_FLAG_SATURATED : 0U;

	commission->accepted_samples++;
	commission->prev_valid = true;
	commission->prev_loop_count = obs->control_loop_count;
	commission->prev_id_a = obs->id_a;
	commission->prev_iq_a = obs->iq_a;
	commission->prev_speed_rad_s = obs->mech_speed_rad_s;
}
