/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/runtime/commission_tune.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>

#include "motor/math/math_constants.h"

#define MOTOR_COMMISSION_TUNE_IQ_LIMIT_MIN_A 0.05f
#define MOTOR_COMMISSION_TUNE_POSITION_BW_RATIO_MIN 0.02f
#define MOTOR_COMMISSION_TUNE_POSITION_BW_RATIO_MAX 0.20f
#define MOTOR_COMMISSION_TUNE_MPR_Q_MIN 0.003f
#define MOTOR_COMMISSION_TUNE_MPR_Q_MAX 0.05f
#define MOTOR_COMMISSION_TUNE_MPR_R_MIN 20.0f
#define MOTOR_COMMISSION_TUNE_MPR_R_MAX 100.0f
#define MOTOR_COMMISSION_TUNE_MPR_DI_MIN_A 0.0005f
#define MOTOR_COMMISSION_TUNE_MPR_DI_MAX_A 0.0020f
#define MOTOR_COMMISSION_TUNE_VELOCITY_KI_TO_KP_MAX 2.0f

static bool tune_is_finite_positive(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static bool tune_is_finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

int motor_commission_tune_validate_config(const struct motor_commission_tune_config *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	if (!tune_is_finite_positive(cfg->pole_pairs) ||
	    !tune_is_finite_positive(cfg->dt_s) ||
	    !tune_is_finite_positive(cfg->velocity_bw_hz) ||
	    !tune_is_finite_positive(cfg->velocity_zeta) ||
	    !tune_is_finite_positive(cfg->position_bw_ratio) ||
	    !tune_is_finite_positive(cfg->position_zeta) ||
	    !tune_is_finite_positive(cfg->iq_limit_a) ||
	    !tune_is_finite_positive(cfg->max_current_a) ||
	    !tune_is_finite_positive(cfg->profile_max_velocity_rad_s) ||
	    !tune_is_finite_positive(cfg->profile_max_accel_rad_s2) ||
	    !tune_is_finite_nonnegative(cfg->min_flux_r2) ||
	    !tune_is_finite_nonnegative(cfg->min_mech_r2) ||
	    !tune_is_finite_positive(cfg->max_flux_rms_v) ||
	    !tune_is_finite_positive(cfg->max_mech_rms_nm) ||
	    cfg->min_flux_samples == 0U ||
	    cfg->min_mech_samples == 0U) {
		return -EINVAL;
	}

	if (cfg->position_bw_ratio > MOTOR_COMMISSION_TUNE_POSITION_BW_RATIO_MAX) {
		return -EINVAL;
	}
	if (cfg->iq_limit_a > cfg->max_current_a) {
		return -EINVAL;
	}

	return 0;
}

int motor_commission_tune_config_default(struct motor_commission_tune_config *cfg,
					 float32_t pole_pairs,
					 float32_t dt_s,
					 float32_t max_current_a,
					 float32_t profile_max_velocity_rad_s,
					 float32_t profile_max_accel_rad_s2)
{
	if (cfg == NULL ||
	    !tune_is_finite_positive(pole_pairs) ||
	    !tune_is_finite_positive(dt_s) ||
	    !tune_is_finite_positive(max_current_a) ||
	    !tune_is_finite_positive(profile_max_velocity_rad_s) ||
	    !tune_is_finite_positive(profile_max_accel_rad_s2)) {
		return -EINVAL;
	}

	cfg->pole_pairs = pole_pairs;
	cfg->dt_s = dt_s;
	cfg->velocity_bw_hz = 10.0f;
	cfg->velocity_zeta = 1.0f;
	cfg->position_bw_ratio = MOTOR_COMMISSION_TUNE_POSITION_BW_RATIO_MAX;
	cfg->position_zeta = 1.0f;
	cfg->max_current_a = max_current_a;
	cfg->iq_limit_a = max_current_a;
	cfg->profile_max_velocity_rad_s = profile_max_velocity_rad_s;
	cfg->profile_max_accel_rad_s2 = profile_max_accel_rad_s2;
	cfg->min_flux_r2 = 0.55f;
	cfg->min_mech_r2 = 0.20f;
	cfg->max_flux_rms_v = 2.0f;
	cfg->max_mech_rms_nm = 0.50f;
	cfg->min_flux_samples = 24U;
	cfg->min_mech_samples = 48U;

	return motor_commission_tune_validate_config(cfg);
}

int motor_commission_tune_compute(const struct motor_commission_fit_summary *fit,
				  const struct motor_commission_tune_config *cfg,
				  struct motor_commission_tune_output *out)
{
	if (fit == NULL || out == NULL) {
		return -EINVAL;
	}

	int ret = motor_commission_tune_validate_config(cfg);
	if (ret != 0) {
		return ret;
	}

	float32_t psi_f = fit->psi_f_wb;
	float32_t inertia = fit->inertia_kgm2;
	float32_t viscous = fit->viscous_friction_nm_per_rad_s;

	uint32_t reject_flags = MOTOR_COMMISSION_TUNE_REJECT_NONE;
	if (!fit->psi_f_valid) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_PSI_INVALID;
	}
	if (fit->psi_f_sample_count < cfg->min_flux_samples) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_PSI_SAMPLES;
	}
	if (!isfinite(fit->psi_f_r2) || fit->psi_f_r2 < cfg->min_flux_r2) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_PSI_R2;
	}
	if (!isfinite(fit->psi_f_residual_rms_v) ||
	    fit->psi_f_residual_rms_v > cfg->max_flux_rms_v) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_PSI_RMS;
	}
	if (!tune_is_finite_positive(psi_f)) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_PSI_SIGN;
	}

	if (!fit->mech_valid) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_MECH_INVALID;
	}
	if (fit->mech_sample_count < cfg->min_mech_samples) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_MECH_SAMPLES;
	}
	if (!isfinite(fit->mech_r2) || fit->mech_r2 < cfg->min_mech_r2) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_MECH_R2;
	}
	if (!isfinite(fit->mech_residual_rms_nm) ||
	    fit->mech_residual_rms_nm > cfg->max_mech_rms_nm) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_MECH_RMS;
	}
	if (!tune_is_finite_positive(inertia)) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_INERTIA_SIGN;
	}
	if (!tune_is_finite_nonnegative(viscous)) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_VISCOUS_SIGN;
	}

	float32_t kt = 1.5f * cfg->pole_pairs * psi_f;
	if (!tune_is_finite_positive(kt)) {
		reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_KT_INVALID;
	}

	*out = (struct motor_commission_tune_output){0};
	out->reject_flags = reject_flags;
	out->accepted = (reject_flags == MOTOR_COMMISSION_TUNE_REJECT_NONE);
	out->kt_nm_per_a = kt;

	if (!out->accepted) {
		return -ERANGE;
	}

	float32_t iq_limit = clampf(cfg->iq_limit_a,
				    MOTOR_COMMISSION_TUNE_IQ_LIMIT_MIN_A,
				    cfg->max_current_a);
	float32_t velocity_bw_hz = cfg->velocity_bw_hz;
	float32_t velocity_omega = 2.0f * PI_F32 * velocity_bw_hz;
	float32_t velocity_kp_model =
		((2.0f * cfg->velocity_zeta * velocity_omega * inertia) - viscous) / kt;
	float32_t velocity_kp_floor = (0.25f * velocity_omega * inertia) / kt;
	float32_t velocity_kp = fmaxf(velocity_kp_model, velocity_kp_floor);
	float32_t velocity_ki_model = (velocity_omega * velocity_omega * inertia) / kt;
	float32_t velocity_ki =
		fminf(velocity_ki_model,
		      MOTOR_COMMISSION_TUNE_VELOCITY_KI_TO_KP_MAX * velocity_kp);

	float32_t position_ratio = clampf(cfg->position_bw_ratio,
					  MOTOR_COMMISSION_TUNE_POSITION_BW_RATIO_MIN,
					  MOTOR_COMMISSION_TUNE_POSITION_BW_RATIO_MAX);
	float32_t position_bw_hz = velocity_bw_hz * position_ratio;
	float32_t position_omega = 2.0f * PI_F32 * position_bw_hz;
	float32_t position_kp = 2.0f * cfg->position_zeta * position_omega;
	float32_t position_ki = position_omega * position_omega;

	if (!tune_is_finite_positive(velocity_kp) ||
	    !tune_is_finite_positive(velocity_ki) ||
	    !tune_is_finite_positive(position_kp) ||
	    !tune_is_finite_positive(position_ki)) {
		out->reject_flags |= MOTOR_COMMISSION_TUNE_REJECT_KT_INVALID;
		out->accepted = false;
		return -ERANGE;
	}

	out->velocity_bw_hz = velocity_bw_hz;
	out->position_bw_hz = position_bw_hz;
	out->velocity_kp_a_per_rad_s = velocity_kp;
	out->velocity_ki_a_per_rad = velocity_ki;
	out->velocity_iq_limit_a = iq_limit;
	out->position_kp_rad_s_per_rad = position_kp;
	out->position_ki_rad_s2_per_rad = position_ki;

	out->velocity_mpr_horizon = 8U;
	/*
	 * Velocity MPR is much more sensitive to sample-to-sample current
	 * changes than the PI path. Keep commissioned defaults conservative and
	 * let the shell bandwidth command raise them after HIL validation.
	 */
	out->velocity_mpr_q_speed =
		clampf((velocity_omega * inertia) / kt,
		       MOTOR_COMMISSION_TUNE_MPR_Q_MIN,
		       MOTOR_COMMISSION_TUNE_MPR_Q_MAX);
	out->velocity_mpr_r_delta_iq =
		clampf(1.0f / (4.0f * out->velocity_mpr_q_speed),
		       MOTOR_COMMISSION_TUNE_MPR_R_MIN,
		       MOTOR_COMMISSION_TUNE_MPR_R_MAX);
	out->velocity_mpr_max_delta_iq_a =
		clampf(iq_limit * 0.003f,
		       MOTOR_COMMISSION_TUNE_MPR_DI_MIN_A,
		       fminf(iq_limit, MOTOR_COMMISSION_TUNE_MPR_DI_MAX_A));
	out->velocity_mpr_disturbance_ki_nm_per_rad_s = 0.0f;

	out->position_mpr_horizon = 16U;
	out->position_mpr_q_position = clampf(2.0f * position_omega, 0.5f, 20.0f);
	out->position_mpr_q_velocity_ff = clampf(position_omega * 0.5f, 0.1f, 10.0f);
	out->position_mpr_r_delta_velocity =
		clampf(1.0f / (5.0f * out->position_mpr_q_position), 0.02f, 0.5f);
	out->position_mpr_max_delta_velocity_rad_s =
		clampf(cfg->profile_max_accel_rad_s2 * cfg->dt_s,
		       0.001f, cfg->profile_max_velocity_rad_s);

	/* Stage DOB limits from the fit, but leave DOB disabled until the PI loops are validated. */
	out->velocity_dob_enable = false;
	out->velocity_dob_observer_gain_nm_per_rad_s =
		clampf(0.01f + (0.001f * velocity_bw_hz), 0.01f, 0.05f);
	out->velocity_dob_iq_ff_limit_a = clampf(iq_limit * 0.40f, 0.05f, iq_limit);
	out->velocity_dob_torque_limit_nm = kt * out->velocity_dob_iq_ff_limit_a;

	return 0;
}
