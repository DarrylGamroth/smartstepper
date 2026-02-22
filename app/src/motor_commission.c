/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>

#include <zephyr/sys/util.h>

#include "motor_commission.h"
#include "config.h"
#include "motor_commission_id.h"
#include "motor_state_utils.h"
#include "motor_states.h"

#define MOTOR_COMMISSION_DERIV_ALPHA 0.2f
#define MOTOR_COMMISSION_FLAG_SATURATED BIT(0)
#define MOTOR_COMMISSION_MIN_FLUX_SAMPLES 16U
#define MOTOR_COMMISSION_MIN_MECH_SAMPLES 32U
#define MOTOR_COMMISSION_MIN_SPEED_RAD_S (2.0f * PI_F32)
#define MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S 0.5f
#define MOTOR_COMMISSION_MIN_KT_NM_PER_A 1.0e-6f

static inline uint32_t motor_commission_default_decimation(void)
{
	uint32_t decim = (uint32_t)(CONTROL_LOOP_FREQUENCY_HZ / 1000.0f);

	return MAX(decim, 1U);
}

static inline bool motor_commission_state_matches_expected(const struct smf_state *state,
							   uint8_t expected_mode)
{
	switch (expected_mode) {
	case MOTOR_COMMISSION_EXPECT_ANY:
		return true;
	case MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED:
		return motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_VELOCITY_CLOSED);
	case MOTOR_COMMISSION_EXPECT_TORQUE:
		return motor_state_ptr_is_mode(state, MOTOR_STATE_ONLINE_TORQUE);
	default:
		return false;
	}
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

bool motor_commission_is_active(const struct motor_parameters *params)
{
	return params != NULL && params->commission.active;
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

static void motor_commission_estimate_flux(struct motor_parameters *params)
{
	struct motor_commission_ctx *ctx = &params->commission;
	struct motor_commission_results *res = &ctx->results;
	struct motor_flux_id_state estimator;
	struct motor_flux_id_result estimate;
	const struct motor_flux_id_config cfg = {
		.rs_ohm = params->Rs_measured_ohm,
		.ld_h = params->Ld_est,
		.lq_h = params->Lq_est,
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
	for (uint32_t i = 0U; i < ctx->sample_count; i++) {
		const struct motor_commission_sample *s = &ctx->samples[i];
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

static void motor_commission_estimate_mech(struct motor_parameters *params)
{
	struct motor_commission_ctx *ctx = &params->commission;
	struct motor_commission_results *res = &ctx->results;
	struct motor_mech_id_state estimator;
	struct motor_mech_id_result estimate;
	const float32_t psi_f = res->psi_f_valid ? res->psi_f_wb : params->flux_linkage_wb_active;
	const float32_t kt = 1.5f * (float32_t)MOTOR_POLE_PAIRS * psi_f;
	const struct motor_mech_id_config cfg = {
		.kt_nm_per_a = kt,
		.sign_deadband_rad_s = MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S,
		.min_samples = MOTOR_COMMISSION_MIN_MECH_SAMPLES,
		.min_r2 = 0.0f,
		.require_positive_inertia = true,
		.require_nonnegative_viscous = true,
	};

	res->mech_valid = false;
	res->mech_sample_count = 0U;
	res->inertia_kgm2 = 0.0f;
	res->viscous_friction_nm_per_rad_s = 0.0f;
	res->coulomb_friction_nm = 0.0f;
	res->offset_friction_nm = 0.0f;
	res->mech_residual_rms_nm = 0.0f;
	res->mech_r2 = 0.0f;

	if (!isfinite(kt) || fabsf(kt) < MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		return;
	}

	motor_mech_id_init(&estimator, &cfg);
	for (uint32_t i = 0U; i < ctx->sample_count; i++) {
		const struct motor_commission_sample *s = &ctx->samples[i];
		(void)motor_mech_id_accumulate(&estimator, s->mech_speed_rad_s,
					       s->mech_accel_rad_s2, s->iq_a);
	}

	if (motor_mech_id_finalize(&estimator, &estimate) < 0) {
		return;
	}

	res->mech_sample_count = estimate.sample_count;
	res->inertia_kgm2 = estimate.inertia_kgm2;
	res->viscous_friction_nm_per_rad_s = estimate.viscous_friction_nm_per_rad_s;
	res->coulomb_friction_nm = estimate.coulomb_friction_nm;
	res->offset_friction_nm = estimate.offset_friction_nm;
	res->mech_residual_rms_nm = estimate.residual_rms_nm;
	res->mech_r2 = estimate.r2;
	res->mech_valid = estimate.valid;
}

static void motor_commission_finalize(struct motor_parameters *params)
{
	struct motor_commission_ctx *ctx = &params->commission;

	switch (ctx->mode) {
	case MOTOR_COMMISSION_MODE_FLUX:
		motor_commission_estimate_flux(params);
		break;
	case MOTOR_COMMISSION_MODE_MECH:
		motor_commission_estimate_mech(params);
		break;
	default:
		break;
	}
}

void motor_commission_reset(struct motor_parameters *params)
{
	if (params == NULL) {
		return;
	}

	struct motor_commission_ctx *ctx = &params->commission;
	ctx->active = false;
	ctx->mode = MOTOR_COMMISSION_MODE_NONE;
	ctx->stage = MOTOR_COMMISSION_STAGE_IDLE;
	ctx->expected_mode = MOTOR_COMMISSION_EXPECT_ANY;
	ctx->start_loop_count = 0U;
	ctx->stop_loop_count = 0U;
	ctx->sample_decimation = motor_commission_default_decimation();
	ctx->flux_cfg.min_speed_hz = 0.0f;
	ctx->flux_cfg.max_speed_hz = 0.0f;
	ctx->flux_cfg.steps = 0U;
	ctx->flux_cfg.settle_ms = 0U;
	ctx->flux_cfg.sample_ms = 0U;
	ctx->flux_cfg.iq_limit_a = 0.0f;
	ctx->mech_cfg.coast_speed_hz = 0.0f;
	ctx->mech_cfg.prbs_amp_a = 0.0f;
	ctx->mech_cfg.prbs_period_ms = 0U;
	ctx->mech_cfg.duration_ms = 0U;
	ctx->results.psi_f_wb = 0.0f;
	ctx->results.psi_f_bias_v = 0.0f;
	ctx->results.psi_f_residual_rms_v = 0.0f;
	ctx->results.psi_f_r2 = 0.0f;
	ctx->results.psi_f_sample_count = 0U;
	ctx->results.inertia_kgm2 = 0.0f;
	ctx->results.viscous_friction_nm_per_rad_s = 0.0f;
	ctx->results.coulomb_friction_nm = 0.0f;
	ctx->results.offset_friction_nm = 0.0f;
	ctx->results.mech_residual_rms_nm = 0.0f;
	ctx->results.mech_r2 = 0.0f;
	ctx->results.mech_sample_count = 0U;
	ctx->results.psi_f_valid = false;
	ctx->results.mech_valid = false;
	ctx->auto_tune_staged = (struct motor_commission_tune_output){0};
	ctx->auto_tune_valid = false;
	ctx->auto_tune_applied = false;
	ctx->auto_tune_last_error = 0;
	(void)motor_commission_tune_config_default(&ctx->auto_tune_cfg,
						   (float32_t)MOTOR_POLE_PAIRS,
						   1.0f / CONTROL_LOOP_FREQUENCY_HZ,
						   MAX(MOTOR_MAX_CURRENT_A, 0.1f),
						   MAX(params->profile_max_velocity_rad_s, 1.0f),
						   MAX(params->profile_max_accel_rad_s2, 1.0f));
	strncpy(ctx->last_abort_reason, "none", sizeof(ctx->last_abort_reason) - 1U);
	ctx->last_abort_reason[sizeof(ctx->last_abort_reason) - 1U] = '\0';
	motor_commission_reset_capture_state(ctx);
}

void motor_commission_init(struct motor_parameters *params)
{
	motor_commission_reset(params);
}

void motor_commission_abort(struct motor_parameters *params, const char *reason)
{
	if (params == NULL) {
		return;
	}

	struct motor_commission_ctx *ctx = &params->commission;
	ctx->active = false;
	ctx->stage = MOTOR_COMMISSION_STAGE_ABORTED;
	ctx->stop_loop_count = params->control_loop_count;

	if (reason == NULL || reason[0] == '\0') {
		reason = "aborted";
	}

	strncpy(ctx->last_abort_reason, reason, sizeof(ctx->last_abort_reason) - 1U);
	ctx->last_abort_reason[sizeof(ctx->last_abort_reason) - 1U] = '\0';
}

static int motor_commission_start_common(struct motor_parameters *params, uint8_t mode,
					 uint8_t expected_mode, uint32_t duration_ms)
{
	if (params == NULL) {
		return -EINVAL;
	}

	struct motor_commission_ctx *ctx = &params->commission;
	motor_commission_reset_capture_state(ctx);
	ctx->active = true;
	ctx->mode = mode;
	ctx->stage = MOTOR_COMMISSION_STAGE_RUNNING;
	ctx->expected_mode = expected_mode;
	ctx->start_loop_count = params->control_loop_count;
	ctx->sample_decimation = motor_commission_default_decimation();
	ctx->stop_loop_count = ctx->start_loop_count +
			       (uint32_t)MAX(1.0f, (CONTROL_LOOP_FREQUENCY_HZ *
						    (float32_t)duration_ms) /
							   1000.0f);
	strncpy(ctx->last_abort_reason, "none", sizeof(ctx->last_abort_reason) - 1U);
	ctx->last_abort_reason[sizeof(ctx->last_abort_reason) - 1U] = '\0';

	return 0;
}

int motor_commission_start_flux(struct motor_parameters *params,
				const struct motor_commission_flux_config *cfg)
{
	if (params == NULL || cfg == NULL) {
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

	int ret = motor_commission_start_common(params, MOTOR_COMMISSION_MODE_FLUX,
						MOTOR_COMMISSION_EXPECT_VELOCITY_CLOSED,
						(uint32_t)total_ms);
	if (ret < 0) {
		return ret;
	}

	params->commission.flux_cfg = *cfg;
	params->commission.results.psi_f_valid = false;
	params->commission.results.psi_f_sample_count = 0U;
	params->commission.results.psi_f_wb = 0.0f;
	params->commission.results.psi_f_bias_v = 0.0f;
	params->commission.results.psi_f_residual_rms_v = 0.0f;
	params->commission.results.psi_f_r2 = 0.0f;
	return 0;
}

int motor_commission_start_mech(struct motor_parameters *params,
				const struct motor_commission_mech_config *cfg)
{
	if (params == NULL || cfg == NULL) {
		return -EINVAL;
	}
	if (!isfinite(cfg->coast_speed_hz) || !isfinite(cfg->prbs_amp_a)) {
		return -EINVAL;
	}
	if (cfg->duration_ms == 0U || cfg->prbs_period_ms == 0U ||
	    cfg->coast_speed_hz <= 0.0f || cfg->prbs_amp_a <= 0.0f) {
		return -EINVAL;
	}

	int ret = motor_commission_start_common(params, MOTOR_COMMISSION_MODE_MECH,
						MOTOR_COMMISSION_EXPECT_TORQUE,
						cfg->duration_ms);
	if (ret < 0) {
		return ret;
	}

	params->commission.mech_cfg = *cfg;
	params->commission.results.mech_valid = false;
	params->commission.results.mech_sample_count = 0U;
	params->commission.results.inertia_kgm2 = 0.0f;
	params->commission.results.viscous_friction_nm_per_rad_s = 0.0f;
	params->commission.results.coulomb_friction_nm = 0.0f;
	params->commission.results.offset_friction_nm = 0.0f;
	params->commission.results.mech_residual_rms_nm = 0.0f;
	params->commission.results.mech_r2 = 0.0f;
	return 0;
}

int motor_commission_apply_results(struct motor_parameters *params)
{
	if (params == NULL) {
		return -EINVAL;
	}

	const struct motor_commission_results *results = &params->commission.results;
	if (!results->psi_f_valid && !results->mech_valid) {
		return -ENOENT;
	}

	if (results->psi_f_valid) {
		params->flux_linkage_wb_active = results->psi_f_wb;
	}
	if (results->mech_valid) {
		params->inertia_kgm2_active = results->inertia_kgm2;
		params->viscous_friction_nm_per_rad_s_active =
			results->viscous_friction_nm_per_rad_s;
		params->coulomb_friction_nm_active = results->coulomb_friction_nm;
	}

	return 0;
}

int motor_commission_stage_auto_tune(struct motor_parameters *params,
				     const struct motor_commission_tune_config *cfg)
{
	if (params == NULL) {
		return -EINVAL;
	}

	struct motor_commission_ctx *ctx = &params->commission;
	const struct motor_commission_tune_config *cfg_sel = cfg;
	if (cfg_sel == NULL) {
		cfg_sel = &ctx->auto_tune_cfg;
	}

	if (cfg != NULL) {
		ctx->auto_tune_cfg = *cfg;
	}

	const struct motor_commission_results *results = &ctx->results;
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

	ctx->auto_tune_staged = (struct motor_commission_tune_output){0};
	int ret = motor_commission_tune_compute(&fit, cfg_sel, &ctx->auto_tune_staged);
	ctx->auto_tune_valid = (ret == 0) && ctx->auto_tune_staged.accepted;
	ctx->auto_tune_applied = false;
	ctx->auto_tune_last_error = ret;
	return ret;
}

int motor_commission_apply_staged_auto_tune(struct motor_parameters *params)
{
	if (params == NULL) {
		return -EINVAL;
	}

	struct motor_commission_ctx *ctx = &params->commission;
	const struct motor_commission_tune_output *staged = &ctx->auto_tune_staged;
	if (!ctx->auto_tune_valid || !staged->accepted) {
		return -ENOENT;
	}

	int ret = motor_commission_apply_results(params);
	if (ret < 0) {
		ctx->auto_tune_last_error = ret;
		return ret;
	}

	params->velocity_cl_kp_A_per_rad_s = staged->velocity_kp_a_per_rad_s;
	params->velocity_cl_ki_A_per_rad = staged->velocity_ki_a_per_rad;
	params->velocity_cl_iq_limit_A = staged->velocity_iq_limit_a;
	params->velocity_cl_i_term_A = 0.0f;

	params->position_cl_kp_rad_s_per_rad = staged->position_kp_rad_s_per_rad;
	params->position_cl_ki_rad_s2_per_rad = staged->position_ki_rad_s2_per_rad;
	params->position_cl_i_term_rad_s = 0.0f;

	params->velocity_mpr_cfg.horizon = staged->velocity_mpr_horizon;
	params->velocity_mpr_cfg.q_speed = staged->velocity_mpr_q_speed;
	params->velocity_mpr_cfg.r_delta_iq = staged->velocity_mpr_r_delta_iq;
	params->velocity_mpr_cfg.iq_limit_a = staged->velocity_iq_limit_a;
	params->velocity_mpr_cfg.max_delta_iq_a = staged->velocity_mpr_max_delta_iq_a;
	params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s =
		staged->velocity_mpr_disturbance_ki_nm_per_rad_s;

	params->position_mpr_cfg.horizon = staged->position_mpr_horizon;
	params->position_mpr_cfg.q_position = staged->position_mpr_q_position;
	params->position_mpr_cfg.q_velocity_ff = staged->position_mpr_q_velocity_ff;
	params->position_mpr_cfg.r_delta_velocity = staged->position_mpr_r_delta_velocity;
	params->position_mpr_cfg.max_delta_velocity_rad_s =
		staged->position_mpr_max_delta_velocity_rad_s;
	params->position_mpr_cfg.velocity_limit_rad_s = params->profile_max_velocity_rad_s;

	params->velocity_dob_cfg.enabled = staged->velocity_dob_enable;
	params->velocity_dob_cfg.observer_gain_nm_per_rad_s =
		staged->velocity_dob_observer_gain_nm_per_rad_s;
	params->velocity_dob_cfg.torque_limit_nm = staged->velocity_dob_torque_limit_nm;
	params->velocity_dob_cfg.iq_ff_limit_a = staged->velocity_dob_iq_ff_limit_a;

	motor_mpr_velocity_reset(&params->velocity_mpr_state, params->velocity_rad_s, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, params->position_rad);
	motor_dob_reset(&params->velocity_dob_state, params->velocity_rad_s);
	params->velocity_dob_iq_ff_a = 0.0f;
	params->velocity_dob_disturbance_nm = 0.0f;
	params->velocity_dob_residual_rad_s = 0.0f;

	ctx->auto_tune_applied = true;
	ctx->auto_tune_last_error = 0;
	return 0;
}

void motor_commission_update(struct motor_parameters *params,
			     const struct motor_commission_observation *obs)
{
	if (params == NULL || obs == NULL) {
		return;
	}

	struct motor_commission_ctx *ctx = &params->commission;
	if (!ctx->active || ctx->stage != MOTOR_COMMISSION_STAGE_RUNNING) {
		return;
	}

	if ((int32_t)(obs->control_loop_count - ctx->stop_loop_count) >= 0) {
		ctx->active = false;
		ctx->stage = MOTOR_COMMISSION_STAGE_COMPLETED;
		motor_commission_finalize(params);
		return;
	}

	ctx->sample_decimation_counter++;
	if (ctx->sample_decimation_counter < ctx->sample_decimation) {
		return;
	}
	ctx->sample_decimation_counter = 0U;

	if (!obs->data_valid) {
		ctx->rejected_samples++;
		ctx->reject_data_invalid++;
		return;
	}
	if (!motor_commission_state_matches_expected(obs->state, ctx->expected_mode)) {
		ctx->rejected_samples++;
		ctx->reject_mode_mismatch++;
		return;
	}
	if (!obs->control_armed) {
		ctx->rejected_samples++;
		ctx->reject_disarmed++;
		return;
	}
	if (obs->fault_active) {
		ctx->rejected_samples++;
		ctx->reject_fault++;
		return;
	}
	if (!obs->encoder_fresh || obs->encoder_warning || obs->encoder_error) {
		ctx->rejected_samples++;
		ctx->reject_encoder++;
		return;
	}
	if (obs->saturation) {
		ctx->rejected_samples++;
		ctx->reject_saturation++;
		return;
	}
	if (ctx->sample_count >= MOTOR_COMMISSION_MAX_SAMPLES) {
		motor_commission_abort(params, "capture buffer full");
		return;
	}

	float32_t did_dt = 0.0f;
	float32_t diq_dt = 0.0f;
	float32_t domega_dt = 0.0f;

	if (ctx->prev_valid) {
		uint32_t delta_cycles = obs->control_loop_count - ctx->prev_loop_count;
		if (delta_cycles == 0U) {
			delta_cycles = 1U;
		}
		float32_t dt_s = (float32_t)delta_cycles / CONTROL_LOOP_FREQUENCY_HZ;
		did_dt = (obs->id_a - ctx->prev_id_a) / dt_s;
		diq_dt = (obs->iq_a - ctx->prev_iq_a) / dt_s;
		domega_dt = (obs->mech_speed_rad_s - ctx->prev_speed_rad_s) / dt_s;
	}

	ctx->did_dt_filt_a_s += MOTOR_COMMISSION_DERIV_ALPHA * (did_dt - ctx->did_dt_filt_a_s);
	ctx->diq_dt_filt_a_s += MOTOR_COMMISSION_DERIV_ALPHA * (diq_dt - ctx->diq_dt_filt_a_s);
	ctx->domega_dt_filt_rad_s2 +=
		MOTOR_COMMISSION_DERIV_ALPHA * (domega_dt - ctx->domega_dt_filt_rad_s2);

	struct motor_commission_sample *sample = &ctx->samples[ctx->sample_count++];
	sample->loop_count = obs->control_loop_count;
	sample->mech_speed_rad_s = obs->mech_speed_rad_s;
	sample->elec_speed_rad_s = obs->elec_speed_rad_s;
	sample->mech_accel_rad_s2 = ctx->domega_dt_filt_rad_s2;
	sample->id_a = obs->id_a;
	sample->iq_a = obs->iq_a;
	sample->did_dt_a_s = ctx->did_dt_filt_a_s;
	sample->diq_dt_a_s = ctx->diq_dt_filt_a_s;
	sample->vd_v = obs->vd_v;
	sample->vq_v = obs->vq_v;
	sample->vbus_v = obs->vbus_v;
	sample->encoder_status = obs->encoder_status;
	sample->flags = obs->saturation ? MOTOR_COMMISSION_FLAG_SATURATED : 0U;

	ctx->accepted_samples++;
	ctx->prev_valid = true;
	ctx->prev_loop_count = obs->control_loop_count;
	ctx->prev_id_a = obs->id_a;
	ctx->prev_iq_a = obs->iq_a;
	ctx->prev_speed_rad_s = obs->mech_speed_rad_s;
}
