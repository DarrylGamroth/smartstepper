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
#include "motor_state_utils.h"
#include "motor_states.h"

#define MOTOR_COMMISSION_DERIV_ALPHA 0.2f
#define MOTOR_COMMISSION_FLAG_SATURATED BIT(0)
#define MOTOR_COMMISSION_MIN_FLUX_SAMPLES 16U
#define MOTOR_COMMISSION_MIN_MECH_SAMPLES 32U
#define MOTOR_COMMISSION_MIN_SPEED_RAD_S (2.0f * PI_F32)
#define MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S 0.5f
#define MOTOR_COMMISSION_MIN_KT_NM_PER_A 1.0e-6f

static bool motor_commission_solve_4x4(float32_t A[4][4], float32_t b[4], float32_t x[4])
{
	float32_t aug[4][5];

	for (uint32_t i = 0U; i < 4U; i++) {
		for (uint32_t j = 0U; j < 4U; j++) {
			aug[i][j] = A[i][j];
		}
		aug[i][4] = b[i];
	}

	for (uint32_t col = 0U; col < 4U; col++) {
		uint32_t pivot = col;
		float32_t pivot_abs = fabsf(aug[pivot][col]);

		for (uint32_t row = col + 1U; row < 4U; row++) {
			float32_t a = fabsf(aug[row][col]);
			if (a > pivot_abs) {
				pivot = row;
				pivot_abs = a;
			}
		}

		if (pivot_abs < 1.0e-9f) {
			return false;
		}

		if (pivot != col) {
			for (uint32_t j = col; j < 5U; j++) {
				float32_t tmp = aug[col][j];
				aug[col][j] = aug[pivot][j];
				aug[pivot][j] = tmp;
			}
		}

		float32_t inv_pivot = 1.0f / aug[col][col];
		for (uint32_t j = col; j < 5U; j++) {
			aug[col][j] *= inv_pivot;
		}

		for (uint32_t row = 0U; row < 4U; row++) {
			if (row == col) {
				continue;
			}

			float32_t f = aug[row][col];
			if (f == 0.0f) {
				continue;
			}

			for (uint32_t j = col; j < 5U; j++) {
				aug[row][j] -= f * aug[col][j];
			}
		}
	}

	for (uint32_t i = 0U; i < 4U; i++) {
		x[i] = aug[i][4];
	}

	return true;
}

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
	float32_t rs = params->Rs_measured_ohm;
	float32_t ld = params->Ld_est;
	float32_t lq = params->Lq_est;
	float32_t sx = 0.0f;
	float32_t sy = 0.0f;
	float32_t sxx = 0.0f;
	float32_t sxy = 0.0f;
	float32_t min_x = 1.0e30f;
	float32_t max_x = -1.0e30f;
	uint32_t n = 0U;

	res->psi_f_valid = false;
	res->psi_f_sample_count = 0U;
	res->psi_f_bias_v = 0.0f;
	res->psi_f_residual_rms_v = 0.0f;
	res->psi_f_r2 = 0.0f;

	for (uint32_t i = 0U; i < ctx->sample_count; i++) {
		const struct motor_commission_sample *s = &ctx->samples[i];
		float32_t x = s->elec_speed_rad_s;

		if (!isfinite(x) || !isfinite(s->id_a) || !isfinite(s->iq_a) ||
		    !isfinite(s->diq_dt_a_s) || !isfinite(s->vq_v)) {
			continue;
		}
		if (fabsf(x) < MOTOR_COMMISSION_MIN_SPEED_RAD_S) {
			continue;
		}

		float32_t y = s->vq_v - rs * s->iq_a - lq * s->diq_dt_a_s - x * ld * s->id_a;
		sx += x;
		sy += y;
		sxx += x * x;
		sxy += x * y;
		min_x = MIN(min_x, x);
		max_x = MAX(max_x, x);
		n++;
	}

	res->psi_f_sample_count = (uint16_t)MIN(n, UINT16_MAX);
	if (n < MOTOR_COMMISSION_MIN_FLUX_SAMPLES ||
	    (max_x - min_x) < (2.0f * MOTOR_COMMISSION_MIN_SPEED_RAD_S)) {
		return;
	}

	float32_t n_f = (float32_t)n;
	float32_t den = n_f * sxx - sx * sx;
	if (fabsf(den) < 1.0e-8f) {
		return;
	}

	float32_t psi_f = (n_f * sxy - sx * sy) / den;
	float32_t bias = (sy - psi_f * sx) / n_f;
	float32_t y_mean = sy / n_f;
	float32_t sse = 0.0f;
	float32_t sst = 0.0f;

	for (uint32_t i = 0U; i < ctx->sample_count; i++) {
		const struct motor_commission_sample *s = &ctx->samples[i];
		float32_t x = s->elec_speed_rad_s;

		if (fabsf(x) < MOTOR_COMMISSION_MIN_SPEED_RAD_S) {
			continue;
		}

		float32_t y = s->vq_v - rs * s->iq_a - lq * s->diq_dt_a_s - x * ld * s->id_a;
		float32_t y_hat = psi_f * x + bias;
		float32_t e = y - y_hat;
		float32_t d = y - y_mean;
		sse += e * e;
		sst += d * d;
	}

	res->psi_f_wb = psi_f;
	res->psi_f_bias_v = bias;
	res->psi_f_residual_rms_v = sqrtf(sse / n_f);
	res->psi_f_r2 = (sst > 1.0e-8f) ? (1.0f - sse / sst) : 0.0f;
	res->psi_f_valid =
		isfinite(psi_f) && psi_f > 0.0f && isfinite(res->psi_f_r2) && res->psi_f_r2 > 0.2f;
}

static void motor_commission_estimate_mech(struct motor_parameters *params)
{
	struct motor_commission_ctx *ctx = &params->commission;
	struct motor_commission_results *res = &ctx->results;
	float32_t psi_f = res->psi_f_valid ? res->psi_f_wb : params->flux_linkage_wb_active;
	float32_t kt = 1.5f * (float32_t)MOTOR_POLE_PAIRS * psi_f;
	float32_t A[4][4] = {0};
	float32_t b[4] = {0};
	float32_t theta[4] = {0};
	uint32_t n = 0U;
	float32_t sum_z = 0.0f;
	float32_t sum_z2 = 0.0f;

	res->mech_valid = false;
	res->mech_sample_count = 0U;
	res->mech_residual_rms_nm = 0.0f;
	res->mech_r2 = 0.0f;

	if (!isfinite(kt) || fabsf(kt) < MOTOR_COMMISSION_MIN_KT_NM_PER_A) {
		return;
	}

	for (uint32_t i = 0U; i < ctx->sample_count; i++) {
		const struct motor_commission_sample *s = &ctx->samples[i];

		if (!isfinite(s->mech_speed_rad_s) || !isfinite(s->mech_accel_rad_s2) ||
		    !isfinite(s->iq_a)) {
			continue;
		}

		float32_t sign_term = 0.0f;
		if (s->mech_speed_rad_s > MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S) {
			sign_term = 1.0f;
		} else if (s->mech_speed_rad_s < -MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S) {
			sign_term = -1.0f;
		}

		float32_t phi[4] = {
			s->mech_accel_rad_s2,
			s->mech_speed_rad_s,
			sign_term,
			1.0f,
		};
		float32_t z = kt * s->iq_a;

		for (uint32_t r = 0U; r < 4U; r++) {
			b[r] += phi[r] * z;
			for (uint32_t c = 0U; c < 4U; c++) {
				A[r][c] += phi[r] * phi[c];
			}
		}

		sum_z += z;
		sum_z2 += z * z;
		n++;
	}

	res->mech_sample_count = (uint16_t)MIN(n, UINT16_MAX);
	if (n < MOTOR_COMMISSION_MIN_MECH_SAMPLES) {
		return;
	}
	if (!motor_commission_solve_4x4(A, b, theta)) {
		return;
	}

	float32_t sse = 0.0f;
	float32_t mean_z = sum_z / (float32_t)n;
	float32_t sst = sum_z2 - (float32_t)n * mean_z * mean_z;

	for (uint32_t i = 0U; i < ctx->sample_count; i++) {
		const struct motor_commission_sample *s = &ctx->samples[i];
		float32_t sign_term = 0.0f;
		if (s->mech_speed_rad_s > MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S) {
			sign_term = 1.0f;
		} else if (s->mech_speed_rad_s < -MOTOR_COMMISSION_SIGN_DEADBAND_RAD_S) {
			sign_term = -1.0f;
		}
		float32_t phi[4] = {
			s->mech_accel_rad_s2,
			s->mech_speed_rad_s,
			sign_term,
			1.0f,
		};
		float32_t z = kt * s->iq_a;
		float32_t z_hat = theta[0] * phi[0] + theta[1] * phi[1] + theta[2] * phi[2] +
				  theta[3] * phi[3];
		float32_t e = z - z_hat;
		sse += e * e;
	}

	res->inertia_kgm2 = theta[0];
	res->viscous_friction_nm_per_rad_s = theta[1];
	res->coulomb_friction_nm = fabsf(theta[2]);
	res->offset_friction_nm = theta[3];
	res->mech_residual_rms_nm = sqrtf(sse / (float32_t)n);
	res->mech_r2 = (sst > 1.0e-8f) ? (1.0f - sse / sst) : 0.0f;
	res->mech_valid = isfinite(res->inertia_kgm2) && res->inertia_kgm2 > 0.0f &&
			  isfinite(res->viscous_friction_nm_per_rad_s) &&
			  res->viscous_friction_nm_per_rad_s >= 0.0f && isfinite(res->mech_r2) &&
			  res->mech_r2 > 0.0f;
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
