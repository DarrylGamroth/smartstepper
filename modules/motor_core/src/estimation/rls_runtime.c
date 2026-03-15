/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/estimation/rls_runtime.h"

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION

#include <math.h>

float32_t motor_rls_prepare_id_reference(struct motor_rls_runtime_ctx *ctx,
					 bool online_control_state,
					 bool control_armed,
					 bool fresh_encoder_sample,
					 bool encoder_frame_error,
					 uint8_t encoder_input_source,
					 float32_t id_ref_a,
					 struct motor_rls_runtime_state *runtime_state)
{
	if (ctx == NULL || runtime_state == NULL) {
		return id_ref_a;
	}

	runtime_state->rls_mask = ctx->rls.decimation - 1U;

	runtime_state->rls_control_enabled =
		ctx->rls_feature_enabled && online_control_state && control_armed;
	bool rls_encoder_ok = fresh_encoder_sample && !encoder_frame_error &&
			      (encoder_input_source == 1U);
	runtime_state->rls_runtime_enabled =
		runtime_state->rls_control_enabled && rls_encoder_ok;

	if (runtime_state->rls_runtime_enabled &&
	    (ctx->control_loop_count & runtime_state->rls_mask) == 0U) {
		uint32_t prbs_bit = prbs_advance(ctx->rls.prbs_gen);
		float32_t i_prbs_d =
			(2.0f * (float32_t)prbs_bit - 1.0f) * ctx->rls.excitation_current_a;
		id_ref_a += i_prbs_d;
	}

	return id_ref_a;
}

void motor_rls_update_estimators(struct motor_rls_runtime_ctx *ctx,
				 const struct motor_rls_runtime_state *runtime_state,
				 float32_t id_a,
				 float32_t iq_a)
{
	if (ctx == NULL || runtime_state == NULL) {
		return;
	}

	/* D-axis RLS parameter estimation with comprehensive gating */
	if (runtime_state->rls_runtime_enabled &&
	    (ctx->control_loop_count & runtime_state->rls_mask) == 0U) {
		/* Use previously applied d-axis voltage to align voltage/current timing. */
		float32_t vd_applied_v = ctx->vd_v;
		float32_t vd_abs = fabsf(vd_applied_v);
		float32_t omega_elec = angle_observer_get_elec_speed(ctx->observer);

		bool voltage_ok = (vd_abs < ctx->rls.max_voltage_v);
		bool pi_ok = (fabsf(vd_applied_v - pi_get_out_max(ctx->pi_id)) > 0.1f) &&
			     (fabsf(vd_applied_v - pi_get_out_min(ctx->pi_id)) > 0.1f);
		bool speed_ok = (fabsf(omega_elec) > ctx->rls.min_speed_rad_s);
		bool residual_ok = (ctx->rls.max_residual_v <= 0.0f) ||
				   (ctx->rls.d->num_updates == 0U) ||
				   (fabsf(ctx->rls.d->residual) <= ctx->rls.max_residual_v);

		if (voltage_ok && pi_ok && speed_ok && residual_ok) {
			if (*ctx->rls.d_prev_valid == 0U) {
				*ctx->rls.id_prev_a = id_a;
				*ctx->rls.d_prev_cycle = ctx->control_loop_count;
				*ctx->rls.d_prev_valid = 1U;
			} else {
				uint32_t sample_cycles =
					ctx->control_loop_count - *ctx->rls.d_prev_cycle;
				if (sample_cycles == 0U) {
					sample_cycles = 1U;
				}
				float32_t sample_period_s =
					(float32_t)sample_cycles / ctx->control_loop_frequency_hz;

				rls_motor_est_update(ctx->rls.d, vd_applied_v, id_a,
						     *ctx->rls.id_prev_a, omega_elec,
						     *ctx->rls.lq_est_h, iq_a, sample_period_s);
				*ctx->rls.id_prev_a = id_a;
				*ctx->rls.d_prev_cycle = ctx->control_loop_count;
			}
		}
	}

	/* Q-axis RLS parameter estimation (staggered by offset for load spreading) */
	const uint32_t rls_offset = ctx->rls.stagger_offset;
	if (runtime_state->rls_runtime_enabled &&
	    ((ctx->control_loop_count & runtime_state->rls_mask) == rls_offset)) {
		float32_t omega_elec = angle_observer_get_elec_speed(ctx->observer);
		float32_t v_bemf = omega_elec * ctx->default_flux_linkage_wb;
		/* Use previously applied q-axis voltage to align voltage/current timing. */
		float32_t vq_applied_v = ctx->vq_v;
		float32_t vq_compensated = vq_applied_v - v_bemf;

		float32_t iq_abs = fabsf(iq_a);
		float32_t vq_comp_abs = fabsf(vq_compensated);

		bool current_ok = (iq_abs > ctx->rls.min_current_a);
		bool voltage_ok = (vq_comp_abs < ctx->rls.max_voltage_v);
		bool pi_ok = (fabsf(vq_applied_v - pi_get_out_max(ctx->pi_iq)) > 0.1f) &&
			     (fabsf(vq_applied_v - pi_get_out_min(ctx->pi_iq)) > 0.1f);
		bool speed_ok = (fabsf(omega_elec) > ctx->rls.min_speed_rad_s);
		bool residual_ok = (ctx->rls.max_residual_v <= 0.0f) ||
				   (ctx->rls.q->num_updates == 0U) ||
				   (fabsf(ctx->rls.q->residual) <= ctx->rls.max_residual_v);

		if (current_ok && voltage_ok && pi_ok && speed_ok && residual_ok) {
			if (*ctx->rls.q_prev_valid == 0U) {
				*ctx->rls.iq_prev_a = iq_a;
				*ctx->rls.q_prev_cycle = ctx->control_loop_count;
				*ctx->rls.q_prev_valid = 1U;
			} else {
				uint32_t sample_cycles =
					ctx->control_loop_count - *ctx->rls.q_prev_cycle;
				if (sample_cycles == 0U) {
					sample_cycles = 1U;
				}
				float32_t sample_period_s =
					(float32_t)sample_cycles / ctx->control_loop_frequency_hz;

				/* Q-axis: pass -omega so RLS subtracts cross-coupling ω·Ld·Id. */
				rls_motor_est_update(ctx->rls.q, vq_compensated, iq_a,
						     *ctx->rls.iq_prev_a, -omega_elec,
						     *ctx->rls.ld_est_h, id_a, sample_period_s);
				*ctx->rls.iq_prev_a = iq_a;
				*ctx->rls.q_prev_cycle = ctx->control_loop_count;
			}
		}

		if (rls_motor_est_is_converged(ctx->rls.d) &&
		    rls_motor_est_is_converged(ctx->rls.q)) {
			*ctx->rls.ld_est_h = rls_motor_est_get_L(ctx->rls.d);
			*ctx->rls.lq_est_h = rls_motor_est_get_L(ctx->rls.q);

			float32_t rs_d = rls_motor_est_get_Rs(ctx->rls.d);
			float32_t rs_q = rls_motor_est_get_Rs(ctx->rls.q);
			*ctx->rs_measured_ohm = (rs_d + rs_q) * 0.5f;

			*ctx->thermal.t_rls_c = thermal_Rs_to_temperature(*ctx->rs_measured_ohm,
									  ctx->thermal.rs_ref_ohm,
									  ctx->thermal.rs_ref_temp_c,
									  ctx->thermal.rs_temp_coeff);
		}
	}

	/* Thermal model update (heavily decimated, ~10Hz). */
	const uint32_t thermal_mask = ctx->thermal.decimation - 1U;
	if (runtime_state->rls_control_enabled &&
	    (ctx->control_loop_count & thermal_mask) == 0U) {
		thermal_model_update(ctx->thermal.model, id_a, iq_a, *ctx->rs_measured_ohm);
	}
}

#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */
