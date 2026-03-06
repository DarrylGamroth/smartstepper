/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/estimation/rls_runtime.h"

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION

#include <math.h>

#include <zephyr/sys/atomic.h>

#include "config.h"

float32_t motor_rls_prepare_id_reference(struct motor_parameters *params,
					 bool online_control_state,
					 bool control_armed,
					 bool fresh_encoder_sample,
					 bool encoder_frame_error,
					 uint8_t encoder_input_source,
					 float32_t id_ref_a,
					 struct motor_rls_runtime_state *runtime_state)
{
	if (params == NULL || runtime_state == NULL) {
		return id_ref_a;
	}

	runtime_state->rls_mask = params->rls.decimation - 1U;

	bool rls_feature_enabled =
		atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION);
	runtime_state->rls_control_enabled =
		rls_feature_enabled && online_control_state && control_armed;
	bool rls_encoder_ok = fresh_encoder_sample && !encoder_frame_error &&
			      (encoder_input_source == MOTOR_ANGLE_INPUT_SRC_ENCODER);
	runtime_state->rls_runtime_enabled =
		runtime_state->rls_control_enabled && rls_encoder_ok;

	if (runtime_state->rls_runtime_enabled &&
	    (params->control_loop_count & runtime_state->rls_mask) == 0U) {
		uint32_t prbs_bit = prbs_advance(&params->rls.prbs_gen);
		float32_t i_prbs_d =
			(2.0f * (float32_t)prbs_bit - 1.0f) * params->rls.excitation_current_a;
		id_ref_a += i_prbs_d;
	}

	return id_ref_a;
}

void motor_rls_update_estimators(struct motor_parameters *params,
				 const struct motor_rls_runtime_state *runtime_state,
				 float32_t id_a,
				 float32_t iq_a)
{
	if (params == NULL || runtime_state == NULL) {
		return;
	}

	/* D-axis RLS parameter estimation with comprehensive gating */
	if (runtime_state->rls_runtime_enabled &&
	    (params->control_loop_count & runtime_state->rls_mask) == 0U) {
		/* Use previously applied d-axis voltage to align voltage/current timing. */
		float32_t vd_applied_v = params->Vd_V;
		float32_t vd_abs = fabsf(vd_applied_v);
		float32_t omega_elec = angle_observer_get_elec_speed(&params->observer);

		bool voltage_ok = (vd_abs < params->rls.max_voltage_v);
		bool pi_ok = (fabsf(vd_applied_v - pi_get_out_max(&params->pi_Id)) > 0.1f) &&
			     (fabsf(vd_applied_v - pi_get_out_min(&params->pi_Id)) > 0.1f);
		bool speed_ok = (fabsf(omega_elec) > params->rls.min_speed_rad_s);
		bool residual_ok = (params->rls.max_residual_v <= 0.0f) ||
				   (params->rls.d.num_updates == 0U) ||
				   (fabsf(params->rls.d.residual) <= params->rls.max_residual_v);

		if (voltage_ok && pi_ok && speed_ok && residual_ok) {
			if (params->rls.d_prev_valid == 0U) {
				params->rls.id_prev_a = id_a;
				params->rls.d_prev_cycle = params->control_loop_count;
				params->rls.d_prev_valid = 1U;
			} else {
				uint32_t sample_cycles =
					params->control_loop_count - params->rls.d_prev_cycle;
				if (sample_cycles == 0U) {
					sample_cycles = 1U;
				}
				float32_t sample_period_s =
					(float32_t)sample_cycles / CONTROL_LOOP_FREQUENCY_HZ;

				rls_motor_est_update(&params->rls.d, vd_applied_v, id_a,
						     params->rls.id_prev_a, omega_elec,
						     params->rls.lq_est_h, iq_a, sample_period_s);
				params->rls.id_prev_a = id_a;
				params->rls.d_prev_cycle = params->control_loop_count;
			}
		}
	}

	/* Q-axis RLS parameter estimation (staggered by offset for load spreading) */
	const uint32_t rls_offset = params->rls.stagger_offset;
	if (runtime_state->rls_runtime_enabled &&
	    ((params->control_loop_count & runtime_state->rls_mask) == rls_offset)) {
		float32_t omega_elec = angle_observer_get_elec_speed(&params->observer);
		float32_t v_bemf = omega_elec * MOTOR_FLUX_LINKAGE_WB;
		/* Use previously applied q-axis voltage to align voltage/current timing. */
		float32_t vq_applied_v = params->Vq_V;
		float32_t vq_compensated = vq_applied_v - v_bemf;

		float32_t iq_abs = fabsf(iq_a);
		float32_t vq_comp_abs = fabsf(vq_compensated);

		bool current_ok = (iq_abs > params->rls.min_current_a);
		bool voltage_ok = (vq_comp_abs < params->rls.max_voltage_v);
		bool pi_ok = (fabsf(vq_applied_v - pi_get_out_max(&params->pi_Iq)) > 0.1f) &&
			     (fabsf(vq_applied_v - pi_get_out_min(&params->pi_Iq)) > 0.1f);
		bool speed_ok = (fabsf(omega_elec) > params->rls.min_speed_rad_s);
		bool residual_ok = (params->rls.max_residual_v <= 0.0f) ||
				   (params->rls.q.num_updates == 0U) ||
				   (fabsf(params->rls.q.residual) <= params->rls.max_residual_v);

		if (current_ok && voltage_ok && pi_ok && speed_ok && residual_ok) {
			if (params->rls.q_prev_valid == 0U) {
				params->rls.iq_prev_a = iq_a;
				params->rls.q_prev_cycle = params->control_loop_count;
				params->rls.q_prev_valid = 1U;
			} else {
				uint32_t sample_cycles =
					params->control_loop_count - params->rls.q_prev_cycle;
				if (sample_cycles == 0U) {
					sample_cycles = 1U;
				}
				float32_t sample_period_s =
					(float32_t)sample_cycles / CONTROL_LOOP_FREQUENCY_HZ;

				/* Q-axis: pass -omega so RLS subtracts cross-coupling ω·Ld·Id. */
				rls_motor_est_update(&params->rls.q, vq_compensated, iq_a,
						     params->rls.iq_prev_a, -omega_elec,
						     params->rls.ld_est_h, id_a, sample_period_s);
				params->rls.iq_prev_a = iq_a;
				params->rls.q_prev_cycle = params->control_loop_count;
			}
		}

		if (rls_motor_est_is_converged(&params->rls.d) &&
		    rls_motor_est_is_converged(&params->rls.q)) {
			params->rls.ld_est_h = rls_motor_est_get_L(&params->rls.d);
			params->rls.lq_est_h = rls_motor_est_get_L(&params->rls.q);

			float32_t rs_d = rls_motor_est_get_Rs(&params->rls.d);
			float32_t rs_q = rls_motor_est_get_Rs(&params->rls.q);
			params->Rs_measured_ohm = (rs_d + rs_q) * 0.5f;

			params->thermal.t_rls_c = thermal_Rs_to_temperature(params->Rs_measured_ohm,
							     params->thermal.rs_ref_ohm,
							     params->thermal.rs_ref_temp_c,
							     params->thermal.rs_temp_coeff);
		}
	}

	/* Thermal model update (heavily decimated, ~10Hz). */
	const uint32_t thermal_mask = params->thermal.decimation - 1U;
	if (runtime_state->rls_control_enabled &&
	    (params->control_loop_count & thermal_mask) == 0U) {
		thermal_model_update(&params->thermal.model, id_a, iq_a, params->Rs_measured_ohm);
	}
}

#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */
