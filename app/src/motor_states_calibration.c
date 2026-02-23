/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "motor_states.h"
#include "motor_states_calibration.h"
#include "motor_control_api.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "angle_observer.h"
#include "angle_gen.h"
#include "angle_wrap.h"
#include "motor_state_utils.h"

LOG_MODULE_DECLARE(motor_states, CONFIG_APP_LOG_LEVEL);

static inline void motor_enable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next |= mask;
}

static inline void motor_disable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next &= ~mask;
}

static inline bool motor_calibration_timeout_elapsed(struct motor_parameters *params)
{
	if (params->event.type == MOTOR_EVENT_TIMEOUT) {
		/* Drain timer status so stale expiries do not trigger the next state immediately. */
		(void)k_timer_status_get(&params->state_timer);
		return true;
	}

	if (k_timer_status_get(&params->state_timer) > 0U) {
		LOG_WRN("State timer expired without queued timeout event; proceeding");
		return true;
	}

	return false;
}

static inline enum motor_state motor_resolve_requested_online_mode(const struct motor_parameters *params)
{
	enum motor_state mode = MOTOR_STATE_ONLINE_VELOCITY_OPEN;

	if (params != NULL) {
		mode = (enum motor_state)params->requested_online_mode;
	}

	if (!motor_state_is_online_submode(mode)) {
		mode = MOTOR_STATE_ONLINE_VELOCITY_OPEN;
	}

	return mode;
}

/* State: CALIBRATION - Hierarchical parent state for all calibration sub-states */
void motor_state_calibration_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	const bool commissioning =
		(params->calibration_mode == MOTOR_CALIBRATION_MODE_COMMISSIONING);

	LOG_INF("=== Starting %s Sequence ===",
		commissioning ? "Motor Commissioning" : "Motor Calibration");

	params->calibration_running = true;

	if (!commissioning) {
		params->calibration_complete = false;
	}

	params->commissioning_complete = false;
}

enum smf_state_result motor_state_calibration_run(void *obj)
{
	/* Parent state run - events propagate to child states.
	 * Initial transition to OFFSET_MEAS handled by SMF engine.
	 */
	return SMF_EVENT_PROPAGATE;
}

void motor_state_calibration_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	const bool commissioning =
		(params->calibration_mode == MOTOR_CALIBRATION_MODE_COMMISSIONING);
	const bool failed = (params->event.type == MOTOR_EVENT_ERROR);

	params->calibration_running = false;
	if (failed) {
		LOG_WRN("=== %s Aborted (error) ===",
			commissioning ? "Commissioning" : "Calibration");
		params->calibration_complete = false;
		params->commissioning_complete = false;
	} else {
		LOG_INF("=== %s Complete ===",
			commissioning ? "Commissioning" : "Calibration");
		LOG_INF("  Rs:    %.4f Ω", (double)params->Rs_measured_ohm);
		LOG_INF("  Ls:    %.6f H", (double)params->Ls_measured_H);
		LOG_INF("  R/L:   %.1f rad/s", (double)params->R_over_L_measured);
		params->calibration_complete = true;
		if (commissioning) {
			params->commissioning_complete = true;
		}
	}
	params->calibration_mode = MOTOR_CALIBRATION_MODE_BOOT;
}

/* State: OFFSET_MEAS - Measure current sensor offsets */
void motor_state_offset_meas_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering OFFSET_MEAS state");

	params->Ia_offset = 0.0f;
	params->Ib_offset = 0.0f;

	filter_fo_set_initial_conditions(&params->filter_Ia, 0.0f, 0.0f);
	filter_fo_set_initial_conditions(&params->filter_Ib, 0.0f, 0.0f);

	/* Start timer for offset measurement duration (1 second) */
	k_timer_start(&params->state_timer, K_SECONDS(1), K_NO_WAIT);
}

void motor_state_offset_meas_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting OFFSET_MEAS state");

	/* Get filtered offset values */
	params->Ia_offset = filter_fo_get_y1(&params->filter_Ia);
	params->Ib_offset = filter_fo_get_y1(&params->filter_Ib);

	LOG_INF("Offset measurement complete: Ia=%.4f, Ib=%.4f",
		(double)params->Ia_offset, (double)params->Ib_offset);
}

enum smf_state_result motor_state_offset_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_timeout_elapsed(params)) {
		/* Offset measurement complete */
		if (params->calibration_mode == MOTOR_CALIBRATION_MODE_COMMISSIONING) {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ROVERL_MEAS]);
		} else {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN]);
		}
		return SMF_EVENT_HANDLED;
	}

	/* Propagate unhandled events */
	return SMF_EVENT_PROPAGATE;
}

/* State: ROVERL_MEAS - Measure R/L time constant via sinusoidal excitation (TI method) */
void motor_state_roverl_meas_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ROVERL_MEAS state");

	/* Additional ROVERL_MEAS requirements (PWM output is provided by OFFLINE). */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));

	/* TI R/L uses small sinusoidal current excitation
	 * Target: 10-20% of rated current
	 * Method: Apply rotating voltage vector, measure phase lag
	 *
	 * Voltage equation: V = R*I + L*dI/dt
	 * For I = I0*sin(ωt):
	 *   V_in_phase = R*I0        → gives R
	 *   V_quadrature = ωL*I0     → gives L
	 *   Time constant: τ = L/R
	 */

	/* Configure trajectory to ramp rotating current amplitude smoothly
	 * This allows PI controllers to settle before measurements begin
	 * Trajectory continues from previous state's current value for smooth transition
	 */
	traj_set_target_value(&params->traj_Id, ROVERL_EST_CURRENT_A);
	float32_t roverl_ramp_rate = ROVERL_EST_CURRENT_A / (ROVERL_EST_SETTLING_S * CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_max_delta(&params->traj_Id, roverl_ramp_rate);

	/* Reset accumulators and initialize angle generator */
	params->roverl_accumulator_Vd_Id = 0.0f;
	params->roverl_accumulator_Vq_Id = 0.0f;
	params->roverl_accumulator_Id2 = 0.0f;
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	/* Convert electrical frequency to mechanical velocity: omega_mech = omega_elec / pole_pairs */
	float32_t omega_rad_s = (ROVERL_EST_FREQ_HZ * 2.0f * PI_F32) / (float32_t)MOTOR_POLE_PAIRS;
	angle_gen_set_velocity(&params->angle_gen, omega_rad_s);

	LOG_INF("RoverL: %.0fHz excitation for %.1fs (I_amplitude=%.3fA)",
		(double)ROVERL_EST_FREQ_HZ, (double)ROVERL_EST_DURATION_S,
		(double)ROVERL_EST_CURRENT_A);

	/* Start timer for total R/L estimation duration */
	k_timer_start(&params->state_timer, K_MSEC((uint32_t)(ROVERL_EST_DURATION_S * 1000)), K_NO_WAIT);
}

enum smf_state_result motor_state_roverl_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_timeout_elapsed(params)) {
		if (params->roverl_accumulator_Id2 <= 1e-9f) {
			LOG_ERR("RoverL failed: insufficient excitation (sum(Id^2)=%.3e)",
				(double)params->roverl_accumulator_Id2);
			params->event.type = MOTOR_EVENT_ERROR;
			params->event.error_code = ERROR_HARDWARE_BREAK;
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
			return SMF_EVENT_HANDLED;
		}

		/* Extract R and ωL from accumulated phase components
		 *
		 * From least-squares fit of V = R*I + jωL*I:
		 *   R = sum(Vd*Id) / sum(Id^2)
		 *   ωL = sum(Vq*Id) / sum(Id^2)
		 *   L = (ωL) / ω
		 *   τ = L/R
		 */

		float32_t R_est = params->roverl_accumulator_Vd_Id / params->roverl_accumulator_Id2;
		float32_t omega_L_est = params->roverl_accumulator_Vq_Id / params->roverl_accumulator_Id2;
		float32_t omega_roverl = 2.0f * PI_F32 * ROVERL_EST_FREQ_HZ;
		float32_t L_est = omega_L_est / omega_roverl;

		if (fabsf(R_est) <= 1e-9f || fabsf(L_est) <= 1e-9f) {
			LOG_ERR("RoverL failed: invalid estimate (R=%.4e, L=%.4e)",
				(double)R_est, (double)L_est);
			params->event.type = MOTOR_EVENT_ERROR;
			params->event.error_code = ERROR_HARDWARE_BREAK;
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
			return SMF_EVENT_HANDLED;
		}

		float32_t RoverL = R_est / L_est;
		float32_t tau = L_est / R_est;

		LOG_INF("RoverL complete: R=%.4f Ω, L=%.2f µH, R/L=%.0f, τ=%.2f ms",
			(double)R_est, (double)(L_est * 1e6f),
			(double)RoverL, (double)(tau * 1000.0f));

		/* Store measured values */
		params->Rs_measured_ohm = R_est;
		params->Ls_measured_H = L_est;
		params->R_over_L_measured = RoverL;

		/* TODO: Use R/L to calculate initial PI current controller gains:
		 * Kp = bandwidth * L
		 * Ki = R/L * Ts
		 * This will be done when PI controllers are reconfigured
		 */

		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_RS_EST]);
		return SMF_EVENT_HANDLED;
	}

	/* Propagate unhandled events */
	return SMF_EVENT_PROPAGATE;
}

void motor_state_roverl_meas_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ROVERL_MEAS state");

	/* Reset angle generator */
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);

	/* Clear this state's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: RS_EST - Measure stator resistance via DC injection (TI method) */
void motor_state_rs_est_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering RS_EST state");

	/* Additional RS_EST requirements (PWM output is provided by OFFLINE). */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));

	/* Rs EST uses DC current injection on d-axis
	 * Two phases:
	 *   1. RampUp: Gradually ramp current to avoid transients
	 *   2. Measurement: Filter voltage and current for accurate Rs
	 *
	 * PI controller automatically generates voltage needed: V = I*R
	 */

	/* Initialize calibration angle generator to stationary frame (0 rad/s) */
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);

	/* Configure trajectory for smooth current ramp
	 * Continues from previous state (ROVERL_MEAS) current value
	 */
	traj_set_target_value(&params->traj_Id, RS_EST_CURRENT_A);
	float32_t rs_est_ramp_rate = RS_EST_CURRENT_A / (RS_EST_RAMPUP_S * CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_max_delta(&params->traj_Id, rs_est_ramp_rate);

	/* Initialize filters for measurement */
	float32_t a1 = expf(-2.0f * PI_F32 * RS_EST_FILTER_BW_HZ / CONTROL_LOOP_FREQUENCY_HZ);
	float32_t b0 = 1.0f - a1;
	filter_fo_init(&params->filter_rs_est_V);
	filter_fo_set_den_coeffs(&params->filter_rs_est_V, a1);
	filter_fo_set_num_coeffs(&params->filter_rs_est_V, b0, 0.0f);
	filter_fo_set_initial_conditions(&params->filter_rs_est_V, 0.0f, 0.0f);

	filter_fo_init(&params->filter_rs_est_I);
	filter_fo_set_den_coeffs(&params->filter_rs_est_I, a1);
	filter_fo_set_num_coeffs(&params->filter_rs_est_I, b0, 0.0f);
	filter_fo_set_initial_conditions(&params->filter_rs_est_I, 0.0f, 0.0f);

	params->Rs_measured_ohm = 0.0f;

	LOG_INF("Rs EST: I_target=%.3fA, rampup=%.1fs, measurement=%.1fs",
		(double)RS_EST_CURRENT_A,
		(double)RS_EST_RAMPUP_S, (double)RS_EST_DURATION_S);

	/* Start timer for total Rs estimation duration (rampup + measurement) */
	float32_t total_duration_s = RS_EST_RAMPUP_S + RS_EST_DURATION_S;
	k_timer_start(&params->state_timer, K_MSEC((uint32_t)(total_duration_s * 1000)), K_NO_WAIT);
}

enum smf_state_result motor_state_rs_est_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_timeout_elapsed(params)) {
		/* Get final filtered values */
		float32_t V_est = filter_fo_get_y1(&params->filter_rs_est_V);
		float32_t I_est = filter_fo_get_y1(&params->filter_rs_est_I);

		if (fabsf(I_est) <= 1e-6f) {
			LOG_ERR("Rs EST failed: filtered current too small (I=%.4e)", (double)I_est);
			params->event.type = MOTOR_EVENT_ERROR;
			params->event.error_code = ERROR_HARDWARE_BREAK;
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
			return SMF_EVENT_HANDLED;
		}

		float32_t Rs_est = V_est / I_est;

		LOG_INF("Rs EST complete: Rs=%.4f Ω (V=%.3fV, I=%.3fA)",
			(double)Rs_est, (double)V_est, (double)I_est);

		/* Update stored Rs value */
		params->Rs_measured_ohm = Rs_est;

		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN]);
		return SMF_EVENT_HANDLED;
	}

	/* Propagate unhandled events */
	return SMF_EVENT_PROPAGATE;
}

void motor_state_rs_est_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting RS_EST state");

	/* Clear this state's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_PI_CONTROL));
}

#define ALIGN_SAMPLE_MIN_FRESH_SAMPLES 4U
#define ALIGN_SAMPLE_MAX_RETRIES 3U
#define ALIGN_OPPOSED_ELEC_TOL_RAD (20.0f * PI_F32 / 180.0f)

static inline void motor_align_set_injection_target(struct motor_parameters *params,
						    float32_t id_target_a)
{
	float32_t current_id_a = traj_get_int_value(&params->traj_Id);
	float32_t steps = MAX(1.0f, ALIGN_INJECT_DURATION_S * CONTROL_LOOP_FREQUENCY_HZ);
	float32_t max_delta_a = fmaxf(fabsf(id_target_a - current_id_a) / steps, 1e-6f);

	traj_set_target_value(&params->traj_Id, id_target_a);
	traj_set_max_delta(&params->traj_Id, max_delta_a);
}

static inline void motor_align_reset_pos_sample_accumulator(struct motor_parameters *params)
{
	params->align_pos_sample_count = 0U;
	params->align_pos_sum_sin = 0.0f;
	params->align_pos_sum_cos = 0.0f;
}

static inline void motor_align_reset_neg_sample_accumulator(struct motor_parameters *params)
{
	params->align_neg_sample_count = 0U;
	params->align_neg_sum_sin = 0.0f;
	params->align_neg_sum_cos = 0.0f;
}

static inline bool motor_align_compute_circular_mean(float32_t sum_sin,
						     float32_t sum_cos,
						     uint16_t sample_count,
						     float32_t *mean_angle_rad)
{
	if (mean_angle_rad == NULL || sample_count == 0U) {
		return false;
	}

	if (!isfinite(sum_sin) || !isfinite(sum_cos)) {
		return false;
	}

	if ((fabsf(sum_sin) < 1e-6f) && (fabsf(sum_cos) < 1e-6f)) {
		return false;
	}

	*mean_angle_rad = wrap_rad_2pi(atan2f(sum_sin, sum_cos));
	return true;
}

static inline void motor_align_start_sample_window(struct motor_parameters *params)
{
	k_timer_start(&params->state_timer,
		      K_MSEC((uint32_t)(ALIGN_STABILIZE_DURATION_S * 1000.0f)),
		      K_NO_WAIT);
}

static inline enum smf_state_result motor_align_apply_offset_and_transition(
	struct motor_parameters *params,
	float32_t offset_rad)
{
	params->observer_alignment_offset_rad = wrap_rad_pi(offset_rad);
	/* ALIGN defines base commutation reference; runtime trim is reset here. */
	params->observer_elec_trim_rad = 0.0f;
	angle_observer_set_offset(&params->observer, params->observer_alignment_offset_rad);

	if (params->calibration_mode == MOTOR_CALIBRATION_MODE_COMMISSIONING) {
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
	} else {
		enum motor_state online_mode =
			motor_resolve_requested_online_mode(params);
		smf_set_state(SMF_CTX(params), &motor_states[online_mode]);
	}

	return SMF_EVENT_HANDLED;
}

static inline enum smf_state_result motor_align_fallback_from_mech(
	struct motor_parameters *params,
	float32_t mech_angle_rad,
	const char *reason)
{
	float32_t mech_wrapped_rad = wrap_rad_2pi(mech_angle_rad);
	float32_t fallback_offset_rad = wrap_rad_pi(-mech_wrapped_rad);

	LOG_WRN("ALIGN fallback: %s (mech=%.2f deg, offset=%.2f deg)",
		reason,
		(double)(mech_wrapped_rad * (180.0f / PI_F32)),
		(double)(fallback_offset_rad * (180.0f / PI_F32)));

	return motor_align_apply_offset_and_transition(params, fallback_offset_rad);
}

static inline enum smf_state_result motor_align_fallback_from_observer(
	struct motor_parameters *params,
	const char *reason)
{
	float32_t mech_angle_rad = angle_observer_get_mech_angle(&params->observer);
	if (!isfinite(mech_angle_rad)) {
		mech_angle_rad = 0.0f;
	}

	return motor_align_fallback_from_mech(params, mech_angle_rad, reason);
}

/* State: ALIGN - parent state for dual-polarity rotor alignment */
void motor_state_align_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN state");
	params->align_pos_sample_retries = 0U;
	params->align_neg_sample_retries = 0U;
	params->align_pos_mech_angle_rad = 0.0f;
	params->align_neg_mech_angle_rad = 0.0f;
	motor_align_reset_pos_sample_accumulator(params);
	motor_align_reset_neg_sample_accumulator(params);
}

enum smf_state_result motor_state_align_run(void *obj)
{
	ARG_UNUSED(obj);
	return SMF_EVENT_PROPAGATE;
}

void motor_state_align_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN state");
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_ENCODER_READ) |
				      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: ALIGN_POS_INJECT - inject +Id with generated angle frame */
void motor_state_align_pos_inject_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN_POS_INJECT state");
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));

	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);

	motor_align_set_injection_target(params, ALIGN_CURRENT_A);
	LOG_INF("ALIGN +Id inject: Id=%.3fA for %.3fs",
		(double)ALIGN_CURRENT_A, (double)ALIGN_INJECT_DURATION_S);

	k_timer_start(&params->state_timer,
		      K_MSEC((uint32_t)(ALIGN_INJECT_DURATION_S * 1000.0f)),
		      K_NO_WAIT);
}

enum smf_state_result motor_state_align_pos_inject_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_timeout_elapsed(params)) {
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN_POS_SAMPLE]);
		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

void motor_state_align_pos_inject_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_POS_INJECT state");
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN));
}

/* State: ALIGN_POS_SAMPLE - hold +Id and capture fresh encoder-driven observer samples */
void motor_state_align_pos_sample_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN_POS_SAMPLE state");
	params->align_pos_sample_retries = 0U;
	motor_align_reset_pos_sample_accumulator(params);
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));
	motor_align_start_sample_window(params);
}

enum smf_state_result motor_state_align_pos_sample_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (!motor_calibration_timeout_elapsed(params)) {
		return SMF_EVENT_PROPAGATE;
	}

	if (params->align_pos_sample_count < ALIGN_SAMPLE_MIN_FRESH_SAMPLES) {
		if (params->align_pos_sample_retries < ALIGN_SAMPLE_MAX_RETRIES) {
			params->align_pos_sample_retries++;
			LOG_WRN("ALIGN +Id sample: only %u fresh samples, extending window (%u/%u)",
				params->align_pos_sample_count,
				params->align_pos_sample_retries,
				ALIGN_SAMPLE_MAX_RETRIES);
			motor_align_start_sample_window(params);
			return SMF_EVENT_HANDLED;
		}

		return motor_align_fallback_from_observer(params,
			"+Id sample window insufficient fresh encoder samples");
	}

	float32_t pos_mean_rad = 0.0f;
	if (!motor_align_compute_circular_mean(params->align_pos_sum_sin,
					       params->align_pos_sum_cos,
					       params->align_pos_sample_count,
					       &pos_mean_rad)) {
		return motor_align_fallback_from_observer(params,
			"+Id sample circular mean invalid");
	}

	params->align_pos_mech_angle_rad = pos_mean_rad;
	LOG_INF("ALIGN +Id sample mean: mech=%.2f deg (%u samples)",
		(double)(pos_mean_rad * (180.0f / PI_F32)),
		params->align_pos_sample_count);

	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN_NEG_INJECT]);
	return SMF_EVENT_HANDLED;
}

void motor_state_align_pos_sample_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_POS_SAMPLE state");
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));
}

/* State: ALIGN_NEG_INJECT - inject -Id with generated angle frame */
void motor_state_align_neg_inject_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN_NEG_INJECT state");
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));

	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);

	motor_align_set_injection_target(params, -ALIGN_CURRENT_A);
	LOG_INF("ALIGN -Id inject: Id=%.3fA for %.3fs",
		(double)(-ALIGN_CURRENT_A), (double)ALIGN_INJECT_DURATION_S);

	k_timer_start(&params->state_timer,
		      K_MSEC((uint32_t)(ALIGN_INJECT_DURATION_S * 1000.0f)),
		      K_NO_WAIT);
}

enum smf_state_result motor_state_align_neg_inject_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_timeout_elapsed(params)) {
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN_NEG_SAMPLE]);
		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

void motor_state_align_neg_inject_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_NEG_INJECT state");
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN));
}

/* State: ALIGN_NEG_SAMPLE - hold -Id, capture observer sample, finalize offset */
void motor_state_align_neg_sample_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN_NEG_SAMPLE state");
	params->align_neg_sample_retries = 0U;
	motor_align_reset_neg_sample_accumulator(params);
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));
	motor_align_start_sample_window(params);
}

enum smf_state_result motor_state_align_neg_sample_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (!motor_calibration_timeout_elapsed(params)) {
		return SMF_EVENT_PROPAGATE;
	}

	if (params->align_neg_sample_count < ALIGN_SAMPLE_MIN_FRESH_SAMPLES) {
		if (params->align_neg_sample_retries < ALIGN_SAMPLE_MAX_RETRIES) {
			params->align_neg_sample_retries++;
			LOG_WRN("ALIGN -Id sample: only %u fresh samples, extending window (%u/%u)",
				params->align_neg_sample_count,
				params->align_neg_sample_retries,
				ALIGN_SAMPLE_MAX_RETRIES);
			motor_align_start_sample_window(params);
			return SMF_EVENT_HANDLED;
		}

		return motor_align_fallback_from_mech(params,
			params->align_pos_mech_angle_rad,
			"-Id sample window insufficient fresh encoder samples");
	}

	float32_t neg_mean_rad = 0.0f;
	if (!motor_align_compute_circular_mean(params->align_neg_sum_sin,
					       params->align_neg_sum_cos,
					       params->align_neg_sample_count,
					       &neg_mean_rad)) {
		return motor_align_fallback_from_mech(params,
			params->align_pos_mech_angle_rad,
			"-Id sample circular mean invalid");
	}
	params->align_neg_mech_angle_rad = neg_mean_rad;

	float32_t expected_delta_mech_rad = PI_F32 / (float32_t)MOTOR_POLE_PAIRS;
	float32_t measured_delta_mech_rad = wrap_rad_pi(
		params->align_neg_mech_angle_rad - params->align_pos_mech_angle_rad);
	float32_t delta_abs_error_rad =
		fabsf(fabsf(measured_delta_mech_rad) - expected_delta_mech_rad);
	float32_t delta_tol_mech_rad = ALIGN_OPPOSED_ELEC_TOL_RAD / (float32_t)MOTOR_POLE_PAIRS;

	if (delta_abs_error_rad > delta_tol_mech_rad) {
		LOG_WRN("ALIGN dual-polarity mismatch: |delta|=%.2f deg expected=%.2f deg (tol=%.2f deg)",
			(double)(fabsf(measured_delta_mech_rad) * (180.0f / PI_F32)),
			(double)(expected_delta_mech_rad * (180.0f / PI_F32)),
			(double)(delta_tol_mech_rad * (180.0f / PI_F32)));
		return motor_align_fallback_from_mech(params,
			params->align_pos_mech_angle_rad,
			"dual-polarity separation out of tolerance");
	}

	float32_t offset_plus_rad = wrap_rad_pi(-params->align_pos_mech_angle_rad);
	float32_t offset_minus_rad =
		wrap_rad_pi((PI_F32 / (float32_t)MOTOR_POLE_PAIRS) - params->align_neg_mech_angle_rad);
	float32_t offset_sum_sin = sinf(offset_plus_rad) + sinf(offset_minus_rad);
	float32_t offset_sum_cos = cosf(offset_plus_rad) + cosf(offset_minus_rad);

	if ((fabsf(offset_sum_sin) < 1e-6f) && (fabsf(offset_sum_cos) < 1e-6f)) {
		return motor_align_fallback_from_mech(params,
			params->align_pos_mech_angle_rad,
			"dual-polarity offset mean is ill-conditioned");
	}

	float32_t final_offset_rad = wrap_rad_pi(atan2f(offset_sum_sin, offset_sum_cos));
	LOG_INF("Alignment complete: +Id=%.2f deg, -Id=%.2f deg, offset=%.2f deg",
		(double)(params->align_pos_mech_angle_rad * (180.0f / PI_F32)),
		(double)(params->align_neg_mech_angle_rad * (180.0f / PI_F32)),
		(double)(final_offset_rad * (180.0f / PI_F32)));
	return motor_align_apply_offset_and_transition(params, final_offset_rad);
}

void motor_state_align_neg_sample_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_NEG_SAMPLE state");
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));
}
