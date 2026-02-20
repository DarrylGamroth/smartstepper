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

LOG_MODULE_DECLARE(motor_states, CONFIG_APP_LOG_LEVEL);

static inline void motor_enable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next |= mask;
}

static inline void motor_disable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next &= ~mask;
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

	LOG_INF("=== %s Complete ===", commissioning ? "Commissioning" : "Calibration");
	LOG_INF("  Rs:    %.4f Ω", (double)params->Rs_measured_ohm);
	LOG_INF("  Ls:    %.6f H", (double)params->Ls_measured_H);
	LOG_INF("  R/L:   %.1f rad/s", (double)params->R_over_L_measured);

	params->calibration_complete = true;
	params->calibration_running = false;
	if (commissioning) {
		params->commissioning_complete = true;
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
		/* Offset measurement complete */
		if (params->calibration_mode == MOTOR_CALIBRATION_MODE_COMMISSIONING) {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ROVERL_MEAS]);
		} else {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN]);
		}
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
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

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
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

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

void motor_state_rs_est_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting RS_EST state");

	/* Clear this state's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: ALIGN - Align rotor to known position (Phase 1: injection in generated frame) */
void motor_state_align_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN state");

	/* Phase 1: use generated angle (no encoder -> observer is driven by angle_gen). */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
			     BIT(MOTOR_FEATURE_PI_CONTROL));

	/* Initialize angle generator to stationary frame (0 rad/s) */
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);

	/* Set trajectory target to alignment current (will ramp smoothly)
	 * Continues from previous state (RS_EST) current value
	 */
	traj_set_target_value(&params->traj_Id, ALIGN_CURRENT_A);
	float32_t align_ramp_rate = ALIGN_CURRENT_A / (ALIGN_INJECT_DURATION_S * CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_max_delta(&params->traj_Id, align_ramp_rate);

	LOG_INF("Applying alignment current: Id=%.3fA for %.3fs (then sample %.3fs)",
		(double)ALIGN_CURRENT_A,
		(double)ALIGN_INJECT_DURATION_S,
		(double)ALIGN_STABILIZE_DURATION_S);

	/* Start timer for alignment injection duration */
	k_timer_start(&params->state_timer,
		      K_MSEC((uint32_t)(ALIGN_INJECT_DURATION_S * 1000.0f)),
		      K_NO_WAIT);
}

enum smf_state_result motor_state_align_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
		/* Move to Phase 2: switch observer input to encoder and let it converge. */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN_SAMPLE]);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

void motor_state_align_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN state");

	/* Clear this phase's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
			      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: ALIGN_SAMPLE - Align rotor to known position (Phase 2: sample encoder/observer) */
void motor_state_align_sample_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN_SAMPLE state");

	/* Phase 2: use encoder-based observer input. Keep PI current control enabled
	 * so we continue holding alignment current while the observer converges.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
			     BIT(MOTOR_FEATURE_PI_CONTROL));

	/* Start timer for stabilization window */
	k_timer_start(&params->state_timer,
		      K_MSEC((uint32_t)(ALIGN_STABILIZE_DURATION_S * 1000.0f)),
		      K_NO_WAIT);
}

enum smf_state_result motor_state_align_sample_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT: {
		/* Get the current mechanical angle from the observer (now encoder-driven). */
		float32_t mech_angle_rad = angle_observer_get_mech_angle(&params->observer);

		/* Set the offset so that electrical angle = 0 at this position
		 * (d-axis aligned with alignment current)
		 * offset = -mech_angle (so elec = (mech + offset) * poles = 0)
		 */
		float32_t offset_rad = -mech_angle_rad;
		angle_observer_set_offset(&params->observer, offset_rad);

		/* Convert to degrees for display */
		float32_t mech_angle_deg = mech_angle_rad * (180.0f / PI_F32);
		float32_t offset_deg = offset_rad * (180.0f / PI_F32);
		LOG_INF("Alignment complete: mech_angle=%.2f deg, offset=%.2f deg",
			(double)mech_angle_deg, (double)offset_deg);

		/* Boot calibration resumes normal flow to ONLINE. Commissioning ends in IDLE
		 * so the user can inspect measurements without immediately entering control.
		 */
		if (params->calibration_mode == MOTOR_CALIBRATION_MODE_COMMISSIONING) {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		} else {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ONLINE]);
		}
		return SMF_EVENT_HANDLED;
	}

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

void motor_state_align_sample_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_SAMPLE state");

	/* Clear this phase's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
			      BIT(MOTOR_FEATURE_PI_CONTROL));
}
