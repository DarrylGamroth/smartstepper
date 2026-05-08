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
#include "motor/filters/pi.h"
#include "motor/motion/traj.h"
#include "motor/calibration/align.h"
#include "motor/calibration/offset.h"
#include "motor/calibration/rl_ident.h"
#include "motor/calibration/timing.h"
#include "motor/calibration/window.h"
#include "motor/observers/angle_observer.h"
#include "motor/motion/angle_gen.h"
#include "motor/math/angle_wrap.h"
#include "motor_state_utils.h"
#include "motor_state_transition.h"

LOG_MODULE_DECLARE(motor_states, CONFIG_APP_LOG_LEVEL);

static inline void motor_enable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next |= mask;
}

static inline void motor_disable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next &= ~mask;
}

static inline void motor_calibration_post_hardware_break(struct motor_parameters *params)
{
	params->event.type = MOTOR_EVENT_ERROR;
	params->event.error_code = ERROR_HARDWARE_BREAK;
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
}

static inline bool motor_calibration_state_timeout_elapsed(struct motor_parameters *params)
{
	bool stale_expiry = false;
	bool elapsed = motor_calibration_timer_has_elapsed(
		&params->state_timer,
		params->event.type == MOTOR_EVENT_TIMEOUT,
		&stale_expiry);

	if (stale_expiry) {
		LOG_WRN("State timer expired without queued timeout event; proceeding");
	}

	return elapsed;
}

static inline bool motor_calibration_start_timer_or_fault(struct motor_parameters *params,
							  float32_t duration_s,
							  const char *name)
{
	int ret = motor_calibration_timer_start_s(&params->state_timer, duration_s);
	if (ret == 0) {
		return true;
	}

	LOG_ERR("%s timer start failed: ret=%d duration=%.6fs",
		name, ret, (double)duration_s);
	motor_calibration_post_hardware_break(params);
	return false;
}

static inline enum smf_state_result
motor_boot_calibration_complete(struct motor_parameters *params)
{
	bool fallback_used = false;
	char reason[96] = {0};
	enum motor_state online_mode =
		motor_state_resolve_requested_online_mode(params, true,
							  &fallback_used,
							  reason, sizeof(reason));

	LOG_INF("Boot calibration complete: current offsets measured");
	LOG_INF("Encoder commutation offset is not set by boot calibration; run generated-sweep encoder commissioning before encoder-control modes");
	motor_transition_status_update(params, MOTOR_EVENT_CALIBRATE_REQUEST,
				       params->calibration.requested_online_mode,
				       MOTOR_STATE_CALIBRATION,
				       online_mode,
				       fallback_used ? online_mode : MOTOR_STATE_ONLINE,
				       fallback_used ?
				       MOTOR_TRANSITION_RESULT_FALLBACK :
				       MOTOR_TRANSITION_RESULT_COMPLETED,
				       ERROR_NONE,
				       fallback_used ? reason : "boot calibration complete");
	smf_set_state(SMF_CTX(params), &motor_states[online_mode]);
	return SMF_EVENT_HANDLED;
}

static inline enum smf_state_result
motor_commissioning_identification_complete(struct motor_parameters *params)
{
	LOG_INF("Commissioning electrical bootstrap complete: RoverL provisional model active");
	LOG_INF("Encoder commutation offset requires explicit generated-sweep encoder commissioning");
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
	return SMF_EVENT_HANDLED;
}

typedef int (*motor_calibration_finalize_fn_t)(struct motor_parameters *params);

static int motor_calibration_apply_current_pi_from_rl(struct motor_parameters *params)
{
	if (params == NULL ||
	    !isfinite(params->Rs_measured_ohm) ||
	    !isfinite(params->Ld_measured_H) ||
	    !isfinite(params->Lq_measured_H) ||
	    params->Rs_measured_ohm <= 0.0f ||
	    params->Ld_measured_H <= 0.0f ||
	    params->Lq_measured_H <= 0.0f) {
		return -ERANGE;
	}

	params->Ls_measured_H = 0.5f * (params->Ld_measured_H + params->Lq_measured_H);
	float32_t rd_over_ld = params->Rs_measured_ohm / params->Ld_measured_H;
	float32_t rq_over_lq = params->Rs_measured_ohm / params->Lq_measured_H;
	float32_t r_over_l = params->Rs_measured_ohm / params->Ls_measured_H;
	if (!isfinite(params->R_over_L_measured) ||
	    params->R_over_L_measured <= 0.0f) {
		params->R_over_L_measured = r_over_l;
	}

	float32_t kp_d = params->Ld_measured_H * CURRENT_LOOP_BANDWIDTH_RPS;
	float32_t kp_q = params->Lq_measured_H * CURRENT_LOOP_BANDWIDTH_RPS;
	float32_t ki_d = rd_over_ld / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t ki_q = rq_over_lq / CONTROL_LOOP_FREQUENCY_HZ;
	if (!isfinite(kp_d) || !isfinite(kp_q) ||
	    !isfinite(ki_d) || !isfinite(ki_q) ||
	    kp_d <= 0.0f || kp_q <= 0.0f ||
	    ki_d <= 0.0f || ki_q <= 0.0f) {
		return -ERANGE;
	}

	pi_set_gains(&params->pi_Id, kp_d, ki_d);
	pi_set_gains(&params->pi_Iq, kp_q, ki_q);
	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);

	LOG_INF("Current PI updated from electrical ID: Id(Kp=%.6f Ki=%.6f) Iq(Kp=%.6f Ki=%.6f) (Rs=%.4f Ω Ld=%.6f H Lq=%.6f H)",
		(double)kp_d, (double)ki_d,
		(double)kp_q, (double)ki_q,
		(double)params->Rs_measured_ohm,
		(double)params->Ld_measured_H,
		(double)params->Lq_measured_H);
	return 0;
}

static inline enum smf_state_result
motor_calibration_timeout_finalize_or_fault(struct motor_parameters *params,
					    enum motor_state next_state,
					    motor_calibration_finalize_fn_t finalize_fn)
{
	if (!motor_calibration_state_timeout_elapsed(params)) {
		return SMF_EVENT_PROPAGATE;
	}

	if (finalize_fn == NULL || finalize_fn(params) != 0) {
		motor_calibration_post_hardware_break(params);
		return SMF_EVENT_HANDLED;
	}

	smf_set_state(SMF_CTX(params), &motor_states[next_state]);
	return SMF_EVENT_HANDLED;
}

/* State: CALIBRATION - Hierarchical parent state for all calibration sub-states */
void motor_state_calibration_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	const bool commissioning =
		(params->calibration.mode == MOTOR_CALIBRATION_MODE_COMMISSIONING);

	LOG_INF("=== Starting %s Sequence ===",
		commissioning ? "Motor Commissioning" : "Motor Calibration");

	params->calibration.running = true;

	if (!commissioning) {
		params->calibration.complete = false;
	}

	params->calibration.commissioning_complete = false;
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
		(params->calibration.mode == MOTOR_CALIBRATION_MODE_COMMISSIONING);
	const bool failed = (params->event.type == MOTOR_EVENT_ERROR);

	params->calibration.running = false;
	if (failed) {
		LOG_WRN("=== %s Aborted (error) ===",
			commissioning ? "Commissioning" : "Calibration");
		params->calibration.complete = false;
		params->calibration.commissioning_complete = false;
	} else {
		LOG_INF("=== %s Complete ===",
			commissioning ? "Commissioning" : "Calibration");
		LOG_INF("  Rs:    %.4f Ω", (double)params->Rs_measured_ohm);
		LOG_INF("  Lavg:  %.6f H", (double)params->Ls_measured_H);
		LOG_INF("  Ld:    %.6f H", (double)params->Ld_measured_H);
		LOG_INF("  Lq:    %.6f H", (double)params->Lq_measured_H);
		LOG_INF("  R/L:   %.1f rad/s", (double)params->R_over_L_measured);
		LOG_INF("  Encoder offset: %.3f deg",
			(double)(params->observer_alignment_offset_rad * (180.0f / PI_F32)));
		params->calibration.complete = true;
		if (commissioning) {
			params->calibration.commissioning_complete = true;
		}
	}
	params->calibration.mode = MOTOR_CALIBRATION_MODE_BOOT;
}

/* State: OFFSET_MEAS - Measure current sensor offsets */
void motor_state_offset_meas_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering OFFSET_MEAS state");

	int ret = motor_offset_measurement_start(&params->filter_Ia, &params->filter_Ib,
						 &params->Ia_offset, &params->Ib_offset);
	if (ret != 0) {
		LOG_ERR("OFFSET_MEAS init failed: %d", ret);
		motor_calibration_post_hardware_break(params);
		return;
	}

	/* Start timer for offset measurement duration. */
	(void)motor_calibration_start_timer_or_fault(params, 1.0f, "OFFSET_MEAS");
}

void motor_state_offset_meas_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting OFFSET_MEAS state");

	int ret = motor_offset_measurement_finalize(&params->filter_Ia, &params->filter_Ib,
						    &params->Ia_offset, &params->Ib_offset);
	if (ret != 0) {
		LOG_ERR("Offset finalize failed: %d (forcing zero offsets)", ret);
		params->Ia_offset = 0.0f;
		params->Ib_offset = 0.0f;
	}

	LOG_INF("Offset measurement complete: Ia=%.4f, Ib=%.4f",
		(double)params->Ia_offset, (double)params->Ib_offset);
}

enum smf_state_result motor_state_offset_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_state_timeout_elapsed(params)) {
		/* Offset measurement complete */
		if (params->calibration.mode == MOTOR_CALIBRATION_MODE_COMMISSIONING) {
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ROVERL_MEAS]);
		} else {
			return motor_boot_calibration_complete(params);
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

	/* Additional ROVERL_MEAS requirements (PWM output is provided by PREPARE_ONLINE). */
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
	const struct motor_roverl_config cfg = {
		.target_current_a = ROVERL_EST_CURRENT_A,
		.settling_s = ROVERL_EST_SETTLING_S,
		.excitation_hz = ROVERL_EST_FREQ_HZ,
		.control_hz = CONTROL_LOOP_FREQUENCY_HZ,
		.pole_pairs = (float32_t)MOTOR_POLE_PAIRS,
	};
	int ret = motor_roverl_prepare_scalars(&params->traj_Id, &params->angle_gen, &cfg,
					       &params->roverl_accumulator_Vd_Id,
					       &params->roverl_accumulator_Vq_Id,
					       &params->roverl_accumulator_Id2);
	if (ret != 0) {
		LOG_ERR("RoverL plan failed: %d", ret);
		motor_calibration_post_hardware_break(params);
		return;
	}

	LOG_INF("RoverL: %.0fHz excitation for %.1fs (I_amplitude=%.3fA)",
		(double)ROVERL_EST_FREQ_HZ, (double)ROVERL_EST_DURATION_S,
		(double)ROVERL_EST_CURRENT_A);

	/* Start timer for total R/L estimation duration */
	(void)motor_calibration_start_timer_or_fault(params, ROVERL_EST_DURATION_S, "ROVERL_MEAS");
}

static int motor_calibration_finalize_roverl(struct motor_parameters *params)
{
	struct motor_roverl_result result = {0};
	int ret = motor_roverl_finalize_from_scalars(
		params->roverl_accumulator_Vd_Id,
		params->roverl_accumulator_Vq_Id,
		params->roverl_accumulator_Id2,
		ROVERL_EST_FREQ_HZ,
		&result);
	if (ret != 0) {
		LOG_ERR("RoverL failed: ret=%d (sum(Id^2)=%.3e)",
			ret, (double)params->roverl_accumulator_Id2);
		return ret;
	}

	LOG_INF("RoverL complete: R=%.4f Ω, L=%.2f µH, R/L=%.0f, τ=%.2f ms",
		(double)result.rs_ohm, (double)(result.ls_h * 1e6f),
		(double)result.r_over_l, (double)(result.tau_s * 1000.0f));

	params->Rs_measured_ohm = result.rs_ohm;
	params->Ls_measured_H = result.ls_h;
	params->Ld_measured_H = result.ls_h;
	params->Lq_measured_H = result.ls_h;
	params->R_over_L_measured = result.r_over_l;
	params->electrical_model_source = MOTOR_ELECTRICAL_MODEL_SOURCE_ROVERL;

	return motor_calibration_apply_current_pi_from_rl(params);
}

enum smf_state_result motor_state_roverl_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	return motor_calibration_timeout_finalize_or_fault(params,
							  MOTOR_STATE_IDLE,
							  motor_calibration_finalize_roverl);
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

#define ALIGN_SAMPLE_MIN_FRESH_SAMPLES 4U
#define ALIGN_SAMPLE_MAX_RETRIES 3U
#define ALIGN_OPPOSED_ELEC_TOL_RAD (20.0f * PI_F32 / 180.0f)

static const struct motor_calibration_window_policy align_sample_window_policy = {
	.min_samples = ALIGN_SAMPLE_MIN_FRESH_SAMPLES,
	.max_retries = ALIGN_SAMPLE_MAX_RETRIES,
};

/* ALIGN current reference contract:
 * - state thread only programs traj_Id target/rate
 * - ADC ISR owns traj_Id advancement and publishes the ramped Id_ref_A
 * - no direct Id_ref_A step changes from state handlers
 */
static inline bool motor_align_plan_injection_or_fault(struct motor_parameters *params,
						       float32_t id_target_a,
						       const char *state_name)
{
	struct motor_align_traj_plan plan = {0};
	int ret = motor_align_plan_id_traj(&params->traj_Id, id_target_a,
					   ALIGN_INJECT_DURATION_S,
					   CONTROL_LOOP_FREQUENCY_HZ, &plan);
	if (ret != 0) {
		LOG_ERR("%s ALIGN traj plan failed: ret=%d", state_name, ret);
		motor_calibration_post_hardware_break(params);
		return false;
	}

	LOG_INF("ALIGN traj plan: Id start=%.4fA target=%.4fA max_delta=%.6fA/tick steps=%.0f",
		(double)plan.id_start_a,
		(double)plan.id_target_a,
		(double)plan.max_delta_a_per_tick,
		(double)plan.steps);
	return true;
}

static inline void motor_align_reset_sample_accumulator(struct motor_parameters *params)
{
	struct motor_align_sample_accum acc = {0};

	motor_align_accum_reset(&acc);
	params->calibration.align_sample_count = acc.count;
	params->calibration.align_sum_sin = acc.sum_sin;
	params->calibration.align_sum_cos = acc.sum_cos;
}

static inline bool motor_align_start_sample_window(struct motor_parameters *params)
{
	return motor_calibration_start_timer_or_fault(params,
						      ALIGN_STABILIZE_DURATION_S,
						      "ALIGN_SAMPLE");
}

static inline enum smf_state_result motor_align_apply_offset_and_transition(
	struct motor_parameters *params,
	float32_t offset_rad)
{
	float32_t base_offset_rad = wrap_rad_pi(offset_rad);

	params->observer_alignment_offset_rad = base_offset_rad;
	/* ALIGN defines base commutation reference; runtime trim is reset here. */
	params->observer_elec_trim_rad = 0.0f;
	angle_observer_set_offset(&params->observer, params->observer_alignment_offset_rad);
	LOG_INF("ALIGN observer offset applied: request=%.3f deg stored=%.3f deg",
		(double)(params->observer_alignment_offset_rad * (180.0f / PI_F32)),
		(double)(params->observer.mech_angle_offset_rad * (180.0f / PI_F32)));

	if (params->calibration.mode == MOTOR_CALIBRATION_MODE_COMMISSIONING) {
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
	} else {
		bool fallback_used = false;
		char reason[96] = {0};
		enum motor_state online_mode =
			motor_state_resolve_requested_online_mode(params, true,
								  &fallback_used,
								  reason, sizeof(reason));
		motor_transition_status_update(params, MOTOR_EVENT_CALIBRATE_REQUEST,
					       params->calibration.requested_online_mode,
					       MOTOR_STATE_ALIGN_POS_SAMPLE,
					       online_mode,
					       fallback_used ? online_mode : MOTOR_STATE_ONLINE,
					       fallback_used ?
					       MOTOR_TRANSITION_RESULT_FALLBACK :
					       MOTOR_TRANSITION_RESULT_COMPLETED,
					       ERROR_NONE,
					       fallback_used ? reason : "alignment complete");
		smf_set_state(SMF_CTX(params), &motor_states[online_mode]);
	}

	/* smf_set_state() may run exit/entry hooks synchronously. Keep ALIGN as
	 * the last writer of the commutation reference for the next control tick.
	 */
	params->observer_alignment_offset_rad = base_offset_rad;
	params->observer_elec_trim_rad = 0.0f;
	angle_observer_set_offset(&params->observer, base_offset_rad);
	LOG_INF("ALIGN observer offset final: base=%.3f deg stored=%.3f deg",
		(double)(params->observer_alignment_offset_rad * (180.0f / PI_F32)),
		(double)(params->observer.mech_angle_offset_rad * (180.0f / PI_F32)));

	return SMF_EVENT_HANDLED;
}

static inline enum smf_state_result motor_align_fallback_from_mech(
	struct motor_parameters *params,
	float32_t mech_angle_rad,
	const char *reason)
{
	float32_t mech_wrapped_rad = wrap_rad_2pi(mech_angle_rad);
	float32_t fallback_offset_rad = motor_align_offset_from_mech_sample(mech_wrapped_rad);

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

/* State: ALIGN - parent state for single-vector rotor alignment */
void motor_state_align_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN state");
	LOG_INF("ALIGN config: current=%.3fA inject=%.3fs sample=%.3fs mode=%s",
		(double)ALIGN_CURRENT_A,
		(double)ALIGN_INJECT_DURATION_S,
		(double)ALIGN_STABILIZE_DURATION_S,
		params->calibration.mode == MOTOR_CALIBRATION_MODE_COMMISSIONING ?
			"commissioning" : "boot");
	params->calibration.align_sample_retries = 0U;
	params->calibration.align_mech_angle_rad = 0.0f;
	motor_align_reset_sample_accumulator(params);
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

	/*
	 * ALIGN defines the base commutation frame.  The generated alignment
	 * vector must therefore start from electrical zero, not from a previous
	 * observer offset left over from an earlier calibration/commissioning
	 * run.
	 */
	params->observer_alignment_offset_rad = 0.0f;
	params->observer_elec_trim_rad = 0.0f;
	angle_observer_set_offset(&params->observer, 0.0f);

	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);

	if (!motor_align_plan_injection_or_fault(params, ALIGN_CURRENT_A, "ALIGN_POS_INJECT")) {
		return;
	}
	LOG_INF("ALIGN +Id inject: Id=%.3fA for %.3fs",
		(double)ALIGN_CURRENT_A, (double)ALIGN_INJECT_DURATION_S);

	(void)motor_calibration_start_timer_or_fault(params,
						     ALIGN_INJECT_DURATION_S,
						     "ALIGN_POS_INJECT");
}

enum smf_state_result motor_state_align_pos_inject_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (motor_calibration_state_timeout_elapsed(params)) {
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

/* State: ALIGN_POS_SAMPLE - hold +Id in generated frame and capture fresh encoder samples */
void motor_state_align_pos_sample_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN_POS_SAMPLE state");
	params->calibration.align_sample_retries = 0U;
	motor_align_reset_sample_accumulator(params);
	/*
	 * Keep generated-angle commutation active while sampling the raw encoder.
	 * The encoder offset is not known yet; using encoder angle for Park/invPark
	 * here would move the alignment vector during the measurement window.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_ENCODER_READ) |
				     BIT(MOTOR_FEATURE_PI_CONTROL));
	(void)motor_align_start_sample_window(params);
}

enum smf_state_result motor_state_align_pos_sample_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	if (!motor_calibration_state_timeout_elapsed(params)) {
		return SMF_EVENT_PROPAGATE;
	}

	LOG_INF("ALIGN +Id sample window complete: fresh=%u retries=%u min_required=%u",
		params->calibration.align_sample_count,
		params->calibration.align_sample_retries,
		ALIGN_SAMPLE_MIN_FRESH_SAMPLES);

	enum motor_calibration_window_action action =
		motor_calibration_window_evaluate(&align_sample_window_policy,
						      params->calibration.align_sample_count,
						      &params->calibration.align_sample_retries);
	if (action == MOTOR_CALIBRATION_WINDOW_EXTEND) {
		LOG_WRN("ALIGN +Id sample: only %u fresh samples, extending window (%u/%u)",
			params->calibration.align_sample_count,
			params->calibration.align_sample_retries,
			ALIGN_SAMPLE_MAX_RETRIES);
		if (!motor_align_start_sample_window(params)) {
			return SMF_EVENT_HANDLED;
		}
		return SMF_EVENT_HANDLED;
	}
	if (action == MOTOR_CALIBRATION_WINDOW_FALLBACK) {
		return motor_align_fallback_from_observer(params,
			"+Id sample window insufficient fresh encoder samples");
	}

	struct motor_align_sample_accum align_acc = {
		.sum_sin = params->calibration.align_sum_sin,
		.sum_cos = params->calibration.align_sum_cos,
		.count = params->calibration.align_sample_count,
	};

	float32_t mean_rad = 0.0f;
	if (!motor_align_circular_mean(&align_acc, &mean_rad)) {
		return motor_align_fallback_from_observer(params,
			"+Id sample circular mean invalid");
	}

	params->calibration.align_mech_angle_rad = mean_rad;
	float32_t offset_rad = motor_align_offset_from_mech_sample(mean_rad);
	LOG_INF("Alignment complete: mech=%.2f deg offset=%.2f deg (%u samples)",
		(double)(mean_rad * (180.0f / PI_F32)),
		(double)(offset_rad * (180.0f / PI_F32)),
		params->calibration.align_sample_count);

	return motor_align_apply_offset_and_transition(params, offset_rad);
}

void motor_state_align_pos_sample_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_POS_SAMPLE state");
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_ENCODER_READ));
}
