/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/smf.h>
#include <zephyr/timing/timing.h>
#include <drivers/mcpwm.h>
#include <drivers/adc_injected.h>
#include <dt-bindings/pwm/stm32-mcpwm.h>

#include "motor_states.h"
#include "motor_isr.h"
#include "motor_math.h"
#include "motor_control_api.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "angle_observer.h"

LOG_MODULE_REGISTER(motor_states, CONFIG_APP_LOG_LEVEL);

/* Forward declarations of state functions */
static void motor_state_hw_init_entry(void *obj);
static enum smf_state_result motor_state_hw_init_run(void *obj);
static void motor_state_ctrl_init_entry(void *obj);
static enum smf_state_result motor_state_ctrl_init_run(void *obj);
static void motor_state_offset_meas_entry(void *obj);
static enum smf_state_result motor_state_offset_meas_run(void *obj);
static void motor_state_offset_meas_exit(void *obj);
static void motor_state_roverl_meas_entry(void *obj);
static enum smf_state_result motor_state_roverl_meas_run(void *obj);
static void motor_state_roverl_meas_exit(void *obj);
static void motor_state_rs_est_entry(void *obj);
static enum smf_state_result motor_state_rs_est_run(void *obj);
static void motor_state_rs_est_exit(void *obj);
static void motor_state_align_entry(void *obj);
static enum smf_state_result motor_state_align_run(void *obj);
static void motor_state_align_exit(void *obj);
static void motor_state_idle_entry(void *obj);
static enum smf_state_result motor_state_idle_run(void *obj);
static void motor_state_offline_entry(void *obj);
static enum smf_state_result motor_state_offline_run(void *obj);
static void motor_state_online_entry(void *obj);
static enum smf_state_result motor_state_online_run(void *obj);
static void motor_state_error_entry(void *obj);
static enum smf_state_result motor_state_error_run(void *obj);

/* State machine table - exported for shell access */
const struct smf_state motor_states[] = {
	[MOTOR_STATE_HW_INIT] = SMF_CREATE_STATE(motor_state_hw_init_entry,
						  motor_state_hw_init_run,
						  NULL, NULL, NULL),
	[MOTOR_STATE_CTRL_INIT] = SMF_CREATE_STATE(motor_state_ctrl_init_entry,
						    motor_state_ctrl_init_run,
						    NULL, NULL, NULL),
	[MOTOR_STATE_OFFSET_MEAS] = SMF_CREATE_STATE(motor_state_offset_meas_entry,
					       motor_state_offset_meas_run,
					       motor_state_offset_meas_exit, NULL, NULL),
	[MOTOR_STATE_ROVERL_MEAS] = SMF_CREATE_STATE(motor_state_roverl_meas_entry,
					  motor_state_roverl_meas_run,
					  motor_state_roverl_meas_exit, NULL, NULL),
	[MOTOR_STATE_RS_EST] = SMF_CREATE_STATE(motor_state_rs_est_entry,
					  motor_state_rs_est_run,
					  motor_state_rs_est_exit, NULL, NULL),
	[MOTOR_STATE_ALIGN] = SMF_CREATE_STATE(motor_state_align_entry,
					motor_state_align_run,
					motor_state_align_exit, NULL, NULL),
	[MOTOR_STATE_IDLE] = SMF_CREATE_STATE(motor_state_idle_entry,
					       motor_state_idle_run,
					       NULL, NULL, NULL),
	[MOTOR_STATE_OFFLINE] = SMF_CREATE_STATE(motor_state_offline_entry,
						  motor_state_offline_run,
						  NULL, NULL, NULL),
	[MOTOR_STATE_ONLINE] = SMF_CREATE_STATE(motor_state_online_entry,
						 motor_state_online_run,
						 NULL, NULL, NULL),
	[MOTOR_STATE_ERROR] = SMF_CREATE_STATE(motor_state_error_entry,
						motor_state_error_run,
						NULL, NULL, NULL),
};

int motor_states_get_current(const struct motor_parameters *params)
{
	if (!params) {
		return -1;
	}
	
	const struct smf_state *current = params->smf.current;
	
	/* Find which state we're in by comparing pointers */
	for (int i = 0; i <= MOTOR_STATE_ERROR; i++) {
		if (current == &motor_states[i]) {
			return i;
		}
	}
	
	return -1;  /* Unknown state */
}

const char *motor_state_to_string(int state)
{
	switch (state) {
	case MOTOR_STATE_HW_INIT:      return "HW_INIT";
	case MOTOR_STATE_CTRL_INIT:    return "CTRL_INIT";
	case MOTOR_STATE_OFFSET_MEAS:  return "OFFSET_MEAS";
	case MOTOR_STATE_ROVERL_MEAS:  return "ROVERL_MEAS";
	case MOTOR_STATE_RS_EST:       return "RS_EST";
	case MOTOR_STATE_ALIGN:        return "ALIGN";
	case MOTOR_STATE_IDLE:         return "IDLE";
	case MOTOR_STATE_OFFLINE:      return "OFFLINE";
	case MOTOR_STATE_ONLINE:       return "ONLINE";
	case MOTOR_STATE_ERROR:        return "ERROR";
	default:                       return "UNKNOWN";
	}
}

const char *motor_error_to_string(int error)
{
	switch (error) {
	case ERROR_NONE:            return "NONE";
	case ERROR_HARDWARE_BREAK:  return "HARDWARE_BREAK";
	case ERROR_OVERCURRENT:     return "OVERCURRENT";
	case ERROR_ENCODER_FAULT:   return "ENCODER_FAULT";
	case ERROR_OVERVOLTAGE:     return "OVERVOLTAGE";
	case ERROR_EMERGENCY_STOP:  return "EMERGENCY_STOP";
	default:                    return "UNKNOWN";
	}
}

/* State: HW_INIT - Initialize hardware */
static void motor_state_hw_init_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering HW_INIT state");

	/* Initialize PWM channels */
	mcpwm_configure(pwm1, 1, 0);
	mcpwm_configure(pwm1, 2, 0);
	mcpwm_configure(pwm1, 4, STM32_PWM_OC_MODE_PWM2);
	mcpwm_configure(pwm8, 1, 0);
	mcpwm_configure(pwm8, 2, 0);
	mcpwm_configure(pwm3, 1, 0);

	/* Set initial duty cycles */
	mcpwm_set_duty_cycle(pwm1, 4, 0x01000000);
	mcpwm_set_duty_cycle(pwm3, 1, 0x47AE147A); /* 56% duty cycle */

	/* Enable PWM for sampling */
	mcpwm_enable(pwm3, 1);
	mcpwm_enable(pwm1, 4);

	/* Set up encoder callback */
	mcpwm_set_compare_callback(pwm3, 1, encoder1_callback, NULL);

	/* Set up break interrupt handler for hardware fault protection */
	mcpwm_set_break_callback(pwm1, break_interrupt_handler, params);
	mcpwm_set_break_callback(pwm8, break_interrupt_handler, params);

	/* Set up ADC callback */
	adc_injected_set_callback(adc1, adc_callback, params);
	adc_injected_enable(adc1);

	/* Initialize timing subsystem for ISR performance measurement */
	timing_init();
	timing_start();

	/* Start master PWM timer */
	mcpwm_start(pwm1);
}

static enum smf_state_result motor_state_hw_init_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Transition to controller initialization */
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CTRL_INIT]);

	return SMF_EVENT_HANDLED;
}

/* State: CTRL_INIT - Initialize controllers and filters */
static void motor_state_ctrl_init_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering CTRL_INIT state");

	/* Initialize filters and PI controllers from devicetree parameters */
	config_init_filters(params);
	config_init_pi_controllers(params);

	/* Initialize angle observer */
	angle_observer_init(&params->observer,
			    1.0f / CONTROL_LOOP_FREQUENCY_HZ,
			    ANGLE_OBSERVER_BANDWIDTH_HZ,
			    MOTOR_POLE_PAIRS);

	/* Initialize Id trajectory generator for smooth current ramping */
	traj_init(&params->traj_Id);
	traj_set_min_value(&params->traj_Id, 0.0f);
	traj_set_max_value(&params->traj_Id, MOTOR_MAX_CURRENT_A);
	/* Max delta will be set by each state that uses it */
}

static enum smf_state_result motor_state_ctrl_init_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Transition to offset measurement */
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_OFFSET_MEAS]);

	return SMF_EVENT_HANDLED;
}

/* State: OFFSET_MEAS - Measure current sensor offsets */
static void motor_state_offset_meas_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering OFFSET_MEAS state");
	params->state_counter = 0;

	filter_fo_set_initial_conditions(&params->filter_Ia, 0.0f, 0.0f);
	filter_fo_set_initial_conditions(&params->filter_Ib, 0.0f, 0.0f);
}

static void motor_state_offset_meas_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting OFFSET_MEAS state");

	/* Get filtered offset values */
	params->Ia_offset = filter_fo_get_y1(&params->filter_Ia);
	params->Ib_offset = filter_fo_get_y1(&params->filter_Ib);

	LOG_INF("Offset measurement complete: Ia=%.4f, Ib=%.4f",
		(double)params->Ia_offset, (double)params->Ib_offset);
}

static enum smf_state_result motor_state_offset_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Wait for sufficient samples to be collected (done in ADC callback) */
	if (params->state_counter >= 1000) {
		/* Transition to Rs measurement */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ROVERL_MEAS]);
	}

	return SMF_EVENT_HANDLED;
}

/* State: ROVERL_MEAS - Measure R/L time constant via sinusoidal excitation (TI method) */
static void motor_state_roverl_meas_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ROVERL_MEAS state");

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
	 */
	traj_set_target_value(&params->traj_Id, ROVERL_EST_CURRENT_A);
	traj_set_int_value(&params->traj_Id, 0.0f);
	float32_t roverl_ramp_rate = ROVERL_EST_CURRENT_A / (ROVERL_EST_SETTLING_S * CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_max_delta(&params->traj_Id, roverl_ramp_rate);
	params->Iq_ref_A = 0.0f;

	/* Reset state counter, accumulators, and phase */
	params->state_counter = 0;
	params->roverl_accumulator_Vd_Id = 0.0f;
	params->roverl_accumulator_Vq_Id = 0.0f;
	params->roverl_accumulator_Id2 = 0.0f;
	params->roverl_phase_degrees = 0.0f;

	LOG_INF("RoverL: %.0fHz excitation for %.1fs (I_amplitude=%.2fA)",
		(double)ROVERL_EST_FREQ_HZ, (double)ROVERL_EST_DURATION_S,
		(double)ROVERL_EST_CURRENT_A);
}

static enum smf_state_result motor_state_roverl_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Total measurement time from devicetree */
	uint32_t total_samples = (uint32_t)(ROVERL_EST_DURATION_S * CONTROL_LOOP_FREQUENCY_HZ);

	if (params->state_counter >= total_samples) {
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
		 * Ki = bandwidth * R
		 * This will be done when PI controllers are reconfigured
		 */

		/* Transition to Rs estimation */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_RS_EST]);
	}

	return SMF_EVENT_HANDLED;
}

static void motor_state_roverl_meas_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ROVERL_MEAS state");

	/* Clear current references and phase */
	params->Id_ref_A = 0.0f;
	params->Iq_ref_A = 0.0f;
	params->roverl_phase_degrees = 0.0f;
}

/* State: RS_EST - Measure stator resistance via DC injection (TI method) */
static void motor_state_rs_est_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering RS_EST state");

	/* TI Rs EST uses DC current injection on d-axis
	 * Two phases:
	 *   1. RampUp (1s): Gradually ramp current to avoid transients
	 *   2. Measurement (6s): Filter voltage and current for accurate Rs
	 *
	 * Total time: 7 seconds (default)
	 * PI controller automatically generates voltage needed: V = I*R
	 */

	/* Configure trajectory for smooth current ramp */
	traj_set_target_value(&params->traj_Id, RS_EST_CURRENT_A);
	traj_set_int_value(&params->traj_Id, 0.0f);
	float32_t rs_est_ramp_rate = RS_EST_CURRENT_A / (RS_EST_RAMPUP_S * CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_max_delta(&params->traj_Id, rs_est_ramp_rate);
	params->Iq_ref_A = 0.0f;

	/* Initialize filters for measurement */
	params->state_counter = 0;
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

	LOG_INF("Rs EST: I_target=%.2fA, rampup=%.1fs, measurement=%.1fs",
		(double)RS_EST_CURRENT_A,
		(double)RS_EST_RAMPUP_S, (double)RS_EST_DURATION_S);
}

static enum smf_state_result motor_state_rs_est_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Calculate phase boundaries */
	uint32_t rampup_samples = (uint32_t)(RS_EST_RAMPUP_S * CONTROL_LOOP_FREQUENCY_HZ);
	uint32_t measurement_samples = (uint32_t)(RS_EST_DURATION_S * CONTROL_LOOP_FREQUENCY_HZ);
	uint32_t total_samples = rampup_samples + measurement_samples;

	if (params->state_counter >= total_samples) {
		/* Get final filtered values */
		float32_t V_est = filter_fo_get_y1(&params->filter_rs_est_V);
		float32_t I_est = filter_fo_get_y1(&params->filter_rs_est_I);
		float32_t Rs_est = V_est / I_est;

		LOG_INF("Rs EST complete: Rs=%.4f Ω (V=%.3fV, I=%.3fA)",
			(double)Rs_est, (double)V_est, (double)I_est);

		/* Update stored Rs value */
		params->Rs_measured_ohm = Rs_est;

		/* Transition to alignment */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN]);
	}

	return SMF_EVENT_HANDLED;
}

static void motor_state_rs_est_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting RS_EST state");

	/* Clear current references */
	params->Id_ref_A = 0.0f;
	params->Iq_ref_A = 0.0f;
}

/* State: ALIGN - Align rotor to known position */
static void motor_state_align_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN state");

	/* Set trajectory target to alignment current (will ramp smoothly) */
	traj_set_target_value(&params->traj_Id, ALIGN_CURRENT_A);
	traj_set_int_value(&params->traj_Id, 0.0f);
	float32_t align_ramp_rate = ALIGN_CURRENT_A / (0.1f * CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_max_delta(&params->traj_Id, align_ramp_rate);
	params->Iq_ref_A = 0.0f;

	/* Reset alignment timer */
	params->state_counter = 0;

	LOG_INF("Applying alignment current: Id=%.3fA for %.1fs",
		(double)ALIGN_CURRENT_A, (double)ALIGN_DURATION_S);
}

static enum smf_state_result motor_state_align_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Check if alignment duration has elapsed */
	uint32_t align_samples = (uint32_t)(ALIGN_DURATION_S * CONTROL_LOOP_FREQUENCY_HZ);
	if (params->state_counter >= align_samples) {
		/* Get the current mechanical angle from the observer */
		float32_t mech_angle_deg = angle_observer_get_mech_angle_deg(&params->observer);

		/* Set the offset so that electrical angle = 0 at this position
		 * (d-axis aligned with alignment current)
		 * offset = -mech_angle (so elec = (mech + offset) * poles = 0)
		 */
		float32_t offset_deg = -mech_angle_deg;
		angle_observer_set_offset(&params->observer, offset_deg);

		LOG_INF("Alignment complete: mech_angle=%.2f deg, offset=%.2f deg",
			(double)mech_angle_deg, (double)offset_deg);

		/* Transition to IDLE */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
	}

	return SMF_EVENT_HANDLED;
}

static void motor_state_align_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN state");

	/* Clear current references (trajectory stops running after state exit) */
	params->Id_ref_A = 0.0f;
	params->Iq_ref_A = 0.0f;
}

/* State: IDLE - Ready but not running */
static void motor_state_idle_entry(void *obj)
{
	LOG_INF("Entering IDLE state");
	/* Disable motor outputs */
}

static enum smf_state_result motor_state_idle_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	struct motor_event evt;

	/* Check for pending events */
	if (motor_api_peek_event(&evt) == 0) {
		switch (evt.type) {
		case MOTOR_EVENT_START_REQUEST:
			motor_api_consume_event();
			LOG_INF("Start request received, transitioning to ONLINE");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ONLINE]);
			break;

		case MOTOR_EVENT_CALIBRATE_REQUEST:
			motor_api_consume_event();
			LOG_INF("Calibrate request received, restarting calibration sequence");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_OFFSET_MEAS]);
			break;

		case MOTOR_EVENT_PARAM_UPDATE:
			/* Explicitly apply parameter update to shadow buffer */
			motor_api_apply_param_update();
			motor_api_consume_event();
			break;

		default:
			/* Ignore other events in IDLE state */
			motor_api_consume_event();
			break;
		}
	}

	return SMF_EVENT_HANDLED;
}

/* State: OFFLINE - Open-loop control */
static void motor_state_offline_entry(void *obj)
{
	LOG_INF("Entering OFFLINE state");
}

static enum smf_state_result motor_state_offline_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	struct motor_event evt;

	/* Check for pending events */
	if (motor_api_peek_event(&evt) == 0) {
		switch (evt.type) {
		case MOTOR_EVENT_STOP_REQUEST:
			motor_api_consume_event();
			LOG_INF("Stop request received, transitioning to IDLE");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
			return SMF_EVENT_HANDLED;

		case MOTOR_EVENT_EMERGENCY_STOP:
			motor_api_consume_event();
			params->error_code = ERROR_EMERGENCY_STOP;
			LOG_WRN("Emergency stop request, transitioning to ERROR");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
			return SMF_EVENT_HANDLED;

		case MOTOR_EVENT_PARAM_UPDATE:
			/* Explicitly apply parameter update to shadow buffer */
			motor_api_apply_param_update();
			motor_api_consume_event();
			break;

		default:
			/* Ignore other events */
			motor_api_consume_event();
			break;
		}
	}

	/* Setpoints are managed by ISR reading from ctrl_buf[ctrl_index]
	 * Parameter updates are handled by event system with double-buffering
	 */

	return SMF_EVENT_HANDLED;
}

/* State: ONLINE - Closed-loop control */
static void motor_state_online_entry(void *obj)
{
	LOG_INF("Entering ONLINE state");
}

static enum smf_state_result motor_state_online_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	struct motor_event evt;

	/* Check for pending events */
	if (motor_api_peek_event(&evt) == 0) {
		switch (evt.type) {
		case MOTOR_EVENT_STOP_REQUEST:
			motor_api_consume_event();
			LOG_INF("Stop request received, transitioning to IDLE");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
			return SMF_EVENT_HANDLED;

		case MOTOR_EVENT_EMERGENCY_STOP:
			motor_api_consume_event();
			params->error_code = ERROR_EMERGENCY_STOP;
			LOG_WRN("Emergency stop request, transitioning to ERROR");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
			return SMF_EVENT_HANDLED;

		case MOTOR_EVENT_PARAM_UPDATE:
			/* Explicitly apply parameter update to shadow buffer */
			motor_api_apply_param_update();
			motor_api_consume_event();
			break;

		default:
			/* Ignore other events */
			motor_api_consume_event();
			break;
		}
	}

	/* Setpoints are managed by ISR reading from ctrl_buf[ctrl_index]
	 * Parameter updates are handled by event system with double-buffering
	 */

	return SMF_EVENT_HANDLED;
}

/* State: ERROR - Fault condition */
static void motor_state_error_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_ERR("Entering ERROR state, code: %d", params->error_code);

	/* Immediately disable PWM outputs to prevent damage
	 * This prevents:
	 * - Dynamic braking from engaging (Vbus spike during fault)
	 * - PI controllers from continuing to drive motor
	 * - Current references from being acted upon
	 *
	 * Hardware faults (overcurrent, break interrupt) may have already
	 * disabled PWM via hardware or ISR - this ensures software faults
	 * also result in safe motor shutdown.
	 */
	mcpwm_disable(pwm1, 1);
	mcpwm_disable(pwm1, 2);
	mcpwm_disable(pwm8, 1);
	mcpwm_disable(pwm8, 2);

	/* Clear current references for good measure */
	params->Id_ref_A = 0.0f;
	params->Iq_ref_A = 0.0f;
}

static enum smf_state_result motor_state_error_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	struct motor_event evt;

	/* Check for pending events */
	if (motor_api_peek_event(&evt) == 0) {
		switch (evt.type) {
		case MOTOR_EVENT_CLEAR_ERROR:
			motor_api_consume_event();
			LOG_INF("Error cleared, transitioning to IDLE");
			params->error_code = ERROR_NONE;
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
			break;

		default:
			/* Ignore all other events in ERROR state */
			motor_api_consume_event();
			break;
		}
	}

	return SMF_EVENT_HANDLED;
}
