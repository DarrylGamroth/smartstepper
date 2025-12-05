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
#include "motor_control_api.h"
#include "motor_hardware.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "angle_observer.h"

LOG_MODULE_REGISTER(motor_states, CONFIG_APP_LOG_LEVEL);

static struct motor_parameters motor_params;
K_MSGQ_DEFINE(motor_event_queue, sizeof(struct motor_event), 16, 4);

/* Timer callback for state timeouts - posts timeout event */
static void state_timer_expiry(struct k_timer *timer)
{
	struct motor_event evt = {
		.type = MOTOR_EVENT_TIMEOUT,
	};

	/* Post timeout event to state machine queue */
	int ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Failed to post timeout event: queue full");
	}
}

/* State machine thread stack and data */
#define MOTOR_SM_THREAD_STACK_SIZE 4096
#define MOTOR_SM_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(motor_sm_thread_stack, MOTOR_SM_THREAD_STACK_SIZE);

static struct k_thread motor_sm_thread_data;
static k_tid_t motor_sm_thread_tid;

/* Forward declarations of state functions */
static void motor_state_hw_init_entry(void *obj);
static enum smf_state_result motor_state_hw_init_run(void *obj);
static void motor_state_ctrl_init_entry(void *obj);
static enum smf_state_result motor_state_ctrl_init_run(void *obj);
static void motor_state_calibration_entry(void *obj);
static enum smf_state_result motor_state_calibration_run(void *obj);
static void motor_state_calibration_exit(void *obj);
static void motor_state_offset_meas_entry(void *obj);
static enum smf_state_result motor_state_offset_meas_run(void *obj);
static void motor_state_offset_meas_exit(void *obj);
static void motor_state_rs_est_entry(void *obj);
static enum smf_state_result motor_state_rs_est_run(void *obj);
static void motor_state_rs_est_exit(void *obj);
static void motor_state_roverl_meas_entry(void *obj);
static enum smf_state_result motor_state_roverl_meas_run(void *obj);
static void motor_state_roverl_meas_exit(void *obj);
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
	[MOTOR_STATE_CALIBRATION] = SMF_CREATE_STATE(motor_state_calibration_entry,
						      motor_state_calibration_run,
						      motor_state_calibration_exit,
						      NULL,
						      &motor_states[MOTOR_STATE_OFFSET_MEAS]),
	[MOTOR_STATE_OFFSET_MEAS] = SMF_CREATE_STATE(motor_state_offset_meas_entry,
				       motor_state_offset_meas_run,
				       motor_state_offset_meas_exit,
				       &motor_states[MOTOR_STATE_CALIBRATION],
				       NULL),
	[MOTOR_STATE_RS_EST] = SMF_CREATE_STATE(motor_state_rs_est_entry,
				  motor_state_rs_est_run,
				  motor_state_rs_est_exit,
				  &motor_states[MOTOR_STATE_CALIBRATION],
				  NULL),
	[MOTOR_STATE_ROVERL_MEAS] = SMF_CREATE_STATE(motor_state_roverl_meas_entry,
				  motor_state_roverl_meas_run,
				  motor_state_roverl_meas_exit,
				  &motor_states[MOTOR_STATE_CALIBRATION],
				  NULL),
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
	case MOTOR_STATE_CALIBRATION:  return "CALIBRATION";
	case MOTOR_STATE_OFFSET_MEAS:  return "OFFSET_MEAS";
	case MOTOR_STATE_RS_EST:       return "RS_EST";
	case MOTOR_STATE_ROVERL_MEAS:  return "ROVERL_MEAS";
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

	/* Initialize hardware GPIO */
	if (motor_hardware_init_gpio() < 0) {
		LOG_ERR("Failed to initialize GPIO");
		motor_api_post_error(ERROR_HARDWARE_BREAK);
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		return;
	}

	/* Check device readiness */
	if (motor_hardware_check_devices() < 0) {
		LOG_ERR("Device readiness check failed");
		motor_api_post_error(ERROR_HARDWARE_BREAK);
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		return;
	}

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
	mcpwm_set_break_callback(pwm1, pwm_break_callback, params);
	mcpwm_set_break_callback(pwm8, pwm_break_callback, params);

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

	config_print_parameters();

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
}

static enum smf_state_result motor_state_ctrl_init_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Transition to calibration parent state (will start with OFFSET_MEAS) */
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);

	return SMF_EVENT_HANDLED;
}

/* State: CALIBRATION - Hierarchical parent state for all calibration sub-states */
static void motor_state_calibration_entry(void *obj)
{
	LOG_INF("=== Starting Motor Calibration Sequence ===");
}

static enum smf_state_result motor_state_calibration_run(void *obj)
{
	/* Parent state run - events propagate to child states.
	 * Initial transition to OFFSET_MEAS handled by SMF engine.
	 */
	return SMF_EVENT_PROPAGATE;
}

static void motor_state_calibration_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("=== Calibration Complete ===");
	LOG_INF("  Rs:    %.4f Ω", (double)params->Rs_measured_ohm);
	LOG_INF("  Ls:    %.6f H", (double)params->Ls_measured_H);
	LOG_INF("  R/L:   %.1f rad/s", (double)params->R_over_L_measured);
}

/* State: OFFSET_MEAS - Measure current sensor offsets */
static void motor_state_offset_meas_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering OFFSET_MEAS state");

	filter_fo_set_initial_conditions(&params->filter_Ia, 0.0f, 0.0f);
	filter_fo_set_initial_conditions(&params->filter_Ib, 0.0f, 0.0f);

	/* Start timer for offset measurement duration (1 second) */
	k_timer_start(&params->state_timer, K_SECONDS(1), K_NO_WAIT);
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
		/* Offset measurement complete - transition to Rs estimation */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_RS_EST]);
		break;

	default:
		/* Ignore other events during calibration */
		break;
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

	/* Reset accumulators and phase */
	params->roverl_accumulator_Vd_Id = 0.0f;
	params->roverl_accumulator_Vq_Id = 0.0f;
	params->roverl_accumulator_Id2 = 0.0f;
	params->roverl_phase_degrees = 0.0f;

	LOG_INF("RoverL: %.0fHz excitation for %.1fs (I_amplitude=%.2fA)",
		(double)ROVERL_EST_FREQ_HZ, (double)ROVERL_EST_DURATION_S,
		(double)ROVERL_EST_CURRENT_A);

	/* Start timer for total R/L estimation duration */
	k_timer_start(&params->state_timer, K_MSEC((uint32_t)(ROVERL_EST_DURATION_S * 1000)), K_NO_WAIT);
}

static enum smf_state_result motor_state_roverl_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
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

		/* Calibration sequence complete - exit to parent, which exits to IDLE */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		break;

	default:
		/* Ignore other events during calibration */
		break;
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

	/* Start timer for total Rs estimation duration (rampup + measurement) */
	float32_t total_duration_s = RS_EST_RAMPUP_S + RS_EST_DURATION_S;
	k_timer_start(&params->state_timer, K_MSEC((uint32_t)(total_duration_s * 1000)), K_NO_WAIT);
}

static enum smf_state_result motor_state_rs_est_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
		/* Get final filtered values */
		float32_t V_est = filter_fo_get_y1(&params->filter_rs_est_V);
		float32_t I_est = filter_fo_get_y1(&params->filter_rs_est_I);
		float32_t Rs_est = V_est / I_est;

		LOG_INF("Rs EST complete: Rs=%.4f Ω (V=%.3fV, I=%.3fA)",
			(double)Rs_est, (double)V_est, (double)I_est);

		/* Update stored Rs value */
		params->Rs_measured_ohm = Rs_est;

		/* Transition to R/L measurement (next calibration step) */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ROVERL_MEAS]);
		break;

	default:
		/* Ignore other events during calibration */
		break;
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

	LOG_INF("Applying alignment current: Id=%.3fA for %.1fs",
		(double)ALIGN_CURRENT_A, (double)ALIGN_DURATION_S);

	/* Start timer for alignment duration */
	k_timer_start(&params->state_timer, K_MSEC((uint32_t)(ALIGN_DURATION_S * 1000)), K_NO_WAIT);
}

static enum smf_state_result motor_state_align_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_TIMEOUT:
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
		break;

	default:
		/* Ignore other events during alignment */
		break;
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_START_REQUEST:
		LOG_INF("Start request received, transitioning to ONLINE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ONLINE]);
		break;

	case MOTOR_EVENT_CALIBRATE_REQUEST:
		LOG_INF("Calibrate request received, starting calibration sequence");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		break;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		break;

	default:
		/* Ignore other events in IDLE state */
		break;
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_STOP_REQUEST:
		LOG_INF("Stop request received, transitioning to IDLE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		break;

	case MOTOR_EVENT_ERROR:
		LOG_WRN("Emergency stop event received, code: %d, transitioning to ERROR", params->event.error_code);
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		break;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		break;

	default:
		/* Ignore other events */
		break;
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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_STOP_REQUEST:
		LOG_INF("Stop request received, transitioning to IDLE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		break;

	case MOTOR_EVENT_ERROR:
		LOG_WRN("Emergency stop event received, code: %d, transitioning to ERROR", params->event.error_code);
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		break;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		break;

	default:
		/* Ignore other events */
		break;
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

	/* Store the error code that triggered this state */
	params->last_error_code = params->event.error_code;

	LOG_ERR("Entering ERROR state, code: %d", params->last_error_code);

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

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_CLEAR_ERROR:
		LOG_INF("Error cleared, transitioning to IDLE");
		params->last_error_code = ERROR_NONE;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		break;

	default:
		/* Ignore all other events in ERROR state */
		break;
	}

	return SMF_EVENT_HANDLED;
}

/* State machine thread function */
static void motor_sm_thread(void *arg1, void *arg2, void *arg3)
{
	int rc;

	/* Initialize motor control API first (zeros the struct, sets defaults) */
	if (motor_control_api_init(&motor_params) < 0) {
		LOG_ERR("Failed to initialize motor control API");
		return;
	}

	/* Initialize state timer */
	k_timer_init(&motor_params.state_timer, state_timer_expiry, NULL);

	/* Initialize state machine */
	smf_set_initial(SMF_CTX(&motor_params), &motor_states[MOTOR_STATE_HW_INIT]);

	/* Event-driven state machine loop */
	while (1) {
		rc = k_msgq_get(&motor_event_queue, &motor_params.event, K_MSEC(10));

		if (rc == 0) {
			/* Run state machine with event */
			rc = smf_run_state(SMF_CTX(&motor_params));

			if (rc) {
				/* State machine terminated */
				LOG_ERR("State machine terminated with code %d", rc);
				break;
			}
		} else if (rc == -EAGAIN) {
			/* No event received within timeout - run state machine with no event */
			struct motor_event evt = {
				.type = MOTOR_EVENT_NONE,
			};
			motor_params.event = evt;
			rc = smf_run_state(SMF_CTX(&motor_params));
			if (rc) {
				/* State machine terminated */
				LOG_ERR("State machine terminated with code %d", rc);
				break;
			}
		}
	}
}

void motor_sm_thread_run(void)
{
	motor_sm_thread_tid = k_thread_create(
		&motor_sm_thread_data,
		motor_sm_thread_stack,
		K_THREAD_STACK_SIZEOF(motor_sm_thread_stack),
		motor_sm_thread,
		NULL, NULL, NULL,
		MOTOR_SM_THREAD_PRIORITY,
		0,
		K_NO_WAIT);

	k_thread_name_set(motor_sm_thread_tid, "motor_sm");
}

struct motor_parameters *motor_sm_get_params(void)
{
	return &motor_params;
}
