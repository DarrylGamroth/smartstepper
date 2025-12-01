/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>
#include <drivers/mcpwm.h>
#include <drivers/adc_injected.h>
#include <dt-bindings/pwm/stm32-mcpwm.h>

#include <drivers/pwm/mcpwm_stm32.h>
#include "motor_math.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "pwmgen.h"
#include <string.h>
#include "angle_observer.h"

/* Include encoder-specific headers based on devicetree */
#if DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955)
#include <drivers/sensor/brcm_aeat9955.h>
#define encoder_decode_position_q31 aeat9955_decode_position_q31
#define encoder_decode_position_f32 aeat9955_decode_position_f32
#elif DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), magntek_mt6835)
#include <drivers/sensor/magntek_mt6835.h>
#define encoder_decode_position_q31 mt6835_decode_position_q31
#define encoder_decode_position_f32 mt6835_decode_position_f32
#else
#error "Unsupported encoder type for encoder1 alias"
#endif

const struct device *const pwm1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
const struct device *const pwm8 = DEVICE_DT_GET(DT_NODELABEL(pwm8));
const struct device *const pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));
const struct device *const encoder1 = DEVICE_DT_GET(DT_ALIAS(encoder1));
const struct device *const adc1 = DEVICE_DT_GET(DT_NODELABEL(adc1));

static const struct gpio_dt_spec pi_enable = GPIO_DT_SPEC_GET(DT_PATH(photo_interruptor_enable), gpios);
static const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(DT_PATH(trig), gpios);

SENSOR_DT_READ_IODEV(encoder1_iodev, DT_ALIAS(encoder1), {SENSOR_CHAN_ROTATION, 0});
RTIO_DEFINE_WITH_MEMPOOL(encoder_rtio_ctx, 8, 8, 16, 16, sizeof(void *));

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* Motor control state machine states */
enum motor_state {
	MOTOR_STATE_HW_INIT,
	MOTOR_STATE_CTRL_INIT,
	MOTOR_STATE_OFFSET_MEAS,
	MOTOR_STATE_ROVERL_MEAS,
	MOTOR_STATE_RS_EST,
	MOTOR_STATE_ALIGN,
	MOTOR_STATE_IDLE,
	MOTOR_STATE_OFFLINE,
	MOTOR_STATE_ONLINE,
	MOTOR_STATE_ERROR,
};

/* Error codes for fault handling */
enum motor_error {
	ERROR_NONE = 0,
	ERROR_HARDWARE_BREAK = 1,
	ERROR_OVERCURRENT = 2,
	ERROR_ENCODER_FAULT = 3,
	ERROR_OVERVOLTAGE = 4,
};

/* Forward declarations */
static const struct smf_state motor_states[];
static inline int read_encoder(struct rtio* ctx, float32_t* angle);

/* Break interrupt handler - hardware fault protection */
static void break_interrupt_handler(const struct device *dev, void *user_data)
{
	struct motor_parameters *params = (struct motor_parameters *)user_data;
	
	/* Hardware has already disabled PWM via break input
	 * This interrupt fires when BKIN pin goes active (overcurrent, external fault)
	 */
	params->error_code = ERROR_HARDWARE_BREAK;
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
	
	LOG_ERR("Hardware break interrupt - PWM disabled by hardware");
}

/* ADC ISR callback - process completed sensor reads */
static void adc_callback(const struct device *dev, const q31_t *values,
					uint8_t count, void *user_data)
{
	gpio_pin_set_dt(&trig, 1);

	struct motor_parameters *params = (struct motor_parameters *)user_data;
	float32_t angle_raw = 0;
	float32_t sinval, cosval;
	float32_t Ia, Ib;
	float32_t Vbus, InvVbus;
	float32_t Id, Iq;
	float32_t Va, Vb;
	float32_t Ua, Ub;
	float32_t Da, Db;
	float32_t Da_high, Da_low, Db_high, Db_low;
	float32_t out_max;
	float32_t Id_ref_A, Iq_ref_A;
	
	q31_t *adc_values[3];
	adc_values[0] = (q31_t *)&values[0]; /* I_b */
	adc_values[1] = (q31_t *)&values[1]; /* I_a */
	adc_values[2] = (q31_t *)&values[2]; /* V_bus */

	/* Read encoder and update observer */
	if (read_encoder(&encoder_rtio_ctx, &angle_raw) < 0) {
		/* No new encoder data, use previous estimate */
		params->encoder_fault_counter++;
		
		/* Fault detection: Too many consecutive encoder failures */
		if (params->encoder_fault_counter > ENCODER_FAULT_THRESHOLD) {
			params->error_code = ERROR_ENCODER_FAULT;
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
			LOG_ERR("Encoder fault: %u consecutive read failures",
				params->encoder_fault_counter);
			goto isr_done;
		}
	} else {
		/* Update observer with new encoder measurement */
		angle_observer_update(&params->observer, angle_raw);
		/* Reset fault counter on successful read */
		params->encoder_fault_counter = 0;
	}	

	Ia = adc_to_current(values[1]);
	Ib = adc_to_current(values[0]);
	Vbus = adc_to_vbus_v(values[2]);
	InvVbus = 1.0f / Vbus;

	/* Fault detection: Check for overvoltage */
	if (Vbus > VBUS_MAX_V) {
		params->error_code = ERROR_OVERVOLTAGE;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		LOG_ERR("Overvoltage fault: Vbus=%.1fV", (double)Vbus);
		goto isr_done;
	}

	/* Handle offset measurement (no control, just filtering) */
	if (params->smf.current == &motor_states[MOTOR_STATE_OFFSET_MEAS]) {
		filter_fo_run(&params->filter_Ia, Ia);
		filter_fo_run(&params->filter_Ib, Ib);
		params->state_counter++;
		goto isr_done;
	}

	/* All other states use PI control - remove offsets first */
	Ia -= params->Ia_offset;
	Ib -= params->Ib_offset;

	/* Fault detection: Check for overcurrent after offset removal */
	if (fabsf(Ia) > OVERCURRENT_THRESHOLD_A || fabsf(Ib) > OVERCURRENT_THRESHOLD_A) {
		params->error_code = ERROR_OVERCURRENT;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		LOG_ERR("Overcurrent fault: Ia=%.2fA, Ib=%.2fA", (double)Ia, (double)Ib);
		goto isr_done;
	}	

	/* RoverL measurement: rotate current reference and accumulate V/I in rotating frame */
	if (params->smf.current == &motor_states[MOTOR_STATE_ROVERL_MEAS]) {
		params->state_counter++;
		
		/* Ramp up current amplitude smoothly (TI method) */
		traj_run(&params->traj_Id);
		float32_t roverl_amplitude_A = traj_get_int_value(&params->traj_Id);
		
		/* Advance excitation phase at configured frequency */
		float32_t phase_increment_deg = 360.0f * ROVERL_EST_FREQ_HZ / CONTROL_LOOP_FREQUENCY_HZ;
		params->roverl_phase_deg += phase_increment_deg;
		if (params->roverl_phase_deg >= 360.0f) {
			params->roverl_phase_deg -= 360.0f;
		}
		
		/* Rotate ramped amplitude by roverl_phase to create rotating vector
		 * Inverse Park: (Id, Iq) = (I_amplitude * cos(θ), I_amplitude * sin(θ))
		 */
		float32_t sin_roverl, cos_roverl;
		arm_sin_cos_f32(params->roverl_phase_deg, &sin_roverl, &cos_roverl);
		params->Id_ref_A = roverl_amplitude_A * cos_roverl;
		params->Iq_ref_A = roverl_amplitude_A * sin_roverl;
		
		/* Wait for PI settling before accumulating measurements */
		uint32_t settling_samples = (uint32_t)(ROVERL_EST_SETTLING_S * CONTROL_LOOP_FREQUENCY_HZ);
		if (params->state_counter > settling_samples) {
			/* Transform to rotating frame aligned with excitation */
			float32_t Id_meas, Iq_meas;
			arm_park_f32(Ia, Ib, &Id_meas, &Iq_meas, sin_roverl, cos_roverl);
			
			/* Transform actual voltage outputs for phase calculation */
			float32_t Vd_meas, Vq_meas;
			arm_park_f32(params->Vd_V, params->Vq_V, &Vd_meas, &Vq_meas,
			             sin_roverl, cos_roverl);
			
			/* Accumulate for R/L extraction */
			params->roverl_sum_Vd_Id += Vd_meas * Id_meas;
			params->roverl_sum_Vq_Id += Vq_meas * Id_meas;
			params->roverl_sum_Id2 += Id_meas * Id_meas;
		}
	}

	/* Rs EST: filter V/I in d-axis for DC resistance */
	if (params->smf.current == &motor_states[MOTOR_STATE_RS_EST]) {
		params->state_counter++;
		
		uint32_t rampup_samples = (uint32_t)(RS_EST_RAMPUP_S * CONTROL_LOOP_FREQUENCY_HZ);
		
		/* Transform to stationary dq (0°) for measurement */
		float32_t Id_meas, Iq_meas;
		arm_park_f32(Ia, Ib, &Id_meas, &Iq_meas, 0.0f, 1.0f);
		
		/* Run trajectory (clamps at target after rampup) */
		traj_run(&params->traj_Id);
		params->Id_ref_A = traj_get_int_value(&params->traj_Id);
		
		/* After rampup: filter voltage and current measurements */
		if (params->state_counter >= rampup_samples) {
			filter_fo_run(&params->filter_rs_est_V, params->Vd_V);
			filter_fo_run(&params->filter_rs_est_I, Id_meas);
		}
	}

	/* ALIGN: ramp alignment current smoothly */
	if (params->smf.current == &motor_states[MOTOR_STATE_ALIGN]) {
		params->state_counter++;
		
		/* Run trajectory for smooth current ramp */
		traj_run(&params->traj_Id);
		params->Id_ref_A = traj_get_int_value(&params->traj_Id);
	}

	/* Run PI controllers for all states (measurement and control) */
	Id_ref_A = params->Id_ref_A;
	Iq_ref_A = params->Iq_ref_A;

	/* Safety: Don't run control if in ERROR state (PWM should be disabled) */
	if (params->smf.current == &motor_states[MOTOR_STATE_ERROR]) {
		goto isr_done;
	}

	/* Get current electrical angle for Park transform */
	float32_t elec_angle_deg = angle_observer_get_elec_angle_deg(&params->observer);
	arm_sin_cos_f32(elec_angle_deg, &sinval, &cosval);

	arm_park_f32(Ia, Ib, &Id, &Iq, sinval, cosval);

	params->maxVsMag_V = params->maxVsMag_pu * Vbus;
	pi_set_min_max(&params->pi_Id, -params->maxVsMag_V, params->maxVsMag_V);
	pi_run_series(&params->pi_Id, Id_ref_A, Id, 0.0f, &params->Vd_V);

	out_max = sqrtf((params->maxVsMag_V * params->maxVsMag_V) - (params->Vd_V * params->Vd_V));
	pi_set_min_max(&params->pi_Iq, -out_max, out_max);
	pi_run_series(&params->pi_Iq, Iq_ref_A, Iq, 0.0f, &params->Vq_V);

	/* Use predicted electrical angle for inverse Park transform */
	float32_t elec_angle_pred_deg = angle_observer_get_elec_angle_pred_deg(&params->observer);
	arm_sin_cos_f32(elec_angle_pred_deg, &sinval, &cosval);

	arm_inv_park_f32(params->Vd_V, params->Vq_V, &Va, &Vb, sinval, cosval);

	/* Normalize voltage commands to available bus voltage */
	Ua = Va * InvVbus;
	Ub = Vb * InvVbus;

	// Apply SVPWM modulation
	pwmgen_svpwm_2phase_f32(Ua, Ub, &Da, &Db);

	// Default: complementary PWM for normal FOC control
	Da_high = Da;
	Da_low = -Da;
	Db_high = Db;
	Db_low = -Db;

	// Vbus-regulated braking: blend between regen and short-circuit
	// Braking occurs when torque opposes motion (Iq and speed have opposite signs)
	float32_t speed_hz = angle_observer_get_mech_speed_hz(&params->observer);
	bool is_braking = (params->Iq_ref_A * speed_hz) < 0.0f;  // Opposite signs = braking
	
	if (is_braking && fabsf(speed_hz) > 0.1f) {  // Braking mode (ignore near-zero speeds)
		if (Vbus > VBUS_REGEN_LIMIT_V) {
			// Calculate short-circuit duty based on overvoltage
			float32_t overvoltage = Vbus - VBUS_REGEN_LIMIT_V;
			float32_t short_duty = fminf(1.0f, overvoltage * VBUS_VOLTAGE_MARGIN_INV);
			
			// Blend: reduce regen FOC PWM, increase short-circuit (both low-side ON)
			float32_t regen_duty = 1.0f - short_duty;
			Da_high = Da * regen_duty;
			Da_low = -Da * regen_duty - short_duty;
			Db_high = Db * regen_duty;
			Db_low = -Db * regen_duty - short_duty;
		}
	}

	// Output PWM values to H-bridges
	// Winding A: pwm1 ch1 and ch2 drive the two sides of H-bridge A
	// Winding B: pwm8 ch1 and ch2 drive the two sides of H-bridge B
	mcpwm_stm32_set_duty_cycle_2phase(pwm1, Da_high, Da_low);
	mcpwm_stm32_set_duty_cycle_2phase(pwm8, Db_high, Db_low);
isr_done:
	gpio_pin_set_dt(&trig, 0);

}

static void encoder1_callback(const struct device *dev, uint32_t channel,
					  void *user_data)
{
	(void)sensor_read_async_mempool(&encoder1_iodev, &encoder_rtio_ctx, (void *)&encoder1_iodev);
}

static inline int read_encoder(struct rtio* ctx, float32_t* angle)
{
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;
	int rc;

	/* Non-blocking: check if a completion is ready */
	cqe = rtio_cqe_consume(ctx);
	if (cqe == NULL) {
		/* No completion ready yet */
		return -1;
	}

	if (cqe->result != 0) {
		rtio_cqe_release(ctx, cqe);
		return -1;
	}

	rc = rtio_cqe_get_mempool_buffer(ctx, cqe, &buf, &buf_len);
	if (rc != 0) {
		rtio_cqe_release(ctx, cqe);
		return -1;
	}

	/* Fast-path decode: directly extract Q31 angle from RTIO buffer */
	*angle = encoder_decode_position_f32(buf);

	rtio_cqe_release(&encoder_rtio_ctx, cqe);
	rtio_release_buffer(ctx, buf, buf_len);

	return 0;
}

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

/* State machine table */
static const struct smf_state motor_states[] = {
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

	/* TI RoverL uses small sinusoidal current excitation
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
	params->roverl_sum_Vd_Id = 0.0f;
	params->roverl_sum_Vq_Id = 0.0f;
	params->roverl_sum_Id2 = 0.0f;
	params->roverl_phase_deg = 0.0f;

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
		
		float32_t R_est = params->roverl_sum_Vd_Id / params->roverl_sum_Id2;
		float32_t omega_L_est = params->roverl_sum_Vq_Id / params->roverl_sum_Id2;
		float32_t omega_roverl = 2.0f * PI_F32 * ROVERL_EST_FREQ_HZ;
		float32_t L_est = omega_L_est / omega_roverl;
		float32_t RoverL = R_est / L_est;
		float32_t tau = L_est / R_est;

		LOG_INF("RoverL complete: R=%.4f Ω, L=%.2f µH, R/L=%.0f, τ=%.2f ms",
			(double)R_est, (double)(L_est * 1e6f),
			(double)RoverL, (double)(tau * 1000.0f));

		/* Store measured values */
		params->Rs_measured = R_est;
		params->L_measured = L_est;
		params->RoverL_measured = RoverL;

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
	params->roverl_phase_deg = 0.0f;
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
		(double)RS_EST_RAMPUP_S, (double)(RS_EST_COARSE_S + RS_EST_FINE_S));
}

static enum smf_state_result motor_state_rs_est_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Calculate phase boundaries */
	uint32_t rampup_samples = (uint32_t)(RS_EST_RAMPUP_S * CONTROL_LOOP_FREQUENCY_HZ);
	uint32_t measurement_samples = (uint32_t)((RS_EST_COARSE_S + RS_EST_FINE_S) * CONTROL_LOOP_FREQUENCY_HZ);
	uint32_t total_samples = rampup_samples + measurement_samples;

	if (params->state_counter >= total_samples) {
		/* Get final filtered values */
		float32_t V_est = filter_fo_get_y1(&params->filter_rs_est_V);
		float32_t I_est = filter_fo_get_y1(&params->filter_rs_est_I);
		float32_t Rs_est = V_est / I_est;

		LOG_INF("Rs EST complete: Rs=%.4f Ω (V=%.3fV, I=%.3fA)",
			(double)Rs_est, (double)V_est, (double)I_est);

		/* Update stored Rs value */
		params->Rs_measured = Rs_est;

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
	/* Wait for command to go OFFLINE or ONLINE */

	return SMF_EVENT_HANDLED;
}

/* State: OFFLINE - Open-loop control */
static void motor_state_offline_entry(void *obj)
{
	LOG_INF("Entering OFFLINE state");
}

static enum smf_state_result motor_state_offline_run(void *obj)
{
	/* Run open-loop control */

	return SMF_EVENT_HANDLED;
}

/* State: ONLINE - Closed-loop control */
static void motor_state_online_entry(void *obj)
{
	LOG_INF("Entering ONLINE state");
}

static enum smf_state_result motor_state_online_run(void *obj)
{
	/* Closed-loop control happens in ADC ISR */

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
	/* Stay in error state until reset */

	return SMF_EVENT_HANDLED;
}

int main(void)
{
	static struct motor_parameters motor_params = {
		.Ia_offset = 0.0f,
		.Ib_offset = 0.0f,
		.Id_setpoint_A = 0.0f,
		.Iq_setpoint_A = 0.0f,
		.Id_ref_A = 0.0f,
		.Iq_ref_A = 0.0f,
		.Vd_ref_V = 0.0f,
		.Vq_ref_V = 0.0f,
		.maxVsMag_pu = MAX_VS_MPU,
		.maxVsMag_V = 0.0f,
		.Vd_V = 0.0f,
		.Vq_V = 0.0f,
		.RoverL_measured = 0.0f,
		.L_measured = 0.0f,
		.Rs_measured = 0.0f,
		.roverl_sum_Vd_Id = 0.0f,
		.roverl_sum_Vq_Id = 0.0f,
		.roverl_sum_Id2 = 0.0f,
		.roverl_phase_deg = 0.0f,
		.state_counter = 0,
		.encoder_fault_counter = 0,
		.error_code = ERROR_NONE,
	};

	/* Print configuration parameters */
	config_print_parameters();

	/* Configure GPIOs */
	if (!gpio_is_ready_dt(&pi_enable)) {
		printk("GPIO PE1 device is not ready\n");
		return 0;
	}
	gpio_pin_configure_dt(&pi_enable, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&trig, GPIO_OUTPUT_INACTIVE);

	/* Check device readiness */
	if (!device_is_ready(pwm1)) {
		printk("PWM1 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm3)) {
		printk("PWM3 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(pwm8)) {
		printk("PWM8 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(encoder1)) {
		printk("encoder1 device is not ready\n");
		return 0;
	}
	if (!device_is_ready(adc1)) {
		printk("ADC1 device is not ready\n");
		return 0;
	}

	/* Initialize state machine */
	smf_set_initial(SMF_CTX(&motor_params), &motor_states[MOTOR_STATE_HW_INIT]);

	while (1) {
		/* Run state machine */
		if (smf_run_state(SMF_CTX(&motor_params)) < 0) {
			LOG_ERR("State machine error\n");
			break;
		}

		k_sleep(K_MSEC(100));
	}

	return 0;
}
