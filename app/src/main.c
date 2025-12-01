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
	MOTOR_STATE_ALIGN,
	MOTOR_STATE_IDLE,
	MOTOR_STATE_OFFLINE,
	MOTOR_STATE_ONLINE,
	MOTOR_STATE_ERROR,
};

/* Forward declarations */
static const struct smf_state motor_states[];
static inline int read_encoder(struct rtio* ctx, float32_t* angle);

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

	Id_ref_A = params->Id_ref_A;
	Iq_ref_A = params->Iq_ref_A;

	/* Read encoder and update observer */
	if (read_encoder(&encoder_rtio_ctx, &angle_raw) < 0) {
		/* No new encoder data, use previous estimate */
	} else {
		/* Update observer with new encoder measurement */
		angle_observer_update(&params->observer, angle_raw);
	}	

	Ia = adc_to_current(values[1]);
	Ib = adc_to_current(values[0]);
	Vbus = adc_to_vbus_v(values[2]);
	InvVbus = 1.0f / Vbus;

	/* Handle offset measurement state */
	if (params->smf.current == &motor_states[MOTOR_STATE_OFFSET_MEAS]) {
		filter_fo_run(&params->filter_Ia, Ia);
		filter_fo_run(&params->filter_Ib, Ib);
		params->state_counter++;
		goto isr_done;
	}

	/* Only run control if in ONLINE state */
	// if (params->smf.current != &motor_states[MOTOR_STATE_ONLINE]) {
	// 	goto isr_done;
	// }

	// Remove offsets
	Ia -= params->Ia_offset;
	Ib -= params->Ib_offset;

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

	/* Enable PWM outputs for offset measurement */
	mcpwm_enable(pwm1, 1);
	mcpwm_enable(pwm1, 2);
	mcpwm_enable(pwm8, 1);
	mcpwm_enable(pwm8, 2);
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

	/* Disable PWM outputs */
	mcpwm_disable(pwm1, 1);
	mcpwm_disable(pwm1, 2);
	mcpwm_disable(pwm8, 1);
	mcpwm_disable(pwm8, 2);
}

static enum smf_state_result motor_state_offset_meas_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Wait for sufficient samples to be collected (done in ADC callback) */
	if (params->state_counter >= 1000) {
		/* Transition to alignment */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ALIGN]);
	}

	return SMF_EVENT_HANDLED;
}

/* State: ALIGN - Align rotor to known position */
static void motor_state_align_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ALIGN state");

	/* Set d-axis current reference to alignment current */
	params->Id_ref_A = ALIGN_CURRENT_A;
	params->Iq_ref_A = 0.0f;

	/* Reset alignment timer */
	params->state_counter = 0;

	/* Enable PWM outputs for alignment */
	mcpwm_enable(pwm1, 1);
	mcpwm_enable(pwm1, 2);
	mcpwm_enable(pwm8, 1);
	mcpwm_enable(pwm8, 2);

	LOG_INF("Applying alignment current: Id=%.3fA for %.1fs",
		(double)ALIGN_CURRENT_A, (double)ALIGN_DURATION_S);
}

static enum smf_state_result motor_state_align_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Increment sample counter (this runs at control loop frequency) */
	params->state_counter++;

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

	/* Clear current references */
	params->Id_ref_A = 0.0f;
	params->Iq_ref_A = 0.0f;

	/* Disable PWM outputs */
	mcpwm_disable(pwm1, 1);
	mcpwm_disable(pwm1, 2);
	mcpwm_disable(pwm8, 1);
	mcpwm_disable(pwm8, 2);
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

	/* Disable all outputs */
	mcpwm_set_duty_cycle(pwm1, 1, 0);
	mcpwm_set_duty_cycle(pwm1, 2, 0);
	mcpwm_set_duty_cycle(pwm8, 1, 0);
	mcpwm_set_duty_cycle(pwm8, 2, 0);
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
		.maxVsMag_pu = MAX_VS_MPU,
		.maxVsMag_V = 0.0f,
		.Vd_V = 0.0f,
		.Vq_V = 0.0f,
		.state_counter = 0,
		.error_code = 0,
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
