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
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>
#include <zephyr/timing/timing.h>
#include <drivers/mcpwm.h>
#include <drivers/adc_injected.h>
#include <drivers/pwm/mcpwm_stm32.h>

#include "motor_control_api.h"
#include "motor_isr.h"
#include "motor_states.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "pwmgen.h"
#include "angle_observer.h"

/**
 * @brief Convert Q31 ADC value to current in Amperes
 *
 * @param q31_value Q31 format ADC reading (-1.0 to +1.0 represented as int32)
 * @return Current in Amperes
 */
static inline float32_t adc_to_current(q31_t q31_value)
{
	float32_t normalized = (float32_t)q31_value / (float32_t)(1U << 31);
	return normalized * CURRENT_SENSE_FULL_SCALE_A;
}

/**
 * @brief Convert Q31 ADC value to bus voltage in Volts
 *
 * @param q31_value Q31 format ADC reading (0.0 to +1.0 represented as int32)
 * @return Bus voltage in Volts
 */
static inline float32_t adc_to_vbus_v(q31_t q31_value)
{
	float32_t normalized = (float32_t)q31_value / (float32_t)(1U << 31);
	return normalized * VBUS_FULL_SCALE_V;
}

/* Include encoder-specific headers based on devicetree */
#if DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), brcm_aeat_9955)
#include <drivers/sensor/brcm_aeat9955.h>
#define encoder_decode_position_f32 aeat9955_decode_position_f32
#elif DT_NODE_HAS_COMPAT(DT_ALIAS(encoder1), magntek_mt6835)
#include <drivers/sensor/magntek_mt6835.h>
#define encoder_decode_position_f32 mt6835_decode_position_f32
#else
#error "Unsupported encoder type for encoder1 alias"
#endif

LOG_MODULE_REGISTER(motor_isr, CONFIG_APP_LOG_LEVEL);

SENSOR_DT_READ_IODEV(encoder1_iodev, DT_ALIAS(encoder1), {SENSOR_CHAN_ROTATION, 0});
RTIO_DEFINE_WITH_MEMPOOL(encoder_rtio_ctx, 8, 8, 16, 16, sizeof(void *));

static inline int encoder_read(struct rtio *ctx, float32_t *angle)
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

    rtio_cqe_release(ctx, cqe);

	/* Fast-path decode: directly extract angle from RTIO buffer */
	*angle = encoder_decode_position_f32(buf);

	rtio_release_buffer(ctx, buf, buf_len);

	return 0;
}

void pwm_break_callback(const struct device *dev, void *user_data)
{
	/* Hardware has already disabled PWM via break input
	 * This interrupt fires when BKIN pin goes active (overcurrent, external fault)
	 */
	motor_api_post_error(ERROR_HARDWARE_BREAK);
}

void adc_callback(const struct device *dev, const q31_t *values,
                  uint8_t count, void *user_data)
{
	gpio_pin_set_dt(&trig, 1);

	/* Start ISR cycle counter */
	timing_t cycles_start = timing_counter_get();

	struct motor_parameters *params = (struct motor_parameters *)user_data;

	/* Double-buffer swap: atomically toggle index between 0 and 1
	 * Shell writes to ctrl_buf[ctrl_index^1], ISR reads from ctrl_buf[ctrl_index].
	 * XOR toggle is single instruction - minimal ISR overhead.
	 */
	if (params->ctrl_swap_pending) {
		params->ctrl_index ^= 1;
		params->ctrl_swap_pending = false;
		params->buffer_swap_count++;
	}

	/* Increment control loop counter */
	params->control_loop_count++;

	float32_t angle_raw_degrees = 0;
	float32_t sin_theta, cos_theta;
	float32_t Ia_A, Ib_A;
	float32_t Vbus_V, Vbus_inv;
	float32_t Id_A, Iq_A;
	float32_t Va_V, Vb_V;
	float32_t Ua_pu, Ub_pu;
	float32_t Da_pu, Db_pu;
	float32_t Da_high_pu, Da_low_pu, Db_high_pu, Db_low_pu;
	float32_t Vq_limit_V;
	float32_t Id_ref_A, Iq_ref_A;

	q31_t *adc_values[3];
	adc_values[0] = (q31_t *)&values[0]; /* I_b */
	adc_values[1] = (q31_t *)&values[1]; /* I_a */
	adc_values[2] = (q31_t *)&values[2]; /* V_bus */

	/* Read encoder and update observer */
	if (encoder_read(&encoder_rtio_ctx, &angle_raw_degrees) < 0) {
		/* No new encoder data, use previous estimate */
		params->encoder_fault_counter++;

		/* Fault detection: Too many consecutive encoder failures */
		if (params->encoder_fault_counter > ENCODER_FAULT_THRESHOLD) {
			motor_api_post_error(ERROR_ENCODER_FAULT);
			goto isr_done;
		}
	} else {
		/* Update observer with new encoder measurement */
		angle_observer_update(&params->observer, angle_raw_degrees);
		/* Reset fault counter on successful read */
		params->encoder_fault_counter = 0;
	}

	Ia_A = adc_to_current(values[1]);
	Ib_A = adc_to_current(values[0]);
	Vbus_V = adc_to_vbus_v(values[2]);
	Vbus_inv = 1.0f / Vbus_V;

	/* Fault detection: Check for overvoltage */
	if (Vbus_V > VBUS_MAX_V) {
		motor_api_post_error(ERROR_OVERVOLTAGE);
		goto isr_done;
	}

	/* Handle offset measurement (no control, just filtering) */
	if (params->smf.current == &motor_states[MOTOR_STATE_OFFSET_MEAS]) {
		filter_fo_run(&params->filter_Ia, Ia_A);
		filter_fo_run(&params->filter_Ib, Ib_A);
		goto isr_done;
	}

	/* All other states use PI control - remove offsets first */
	Ia_A -= params->Ia_offset;
	Ib_A -= params->Ib_offset;

	/* Fault detection: Check for overcurrent after offset removal */
	if (fabsf(Ia_A) > OVERCURRENT_THRESHOLD_A || fabsf(Ib_A) > OVERCURRENT_THRESHOLD_A) {
		motor_api_post_error(ERROR_OVERCURRENT);
		goto isr_done;
	}

	/* R/L measurement: rotate current reference and accumulate V/I in rotating frame */
	if (params->smf.current == &motor_states[MOTOR_STATE_ROVERL_MEAS]) {
		/* Ramp up current amplitude smoothly (TI method) */
		traj_run(&params->traj_Id);
		float32_t roverl_amplitude_A = traj_get_int_value(&params->traj_Id);

		/* Advance excitation phase at configured frequency */
		float32_t phase_increment_deg = 360.0f * ROVERL_EST_FREQ_HZ / CONTROL_LOOP_FREQUENCY_HZ;
		params->roverl_phase_degrees += phase_increment_deg;
		if (params->roverl_phase_degrees >= 360.0f) {
			params->roverl_phase_degrees -= 360.0f;
		}

		/* Rotate ramped amplitude by roverl_phase to create rotating vector
		 * Inverse Park: (Id, Iq) = (I_amplitude * cos(θ), I_amplitude * sin(θ))
		 * Write to double buffer (state machine has write access)
		 */
		float32_t sin_roverl, cos_roverl;
		arm_sin_cos_f32(params->roverl_phase_degrees, &sin_roverl, &cos_roverl);
		params->Id_ref_A = roverl_amplitude_A * cos_roverl;
		params->Iq_ref_A = roverl_amplitude_A * sin_roverl;

		/* Check if settling period complete using trajectory target */
		if (traj_is_at_target(&params->traj_Id)) {
			/* Transform to rotating frame aligned with excitation */
			float32_t Id_meas, Iq_meas;
			arm_park_f32(Ia_A, Ib_A, &Id_meas, &Iq_meas, sin_roverl, cos_roverl);

			/* Transform actual voltage outputs for phase calculation */
			float32_t Vd_meas, Vq_meas;
			arm_park_f32(params->Vd_V, params->Vq_V, &Vd_meas, &Vq_meas,
			             sin_roverl, cos_roverl);

			/* Accumulate for R/L extraction */
			params->roverl_accumulator_Vd_Id += Vd_meas * Id_meas;
			params->roverl_accumulator_Vq_Id += Vq_meas * Id_meas;
			params->roverl_accumulator_Id2 += Id_meas * Id_meas;
		}
	}

	/* Rs EST: filter V/I in d-axis for DC resistance */
	if (params->smf.current == &motor_states[MOTOR_STATE_RS_EST]) {
		/* Transform to stationary dq (0°) for measurement */
		float32_t Id_meas, Iq_meas;
		arm_park_f32(Ia_A, Ib_A, &Id_meas, &Iq_meas, 0.0f, 1.0f);

		/* Run trajectory (clamps at target after rampup) */
		traj_run(&params->traj_Id);
		params->Id_ref_A = traj_get_int_value(&params->traj_Id);

		/* After rampup complete: filter voltage and current measurements */
		if (traj_is_at_target(&params->traj_Id)) {
			filter_fo_run(&params->filter_rs_est_V, params->Vd_V);
			filter_fo_run(&params->filter_rs_est_I, Id_meas);
		}
	}

	/* ALIGN: ramp alignment current smoothly */
	if (params->smf.current == &motor_states[MOTOR_STATE_ALIGN]) {
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
	arm_sin_cos_f32(elec_angle_deg, &sin_theta, &cos_theta);

	arm_park_f32(Ia_A, Ib_A, &Id_A, &Iq_A, sin_theta, cos_theta);

	/* Update telemetry snapshot (convert from deg/Hz to rad/rad_s) */
	params->position_rad = angle_observer_get_mech_angle_deg(&params->observer) * (PI_F32 / 180.0f);
	params->velocity_rad_s = angle_observer_get_mech_speed_hz(&params->observer) * (2.0f * PI_F32);
	params->Id_A = Id_A;
	params->Iq_A = Iq_A;
	params->Ia_A = Ia_A;
	params->Ib_A = Ib_A;
	params->elec_angle_rad = elec_angle_deg * (PI_F32 / 180.0f);
	params->dc_bus_voltage_V = Vbus_V;

	params->max_voltage_magnitude_V = params->max_modulation_index * Vbus_V;
	pi_set_min_max(&params->pi_Id, -params->max_voltage_magnitude_V, params->max_voltage_magnitude_V);
	pi_run_series(&params->pi_Id, Id_ref_A, Id_A, 0.0f, &params->Vd_V);

	Vq_limit_V = sqrtf((params->max_voltage_magnitude_V * params->max_voltage_magnitude_V) - (params->Vd_V * params->Vd_V));
	pi_set_min_max(&params->pi_Iq, -Vq_limit_V, Vq_limit_V);
	pi_run_series(&params->pi_Iq, Iq_ref_A, Iq_A, 0.0f, &params->Vq_V);

	/* Use predicted electrical angle for inverse Park transform */
	float32_t elec_angle_pred_deg = angle_observer_get_elec_angle_pred_deg(&params->observer);
	arm_sin_cos_f32(elec_angle_pred_deg, &sin_theta, &cos_theta);

	arm_inv_park_f32(params->Vd_V, params->Vq_V, &Va_V, &Vb_V, sin_theta, cos_theta);

	/* Update voltage telemetry snapshot */
	params->Va_V = Va_V;
	params->Vb_V = Vb_V;

	/* Normalize voltage commands to available bus voltage */
	Ua_pu = Va_V * Vbus_inv;
	Ub_pu = Vb_V * Vbus_inv;

	/* Apply SVPWM modulation */
	pwmgen_svpwm_2phase_f32(Ua_pu, Ub_pu, &Da_pu, &Db_pu);

	/* Default: complementary PWM for normal FOC control */
	Da_high_pu = Da_pu;
	Da_low_pu = -Da_pu;
	Db_high_pu = Db_pu;
	Db_low_pu = -Db_pu;

	/* Vbus-regulated braking: blend between regen and short-circuit
	 * Braking occurs when torque opposes motion (Iq and speed have opposite signs)
	 */
	float32_t speed_hz = angle_observer_get_mech_speed_hz(&params->observer);
	bool is_braking = (params->Iq_ref_A * speed_hz) < 0.0f;  /* Opposite signs = braking */

	if (is_braking && fabsf(speed_hz) > 0.1f) {  /* Braking mode (ignore near-zero speeds) */
		if (Vbus_V > VBUS_REGEN_LIMIT_V) {
			/* Calculate short-circuit duty based on overvoltage */
			float32_t overvoltage = Vbus_V - VBUS_REGEN_LIMIT_V;
			float32_t short_duty = fminf(1.0f, overvoltage * VBUS_VOLTAGE_MARGIN_INV);

			/* Blend: reduce regen FOC PWM, increase short-circuit (both low-side ON) */
			float32_t regen_duty = 1.0f - short_duty;
			Da_high_pu = Da_pu * regen_duty;
			Da_low_pu = -Da_pu * regen_duty - short_duty;
			Db_high_pu = Db_pu * regen_duty;
			Db_low_pu = -Db_pu * regen_duty - short_duty;
		}
	}

	/* Output PWM values to H-bridges
	 * Winding A: pwm1 ch1 and ch2 drive the two sides of H-bridge A
	 * Winding B: pwm8 ch1 and ch2 drive the two sides of H-bridge B
	 */
	mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1, Da_high_pu, Da_low_pu);
	mcpwm_stm32_set_duty_cycle_2phase_f32(pwm8, Db_high_pu, Db_low_pu);

isr_done:
	/* Measure ISR execution time */
	timing_t cycles_end = timing_counter_get();
	uint64_t cycles_elapsed = timing_cycles_get(&cycles_start, &cycles_end);
	params->total_isr_cycles += (uint32_t)cycles_elapsed;
	if (cycles_elapsed > params->max_isr_cycles) {
		params->max_isr_cycles = (uint32_t)cycles_elapsed;
	}

	gpio_pin_set_dt(&trig, 0);
}

void encoder1_callback(const struct device *dev, uint32_t channel,
                       void *user_data)
{
	(void)sensor_read_async_mempool(&encoder1_iodev, &encoder_rtio_ctx, NULL);
}
