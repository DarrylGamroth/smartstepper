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
#include <zephyr/sys/atomic.h>
#include <drivers/mcpwm.h>
#include <drivers/adc_injected.h>
#include <drivers/pwm/mcpwm_stm32.h>
#include <drivers/gate_driver/ti_drv8328.h>

#include "motor_control_api.h"
#include "motor_isr.h"
#include "motor_states.h"
#include "math_constants.h"
#include "config.h"
#include "pi.h"
#include "filter_fo.h"
#include "traj.h"
#include "pwmgen.h"
#include "angle_observer.h"
#include "angle_gen.h"
#include "angle_wrap.h"
#include "motor_state_utils.h"

/**
 * @brief Convert Q31 ADC value to current in Amperes
 *
 * @param q31_value Q31 format ADC reading (0.0 to +1.0 represented as int32)
 * @param polarity Polarity multiplier (1 or -1)
 * @return Current in Amperes (offset removal done separately)
 */
static inline float32_t adc_to_current(q31_t q31_value, int polarity)
{
	float32_t normalized = (float32_t)q31_value / (float32_t)(1U << 31);
	/* Direct scaling - offset measurement handles midpoint centering */
	return (float32_t)polarity * normalized * CURRENT_SENSE_FULL_SCALE_A * 2.0f;
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
static atomic_t encoder_read_in_flight;

#define VBUS_MIN_VALID_V 0.1f

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
		atomic_set(&encoder_read_in_flight, 0);
		return -1;
	}

	rc = rtio_cqe_get_mempool_buffer(ctx, cqe, &buf, &buf_len);
	if (rc != 0) {
		rtio_cqe_release(ctx, cqe);
		atomic_set(&encoder_read_in_flight, 0);
		return -1;
	}

	rtio_cqe_release(ctx, cqe);
	atomic_set(&encoder_read_in_flight, 0);

	/* Fast-path decode: directly extract angle from RTIO buffer */
	*angle = encoder_decode_position_f32(buf);

	rtio_release_buffer(ctx, buf, buf_len);

	return 0;
}

void gate_driver_a_break_callback(const struct device *dev, void *user_data)
{
	/* Hardware has already disabled PWM via break input
	 * This interrupt fires when BKIN pin goes active (overcurrent, external fault)
	 */

	drv8328_disable_all_channels(gate_driver_a);

	/* Post error after driver handles disable all channels */
	motor_api_post_error(ERROR_HARDWARE_BREAK);
}

void gate_driver_b_break_callback(const struct device *dev, void *user_data)
{
	/* Hardware has already disabled PWM via break input
	 * This interrupt fires when BKIN pin goes active (overcurrent, external fault)
	 */

	drv8328_disable_all_channels(gate_driver_b);

	/* Post error after driver handles disable all channels */
	motor_api_post_error(ERROR_HARDWARE_BREAK);
}

void adc_callback(const struct device *dev, const q31_t *values,
                  uint8_t count, void *user_data)
{
	gpio_pin_set_dt(&trig, 1);

	/* Start ISR cycle counter */
	timing_t cycles_start = timing_counter_get();

	struct motor_parameters *params = (struct motor_parameters *)user_data;
	const struct smf_state *state = params->state_for_isr;
	bool online_control_state = motor_state_ptr_is_online_control_state(state);
	bool control_armed = atomic_get(&params->control_armed) != 0;

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
	float32_t Da_hb1_pu, Da_hb2_pu, Db_hb1_pu, Db_hb2_pu;
	float32_t Vq_limit_V;
	float32_t Id_ref_A = 0.0f, Iq_ref_A = 0.0f;
	float32_t Vd_V, Vq_V;
	float32_t max_voltage_magnitude_V;
	float32_t velocity_target_rad_s = params->velocity_target_rad_s;
	float32_t velocity_ref_rad_s = params->velocity_ref_rad_s;
	uint8_t encoder_input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;

	/* Timeout disarms output commands when command updates stop. */
	if (online_control_state && control_armed && params->command_timeout_ms > 0U) {
		uint32_t now_ms = k_uptime_get_32();
		uint32_t elapsed_ms = now_ms - params->last_command_update_ms;

		if (elapsed_ms > params->command_timeout_ms) {
			control_armed = false;
			atomic_set(&params->control_armed, 0);
			if (!params->command_timeout_latched) {
				params->command_timeout_latched = true;
				params->command_timeout_count++;
			}
		}
	}

	/* Read encoder if feature is enabled */
	float32_t angle_raw_rad;
	bool fresh_encoder_sample = false;
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ENCODER_READ)) {
		/* Read encoder to consume RTIO queue data (callback keeps queuing reads) */
		if (encoder_read(&encoder_rtio_ctx, &angle_raw_degrees) < 0) {
			/* No new encoder data this cycle. */
			params->encoder_fault_counter++;

			/* Fault detection: Too many consecutive encoder failures */
			if (params->encoder_fault_counter > ENCODER_FAULT_THRESHOLD) {
				motor_api_post_error(ERROR_ENCODER_FAULT);
				goto isr_done;
			}
			
		} else {
			/* Reset fault counter on successful read */
			params->encoder_fault_counter = 0;
			fresh_encoder_sample = true;
		}
	} else {
		/* Encoder not active - reset fault counter */
		params->encoder_fault_counter = 0;
	}
	
	/* Select angle source based on feature flag */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ANGLE_GEN)) {
		/* Calibration/open-loop: use generated angle (no delay) */
		angle_raw_rad = angle_gen_get_angle(&params->angle_gen);
		angle_observer_set_delay(&params->observer, 0.0f);
		encoder_input_source = MOTOR_ANGLE_INPUT_SRC_GENERATED;
	} else if (fresh_encoder_sample) {
		/* Normal operation: use fresh encoder reading (1-cycle pipelined delay) */
		angle_raw_rad = angle_raw_degrees * (PI_F32 / 180.0f);
		angle_observer_set_delay(&params->observer, 1.0f);
		encoder_input_source = MOTOR_ANGLE_INPUT_SRC_ENCODER;
		params->encoder_raw_deg = angle_raw_degrees;
		params->encoder_raw_rad = angle_raw_rad;
	} else {
		/* No fresh encoder sample: propagate using prior estimate only. */
		angle_raw_rad = angle_observer_get_mech_angle(&params->observer);
		angle_observer_set_delay(&params->observer, 0.0f);
		encoder_input_source = MOTOR_ANGLE_INPUT_SRC_PROPAGATED;
	}
	
	/* Update observer with angle (encoder or generated) */
	angle_observer_update(&params->observer, angle_raw_rad);

	/* Skip control if PWM output not enabled */
	if (!atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_PWM_OUTPUT)) {
		goto isr_done;
	}

	Ia_A = adc_to_current(values[CURRENT_SENSE_ADC_BUFFER_INDEX_0], CURRENT_SENSE_POLARITY_0);
	Ib_A = adc_to_current(values[CURRENT_SENSE_ADC_BUFFER_INDEX_1], CURRENT_SENSE_POLARITY_1);
	Vbus_V = adc_to_vbus_v(values[VBUS_ADC_BUFFER_INDEX]);

	/* Validate bus voltage before reciprocal to avoid Inf/NaN propagation. */
	if (Vbus_V < VBUS_MIN_VALID_V) {
		motor_api_post_error(ERROR_HARDWARE_BREAK);
		goto isr_done;
	}

	/* Fault detection: Check for overvoltage */
	if (Vbus_V > VBUS_MAX_V) {
		motor_api_post_error(ERROR_OVERVOLTAGE);
		goto isr_done;
	}

	Vbus_inv = 1.0f / Vbus_V;

	/* Handle offset measurement (no control, just filtering) */
	if (state == &motor_states[MOTOR_STATE_OFFSET_MEAS]) {
		filter_fo_run(&params->filter_Ia, Ia_A);
		filter_fo_run(&params->filter_Ib, Ib_A);
		goto isr_done;
	}

	/* Remove offsets for current measurements */
	Ia_A -= params->Ia_offset;
	Ib_A -= params->Ib_offset;

	/* Fault detection: Check for overcurrent after offset removal */
	if (fabsf(Ia_A) > OVERCURRENT_THRESHOLD_A || fabsf(Ib_A) > OVERCURRENT_THRESHOLD_A) {
		motor_api_post_error(ERROR_OVERCURRENT);
		goto isr_done;
	}

	/* Skip PI control if not enabled */
	if (!atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_PI_CONTROL)) {
		goto isr_done;
	}

	/* Select control frame angle based on state and transform currents */
	float32_t ctrl_angle_rad = angle_observer_get_elec_angle(&params->observer);

	/* Park transform to dq frame - convert to degrees for arm_sin_cos_f32 */
	float32_t ctrl_angle_deg = ctrl_angle_rad * (180.0f / PI_F32);
	arm_sin_cos_f32(ctrl_angle_deg, &sin_theta, &cos_theta);
	arm_park_f32(Ia_A, Ib_A, &Id_A, &Iq_A, sin_theta, cos_theta);

	/* R/L measurement: set current reference and accumulate V/I in rotating frame */
	if (state == &motor_states[MOTOR_STATE_ROVERL_MEAS]) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;

		/* Check if settling period complete using trajectory target */
		if (traj_is_at_target(&params->traj_Id)) {
			/* Accumulate for R/L extraction using previous cycle's voltage */
			params->roverl_accumulator_Vd_Id += params->Vd_V * Id_A;
			params->roverl_accumulator_Vq_Id += params->Vq_V * Id_A;
			params->roverl_accumulator_Id2 += Id_A * Id_A;
		}
	}

	/* Rs EST: filter V/I in d-axis for DC resistance */
	if (state == &motor_states[MOTOR_STATE_RS_EST]) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;

		/* After rampup complete: filter voltage and current measurements using previous cycle's voltage */
		if (traj_is_at_target(&params->traj_Id)) {
			filter_fo_run(&params->filter_rs_est_V, params->Vd_V);
			filter_fo_run(&params->filter_rs_est_I, Id_A);
		}
	}

	/* ALIGN: ramp/hold alignment current */
	if (state == &motor_states[MOTOR_STATE_ALIGN] ||
	    state == &motor_states[MOTOR_STATE_ALIGN_SAMPLE]) {
		traj_run(&params->traj_Id);

		Id_ref_A = traj_get_int_value(&params->traj_Id);
		Iq_ref_A = 0.0f;
	}

	/* Position cascade: generate velocity target from position error. */
	if (state == &motor_states[MOTOR_STATE_ONLINE_POSITION]) {
		float32_t position_mech_rad = angle_observer_get_mech_angle(&params->observer);
		float32_t position_error_rad;

		/* Position mode supports optional quintic profile feedforward. */
		if (motion_profile_quintic_is_active(&params->position_profile)) {
			if (control_armed) {
				motion_profile_quintic_step(&params->position_profile);
			}

			float32_t profile_pos_rad =
				motion_profile_quintic_get_position(&params->position_profile);
			float32_t profile_vel_rad_s =
				motion_profile_quintic_get_velocity(&params->position_profile);
			params->position_target_rad = wrap_rad_2pi(profile_pos_rad);
			position_error_rad = wrap_rad_pi(profile_pos_rad - position_mech_rad);
			velocity_target_rad_s =
				profile_vel_rad_s +
				params->position_cl_kp_rad_s_per_rad * position_error_rad;
		} else if (params->position_profile.valid) {
			/* Completed profile: hold final position with pure feedback. */
			float32_t profile_pos_rad =
				motion_profile_quintic_get_position(&params->position_profile);
			params->position_target_rad = wrap_rad_2pi(profile_pos_rad);
			position_error_rad = wrap_rad_pi(profile_pos_rad - position_mech_rad);
			velocity_target_rad_s =
				params->position_cl_kp_rad_s_per_rad * position_error_rad;
		} else {
			position_error_rad =
				wrap_rad_pi(params->position_target_rad - position_mech_rad);
			velocity_target_rad_s =
				params->position_cl_kp_rad_s_per_rad * position_error_rad;
		}

		velocity_target_rad_s =
			clampf(velocity_target_rad_s, -params->profile_max_velocity_rad_s,
			       params->profile_max_velocity_rad_s);
		traj_set_target_value(&params->traj_velocity, velocity_target_rad_s);
	}

	/* Update velocity trajectory if enabled */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_VELOCITY_TRAJ)) {
		velocity_target_rad_s = traj_get_target_value(&params->traj_velocity);
		traj_run(&params->traj_velocity);
		velocity_ref_rad_s = traj_get_int_value(&params->traj_velocity);

		/* Open-loop commutation uses the trajectory directly. */
		if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ANGLE_GEN)) {
			angle_gen_set_velocity(&params->angle_gen, velocity_ref_rad_s);
		}
	}

	/* Closed-loop velocity and position share the same inner velocity->Iq stage. */
	if (state == &motor_states[MOTOR_STATE_ONLINE_VELOCITY_CLOSED] ||
	    state == &motor_states[MOTOR_STATE_ONLINE_POSITION]) {
		float32_t speed_mech_rad_s = angle_observer_get_mech_speed(&params->observer);
		float32_t speed_error_rad_s = velocity_ref_rad_s - speed_mech_rad_s;

		Id_ref_A = params->Id_setpoint_A;
		Iq_ref_A =
			clampf(params->velocity_cl_kp_A_per_rad_s * speed_error_rad_s,
			       -params->velocity_cl_iq_limit_A,
			       params->velocity_cl_iq_limit_A);
	}

	/* Select current references based on mode */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_USE_COMMANDED_CURRENTS)) {
		/* Normal FOC operation: use commanded current references */
		Id_ref_A = params->Id_setpoint_A;
		Iq_ref_A = params->Iq_setpoint_A;
	}

	/* Arm/disarm interlock only applies in ONLINE control states. */
	if (online_control_state && !control_armed) {
		Id_ref_A = Id_A;
		Iq_ref_A = Iq_A;
		params->Id_setpoint_A = 0.0f;
		params->Iq_setpoint_A = 0.0f;
		velocity_target_rad_s = 0.0f;
		velocity_ref_rad_s = 0.0f;
		params->velocity_target_rad_s = 0.0f;
		params->velocity_ref_rad_s = 0.0f;
		traj_set_target_value(&params->traj_velocity, 0.0f);
		traj_set_int_value(&params->traj_velocity, 0.0f);
		angle_gen_set_velocity(&params->angle_gen, 0.0f);
		pi_set_ui(&params->pi_Id, 0.0f);
		pi_set_ui(&params->pi_Iq, 0.0f);
	}

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
	/* PRBS injection for parameter estimation (d-axis current reference) */
	const uint32_t rls_mask = params->rls_decimation - 1u;
	float32_t I_prbs_d = 0.0f;
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION) &&
	    (params->control_loop_count & rls_mask) == 0u) {
		uint32_t prbs_bit = prbs_advance(&params->prbs_gen);
		I_prbs_d = (2.0f * (float32_t)prbs_bit - 1.0f) * ROVERL_EST_CURRENT_A;
	}

	/* Apply PRBS excitation to d-axis current reference */
	Id_ref_A += I_prbs_d;
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

	max_voltage_magnitude_V = params->max_modulation_index * Vbus_V;
	pi_set_min_max(&params->pi_Id, -max_voltage_magnitude_V, max_voltage_magnitude_V);
	pi_run_series(&params->pi_Id, Id_ref_A, Id_A, 0.0f, &Vd_V);

	Vq_limit_V = sqrtf((max_voltage_magnitude_V * max_voltage_magnitude_V) - (Vd_V * Vd_V));
	pi_set_min_max(&params->pi_Iq, -Vq_limit_V, Vq_limit_V);
	pi_run_series(&params->pi_Iq, Iq_ref_A, Iq_A, 0.0f, &Vq_V);

	/* Advance angle generator if enabled
	 * This compensates for the fact that computed voltages will be applied in the next cycle
	 */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ANGLE_GEN)) {
		angle_gen_run(&params->angle_gen);
	}

	ctrl_angle_rad = angle_observer_get_elec_angle_pred(&params->observer);

	/* Calculate sin and cos of control angle - convert to degrees for arm_sin_cos_f32 */
	ctrl_angle_deg = ctrl_angle_rad * (180.0f / PI_F32);
	arm_sin_cos_f32(ctrl_angle_deg, &sin_theta, &cos_theta);

	/* Transform voltages to stationary frame */
	arm_inv_park_f32(Vd_V, Vq_V, &Va_V, &Vb_V, sin_theta, cos_theta);

	/* Normalize voltage commands to available bus voltage */
	Ua_pu = Va_V * Vbus_inv;
	Ub_pu = Vb_V * Vbus_inv;

	/* Apply SVPWM modulation - outputs duty cycles in [0,1] range */
	pwmgen_spwm_2phase_f32(Ua_pu, Ub_pu, &Da_pu, &Db_pu);

	/* Complementary PWM for H-bridge control
	 * Half-bridge 1: Da_pu [0,1]
	 * Half-bridge 2: inverted (1 - Da_pu) [0,1]
	 * Creates bidirectional voltage across winding
	 */
	Da_hb1_pu = Da_pu;
	Da_hb2_pu = 1.0f - Da_pu;
	Db_hb1_pu = Db_pu;
	Db_hb2_pu = 1.0f - Db_pu;

	/* Vbus-regulated braking: blend between regen and short-circuit
	 * Only active during ONLINE modes (not calibration states)
	 * Braking occurs when torque opposes motion (Iq and speed have opposite signs)
	 */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_BRAKING)) {
		float32_t speed_rad_s = angle_observer_get_mech_speed(&params->observer);
		bool is_braking = (params->Iq_ref_A * speed_rad_s) < 0.0f;  /* Opposite signs = braking */

		/* Braking mode (ignore near-zero speeds, 0.628 rad/s = 0.1 Hz) */
		if (is_braking && fabsf(speed_rad_s) > 0.628f) {
			if (Vbus_V > VBUS_REGEN_LIMIT_V) {
				/* Calculate short-circuit duty based on overvoltage */
				float32_t overvoltage = Vbus_V - VBUS_REGEN_LIMIT_V;
				float32_t short_duty = fminf(1.0f, overvoltage * VBUS_VOLTAGE_MARGIN_INV);

				/* Scale down FOC PWM and add offset to hb2 for short-circuit braking
				 * As short_duty → 1.0, hb1 → 0, hb2 → 1.0 (full short-circuit)
				 */
				float32_t scale = 1.0f - short_duty;
				Da_hb1_pu *= scale;
				Da_hb2_pu = Da_hb2_pu * scale + short_duty;
				Db_hb1_pu *= scale;
				Db_hb2_pu = Db_hb2_pu * scale + short_duty;
			}
		}
	}

	/* Output PWM values to H-bridges
	 * Winding A: pwm1 ch1 and ch2 drive the two sides of H-bridge A
	 * Winding B: pwm8 ch1 and ch2 drive the two sides of H-bridge B
	 */
	mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1, Da_hb1_pu, Da_hb2_pu);
	mcpwm_stm32_set_duty_cycle_2phase_f32(pwm8, Db_hb1_pu, Db_hb2_pu);

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
	/* D-axis RLS parameter estimation with comprehensive gating */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION) &&
	    (params->control_loop_count & rls_mask) == 0u) {
		/* Gating conditions for robust estimation:
		 * 1. Voltage within valid measurement range
		 * 2. PI controller not saturated (would corrupt voltage measurement)
		 * 3. Speed sufficient for back-EMF observability (bypassed in open-loop)
		 * Note: D-axis current check omitted since we command Id=0 (only PRBS excitation)
		 */
		float32_t Vd_abs = fabsf(Vd_V);
		float32_t omega_elec;

		omega_elec = angle_observer_get_elec_speed(&params->observer);

		bool voltage_ok = (Vd_abs < params->rls_max_voltage_V);
		bool pi_ok = (fabsf(Vd_V - pi_get_out_max(&params->pi_Id)) > 0.1f) &&
		             (fabsf(Vd_V - pi_get_out_min(&params->pi_Id)) > 0.1f);
		bool speed_ok = (fabsf(omega_elec) > params->rls_min_speed_rad_s);

		if (voltage_ok && pi_ok && speed_ok) {
			rls_motor_est_update(&params->rls_d, Vd_V, Id_A, params->Id_rls_prev,
			                     omega_elec, params->Lq_est, Iq_A);
			params->Id_rls_prev = Id_A;  /* Store for next RLS update */
		}
	}

	/* Q-axis RLS parameter estimation (staggered by offset for load spreading) */
	const uint32_t rls_offset = params->rls_stagger_offset;
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION) &&
	    ((params->control_loop_count & rls_mask) == rls_offset)) {
		/* Compensate back-EMF first for gating check */
		float32_t omega_elec = angle_observer_get_elec_speed(&params->observer);
		float32_t V_bemf = omega_elec * MOTOR_FLUX_LINKAGE_WB;
		float32_t Vq_compensated = Vq_V - V_bemf;
		
		/* Gating conditions using compensated voltage */
		float32_t Iq_abs = fabsf(Iq_A);
		float32_t Vq_comp_abs = fabsf(Vq_compensated);

		bool current_ok = (Iq_abs > params->rls_min_current_A);
		bool voltage_ok = (Vq_comp_abs < params->rls_max_voltage_V);
		bool pi_ok = (fabsf(Vq_V - pi_get_out_max(&params->pi_Iq)) > 0.1f) &&
		             (fabsf(Vq_V - pi_get_out_min(&params->pi_Iq)) > 0.1f);
		bool speed_ok = (fabsf(omega_elec) > params->rls_min_speed_rad_s);

		if (current_ok && voltage_ok && pi_ok && speed_ok) {
			/* Q-axis: Pass -omega to RLS so it subtracts cross-coupling ω·Ld·Id */
			rls_motor_est_update(&params->rls_q, Vq_compensated, Iq_A, params->Iq_rls_prev,
			                     -omega_elec, params->Ld_est, Id_A);
			params->Iq_rls_prev = Iq_A;  /* Store for next RLS update */
		}

		/* Parameter synthesis: update cross-coupling estimates and average Rs */
		if (rls_motor_est_is_converged(&params->rls_d) &&
		    rls_motor_est_is_converged(&params->rls_q)) {
			/* Update inductance estimates for cross-coupling compensation */
			params->Ld_est = rls_motor_est_get_L(&params->rls_d);
			params->Lq_est = rls_motor_est_get_L(&params->rls_q);

			/* Average Rs from both axes (they should converge to same value) */
			float32_t Rs_d = rls_motor_est_get_Rs(&params->rls_d);
			float32_t Rs_q = rls_motor_est_get_Rs(&params->rls_q);
			params->Rs_measured_ohm = (Rs_d + Rs_q) * 0.5f;

			/* Compute temperature from RLS Rs estimate */
			params->T_rls_C = thermal_Rs_to_temperature(params->Rs_measured_ohm,
			                                             params->Rs_ref_ohm,
			                                             params->Rs_ref_temp_C,
			                                             params->Rs_temp_coeff);
		}
	}

	/* Thermal model update (heavily decimated, ~10Hz) */
	const uint32_t thermal_mask = params->thermal_decimation - 1u;
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_RLS_ESTIMATION) &&
	    (params->control_loop_count & thermal_mask) == 0u) {
		thermal_model_update(&params->thermal, Id_A, Iq_A, params->Rs_measured_ohm);
	}
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

	/* Update telemetry snapshot (observer already returns rad/rad_s) */
	params->position_rad = angle_observer_get_mech_angle(&params->observer);
	params->velocity_rad_s = angle_observer_get_mech_speed(&params->observer);
	params->velocity_target_rad_s = velocity_target_rad_s;
	params->velocity_ref_rad_s = velocity_ref_rad_s;

	params->Id_ref_A = Id_ref_A;
	params->Iq_ref_A = Iq_ref_A;
	params->Id_A = Id_A;
	params->Iq_A = Iq_A;
	params->Ia_A = Ia_A;
	params->Ib_A = Ib_A;
	params->Vd_V = Vd_V;
	params->Vq_V = Vq_V;
	params->Va_V = Va_V;
	params->Vb_V = Vb_V;
	params->max_voltage_magnitude_V = max_voltage_magnitude_V;
	params->elec_angle_rad = ctrl_angle_rad;
	params->dc_bus_voltage_V = Vbus_V;
	params->encoder_observer_input_rad = angle_raw_rad;
	params->encoder_sample_fresh = fresh_encoder_sample ? 1U : 0U;
	params->encoder_input_source = encoder_input_source;

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
	struct motor_parameters *params = (struct motor_parameters *)user_data;
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	if (params == NULL) {
		return;
	}

	/* Trigger continuous encoder reads when feature is enabled
	 * This keeps SPI bus free during calibration and reduces interrupt load
	 */
	if (atomic_test_bit(&params->feature_flags, MOTOR_FEATURE_ENCODER_READ)) {
		if (atomic_cas(&encoder_read_in_flight, 0, 1)) {
			int ret = sensor_read_async_mempool(&encoder1_iodev, &encoder_rtio_ctx, NULL);
			if (ret != 0) {
				atomic_set(&encoder_read_in_flight, 0);
			}
		}
	}

	/* Hardware-timer-driven position-sequence tick source. */
	if (params->profile_sequence_running &&
	    atomic_get(&params->control_armed) != 0 &&
	    params->profile_sequence_trigger_source == PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL &&
	    motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION)) {
		uint32_t period_ticks = params->profile_sequence_period_ticks;
		if (period_ticks == 0U) {
			period_ticks = 1U;
		}

		uint32_t tick_counter = params->profile_sequence_tick_counter + 1U;
		if (tick_counter >= period_ticks) {
			struct motor_event evt = {
				.type = MOTOR_EVENT_PROFILE_SEQ_TICK,
			};
			int ret;

			params->profile_sequence_tick_counter = 0U;
			ret = k_msgq_put(&motor_event_queue, &evt, K_NO_WAIT);
			if (ret != 0) {
				params->profile_sequence_event_drop_count++;
			}
		} else {
			params->profile_sequence_tick_counter = tick_counter;
		}
	} else {
		params->profile_sequence_tick_counter = 0U;
	}
}
