/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <zephyr/kernel.h>
#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>
#include "pi.h"
#include "filter_fo.h"
#include "angle_observer.h"
#include "rs_online.h"
#include "traj.h"

/**
 * @brief Double-buffered motor control parameters
 *
 * Shell-writable STATELESS parameters that can be modified at runtime without
 * disrupting the control loop. ISR reads from read_buffer, shell writes
 * to write_buffer, and swap is atomic.
 *
 * IMPORTANT: Only put stateless values here (setpoints, references).
 * Do NOT put stateful objects (PI controllers with integrators, filters).
 */
struct motor_control_params {
	float32_t Id_setpoint_A;
	float32_t Iq_setpoint_A;
};

/**
 * @brief Main motor control parameters structure
 *
 * Contains all state needed for motor control including SMF state machine,
 * control parameters (double-buffered), filters, observers, and telemetry.
 */
struct motor_parameters {
	/* State machine */
	struct smf_ctx smf;

	/* Double-buffered control parameters (index-based for minimal overhead) */
	struct motor_control_params ctrl_buf[2];     /* Two buffers */
	volatile uint8_t ctrl_index;                 /* ISR reads ctrl_buf[ctrl_index], shell writes ctrl_buf[ctrl_index^1] */
	volatile bool ctrl_swap_pending;             /* Set by shell, cleared by ISR */

	/* PI controllers (stateful - NOT double buffered) */
	struct pi_f32 pi_Id;
	struct pi_f32 pi_Iq;
	float32_t maxVsMag_pu;  /* Max voltage magnitude in per-unit */

	/* Current sensing filters and offsets */
	struct filter_fo_f32 filter_Ia;
	struct filter_fo_f32 filter_Ib;
	float32_t Ia_offset;
	float32_t Ib_offset;

	/* Rs estimation filters */
	struct filter_fo_f32 filter_rs_est_V;
	struct filter_fo_f32 filter_rs_est_I;

	/* Current references (computed by control loop) */
	float32_t Id_ref_A;
	float32_t Iq_ref_A;

	/* Voltage references (computed by PI controllers) */
	float32_t Vd_ref_V;
	float32_t Vq_ref_V;
	float32_t maxVsMag_V;

	/* Applied voltages (after SVPWM limiting) */
	float32_t Vd_V;
	float32_t Vq_V;

	/* Observers and estimators */
	struct angle_observer_state observer;
	struct rs_online_estimator rs_est;
	struct traj_f32 traj_Id;

	/* Measured parameters (from calibration) */
	float32_t RoverL_measured;
	float32_t L_measured;
	float32_t Rs_measured;

	/* R/L estimation accumulators */
	float32_t roverl_sum_Vd_Id;
	float32_t roverl_sum_Vq_Id;
	float32_t roverl_sum_Id2;
	float32_t roverl_phase_deg;

	/* Telemetry and diagnostics */
	uint32_t state_counter;
	uint32_t encoder_fault_counter;
	uint32_t control_loop_count;
	uint32_t adc_conversion_count;
	uint32_t max_isr_cycles;
	uint32_t total_isr_cycles;
	uint32_t buffer_swap_count;
	uint32_t overrun_count;
	int error_code;

	/* Live telemetry snapshot (updated in ISR) */
	float32_t position_rad;
	float32_t velocity_rad_s;
	float32_t Id_A;
	float32_t Iq_A;
	float32_t Ia_A;
	float32_t Ib_A;
	float32_t Va_V;
	float32_t Vb_V;
	float32_t elec_angle_rad;
	float32_t dc_bus_voltage_V;
};

/* M_PI is not guaranteed by C standard, define float version */
#define PI_F32 3.14159265358979323846f

/* Devicetree parameter extraction with unit conversion */
#define USER_PARAMS_NODE DT_PATH(user_parameters)
#define NOMINAL_VOLTAGE_V ((float32_t)DT_PROP(USER_PARAMS_NODE, nominal_voltage_mv) / 1000.0f)
#define PWM_FREQUENCY_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, pwm_frequency_hz))
#define CONTROL_LOOP_FREQUENCY_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, control_loop_frequency_hz))
#define CURRENT_LOOP_BANDWIDTH_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, current_loop_bandwidth_hz))
#define CURRENT_LOOP_BANDWIDTH_RPS (2.0f * PI_F32 * (float32_t)DT_PROP(USER_PARAMS_NODE, current_loop_bandwidth_hz))
#define OFFSET_POLE_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, offset_pole_hz))
#define ALIGN_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, align_current_ma) / 1000.0f)
#define ALIGN_DURATION_S ((float32_t)DT_PROP(USER_PARAMS_NODE, align_duration_ms) / 1000.0f)
#define BRAKE_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, brake_current_ma) / 1000.0f)
#define MAX_VS_MPU ((float32_t)DT_PROP(USER_PARAMS_NODE, max_vs_mpu) / 1000.0f)
#define ROVERL_EST_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_current_ma) / 1000.0f)
#define ROVERL_EST_FREQ_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_freq_hz))
#define ROVERL_EST_SETTLING_S ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_settling_ms) / 1000.0f)
#define ROVERL_EST_DURATION_S ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_duration_ms) / 1000.0f)
#define RS_EST_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_current_ma) / 1000.0f)
#define RS_EST_RAMPUP_S ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_rampup_ms) / 1000.0f)
#define RS_EST_COARSE_S ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_coarse_ms) / 1000.0f)
#define RS_EST_FINE_S ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_fine_ms) / 1000.0f)
#define RS_EST_FILTER_BW_HZ 5.0f      /* Heavy filtering for accurate measurement */

#define VBUS_SENSE_NODE DT_PATH(vbus_sense)
#define VBUS_CHANNEL DT_PROP(VBUS_SENSE_NODE, channel)
#define VBUS_VREF_V ((float32_t)DT_PROP(VBUS_SENSE_NODE, vref_mv) / 1000.0f)
#define VBUS_OUTPUT_OHMS ((float32_t)DT_PROP(VBUS_SENSE_NODE, output_ohms))
#define VBUS_FULL_OHMS ((float32_t)DT_PROP(VBUS_SENSE_NODE, full_ohms))
#define VBUS_FULL_SCALE_V (VBUS_VREF_V * (VBUS_FULL_OHMS / VBUS_OUTPUT_OHMS))

/* Braking voltage limits */
#define VBUS_MAX_V (VBUS_FULL_SCALE_V * 0.95f)  /* 95% of full scale for safety margin */
#define VBUS_REGEN_LIMIT_V (NOMINAL_VOLTAGE_V * 1.1f)  /* Start blending to short-circuit at 110% nominal */
#define VBUS_VOLTAGE_MARGIN_INV (1.0f / (VBUS_MAX_V - VBUS_REGEN_LIMIT_V))  /* Inverse for fast computation */


#define CURRENT_SENSE_NODE DT_PATH(current_sense)
#define CURRENT_SENSE_VREF_V ((float32_t)DT_PROP(CURRENT_SENSE_NODE, vref_mv) / 1000.0f)
#define CURRENT_SENSE_RESISTOR_OHM ((float32_t)DT_PROP(CURRENT_SENSE_NODE, current_sense_resistor_uohms) / 1000000.0f)
#define CURRENT_SENSE_GAIN ((float32_t)DT_PROP(CURRENT_SENSE_NODE, current_sense_gain))
#define CURRENT_SENSE_FULL_SCALE_A (CURRENT_SENSE_VREF_V / (CURRENT_SENSE_RESISTOR_OHM * CURRENT_SENSE_GAIN))

#define MOTOR_PARAMS_NODE DT_PATH(motor_parameters)
#define MOTOR_INDUCTANCE_D_H ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, inductance_d_uh) / 1000000.0f)
#define MOTOR_INDUCTANCE_Q_H ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, inductance_q_uh) / 1000000.0f)
#define MOTOR_RESISTANCE_OHM ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, resistance_mohms) / 1000.0f)
#define MOTOR_FLUX_LINKAGE_VPH ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, flux_linkage_mvphz) / 1000.0f)
#define MOTOR_POLE_PAIRS DT_PROP(MOTOR_PARAMS_NODE, pole_pairs)
#define MOTOR_MAX_CURRENT_A ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, max_current_ma) / 1000.0f)
#define MOTOR_INERTIA_KGM2 ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, inertia_mgcm2) / 10000000.0f)
#define MOTOR_MAX_SPEED_HZ ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, max_speed_hz))

#define ANGLE_OBSERVER_NODE DT_PATH(angle_observer)
#define ANGLE_OBSERVER_BANDWIDTH_HZ ((float32_t)DT_PROP(ANGLE_OBSERVER_NODE, bandwidth_hz))

#define FAULT_DETECT_NODE DT_PATH(fault_detection)
#define ENCODER_FAULT_THRESHOLD DT_PROP(FAULT_DETECT_NODE, encoder_fault_threshold)
#define OVERCURRENT_THRESHOLD_A ((float32_t)DT_PROP(FAULT_DETECT_NODE, overcurrent_threshold_ma) / 1000.0f)

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

/**
 * @brief Initialize filters with devicetree parameters
 *
 * @param params Motor parameters structure containing filters
 */
void config_init_filters(struct motor_parameters *params);

/**
 * @brief Initialize PI controllers with devicetree parameters
 *
 * @param params Motor parameters structure containing PI controllers
 */
void config_init_pi_controllers(struct motor_parameters *params);

/**
 * @brief Print all configuration parameters to log
 */
void config_print_parameters(void);

#endif /* CONFIG_H_ */
