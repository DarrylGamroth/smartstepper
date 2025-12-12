/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "config.h"

LOG_MODULE_REGISTER(config, CONFIG_APP_LOG_LEVEL);

/* M_PI is not guaranteed by C standard, define float version */
#define PI_F32 3.14159265358979323846f

void config_init_filters(struct motor_parameters *params)
{
	/* Calculate filter coefficients from offset measurement pole frequency */
	float32_t a1 = expf(-2.0f * PI_F32 * OFFSET_POLE_HZ / CONTROL_LOOP_FREQUENCY_HZ);
	float32_t b0 = 1.0f - a1;

	filter_fo_init(&params->filter_Ia);
	filter_fo_set_den_coeffs(&params->filter_Ia, a1);
	filter_fo_set_num_coeffs(&params->filter_Ia, b0, 0.0f);

	filter_fo_init(&params->filter_Ib);
	filter_fo_set_den_coeffs(&params->filter_Ib, a1);
	filter_fo_set_num_coeffs(&params->filter_Ib, b0, 0.0f);

	LOG_DBG("Filters initialized: a1=%.6f, b0=%.6f",
		(double)a1, (double)b0);
}

void config_init_pi_controllers(struct motor_parameters *params)
{
	/* Calculate intermediate values */
	float32_t rd_over_ld_rps = MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_D_H;
	float32_t rq_over_lq_rps = MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_Q_H;
	float32_t bwc_rps = CURRENT_LOOP_BANDWIDTH_RPS;
	float32_t current_ctrl_period_sec = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;

	/* D-axis PI controller
	 * Kp_Id = Ls_d * BWc_rps
	 * Ki_Id = (Rs/Ls_d) * Ti
	 */
	float32_t kp_d = MOTOR_INDUCTANCE_D_H * bwc_rps;
	float32_t ki_d = rd_over_ld_rps * current_ctrl_period_sec;

	/* Initialize PI controller (NOT double buffered - single instance in main struct) */
	pi_init(&params->pi_Id);
	pi_set_gains(&params->pi_Id, kp_d, ki_d);

	/* Q-axis PI controller
	 * Kp_Iq = Ls_q * BWc_rps
	 * Ki_Iq = (Rs/Ls_q) * Ti
	 */
	float32_t kp_q = MOTOR_INDUCTANCE_Q_H * bwc_rps;
	float32_t ki_q = rq_over_lq_rps * current_ctrl_period_sec;

	/* Initialize PI controller (NOT double buffered - single instance in main struct) */
	pi_init(&params->pi_Iq);
	pi_set_gains(&params->pi_Iq, kp_q, ki_q);

	LOG_DBG("PI controllers initialized: Ti=%.6f s",
		(double)current_ctrl_period_sec);
	LOG_DBG("  D-axis: Kp=%.6f, Ki=%.6f (R/L=%.1f rad/s)", 
		(double)kp_d, (double)ki_d, (double)rd_over_ld_rps);
	LOG_DBG("  Q-axis: Kp=%.6f, Ki=%.6f (R/L=%.1f rad/s)", 
		(double)kp_q, (double)ki_q, (double)rq_over_lq_rps);
}

void config_print_parameters(void)
{
	LOG_INF("Motor Controller Configuration");
	LOG_INF("================================");
	LOG_INF("System Parameters:");
	LOG_INF("  Nominal Voltage: %.2f V", (double)NOMINAL_VOLTAGE_V);
	LOG_INF("  PWM Frequency: %.0f Hz", (double)PWM_FREQUENCY_HZ);
	LOG_INF("  Control Loop Frequency: %.0f Hz", (double)CONTROL_LOOP_FREQUENCY_HZ);
	LOG_INF("  Current Loop Bandwidth: %.0f Hz", (double)CURRENT_LOOP_BANDWIDTH_HZ);
	LOG_INF("  Offset Filter Pole: %.0f Hz", (double)OFFSET_POLE_HZ);
	LOG_INF("  Max Voltage: %.1f%% of bus", (double)(MAX_VS_MPU * 100.0f));

	LOG_INF("Current Sensing:");
	LOG_INF("  Resistor: %.0fmOhm, Gain: %.0fx, Full Scale: %.1fA",
		(double)(CURRENT_SENSE_RESISTOR_OHM * 1000.0f),
		(double)CURRENT_SENSE_GAIN,
		(double)CURRENT_SENSE_FULL_SCALE_A);

	LOG_INF("Motor Parameters:");
	LOG_INF("  Ld=%.1fuH, Lq=%.1fuH",
		(double)(MOTOR_INDUCTANCE_D_H * 1e6f),
		(double)(MOTOR_INDUCTANCE_Q_H * 1e6f));
	LOG_INF("  R=%.1fmOhm", (double)(MOTOR_RESISTANCE_OHM * 1000.0f));
	LOG_INF("  Flux linkage=%.1fmV/Hz", (double)(MOTOR_FLUX_LINKAGE_VPH * 1000.0f));
	LOG_INF("  Pole pairs=%d", MOTOR_POLE_PAIRS);
	LOG_INF("  Max current=%.1fA", (double)MOTOR_MAX_CURRENT_A);
	LOG_INF("  Max speed=%.0fHz", (double)MOTOR_MAX_SPEED_HZ);
	LOG_INF("  Inertia=%.3fkgcm²", (double)(MOTOR_INERTIA_KGM2 * 10000.0f));

	LOG_INF("Alignment:");
	LOG_INF("  Current=%.2fA, Duration=%.1fs",
		(double)ALIGN_CURRENT_A,
		(double)ALIGN_DURATION_S);
}
