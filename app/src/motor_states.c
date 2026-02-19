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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <drivers/mcpwm.h>
#include <drivers/adc_injected.h>
#include <drivers/gate_driver/ti_drv8328.h>
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
#include "angle_wrap.h"
#include "motor_state_utils.h"

LOG_MODULE_REGISTER(motor_states, CONFIG_APP_LOG_LEVEL);

/**
 * @brief Stage ISR feature flags for the next stable state
 *
 * The ISR reads params->feature_flags (atomic). The state-machine thread builds up
 * params->feature_flags_next while SMF runs (including any hierarchical entry/exit
 * actions). After SMF completes for the current cycle, feature_flags_next is
 * atomically published.
 */
static inline void motor_set_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next = mask;
}

static inline void motor_enable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next |= mask;
}

static inline void motor_disable_isr_feature_flags(struct motor_parameters *params, atomic_val_t mask)
{
	params->feature_flags_next &= ~mask;
}

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

static int motor_position_plan_sequence_move(struct motor_parameters *params, float32_t target_wrapped_rad)
{
	float32_t start_pos_rad = params->position_rad;
	float32_t start_vel_rad_s = params->velocity_rad_s;
	float32_t delta_rad = wrap_rad_pi(target_wrapped_rad - start_pos_rad);
	float32_t end_pos_rad = start_pos_rad + delta_rad;

	int ret = motion_profile_quintic_plan(&params->position_profile,
					      start_pos_rad, start_vel_rad_s, 0.0f,
					      end_pos_rad, params->profile_sequence_end_velocity_rad_s,
					      0.0f, params->profile_sequence_move_duration_s);
	if (ret != 0) {
		return ret;
	}

	ret = motion_profile_quintic_check_limits(&params->position_profile,
						  params->profile_max_velocity_rad_s,
						  params->profile_max_accel_rad_s2, 64U,
						  NULL, NULL);
	if (ret != 0) {
		motion_profile_quintic_cancel(&params->position_profile, start_pos_rad);
		return ret;
	}

	params->position_target_rad = wrap_rad_2pi(start_pos_rad);
	params->last_command_update_ms = k_uptime_get_32();
	params->command_timeout_latched = false;

	return 0;
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
static void motor_state_align_sample_entry(void *obj);
static enum smf_state_result motor_state_align_sample_run(void *obj);
static void motor_state_align_sample_exit(void *obj);
static void motor_state_idle_entry(void *obj);
static enum smf_state_result motor_state_idle_run(void *obj);
static void motor_state_offline_entry(void *obj);
static enum smf_state_result motor_state_offline_run(void *obj);
static void motor_state_offline_exit(void *obj);
static void motor_state_online_entry(void *obj);
static enum smf_state_result motor_state_online_run(void *obj);
static void motor_state_online_exit(void *obj);
static void motor_state_online_torque_entry(void *obj);
static enum smf_state_result motor_state_online_torque_run(void *obj);
static void motor_state_online_torque_exit(void *obj);
static void motor_state_online_velocity_open_entry(void *obj);
static enum smf_state_result motor_state_online_velocity_open_run(void *obj);
static void motor_state_online_velocity_open_exit(void *obj);
static void motor_state_online_velocity_closed_entry(void *obj);
static enum smf_state_result motor_state_online_velocity_closed_run(void *obj);
static void motor_state_online_velocity_closed_exit(void *obj);
static void motor_state_online_position_entry(void *obj);
static enum smf_state_result motor_state_online_position_run(void *obj);
static void motor_state_online_position_exit(void *obj);
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
						      &motor_states[MOTOR_STATE_OFFLINE],
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
					motor_state_align_exit,
					&motor_states[MOTOR_STATE_CALIBRATION],
					NULL),
	[MOTOR_STATE_ALIGN_SAMPLE] = SMF_CREATE_STATE(motor_state_align_sample_entry,
					     motor_state_align_sample_run,
					     motor_state_align_sample_exit,
					     &motor_states[MOTOR_STATE_CALIBRATION],
					     NULL),
	[MOTOR_STATE_IDLE] = SMF_CREATE_STATE(motor_state_idle_entry,
					       motor_state_idle_run,
					       NULL, NULL, NULL),
	[MOTOR_STATE_OFFLINE] = SMF_CREATE_STATE(motor_state_offline_entry,
						  motor_state_offline_run,
					  motor_state_offline_exit, NULL,
						  &motor_states[MOTOR_STATE_CALIBRATION]),
	[MOTOR_STATE_ONLINE] = SMF_CREATE_STATE(motor_state_online_entry,
						 motor_state_online_run,
					 motor_state_online_exit, NULL,
						 &motor_states[MOTOR_STATE_ONLINE_VELOCITY_OPEN]),
	[MOTOR_STATE_ONLINE_TORQUE] = SMF_CREATE_STATE(motor_state_online_torque_entry,
							 motor_state_online_torque_run,
						 motor_state_online_torque_exit,
							 &motor_states[MOTOR_STATE_ONLINE],
							 NULL),
	[MOTOR_STATE_ONLINE_VELOCITY_OPEN] = SMF_CREATE_STATE(motor_state_online_velocity_open_entry,
								motor_state_online_velocity_open_run,
								motor_state_online_velocity_open_exit,
								&motor_states[MOTOR_STATE_ONLINE],
								NULL),
	[MOTOR_STATE_ONLINE_VELOCITY_CLOSED] = SMF_CREATE_STATE(motor_state_online_velocity_closed_entry,
								  motor_state_online_velocity_closed_run,
								  motor_state_online_velocity_closed_exit,
								  &motor_states[MOTOR_STATE_ONLINE],
								  NULL),
	[MOTOR_STATE_ONLINE_POSITION] = SMF_CREATE_STATE(motor_state_online_position_entry,
							  motor_state_online_position_run,
							  motor_state_online_position_exit,
							  &motor_states[MOTOR_STATE_ONLINE],
							  NULL),
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
	case MOTOR_STATE_ALIGN_SAMPLE: return "ALIGN_SAMPLE";
	case MOTOR_STATE_IDLE:         return "IDLE";
	case MOTOR_STATE_OFFLINE:      return "OFFLINE";
	case MOTOR_STATE_ONLINE:       return "ONLINE";
	case MOTOR_STATE_ONLINE_TORQUE:          return "ONLINE_TORQUE";
	case MOTOR_STATE_ONLINE_VELOCITY_OPEN:   return "ONLINE_VELOCITY_OPEN";
	case MOTOR_STATE_ONLINE_VELOCITY_CLOSED: return "ONLINE_VELOCITY_CLOSED";
	case MOTOR_STATE_ONLINE_POSITION:        return "ONLINE_POSITION";
	case MOTOR_STATE_ERROR:        return "ERROR";
	default:                       return "UNKNOWN";
	}
}

const char *motor_event_to_string(enum motor_event_type event_type)
{
	switch (event_type) {
	case MOTOR_EVENT_INIT:              return "INIT";
	case MOTOR_EVENT_RUN:               return "RUN";
	case MOTOR_EVENT_IDLE:              return "IDLE";
	case MOTOR_EVENT_OFFLINE:           return "OFFLINE";
	case MOTOR_EVENT_ONLINE:            return "ONLINE";
	case MOTOR_EVENT_CALIBRATE_REQUEST: return "CALIBRATE_REQUEST";
	case MOTOR_EVENT_MODE_CHANGE:       return "MODE_CHANGE";
	case MOTOR_EVENT_PARAM_UPDATE:      return "PARAM_UPDATE";
	case MOTOR_EVENT_CLEAR_ERROR:       return "CLEAR_ERROR";
	case MOTOR_EVENT_ERROR:             return "ERROR";
	case MOTOR_EVENT_PROFILE_SEQ_TICK:  return "PROFILE_SEQ_TICK";
	case MOTOR_EVENT_TIMEOUT:           return "TIMEOUT";
	case MOTOR_EVENT_NONE:              return "NONE";
	default:                            return "UNKNOWN";
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
	case ERROR_INVALID_ANGLE:   return "INVALID_ANGLE";
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

	mcpwm_stop(pwm1);
	mcpwm_stop(pwm3);
	mcpwm_stop(pwm8);

	drv8328_disable_all_channels(gate_driver_a);
	drv8328_disable_all_channels(gate_driver_b);
	mcpwm_disable(pwm1, 4);
	mcpwm_disable(pwm3, 1);

	/* Initialize PWM channels */
	mcpwm_configure(pwm1, 4, STM32_PWM_OC_MODE_PWM2);
	mcpwm_configure(pwm3, 1, 0);

	/* Set initial duty cycles */
	mcpwm_set_duty_cycle(pwm1, 4, 0x01000000);
	// mcpwm_set_duty_cycle(pwm3, 1, 0x40000000);
	mcpwm_set_duty_cycle(pwm3, 1, 0x47AE147A); /* 56% duty cycle */

	/* Set up encoder callback */
	mcpwm_set_compare_callback(pwm3, 1, encoder1_callback, params);

	/* Set up break interrupt handler for hardware fault protection */
	mcpwm_set_break_callback(pwm1, gate_driver_a_break_callback, params);
	mcpwm_set_break_callback(pwm8, gate_driver_b_break_callback, params);

	/* Set up ADC callback */
	adc_injected_set_callback(adc1, adc_callback, params);
	adc_injected_enable(adc1);

	/* Initialize timing subsystem for ISR performance measurement */
	timing_init();
	timing_start();

	/* Enable PWM for sampling */
	mcpwm_enable(pwm3, 1);
	mcpwm_enable(pwm1, 4);

	/* Start master timers. pwm1 is the master timer */
	mcpwm_start(pwm3);
	mcpwm_start(pwm8);
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

	/* Initialize angle observer with 1-sample delay compensation for pipelined SPI4-16 reads */
	angle_observer_init(&params->observer,
			    1.0f / CONTROL_LOOP_FREQUENCY_HZ,
			    ANGLE_OBSERVER_BANDWIDTH_HZ,
			    MOTOR_POLE_PAIRS,
			    1.0f); /* SPI4-16 pipelined reads have 1-cycle delay */

	/* Initialize Id trajectory generator for smooth current ramping */
	traj_init(&params->traj_Id);
	traj_set_min_value(&params->traj_Id, 0.0f);
	traj_set_max_value(&params->traj_Id, MOTOR_MAX_CURRENT_A);

	/* Initialize velocity/position scaffold defaults */
	params->position_target_rad = 0.0f;
	params->profile_max_velocity_rad_s = VELOCITY_MAX_RAD_S;
	params->profile_max_accel_rad_s2 = VELOCITY_MAX_ACCEL_RAD_S2;
	params->velocity_cl_iq_limit_A = MOTOR_MAX_CURRENT_A;
	params->velocity_cl_kp_A_per_rad_s =
		MOTOR_MAX_CURRENT_A / MAX(params->profile_max_velocity_rad_s, 1.0f);
	params->position_cl_kp_rad_s_per_rad = params->profile_max_velocity_rad_s / PI_F32;
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
	atomic_set(&params->control_armed, 0);
	params->command_timeout_ms = COMMAND_TIMEOUT_DEFAULT_MS;
	params->last_command_update_ms = k_uptime_get_32();
	params->command_timeout_count = 0U;
	params->command_timeout_latched = false;

	traj_init(&params->traj_velocity);
	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
	motion_profile_quintic_init(&params->position_profile, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	motion_profile_quintic_cancel(&params->position_profile, 0.0f);
	params->profile_sequence_running = false;
	params->profile_sequence_loop = false;
	params->profile_sequence_count = 0U;
	params->profile_sequence_next_idx = 0U;
	params->profile_sequence_period_ms = 1000U;
	params->profile_sequence_period_ticks =
		MAX(1U, (uint32_t)((CONTROL_LOOP_FREQUENCY_HZ * params->profile_sequence_period_ms) / 1000.0f));
	params->profile_sequence_tick_counter = 0U;
	params->profile_sequence_event_drop_count = 0U;
	params->profile_sequence_move_duration_s = 0.100f;
	params->profile_sequence_end_velocity_rad_s = 0.0f;

	params->chopper_cal_active = false;
	params->chopper_cal_complete = false;
	params->chopper_cal_valid = false;
	params->chopper_cal_slots = 0U;
	params->chopper_cal_revs_target = 0U;
	params->chopper_cal_samples_per_edge = 0U;
	params->chopper_cal_midpoint_count = 0U;
	params->chopper_cal_total_edges_target = 0U;
	params->chopper_cal_total_edges_captured = 0U;
	params->chopper_cal_discarded_edges = 0U;
	params->chopper_cal_saved_timeout_ms = COMMAND_TIMEOUT_DEFAULT_MS;
	params->chopper_cal_edge_min_step_rad = 0.0f;
	params->chopper_cal_speed_target_rad_s = 0.0f;
	params->chopper_cal_last_wrapped_rad = 0.0f;
	params->chopper_cal_last_unwrapped_rad = 0.0f;
	params->chopper_cal_start_unwrapped_rad = 0.0f;

	for (uint32_t i = 0U; i < CHOPPER_CAL_MAX_EDGES; i++) {
		params->chopper_cal_edge_sum_rad[i] = 0.0f;
		params->chopper_cal_edge_count[i] = 0U;
	}
	for (uint32_t i = 0U; i < CHOPPER_CAL_MAX_SLOTS; i++) {
		params->chopper_blade_midpoints_rad[i] = 0.0f;
	}

	#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
	/* Initialize PRBS generator and RLS parameters */
	prbs_init(&params->prbs_gen);
	params->rls_decimation = RLS_DECIMATION;
	if (!is_power_of_two(params->rls_decimation)) {
		LOG_WRN("Invalid rls_decimation=%u, forcing 1", params->rls_decimation);
		params->rls_decimation = 1u;
	}
	params->rls_stagger_offset = RLS_STAGGER_OFFSET;
	if (params->rls_stagger_offset >= params->rls_decimation) {
		params->rls_stagger_offset &= (params->rls_decimation - 1u);
	}
	params->prbs_amplitude_V = PRBS_AMPLITUDE_V;
	params->Ld_est = params->Ls_measured_H;  /* Initial estimate from calibration */
	params->Lq_est = RLS_INITIAL_LQ_H;
	params->Id_rls_prev = 0.0f;  /* Initialize previous RLS current for dI/dt */
	params->Iq_rls_prev = 0.0f;

	/* Initialize d-axis RLS estimator */
	rls_motor_est_init(&params->rls_d,
	                   RLS_LAMBDA,
	                   CONTROL_LOOP_FREQUENCY_HZ / (float32_t)params->rls_decimation,
	                   RLS_CONVERGENCE_THRESHOLD,
	                   params->Rs_measured_ohm,
	                   params->Ls_measured_H,
	                   RLS_INITIAL_COVARIANCE);

	/* Initialize q-axis RLS estimator */
	rls_motor_est_init(&params->rls_q,
	                   RLS_LAMBDA,
	                   CONTROL_LOOP_FREQUENCY_HZ / (float32_t)params->rls_decimation,
	                   RLS_CONVERGENCE_THRESHOLD,
	                   params->Rs_measured_ohm,
	                   RLS_INITIAL_LQ_H,
	                   RLS_INITIAL_COVARIANCE);

	/* Initialize thermal model */
	params->thermal_decimation = THERMAL_DECIMATION;
	if (!is_power_of_two(params->thermal_decimation)) {
		LOG_WRN("Invalid thermal_decimation=%u, forcing 1", params->thermal_decimation);
		params->thermal_decimation = 1u;
	}
	params->Rs_ref_ohm = params->Rs_measured_ohm;  /* Save calibrated Rs as reference */
	params->Rs_ref_temp_C = RS_REF_TEMP_C;
	params->Rs_temp_coeff = RS_TEMP_COEFF;
	params->T_rls_C = THERMAL_T_AMBIENT;  /* Initialize to ambient */

	float32_t thermal_update_freq = CONTROL_LOOP_FREQUENCY_HZ / (float32_t)params->thermal_decimation;
	thermal_model_init(&params->thermal,
	                   THERMAL_R_TH,
	                   THERMAL_C_TH,
	                   THERMAL_T_AMBIENT,
	                   thermal_update_freq);

	/* Initialize RLS gating conditions */
	params->rls_min_current_A = RLS_MIN_CURRENT_A;
	params->rls_min_speed_rad_s = RLS_MIN_SPEED_RAD_S;
	params->rls_max_residual = RLS_MAX_RESIDUAL;
	params->rls_max_voltage_V = RLS_MAX_VOLTAGE_V;
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */
}

static enum smf_state_result motor_state_ctrl_init_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Transition to IDLE - ready but unpowered */
	smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);

	return SMF_EVENT_HANDLED;
}

/* State: CALIBRATION - Hierarchical parent state for all calibration sub-states */
static void motor_state_calibration_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("=== Starting Motor Calibration Sequence ===");

	params->calibration_complete = false;
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

	params->calibration_complete = true;
}

/* State: OFFSET_MEAS - Measure current sensor offsets */
static void motor_state_offset_meas_entry(void *obj)
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
		/* Offset measurement complete */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ROVERL_MEAS]);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

/* State: ROVERL_MEAS - Measure R/L time constant via sinusoidal excitation (TI method) */
static void motor_state_roverl_meas_entry(void *obj)
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

static enum smf_state_result motor_state_roverl_meas_run(void *obj)
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

static void motor_state_roverl_meas_exit(void *obj)
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
static void motor_state_rs_est_entry(void *obj)
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

static enum smf_state_result motor_state_rs_est_run(void *obj)
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

static void motor_state_rs_est_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting RS_EST state");

	/* Clear this state's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: ALIGN - Align rotor to known position (Phase 1: injection in generated frame) */
static void motor_state_align_entry(void *obj)
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

static enum smf_state_result motor_state_align_run(void *obj)
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

static void motor_state_align_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN state");

	/* Clear this phase's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
			      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: ALIGN_SAMPLE - Align rotor to known position (Phase 2: sample encoder/observer) */
static void motor_state_align_sample_entry(void *obj)
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

static enum smf_state_result motor_state_align_sample_run(void *obj)
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

		/* Transition from CALIBRATION (ALIGN_SAMPLE) to ONLINE */
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ONLINE]);
		return SMF_EVENT_HANDLED;
	}

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void motor_state_align_sample_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ALIGN_SAMPLE state");

	/* Clear this phase's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
			      BIT(MOTOR_FEATURE_PI_CONTROL));
}

/* State: IDLE - Ready but not running */
static void motor_state_idle_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering IDLE state");

	/* Disable all ISR features */
	motor_set_isr_feature_flags(params, 0);

	/* Disable gate drivers - motor unpowered */
	drv8328_disable_all_channels(gate_driver_a);
	drv8328_disable_all_channels(gate_driver_b);

	atomic_set(&params->control_armed, 0);
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
}

static enum smf_state_result motor_state_idle_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_ONLINE:
		if (params->calibration_complete) {
			LOG_INF("ONLINE request received, transitioning to ONLINE");
			smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ONLINE]);
			return SMF_EVENT_HANDLED;
		}

		LOG_WRN("ONLINE request ignored: calibration not complete");
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_OFFLINE:
		LOG_INF("OFFLINE request received, transitioning to OFFLINE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_OFFLINE]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_CALIBRATE_REQUEST:
		LOG_INF("Calibrate request received, forcing recalibration");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

/* State: OFFLINE - Motor energized, calibration and open-loop control */
static void motor_state_offline_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering OFFLINE state");

	/* OFFLINE baseline requirements (shared by all OFFLINE substates).
	 * CALIBRATION is a child of OFFLINE and should not own PWM-output baseline.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_PWM_OUTPUT));

	/* Enable gate driver channels - motor now energized */
	drv8328_enable_channel(gate_driver_a, 0);
	drv8328_enable_channel(gate_driver_a, 1);
	drv8328_enable_channel(gate_driver_b, 0);
	drv8328_enable_channel(gate_driver_b, 1);

	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
}

static void motor_state_offline_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting OFFLINE state");

	/* Clear OFFLINE baseline requirements on exit. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_PWM_OUTPUT));
}

static enum smf_state_result motor_state_offline_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Check if calibration is needed on first run */
	if (!params->calibration_complete) {
		/* SMF will automatically enter CALIBRATION child state */
		/* Just propagate events to child states */
	} else {
		/* Calibration already done, skip to ONLINE */
		LOG_INF("Calibration already complete, transitioning to ONLINE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ONLINE]);
		return SMF_EVENT_HANDLED;
	}

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_IDLE:
		LOG_INF("IDLE request received, transitioning to IDLE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_ERROR:
		LOG_WRN("%s event received, code: %s, transitioning to ERROR",
			motor_event_to_string(params->event.type),
			motor_error_to_string(params->event.error_code));
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate events to child states (CALIBRATION sequence) */
		return SMF_EVENT_PROPAGATE;
	}
}

static void motor_state_online_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE state");

	/* ONLINE baseline requirements (shared by all ONLINE substates). */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_BRAKING) |
				     BIT(MOTOR_FEATURE_PWM_OUTPUT) |
				     BIT(MOTOR_FEATURE_PI_CONTROL) |
				     BIT(MOTOR_FEATURE_RLS_ESTIMATION) |
				     BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

static void motor_state_online_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE state");

	/* Clear ONLINE baseline requirements on exit. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_BRAKING) |
				      BIT(MOTOR_FEATURE_PWM_OUTPUT) |
				      BIT(MOTOR_FEATURE_PI_CONTROL) |
				      BIT(MOTOR_FEATURE_RLS_ESTIMATION) |
				      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

static enum smf_state_result motor_state_online_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_IDLE:
		LOG_INF("IDLE request received, transitioning to IDLE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_IDLE]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_ERROR:
		LOG_WRN("%s event received, code: %s, transitioning to ERROR",
			motor_event_to_string(params->event.type),
			motor_error_to_string(params->event.error_code));
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_ERROR]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_PARAM_UPDATE:
		/* Apply parameter update to shadow buffer */
		motor_api_apply_param_update(params);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_MODE_CHANGE:
		/* Transition to requested control mode substate */
		LOG_INF("%s event: changing to state %s",
			motor_event_to_string(params->event.type),
			motor_state_to_string(params->event.target_mode));

		/* Validate target is an ONLINE substate */
		if (!motor_state_is_online_submode(params->event.target_mode)) {
			LOG_ERR("Invalid mode change target: %s",
				motor_state_to_string(params->event.target_mode));
			return SMF_EVENT_HANDLED;
		}

		smf_set_state(SMF_CTX(params), &motor_states[params->event.target_mode]);
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
}

/* Substate: ONLINE_TORQUE - Direct Id/Iq control mode */
static void motor_state_online_torque_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Entering ONLINE_TORQUE substate (direct Id/Iq control)");

	/* Encoder-based control: add encoder read; ONLINE provides the baseline. */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));
	/* Start torque mode from a neutral current command for bumpless handover. */
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
}

static void motor_state_online_torque_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_TORQUE substate");

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ));
}

static enum smf_state_result motor_state_online_torque_run(void *obj)
{
	/* Direct torque control - no additional processing needed
	 * Id/Iq setpoints are controlled via shell commands
	 * Parent ONLINE state handles stop/error events
	 */
	return SMF_EVENT_PROPAGATE;
}

/* Substate: ONLINE_VELOCITY_OPEN - Open-loop velocity control */
static void motor_state_online_velocity_open_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t mech_angle_rad = angle_observer_get_mech_angle(&params->observer);

	LOG_INF("Entering ONLINE_VELOCITY_OPEN substate");

	/* Open-loop velocity control uses angle generator and velocity trajectory.
	 * ONLINE provides the baseline.
	 */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));

	/* Initialize angle generator for open-loop mode */
	angle_gen_init(&params->angle_gen, 1.0f / CONTROL_LOOP_FREQUENCY_HZ);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	/* Preserve commutation frame across mode transitions. */
	angle_gen_set_angle(&params->angle_gen, mech_angle_rad);

	/* Initialize velocity trajectory */
	traj_init(&params->traj_velocity);
	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);

	LOG_INF("Open-loop velocity mode initialized: max=%.1f Hz, accel=%.1f Hz/s",
		(double)(params->profile_max_velocity_rad_s / (2.0f * PI_F32)),
		(double)(params->profile_max_accel_rad_s2 / (2.0f * PI_F32)));
}

static enum smf_state_result motor_state_online_velocity_open_run(void *obj)
{
	/* Velocity control happens in motor_isr based on active substate
	 * This handler just maintains state and propagates events to parent
	 */
	return SMF_EVENT_PROPAGATE;
}

static void motor_state_online_velocity_open_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_VELOCITY_OPEN substate");

	/* Reset angle generator and trajectory */
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	angle_gen_set_angle(&params->angle_gen, 0.0f);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
				      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
}

/* Substate: ONLINE_VELOCITY_CLOSED - Closed-loop velocity control */
static void motor_state_online_velocity_closed_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t speed_mech_rad_s = angle_observer_get_mech_speed(&params->observer);

	LOG_INF("Entering ONLINE_VELOCITY_CLOSED substate");

	/* Closed-loop velocity uses measured speed and acceleration-limited velocity profile. */
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
					      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));

	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, speed_mech_rad_s);
	traj_set_int_value(&params->traj_velocity, speed_mech_rad_s);
	params->velocity_target_rad_s = speed_mech_rad_s;
	params->velocity_ref_rad_s = speed_mech_rad_s;
}

static enum smf_state_result motor_state_online_velocity_closed_run(void *obj)
{
	ARG_UNUSED(obj);
	return SMF_EVENT_PROPAGATE;
}

static void motor_state_online_velocity_closed_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_VELOCITY_CLOSED substate");

	/* Clear this substate's additional requirements. */
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

/* Substate: ONLINE_POSITION - Cascaded position->velocity->current scaffold */
static void motor_state_online_position_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;
	float32_t speed_mech_rad_s = angle_observer_get_mech_speed(&params->observer);
	float32_t position_mech_rad = angle_observer_get_mech_angle(&params->observer);

	LOG_INF("Entering ONLINE_POSITION substate");

	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					     BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ANGLE_GEN) |
					      BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));

	/* Use current angle as initial target for bumpless mode entry. */
	params->position_target_rad = position_mech_rad;
	motion_profile_quintic_cancel(&params->position_profile, position_mech_rad);

	traj_set_min_value(&params->traj_velocity, -params->profile_max_velocity_rad_s);
	traj_set_max_value(&params->traj_velocity, params->profile_max_velocity_rad_s);
	traj_set_max_delta(&params->traj_velocity,
			   params->profile_max_accel_rad_s2 / CONTROL_LOOP_FREQUENCY_HZ);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, speed_mech_rad_s);
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = speed_mech_rad_s;
}

static enum smf_state_result motor_state_online_position_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	switch (params->event.type) {
	case MOTOR_EVENT_PROFILE_SEQ_TICK: {
		if (!params->profile_sequence_running) {
			return SMF_EVENT_HANDLED;
		}
		if (params->profile_sequence_count == 0U) {
			params->profile_sequence_running = false;
			params->profile_sequence_tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}
		if (atomic_get(&params->control_armed) == 0) {
			return SMF_EVENT_HANDLED;
		}

		uint16_t idx = params->profile_sequence_next_idx;
		if (idx >= params->profile_sequence_count) {
			if (params->profile_sequence_loop) {
				idx = 0U;
			} else {
				params->profile_sequence_running = false;
				params->profile_sequence_tick_counter = 0U;
				return SMF_EVENT_HANDLED;
			}
		}

		int ret = motor_position_plan_sequence_move(params, params->profile_sequence_points_rad[idx]);
		if (ret != 0) {
			LOG_ERR("Profile sequence move %u failed (%d), stopping", idx, ret);
			params->profile_sequence_running = false;
			params->profile_sequence_tick_counter = 0U;
			return SMF_EVENT_HANDLED;
		}

		idx++;
		if (idx >= params->profile_sequence_count) {
			if (params->profile_sequence_loop) {
				idx = 0U;
			} else {
				params->profile_sequence_running = false;
				params->profile_sequence_tick_counter = 0U;
				LOG_INF("Profile sequence completed");
			}
		}
		params->profile_sequence_next_idx = idx;
		return SMF_EVENT_HANDLED;
	}

	default:
		return SMF_EVENT_PROPAGATE;
	}
}

static void motor_state_online_position_exit(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	LOG_INF("Exiting ONLINE_POSITION substate");

	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	params->velocity_target_rad_s = 0.0f;
	motion_profile_quintic_cancel(&params->position_profile,
				      angle_observer_get_mech_angle(&params->observer));
	params->velocity_ref_rad_s = 0.0f;
	params->profile_sequence_running = false;
	params->profile_sequence_tick_counter = 0U;

	motor_disable_isr_feature_flags(params, BIT(MOTOR_FEATURE_ENCODER_READ) |
					      BIT(MOTOR_FEATURE_VELOCITY_TRAJ));
	motor_enable_isr_feature_flags(params, BIT(MOTOR_FEATURE_USE_COMMANDED_CURRENTS));
}

/* State: ERROR - Fault condition */
static void motor_state_error_entry(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Disable all ISR features */
	motor_set_isr_feature_flags(params, 0);

	/* Store the error code that triggered this state */
	params->last_error_code = params->event.error_code;

	LOG_ERR("Entering ERROR state, code: %s",
		motor_error_to_string(params->last_error_code));

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

	/* Emergency stop gate drivers (sets INL GPIOs low) */
	drv8328_disable_all_channels(gate_driver_a);
	drv8328_disable_all_channels(gate_driver_b);

	atomic_set(&params->control_armed, 0);
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
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
		return SMF_EVENT_HANDLED;

	default:
		/* Propagate unhandled events */
		return SMF_EVENT_PROPAGATE;
	}
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

	/* Update ISR-safe state after initialization complete */
	motor_params.state_for_isr = motor_params.smf.current;

	/* Event-driven state machine loop */
	while (1) {
		rc = k_msgq_get(&motor_event_queue, &motor_params.event, K_MSEC(10));

		if (rc == -EAGAIN) {
			/* No event received within timeout - run state machine with no event */
			struct motor_event evt = {
				.type = MOTOR_EVENT_NONE,
			};
			motor_params.event = evt;

		}

		/* Start each SMF cycle from the currently published stable flags so that
		 * hierarchical entry/exit delta updates compose correctly.
		 */
		motor_params.feature_flags_next = atomic_get(&motor_params.feature_flags);

		rc = smf_run_state(SMF_CTX(&motor_params));

		if (rc) {
			/* State machine terminated */
			LOG_ERR("State machine terminated with code %d", rc);
			break;
		}

		/* Update ISR-safe state and feature flags after all actions complete */
		motor_params.state_for_isr = motor_params.smf.current;
		atomic_set(&motor_params.feature_flags, motor_params.feature_flags_next);
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
