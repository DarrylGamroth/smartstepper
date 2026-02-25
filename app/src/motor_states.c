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
#include <drivers/pwm/mcpwm_stm32.h>
#include <drivers/adc_injected.h>
#include <drivers/gate_driver/ti_drv8328.h>
#include <dt-bindings/pwm/stm32-mcpwm.h>

#include "motor_states.h"
#include "motor_isr.h"
#include "motor_control_api.h"
#include "motor_hardware.h"
#include "config.h"
#include "motor/filters/pi.h"
#include "motor/filters/filter_fo.h"
#include "motor/motion/traj.h"
#include "motor/observers/angle_observer.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/motion_planner.h"
#include "motor_state_utils.h"
#include "motor_states_calibration.h"
#include "motor_states_online.h"
#include "motor_commission.h"
#include "motor_torque.h"
#include "motor/runtime/config_snapshot.h"

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

static inline void motor_publish_isr_config_snapshot(struct motor_parameters *params)
{
	struct motor_rt_config_snapshot snapshot = {
		.epoch = 0U,
		.state = params->smf.current,
		.feature_flags = params->feature_flags_next,
		.velocity_loop_decimation = params->velocity_loop_decimation,
		.position_loop_decimation = params->position_loop_decimation,
		.profile_sequence_running = params->profile_sequence_running,
		.profile_sequence_loop = params->profile_sequence_loop,
		.profile_sequence_trigger_source = params->profile_sequence_trigger_source,
		.profile_sequence_trigger_edge = params->profile_sequence_trigger_edge,
		.profile_sequence_trigger_channel = params->profile_sequence_trigger_channel,
		.profile_sequence_period_ticks = params->profile_sequence_period_ticks,
		.profile_sequence_period_ms = params->profile_sequence_period_ms,
	};

	/* Keep legacy published fields in sync during transition. */
	params->state_for_isr = snapshot.state;
	atomic_set(&params->feature_flags, snapshot.feature_flags);
	motor_config_snapshot_publish(&snapshot);
}

static inline void motor_force_safe_pwm_outputs(void)
{
	/* Program a neutral duty vector so re-enabling gate drivers never replays
	 * stale PWM compare values from a previous control mode/fault condition.
	 */
	mcpwm_stm32_set_duty_cycle_2phase_f32(pwm1, 0.5f, 0.5f);
	mcpwm_stm32_set_duty_cycle_2phase_f32(pwm8, 0.5f, 0.5f);
}

static inline void motor_reset_control_runtime(struct motor_parameters *params)
{
	params->Id_setpoint_A = 0.0f;
	params->Iq_setpoint_A = 0.0f;
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	traj_set_target_value(&params->traj_Id, 0.0f);
	traj_set_int_value(&params->traj_Id, 0.0f);
	traj_set_target_value(&params->traj_velocity, 0.0f);
	traj_set_int_value(&params->traj_velocity, 0.0f);
	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);
	angle_gen_set_velocity(&params->angle_gen, 0.0f);
	motor_mpr_velocity_reset(&params->velocity_mpr_state,
				 params->velocity_rad_s,
				 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	motor_dob_reset(&params->velocity_dob_state,
			params->velocity_rad_s);
	params->position_quality_flags = 0U;
	params->position_stale_count = 0U;
	params->position_stale_events = 0U;
	params->position_glitch_count = 0U;
	params->position_jitter_count = 0U;
	params->velocity_dob_iq_ff_a = 0.0f;
	params->velocity_dob_disturbance_nm = 0.0f;
	params->velocity_dob_residual_rad_s = 0.0f;
}

static inline enum motor_state motor_resolve_requested_online_mode(const struct motor_parameters *params)
{
	enum motor_state mode = MOTOR_STATE_ONLINE_VELOCITY_OPEN;

	if (params != NULL) {
		mode = (enum motor_state)params->requested_online_mode;
	}

	if (!motor_state_is_online_submode(mode)) {
		mode = MOTOR_STATE_ONLINE_VELOCITY_OPEN;
	}

	return mode;
}

static struct motor_parameters motor_params;
K_MSGQ_DEFINE(motor_event_queue, sizeof(struct motor_event), 16, 4);

#define MOTOR_ISR_EVENT_RING_SIZE 16U
static struct motor_event motor_isr_event_ring[MOTOR_ISR_EVENT_RING_SIZE];
static atomic_t motor_isr_event_head;
static atomic_t motor_isr_event_tail;

static int motor_isr_event_ring_pop(struct motor_event *evt)
{
	if (evt == NULL) {
		return -EINVAL;
	}

	uint32_t tail = (uint32_t)atomic_get(&motor_isr_event_tail);
	uint32_t head = (uint32_t)atomic_get(&motor_isr_event_head);
	if (tail == head) {
		return -ENOENT;
	}

	*evt = motor_isr_event_ring[tail];
	atomic_set(&motor_isr_event_tail, (atomic_val_t)((tail + 1U) % MOTOR_ISR_EVENT_RING_SIZE));
	return 0;
}

int motor_api_enqueue_event_from_isr(const struct motor_event *evt)
{
	if (evt == NULL) {
		return -EINVAL;
	}

	uint32_t head = (uint32_t)atomic_get(&motor_isr_event_head);
	uint32_t tail = (uint32_t)atomic_get(&motor_isr_event_tail);
	uint32_t next = (head + 1U) % MOTOR_ISR_EVENT_RING_SIZE;
	if (next == tail) {
		return -ENOSPC;
	}

	motor_isr_event_ring[head] = *evt;
	atomic_set(&motor_isr_event_head, (atomic_val_t)next);
	return 0;
}

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
static void motor_state_idle_entry(void *obj);
static enum smf_state_result motor_state_idle_run(void *obj);
static void motor_state_offline_entry(void *obj);
static enum smf_state_result motor_state_offline_run(void *obj);
static void motor_state_offline_exit(void *obj);
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
					&motor_states[MOTOR_STATE_ALIGN_POS_INJECT]),
	[MOTOR_STATE_ALIGN_POS_INJECT] = SMF_CREATE_STATE(motor_state_align_pos_inject_entry,
						  motor_state_align_pos_inject_run,
						  motor_state_align_pos_inject_exit,
						  &motor_states[MOTOR_STATE_ALIGN],
						  NULL),
	[MOTOR_STATE_ALIGN_POS_SAMPLE] = SMF_CREATE_STATE(motor_state_align_pos_sample_entry,
						  motor_state_align_pos_sample_run,
						  motor_state_align_pos_sample_exit,
						  &motor_states[MOTOR_STATE_ALIGN],
						  NULL),
	[MOTOR_STATE_ALIGN_NEG_INJECT] = SMF_CREATE_STATE(motor_state_align_neg_inject_entry,
						  motor_state_align_neg_inject_run,
						  motor_state_align_neg_inject_exit,
						  &motor_states[MOTOR_STATE_ALIGN],
						  NULL),
	[MOTOR_STATE_ALIGN_NEG_SAMPLE] = SMF_CREATE_STATE(motor_state_align_neg_sample_entry,
						  motor_state_align_neg_sample_run,
						  motor_state_align_neg_sample_exit,
						  &motor_states[MOTOR_STATE_ALIGN],
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
	case MOTOR_STATE_ALIGN_POS_INJECT: return "ALIGN_POS_INJECT";
	case MOTOR_STATE_ALIGN_POS_SAMPLE: return "ALIGN_POS_SAMPLE";
	case MOTOR_STATE_ALIGN_NEG_INJECT: return "ALIGN_NEG_INJECT";
	case MOTOR_STATE_ALIGN_NEG_SAMPLE: return "ALIGN_NEG_SAMPLE";
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
	case MOTOR_EVENT_COMMISSION_REQUEST: return "COMMISSION_REQUEST";
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
	/* Fire encoder compare callback in mid-early PWM period to balance SPI
	 * completion slack against switching-noise susceptibility.
	 */
	mcpwm_set_duty_cycle(pwm3, 1, 0x5999999A); /* 35% duty cycle */

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

	/* Initialize angle observer delay compensation per encoder transport. */
	angle_observer_init(&params->observer,
			    1.0f / CONTROL_LOOP_FREQUENCY_HZ,
			    ANGLE_OBSERVER_BANDWIDTH_HZ,
			    MOTOR_POLE_PAIRS,
			    ENCODER_SPI_PIPELINE_DELAY_SAMPLES);

	/* Initialize Id trajectory generator for smooth current ramping */
	traj_init(&params->traj_Id);
	traj_set_min_value(&params->traj_Id, -MOTOR_MAX_CURRENT_A);
	traj_set_max_value(&params->traj_Id, MOTOR_MAX_CURRENT_A);

	/* Initialize velocity/position scaffold defaults */
	params->position_target_rad = 0.0f;
	params->encoder_direction_sign = (ENCODER_DIRECTION_SIGN >= 0) ? 1 : -1;
	params->requested_online_mode = MOTOR_STATE_ONLINE_VELOCITY_OPEN;
	params->profile_max_velocity_rad_s = VELOCITY_MAX_RAD_S;
	params->profile_max_accel_rad_s2 = VELOCITY_MAX_ACCEL_RAD_S2;
	params->outer_loop_mode = OUTER_LOOP_MPR_DEFAULT_ENABLED ?
		MOTOR_OUTER_LOOP_MODE_MPR : MOTOR_OUTER_LOOP_MODE_PI;
	params->velocity_loop_decimation = CLAMP(VELOCITY_LOOP_DECIMATION_DEFAULT,
						 OUTER_LOOP_DECIMATION_MIN,
						 OUTER_LOOP_DECIMATION_MAX);
	params->position_loop_decimation = CLAMP(POSITION_LOOP_DECIMATION_DEFAULT,
						 OUTER_LOOP_DECIMATION_MIN,
						 OUTER_LOOP_DECIMATION_MAX);
	params->velocity_loop_phase = 0U;
	params->position_loop_phase = 0U;
	float32_t velocity_loop_dt_s =
		(float32_t)params->velocity_loop_decimation / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t position_loop_dt_s =
		(float32_t)params->position_loop_decimation / CONTROL_LOOP_FREQUENCY_HZ;
	params->velocity_cl_iq_limit_A = MOTOR_MAX_CURRENT_A;
	params->velocity_cl_kp_A_per_rad_s =
		MOTOR_MAX_CURRENT_A / MAX(params->profile_max_velocity_rad_s, 1.0f);
	params->velocity_cl_ki_A_per_rad = 2.0f * params->velocity_cl_kp_A_per_rad_s;
	params->position_cl_kp_rad_s_per_rad = params->profile_max_velocity_rad_s / PI_F32;
	params->position_cl_ki_rad_s2_per_rad = 0.5f * params->position_cl_kp_rad_s_per_rad;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_mpr_cfg.dt_s = velocity_loop_dt_s;
	params->velocity_mpr_cfg.horizon = 8U;
	params->velocity_mpr_cfg.q_speed = 1.5f;
	params->velocity_mpr_cfg.r_delta_iq = 0.05f;
	params->velocity_mpr_cfg.iq_limit_a = params->velocity_cl_iq_limit_A;
	params->velocity_mpr_cfg.max_delta_iq_a = 0.0f;
	params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s = 0.02f;
	motor_mpr_velocity_reset(&params->velocity_mpr_state, 0.0f, 0.0f);

	params->position_mpr_cfg.dt_s = position_loop_dt_s;
	params->position_mpr_cfg.horizon = 16U;
	params->position_mpr_cfg.q_position = 2.0f;
	params->position_mpr_cfg.q_velocity_ff = 0.4f;
	params->position_mpr_cfg.r_delta_velocity = 0.2f;
	params->position_mpr_cfg.velocity_limit_rad_s = params->profile_max_velocity_rad_s;
	params->position_mpr_cfg.max_delta_velocity_rad_s =
		params->profile_max_accel_rad_s2 * position_loop_dt_s;
	motor_mpr_position_reset(&params->position_mpr_state, 0.0f);
	params->observer_alignment_offset_rad = 0.0f;
	params->observer_elec_trim_rad = 0.0f;
	params->flux_linkage_wb_active = MOTOR_FLUX_LINKAGE_WB;
	params->torque_gain_nm_per_a_active =
		motor_torque_gain_from_flux(params->flux_linkage_wb_active);
	params->velocity_dob_cfg.enabled = true;
	params->velocity_dob_cfg.dt_s = velocity_loop_dt_s;
	params->velocity_dob_cfg.observer_gain_nm_per_rad_s = 0.02f;
	params->velocity_dob_cfg.iq_ff_limit_a = params->velocity_cl_iq_limit_A;
	params->velocity_dob_cfg.torque_limit_nm = params->torque_gain_nm_per_a_active *
						   params->velocity_cl_iq_limit_A;
	motor_dob_reset(&params->velocity_dob_state, 0.0f);
	params->velocity_target_rad_s = 0.0f;
	params->velocity_ref_rad_s = 0.0f;
	params->velocity_filtered_rad_s = 0.0f;
	params->position_rad = 0.0f;
	params->position_unwrapped_rad = 0.0f;
	params->position_innovation_rad = 0.0f;
	params->velocity_rad_s = 0.0f;
	params->acceleration_rad_s2 = 0.0f;
	params->position_quality_flags = 0U;
	params->position_stale_count = 0U;
	params->position_stale_events = 0U;
	params->position_glitch_count = 0U;
	params->position_jitter_count = 0U;
	params->encoder_capture_enabled = false;
	params->encoder_capture_decimation = 1U;
	params->encoder_capture_phase = 0U;
	params->encoder_capture_write_idx = 0U;
	params->encoder_capture_count = 0U;
	params->encoder_capture_overrun_count = 0U;
	params->velocity_dob_iq_ff_a = 0.0f;
	params->velocity_dob_disturbance_nm = 0.0f;
	params->velocity_dob_residual_rad_s = 0.0f;
	params->Rs_measured_ohm = MOTOR_RESISTANCE_OHM;
	params->Ls_measured_H = MOTOR_INDUCTANCE_D_H;
	params->R_over_L_measured =
		(params->Ls_measured_H > 0.0f) ? (params->Rs_measured_ohm / params->Ls_measured_H) : 0.0f;
	params->inertia_kgm2_active = MOTOR_INERTIA_KGM2;
	params->viscous_friction_nm_per_rad_s_active = 0.0f;
	params->coulomb_friction_nm_active = 0.0f;
	params->Ld_est = MOTOR_INDUCTANCE_D_H;
	params->Lq_est = MOTOR_INDUCTANCE_Q_H;
	atomic_set(&params->control_armed, 0);
	params->command_timeout_ms = COMMAND_TIMEOUT_DEFAULT_MS;
	params->last_command_update_ms = k_uptime_get_32();
	params->command_timeout_count = 0U;
	params->command_timeout_latched = false;
	params->calibration_complete = false;
	params->calibration_running = false;
	params->commissioning_complete = false;
	params->calibration_mode = MOTOR_CALIBRATION_MODE_BOOT;
	params->align_pos_sample_retries = 0U;
	params->align_neg_sample_retries = 0U;
	params->align_pos_sample_count = 0U;
	params->align_neg_sample_count = 0U;
	params->align_pos_sum_sin = 0.0f;
	params->align_pos_sum_cos = 0.0f;
	params->align_neg_sum_sin = 0.0f;
	params->align_neg_sum_cos = 0.0f;
	params->align_pos_mech_angle_rad = 0.0f;
	params->align_neg_mech_angle_rad = 0.0f;
	motor_commission_init(params);

	motor_velocity_plan_init(&params->traj_velocity,
				 params->profile_max_velocity_rad_s,
				 params->profile_max_accel_rad_s2,
				 1.0f / CONTROL_LOOP_FREQUENCY_HZ,
				 0.0f);
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
	params->profile_sequence_trigger_source = PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL;
	params->profile_sequence_trigger_edge = PROFILE_SEQUENCE_TRIGGER_EDGE_RISING;
	params->profile_sequence_trigger_channel = 0U;
	params->profile_sequence_ext_capture_enabled = false;
	params->profile_sequence_ext_last_capture_valid = false;
	params->profile_sequence_ext_min_interval_us = 0U;
	params->profile_sequence_ext_min_interval_cycles = 0U;
	params->profile_sequence_ext_last_capture_cycles = 0U;
	params->profile_sequence_ext_trigger_count = 0U;
	params->profile_sequence_ext_reject_count = 0U;
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
	params->rls_excitation_current_A = RLS_EXCITATION_CURRENT_A;
	params->Ld_est = params->Ls_measured_H;  /* Initial estimate from calibration */
	params->Lq_est = RLS_INITIAL_LQ_H;
	params->Id_rls_prev = 0.0f;  /* Initialize previous RLS current for dI/dt */
	params->Iq_rls_prev = 0.0f;
	params->rls_d_prev_cycle = 0u;
	params->rls_q_prev_cycle = 0u;
	params->rls_d_prev_valid = 0u;
	params->rls_q_prev_valid = 0u;

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
	motor_reset_control_runtime(params);
	motor_force_safe_pwm_outputs();
}

static enum smf_state_result motor_state_idle_run(void *obj)
{
	struct motor_parameters *params = (struct motor_parameters *)obj;

	/* Process current event */
	switch (params->event.type) {
	case MOTOR_EVENT_ONLINE:
		if (params->calibration_complete) {
			enum motor_state online_mode =
				motor_resolve_requested_online_mode(params);
			LOG_INF("ONLINE request received, transitioning to ONLINE");
			smf_set_state(SMF_CTX(params), &motor_states[online_mode]);
			return SMF_EVENT_HANDLED;
		}

		LOG_WRN("ONLINE request ignored: calibration not complete");
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_OFFLINE:
		LOG_INF("OFFLINE request received, transitioning to OFFLINE");
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_OFFLINE]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_CALIBRATE_REQUEST:
		LOG_INF("Calibrate request received, running boot calibration");
		params->calibration_mode = MOTOR_CALIBRATION_MODE_BOOT;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_COMMISSION_REQUEST:
		LOG_INF("Commission request received, running commissioning sequence");
		params->calibration_mode = MOTOR_CALIBRATION_MODE_COMMISSIONING;
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

	/* Ensure a clean bumpless restart after any fault or mode transition. */
	motor_reset_control_runtime(params);
	motor_force_safe_pwm_outputs();

	/* Enable gate driver channels - motor now energized */
	drv8328_enable_channel(gate_driver_a, 0);
	drv8328_enable_channel(gate_driver_a, 1);
	drv8328_enable_channel(gate_driver_b, 0);
	drv8328_enable_channel(gate_driver_b, 1);

	/* First OFFLINE entry after boot runs fast boot calibration sequence. */
	if (!params->calibration_complete &&
	    params->calibration_mode != MOTOR_CALIBRATION_MODE_COMMISSIONING) {
		params->calibration_mode = MOTOR_CALIBRATION_MODE_BOOT;
	}
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

	case MOTOR_EVENT_CALIBRATE_REQUEST:
		LOG_INF("Calibrate request received, running boot calibration");
		params->calibration_mode = MOTOR_CALIBRATION_MODE_BOOT;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		return SMF_EVENT_HANDLED;

	case MOTOR_EVENT_COMMISSION_REQUEST:
		LOG_INF("Commission request received, running commissioning sequence");
		params->calibration_mode = MOTOR_CALIBRATION_MODE_COMMISSIONING;
		smf_set_state(SMF_CTX(params), &motor_states[MOTOR_STATE_CALIBRATION]);
		return SMF_EVENT_HANDLED;

	default:
		break;
	}

	/* Auto-enter ONLINE once OFFLINE has no active calibration and the required
	 * boot calibration has already completed.
	 */
	if (!params->calibration_running && params->calibration_complete) {
		enum motor_state online_mode = motor_resolve_requested_online_mode(params);
		LOG_INF("Calibration already complete, transitioning to ONLINE");
		smf_set_state(SMF_CTX(params), &motor_states[online_mode]);
		return SMF_EVENT_HANDLED;
	}

	/* Propagate events to child states (CALIBRATION sequence). */
	return SMF_EVENT_PROPAGATE;
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
	motor_reset_control_runtime(params);
	motor_force_safe_pwm_outputs();
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
	motor_config_snapshot_init();

	/* Initialize state timer */
	k_timer_init(&motor_params.state_timer, state_timer_expiry, NULL);

	/* Initialize state machine */
	smf_set_initial(SMF_CTX(&motor_params), &motor_states[MOTOR_STATE_HW_INIT]);
	atomic_set(&motor_isr_event_head, 0);
	atomic_set(&motor_isr_event_tail, 0);

	/* Publish initial coherent ISR snapshot. */
	motor_params.feature_flags_next = atomic_get(&motor_params.feature_flags);
	motor_publish_isr_config_snapshot(&motor_params);

	/* Event-driven state machine loop */
	while (1) {
		rc = motor_isr_event_ring_pop(&motor_params.event);
		if (rc == -ENOENT) {
			rc = k_msgq_get(&motor_event_queue, &motor_params.event, K_MSEC(10));
			if (rc == -EAGAIN) {
				/* No event received within timeout - run state machine with no event */
				struct motor_event evt = {
					.type = MOTOR_EVENT_NONE,
				};
				motor_params.event = evt;
			}
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

		/* Publish coherent ISR config after SMF actions complete. */
		motor_publish_isr_config_snapshot(&motor_params);
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
