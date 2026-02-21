/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <zephyr/kernel.h>
#include <zephyr/dsp/utils.h>
#include <zephyr/smf.h>
#include <zephyr/sys/util.h>
#include "pi.h"
#include "filter_fo.h"
#include "filter_so.h"
#include "angle_observer.h"
#include "angle_gen.h"
#include "rs_online.h"
#include "traj.h"
#include "motion_profile.h"
#include "motor_commission.h"
#include "motor_events.h"
#include "motor_states.h"
#include "prbs.h"
#include "rls_motor_est.h"
#include "thermal_model.h"
#include "math_constants.h"

#define MOTOR_PROFILE_SEQUENCE_MAX_POINTS 64U
#define CHOPPER_CAL_MAX_SLOTS MOTOR_PROFILE_SEQUENCE_MAX_POINTS
#define CHOPPER_CAL_MAX_EDGES (2U * CHOPPER_CAL_MAX_SLOTS)

#define PROFILE_SEQUENCE_TRIGGER_SRC_INTERNAL 0U
#define PROFILE_SEQUENCE_TRIGGER_SRC_EXTERNAL 1U

#define PROFILE_SEQUENCE_TRIGGER_EDGE_RISING 0U
#define PROFILE_SEQUENCE_TRIGGER_EDGE_FALLING 1U
#define PROFILE_SEQUENCE_TRIGGER_EDGE_BOTH 2U

#define MOTOR_ANGLE_INPUT_SRC_GENERATED 0U
#define MOTOR_ANGLE_INPUT_SRC_ENCODER 1U
#define MOTOR_ANGLE_INPUT_SRC_PROPAGATED 2U

#define MOTOR_CALIBRATION_MODE_BOOT 0U
#define MOTOR_CALIBRATION_MODE_COMMISSIONING 1U

/**
 * @brief Main motor control parameters structure
 *
 * Contains all state needed for motor control including SMF state machine,
 * control parameters, filters, observers, and telemetry.
 */
struct motor_parameters {
	/* State machine */
	struct smf_ctx smf;
	const struct smf_state *state_for_isr;
	struct motor_event event;  /* Current event being processed */
	struct k_timer state_timer;  /* Timer for state timeouts */

	/* PI controllers (stateful - NOT double buffered) */
	struct pi_f32 pi_Id;
	struct pi_f32 pi_Iq;
	float32_t max_modulation_index;  /* Maximum modulation index (0.0 to ~0.907) */

	/* Current sensing filters and offsets */
	struct filter_fo_f32 filter_Ia;
	struct filter_fo_f32 filter_Ib;
	struct filter_so_f32 filter_velocity_notch; /* Optional speed notch for velocity loop */
	float32_t Ia_offset;
	float32_t Ib_offset;

	/* Rs estimation filters */
	struct filter_fo_f32 filter_rs_est_V;
	struct filter_fo_f32 filter_rs_est_I;

	/* Current setpoints */
	float32_t Id_setpoint_A;
	float32_t Iq_setpoint_A;

	/* Safety interlock and command-timeout state */
	atomic_t control_armed;          /* 1 when torque-producing commands are allowed */
	uint32_t command_timeout_ms;     /* 0 disables timeout */
	uint32_t last_command_update_ms; /* Last command activity timestamp (k_uptime_get_32) */
	uint32_t command_timeout_count;  /* Number of timeout-triggered disarms */
	bool command_timeout_latched;    /* Prevent repeated timeout handling */

	/* Voltage references (computed by PI controllers) */
	float32_t max_voltage_magnitude_V;

	/* Applied voltages (after SVPWM limiting) */
	float32_t Vd_V;
	float32_t Vq_V;

	/* Observers and estimators */
	struct angle_observer_state observer;
	struct rs_online_estimator rs_est;
	struct traj_f32 traj_Id;
	struct traj_f32 traj_velocity;  /* Velocity trajectory for open-loop mode */
	struct motion_profile_quintic position_profile; /* Optional quintic position profile */
	float32_t position_target_rad;  /* Position target for closed-loop position mode */

	/* Hardware-timer-driven position sequence profile */
	bool profile_sequence_running;      /* Sequence engine active */
	bool profile_sequence_loop;         /* Loop sequence when last point is reached */
	uint16_t profile_sequence_count;    /* Number of valid points in sequence array */
	uint16_t profile_sequence_next_idx; /* Next point index to trigger */
	uint32_t profile_sequence_period_ms;      /* Target trigger period */
	uint32_t profile_sequence_period_ticks;   /* Hardware timer ISR ticks per trigger */
	uint32_t profile_sequence_tick_counter;   /* Runtime tick accumulator */
	uint32_t profile_sequence_event_drop_count; /* Dropped sequence-tick events */
	uint8_t profile_sequence_trigger_source; /* PROFILE_SEQUENCE_TRIGGER_SRC_* */
	uint8_t profile_sequence_trigger_edge;   /* PROFILE_SEQUENCE_TRIGGER_EDGE_* */
	uint8_t profile_sequence_trigger_channel; /* External capture channel index */
	bool profile_sequence_ext_capture_enabled; /* External capture currently armed */
	bool profile_sequence_ext_last_capture_valid; /* External filter history valid */
	uint32_t profile_sequence_ext_min_interval_us; /* Reject edges closer than this interval */
	uint32_t profile_sequence_ext_min_interval_cycles; /* Converted from min_interval_us */
	uint32_t profile_sequence_ext_last_capture_cycles; /* Last accepted capture cycle */
	uint32_t profile_sequence_ext_trigger_count; /* Accepted external triggers */
	uint32_t profile_sequence_ext_reject_count;  /* Rejected external edges (filter/status) */
	float32_t profile_sequence_move_duration_s; /* Quintic segment duration */
	float32_t profile_sequence_end_velocity_rad_s; /* Segment end velocity */
	float32_t profile_sequence_points_rad[MOTOR_PROFILE_SEQUENCE_MAX_POINTS]; /* Absolute target points [0, 2pi) */

	/* Chopper edge calibration (photo-interrupter + encoder angle snapshots) */
	bool chopper_cal_active;            /* Edge capture in progress */
	bool chopper_cal_complete;          /* Capture complete and midpoint table valid */
	bool chopper_cal_valid;             /* Midpoint table can be used */
	uint16_t chopper_cal_slots;         /* Number of blades/slots being calibrated */
	uint16_t chopper_cal_revs_target;   /* Number of revolutions to average */
	uint16_t chopper_cal_samples_per_edge; /* Expected samples per edge bin */
	uint16_t chopper_cal_midpoint_count; /* Number of valid midpoint entries */
	uint32_t chopper_cal_total_edges_target;   /* 2 * slots * revs */
	uint32_t chopper_cal_total_edges_captured; /* Number of accepted edges */
	uint32_t chopper_cal_discarded_edges;      /* Edges rejected by sanity checks */
	uint32_t chopper_cal_saved_timeout_ms;     /* Timeout restored after calibration */
	float32_t chopper_cal_edge_min_step_rad;   /* Reject edges too close together */
	float32_t chopper_cal_speed_target_rad_s;  /* Open-loop speed command used for calibration */
	float32_t chopper_cal_last_wrapped_rad;    /* Last wrapped encoder angle sample */
	float32_t chopper_cal_last_unwrapped_rad;  /* Last unwrapped encoder angle sample */
	float32_t chopper_cal_start_unwrapped_rad; /* Unwrapped angle when capture started */
	float32_t chopper_cal_edge_sum_rad[CHOPPER_CAL_MAX_EDGES];      /* Unwrapped angle sum per edge bin */
	uint32_t chopper_cal_edge_count[CHOPPER_CAL_MAX_EDGES];         /* Sample count per edge bin */
	float32_t chopper_blade_midpoints_rad[CHOPPER_CAL_MAX_SLOTS];   /* Final midpoint table [0, 2pi) */

	/* Cascaded control scaffolding (velocity/position/motion profile) */
	float32_t velocity_cl_kp_A_per_rad_s;   /* Velocity P gain: speed error -> Iq reference */
	float32_t velocity_cl_ki_A_per_rad;     /* Velocity I gain: speed error integral -> Iq reference */
	float32_t velocity_cl_iq_limit_A;       /* Closed-loop velocity Iq limit */
	float32_t velocity_cl_i_term_A;         /* Velocity PI integrator state */
	float32_t position_cl_kp_rad_s_per_rad; /* Position P gain: position error -> velocity target */
	float32_t position_cl_ki_rad_s2_per_rad;/* Position I gain: position error integral -> velocity */
	float32_t position_cl_i_term_rad_s;     /* Position PI integrator state */
	float32_t profile_max_velocity_rad_s;   /* Motion profile velocity limit */
	float32_t profile_max_accel_rad_s2;     /* Motion profile acceleration limit */

	/* Measured parameters (from calibration) */
	float32_t R_over_L_measured;
	float32_t Ls_measured_H;
	float32_t Rs_measured_ohm;
	float32_t flux_linkage_wb_active;             /* Active psi_f used by FOC */
	float32_t inertia_kgm2_active;                /* Active inertia estimate */
	float32_t viscous_friction_nm_per_rad_s_active; /* Active viscous friction */
	float32_t coulomb_friction_nm_active;         /* Active Coulomb friction */

	/* R/L estimation accumulators and angle generator */
	float32_t roverl_accumulator_Vd_Id;
	float32_t roverl_accumulator_Vq_Id;
	float32_t roverl_accumulator_Id2;

	/* Generic angle generator (used by calibration and open-loop velocity states) */
	angle_gen_t angle_gen;

	/* Telemetry and diagnostics */
	uint32_t state_counter;
	uint32_t encoder_fault_counter;
	uint32_t encoder_warning_count;
	uint32_t encoder_error_count;
	uint32_t control_loop_count;
	uint32_t max_isr_cycles;
	uint32_t total_isr_cycles;
	uint32_t overrun_count;

	/* Error tracking */
	uint32_t last_error_code;  /* Last error that caused ERROR state entry */

	/* Calibration status */
	bool calibration_complete;  /* True if calibration has been run successfully */
	bool calibration_running;   /* True while calibration/commissioning state machine is active */
	bool commissioning_complete; /* True if commissioning sequence has completed at least once */
	uint8_t calibration_mode;   /* MOTOR_CALIBRATION_MODE_* for active sequence */
	struct motor_commission_ctx commission; /* Commissioning runtime and capture buffers */

	/* ISR feature flags (atomic for thread-safe access) */
	atomic_t feature_flags;
	atomic_val_t feature_flags_next;  /* Pending flags to apply after state entry completes (state machine thread only) */

	/* RLS parameter estimation */
	struct prbs_gen prbs_gen;        /* PRBS generator state */
	struct rls_motor_est rls_d;      /* D-axis RLS estimator */
	struct rls_motor_est rls_q;      /* Q-axis RLS estimator */
	uint32_t rls_decimation;         /* RLS update rate decimation (power-of-2) */
	uint32_t rls_stagger_offset;     /* Q-axis RLS stagger offset for load spreading */
	float32_t rls_excitation_current_A; /* PRBS d-axis current excitation amplitude */
	float32_t Ld_est;                /* D-axis inductance estimate for cross-coupling */
	float32_t Lq_est;                /* Q-axis inductance estimate for cross-coupling */
	float32_t Id_rls_prev;           /* Previous accepted RLS Id sample for dI/dt */
	float32_t Iq_rls_prev;           /* Previous accepted RLS Iq sample for dI/dt */
	uint32_t rls_d_prev_cycle;       /* Control-loop count of previous accepted d-axis sample */
	uint32_t rls_q_prev_cycle;       /* Control-loop count of previous accepted q-axis sample */
	uint8_t rls_d_prev_valid;        /* Previous d-axis sample initialized */
	uint8_t rls_q_prev_valid;        /* Previous q-axis sample initialized */

	/* Thermal model */
	struct thermal_model thermal;    /* Thermal model state */
	uint32_t thermal_decimation;     /* Thermal update rate decimation (power-of-2) */
	float32_t Rs_ref_ohm;            /* Reference Rs from calibration (at ref temp) */
	float32_t Rs_ref_temp_C;         /* Reference temperature for Rs measurement (°C) */
	float32_t Rs_temp_coeff;         /* Rs temperature coefficient (1/°C) */
	float32_t T_rls_C;               /* Temperature from RLS Rs estimate (°C) */

	/* RLS gating conditions */
	float32_t rls_min_current_A;    /* Minimum current for observability */
	float32_t rls_min_speed_rad_s;  /* Minimum electrical speed for back-EMF observability */
	float32_t rls_max_residual;     /* Maximum residual before disabling RLS */
	float32_t rls_max_voltage_V;    /* Maximum voltage magnitude for validity check */

	/* Live telemetry snapshot (updated in ISR) */
	float32_t position_rad;
	float32_t velocity_rad_s;
	float32_t velocity_filtered_rad_s;
	float32_t encoder_raw_deg;
	float32_t encoder_raw_rad;
	float32_t encoder_observer_input_rad;
	float32_t velocity_target_rad_s; /* Velocity target before profile limiting */
	float32_t velocity_ref_rad_s;    /* Velocity reference after profile limiting */
	float32_t Id_ref_A;
	float32_t Iq_ref_A;	
	float32_t Id_A;
	float32_t Iq_A;
	float32_t Ia_A;
	float32_t Ib_A;
	float32_t Va_V;
	float32_t Vb_V;
	float32_t elec_angle_rad;
	float32_t dc_bus_voltage_V;
	uint8_t encoder_last_status;
	uint8_t encoder_sample_fresh;
	uint8_t encoder_sample_warning;
	uint8_t encoder_sample_error;
	uint8_t encoder_input_source;
};

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
#define ALIGN_STABILIZE_MS 20U
#define ALIGN_INJECT_MS (DT_PROP(USER_PARAMS_NODE, align_duration_ms) - ALIGN_STABILIZE_MS)
#define ALIGN_INJECT_DURATION_S ((float32_t)ALIGN_INJECT_MS / 1000.0f)
#define ALIGN_STABILIZE_DURATION_S ((float32_t)ALIGN_STABILIZE_MS / 1000.0f)
#define BRAKE_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, brake_current_ma) / 1000.0f)
#define MAX_VS_MPU ((float32_t)DT_PROP(USER_PARAMS_NODE, max_modulation_index_mpu) / 1000.0f)
#define ROVERL_EST_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_current_ma) / 1000.0f)
#define ROVERL_EST_FREQ_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_freq_hz))
#define ROVERL_EST_SETTLING_S ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_settling_ms) / 1000.0f)
#define ROVERL_EST_DURATION_S ((float32_t)DT_PROP(USER_PARAMS_NODE, roverl_est_duration_ms) / 1000.0f)
#define RS_EST_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_current_ma) / 1000.0f)
#define RS_EST_RAMPUP_S ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_rampup_ms) / 1000.0f)
#define RS_EST_DURATION_S ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_est_duration_ms) / 1000.0f)
#define RS_EST_FILTER_BW_HZ 5.0f      /* Heavy filtering for accurate measurement */

/* RLS and Thermal parameters - all values from devicetree (motor/system specific) */
#define RLS_DECIMATION DT_PROP(USER_PARAMS_NODE, rls_decimation)
#if DT_NODE_HAS_PROP(USER_PARAMS_NODE, rls_excitation_current_ma)
#define RLS_EXCITATION_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_excitation_current_ma) / 1000.0f)
#elif DT_NODE_HAS_PROP(USER_PARAMS_NODE, prbs_amplitude_millivolts)
/* Backward compatibility: legacy property name interpreted as current in mA. */
#define RLS_EXCITATION_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, prbs_amplitude_millivolts) / 1000.0f)
#else
#define RLS_EXCITATION_CURRENT_A ROVERL_EST_CURRENT_A
#endif
#define RLS_LAMBDA ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_lambda_mppu) / 10000.0f)
#define RLS_CONVERGENCE_THRESHOLD ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_convergence_threshold_mpu) / 1000.0f)
#define RLS_INITIAL_COVARIANCE ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_initial_covariance_mpu) / 1000.0f)
#define RLS_INITIAL_LQ_H ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_initial_lq_millihenries) / 1000000.0f)
#define RLS_STAGGER_OFFSET DT_PROP(USER_PARAMS_NODE, rls_stagger_offset)
#define THERMAL_DECIMATION DT_PROP(USER_PARAMS_NODE, thermal_decimation)
#define THERMAL_R_TH ((float32_t)DT_PROP(USER_PARAMS_NODE, thermal_resistance_c_per_w_milli) / 1000.0f)
#define THERMAL_C_TH ((float32_t)DT_PROP(USER_PARAMS_NODE, thermal_capacitance_j_per_c))
#define THERMAL_T_AMBIENT ((float32_t)DT_PROP(USER_PARAMS_NODE, thermal_ambient_temp_c))
#define RS_TEMP_COEFF ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_temp_coeff_ppm_per_c) / 1000000.0f)
#define RS_REF_TEMP_C ((float32_t)DT_PROP(USER_PARAMS_NODE, rs_ref_temp_c))
#define RLS_MIN_CURRENT_A ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_min_current_ma) / 1000.0f)
#define RLS_MIN_SPEED_RAD_S ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_min_speed_hz) * 2.0f * PI_F32)
#define RLS_MAX_RESIDUAL ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_max_residual_volts))
#define RLS_MAX_VOLTAGE_V ((float32_t)DT_PROP(USER_PARAMS_NODE, rls_max_voltage_volts))

#define VOLTAGE_SENSE_NODE DT_PATH(voltage_sense)
#define VBUS_CHANNEL DT_PROP(VOLTAGE_SENSE_NODE, channel)
#define VBUS_ADC_BUFFER_INDEX DT_PROP(VOLTAGE_SENSE_NODE, adc_buffer_index)
#define VBUS_VREF_V ((float32_t)DT_PROP(VOLTAGE_SENSE_NODE, vref_mv) / 1000.0f)
#define VBUS_OUTPUT_OHMS ((float32_t)DT_PROP(VOLTAGE_SENSE_NODE, output_ohms))
#define VBUS_FULL_OHMS ((float32_t)DT_PROP(VOLTAGE_SENSE_NODE, full_ohms))
#define VBUS_FULL_SCALE_V (VBUS_VREF_V * (VBUS_FULL_OHMS / VBUS_OUTPUT_OHMS))

/* Braking voltage limits */
#define VBUS_MAX_V (VBUS_FULL_SCALE_V * 0.95f)  /* 95% of full scale for safety margin */
#define VBUS_REGEN_LIMIT_V (NOMINAL_VOLTAGE_V * 1.1f)  /* Start blending to short-circuit at 110% nominal */
#define VBUS_VOLTAGE_MARGIN_INV (1.0f / (VBUS_MAX_V - VBUS_REGEN_LIMIT_V))  /* Inverse for fast computation */


#define CURRENT_SENSE_NODE DT_PATH(current_sense)
#define CURRENT_SENSE_VREF_V ((float32_t)DT_PROP(CURRENT_SENSE_NODE, vref_mv) / 1000.0f)
#define CURRENT_SENSE_RESISTOR_OHM ((float32_t)DT_PROP(CURRENT_SENSE_NODE, current_sense_resistor_uohms) / 1000000.0f)
#define CURRENT_SENSE_GAIN ((float32_t)DT_PROP(CURRENT_SENSE_NODE, current_sense_gain))
#define CURRENT_SENSE_FULL_SCALE_A (CURRENT_SENSE_VREF_V / (2.0f * CURRENT_SENSE_RESISTOR_OHM * CURRENT_SENSE_GAIN))

/* Current sense channels are stored as separate arrays after devicetree preprocessing */
#define CURRENT_SENSE_CHANNEL_0  DT_PROP_BY_IDX(CURRENT_SENSE_NODE, channels, 0)
#define CURRENT_SENSE_POLARITY_0 DT_PROP_BY_IDX(CURRENT_SENSE_NODE, channels, 1)
#define CURRENT_SENSE_CHANNEL_1  DT_PROP_BY_IDX(CURRENT_SENSE_NODE, channels, 2)
#define CURRENT_SENSE_POLARITY_1 DT_PROP_BY_IDX(CURRENT_SENSE_NODE, channels, 3)
#define CURRENT_SENSE_ADC_BUFFER_INDEX_0 DT_PROP_BY_IDX(CURRENT_SENSE_NODE, adc_buffer_indices, 0)
#define CURRENT_SENSE_ADC_BUFFER_INDEX_1 DT_PROP_BY_IDX(CURRENT_SENSE_NODE, adc_buffer_indices, 1)

#define MOTOR_PARAMS_NODE DT_PATH(motor_parameters)
#define MOTOR_INDUCTANCE_D_H ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, inductance_d_uh) / 1000000.0f)
#define MOTOR_INDUCTANCE_Q_H ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, inductance_q_uh) / 1000000.0f)
#define MOTOR_RESISTANCE_OHM ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, resistance_mohms) / 1000.0f)
#define MOTOR_FLUX_LINKAGE_VPH_ELEC ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, flux_linkage_uvphz) / 1000000.0f)
#define MOTOR_POLE_PAIRS DT_PROP(MOTOR_PARAMS_NODE, pole_pairs)
#define MOTOR_FLUX_LINKAGE_WB (MOTOR_FLUX_LINKAGE_VPH_ELEC / (2.0f * PI_F32))
#define MOTOR_MAX_CURRENT_A ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, max_current_ma) / 1000.0f)
#define MOTOR_INERTIA_KGM2 ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, inertia_mgcm2) / 10000000.0f)
#define MOTOR_MAX_SPEED_HZ ((float32_t)DT_PROP(MOTOR_PARAMS_NODE, max_speed_hz))

#define ANGLE_OBSERVER_NODE DT_PATH(angle_observer)
#define ANGLE_OBSERVER_BANDWIDTH_HZ ((float32_t)DT_PROP(ANGLE_OBSERVER_NODE, bandwidth_hz))

#define FAULT_DETECT_NODE DT_PATH(fault_detection)
#define ENCODER_FAULT_THRESHOLD DT_PROP(FAULT_DETECT_NODE, encoder_fault_threshold)
#define OVERCURRENT_THRESHOLD_A ((float32_t)DT_PROP(FAULT_DETECT_NODE, overcurrent_threshold_ma) / 1000.0f)

/* Velocity control parameters */
#define VELOCITY_MAX_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, velocity_max_hz))
#define VELOCITY_MAX_ACCEL_HZ_S ((float32_t)DT_PROP(USER_PARAMS_NODE, velocity_max_accel_hz_per_s))
#define VELOCITY_MAX_RAD_S (VELOCITY_MAX_HZ * 2.0f * PI_F32)
#define VELOCITY_MAX_ACCEL_RAD_S2 (VELOCITY_MAX_ACCEL_HZ_S * 2.0f * PI_F32)
#define VELOCITY_INITIAL_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, velocity_initial_hz))
#define COMMAND_TIMEOUT_DEFAULT_MS 1000U
#define CURRENT_DECOUPLING_ENABLED IS_ENABLED(CONFIG_MOTOR_CURRENT_DECOUPLING)
#define VELOCITY_NOTCH_FILTER_ENABLED IS_ENABLED(CONFIG_MOTOR_VELOCITY_NOTCH_FILTER)
#if defined(CONFIG_MOTOR_VELOCITY_NOTCH_FREQ_HZ)
#define VELOCITY_NOTCH_FREQ_HZ_CFG ((float32_t)CONFIG_MOTOR_VELOCITY_NOTCH_FREQ_HZ)
#else
#define VELOCITY_NOTCH_FREQ_HZ_CFG 300.0f
#endif
#if defined(CONFIG_MOTOR_VELOCITY_NOTCH_Q_MILLI)
#define VELOCITY_NOTCH_Q_CFG ((float32_t)CONFIG_MOTOR_VELOCITY_NOTCH_Q_MILLI / 1000.0f)
#else
#define VELOCITY_NOTCH_Q_CFG 2.0f
#endif
#if DT_NODE_HAS_PROP(USER_PARAMS_NODE, velocity_notch_freq_hz)
#define VELOCITY_NOTCH_FREQ_HZ ((float32_t)DT_PROP(USER_PARAMS_NODE, velocity_notch_freq_hz))
#else
#define VELOCITY_NOTCH_FREQ_HZ VELOCITY_NOTCH_FREQ_HZ_CFG
#endif
#if DT_NODE_HAS_PROP(USER_PARAMS_NODE, velocity_notch_q_milli)
#define VELOCITY_NOTCH_Q ((float32_t)DT_PROP(USER_PARAMS_NODE, velocity_notch_q_milli) / 1000.0f)
#else
#define VELOCITY_NOTCH_Q VELOCITY_NOTCH_Q_CFG
#endif

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
