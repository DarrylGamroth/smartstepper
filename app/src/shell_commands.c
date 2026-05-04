/*
 * Copyright (c) 2025 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "shell_commands.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "motor_hardware.h"
#include "config.h"
#include "motor_torque.h"
#include "motor/math/angle_wrap.h"
#include "shell_commands_motion.h"
#include "shell_commands_commission.h"
#include "shell_commands_state.h"
#include "shell_parse.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(shell_commands, CONFIG_APP_LOG_LEVEL);

/* Global motor parameters pointer (set by main) */
struct motor_parameters *g_motor_params = NULL;

/**
 * @brief Set global motor parameters pointer
 * Called from main() to provide access to motor_parameters structure
 */
void shell_set_motor_params(struct motor_parameters *params)
{
	g_motor_params = params;
}

/**
 * @brief Get global motor parameters pointer
 * @return Pointer to motor parameters, or NULL if not set
 */
struct motor_parameters *shell_get_motor_params(void)
{
	return g_motor_params;
}

bool motor_control_is_armed(const struct motor_parameters *params)
{
	return params && (atomic_get(&params->control_armed) != 0);
}

void motor_command_feed_watchdog(struct motor_parameters *params)
{
	if (!params) {
		return;
	}

	params->last_command_update_ms = k_uptime_get_32();
	params->last_command_update_loop = params->control_loop_count;
	params->command_timeout_latched = false;
}

enum motor_gains_profile {
	MOTOR_GAINS_PROFILE_NOMINAL = 0,
	MOTOR_GAINS_PROFILE_SAFE,
};

#define OUTER_LOOP_ZETA_DEFAULT 1.0f
#define OUTER_LOOP_ZETA_MIN 0.2f
#define OUTER_LOOP_ZETA_MAX 2.0f
#define POSITION_TO_VELOCITY_BW_RATIO_MAX 0.2f
#define VELOCITY_STATUS_TRACK_TOL_HZ 0.2f

static int motor_parse_gains_profile(const char *token, enum motor_gains_profile *profile)
{
	if (token == NULL || profile == NULL) {
		return -EINVAL;
	}

	if (strcmp(token, "nominal") == 0) {
		*profile = MOTOR_GAINS_PROFILE_NOMINAL;
		return 0;
	}
	if (strcmp(token, "safe") == 0) {
		*profile = MOTOR_GAINS_PROFILE_SAFE;
		return 0;
	}

	return -EINVAL;
}

static void motor_compute_nominal_outer_gains(const struct motor_parameters *params,
					       float *vel_kp, float *vel_ki,
					       float *vel_iq_limit,
					       float *pos_kp, float *pos_ki)
{
	float max_vel = MAX(params->profile_max_velocity_rad_s, 1.0f);

	*vel_kp = MOTOR_MAX_CURRENT_A / max_vel;
	*vel_ki = 2.0f * (*vel_kp);
	*vel_iq_limit = MOTOR_MAX_CURRENT_A;
	*pos_kp = max_vel / PI_F32;
	*pos_ki = 0.5f * (*pos_kp);
}

static void motor_compute_safe_outer_gains(const struct motor_parameters *params,
					    float *vel_kp, float *vel_ki,
					    float *vel_iq_limit,
					    float *pos_kp, float *pos_ki)
{
	motor_compute_nominal_outer_gains(params, vel_kp, vel_ki, vel_iq_limit, pos_kp, pos_ki);

	*vel_kp *= 0.5f;
	*vel_ki = *vel_kp;
	*vel_iq_limit *= 0.25f;
	*pos_kp *= 0.5f;
	*pos_ki *= 0.1f;
}

static int motor_apply_velocity_gains(float kp, float ki, float iq_limit)
{
	int ret = motor_api_set_param("velocity_cl_kp_A_per_rad_s", kp);
	if (ret != 0) {
		return ret;
	}
	ret = motor_api_set_param("velocity_cl_ki_A_per_rad", ki);
	if (ret != 0) {
		return ret;
	}
	ret = motor_api_set_param("velocity_cl_iq_limit_A", iq_limit);
	if (ret != 0) {
		return ret;
	}

	if (g_motor_params) {
		g_motor_params->velocity_cl_i_term_A = 0.0f;
	}

	return 0;
}

static int motor_apply_position_gains(float kp, float ki)
{
	int ret = motor_api_set_param("position_cl_kp_rad_s_per_rad", kp);
	if (ret != 0) {
		return ret;
	}
	ret = motor_api_set_param("position_cl_ki_rad_s2_per_rad", ki);
	if (ret != 0) {
		return ret;
	}

	if (g_motor_params) {
		g_motor_params->position_cl_i_term_rad_s = 0.0f;
	}

	return 0;
}

static int motor_compute_velocity_dob_defaults(const struct motor_parameters *params,
					       enum motor_gains_profile profile,
					       float *gain_out,
					       float *torque_limit_out,
					       float *iq_ff_limit_out,
					       float *kt_out)
{
	if (params == NULL || gain_out == NULL || torque_limit_out == NULL ||
	    iq_ff_limit_out == NULL || kt_out == NULL) {
		return -EINVAL;
	}

	float iq_limit = params->velocity_cl_iq_limit_A;
	float kt = motor_torque_gain_resolve_active(params);

	if (!isfinite(iq_limit) || iq_limit <= 0.0f || !isfinite(kt) || kt <= 0.0f) {
		return -ERANGE;
	}

	float gain = (profile == MOTOR_GAINS_PROFILE_SAFE) ? 0.01f : 0.02f;
	float iq_ff_limit = (profile == MOTOR_GAINS_PROFILE_SAFE) ?
				    (0.25f * iq_limit) :
				    (0.50f * iq_limit);
	iq_ff_limit = clampf(iq_ff_limit, 0.05f, iq_limit);

	*gain_out = gain;
	*iq_ff_limit_out = iq_ff_limit;
	*torque_limit_out = kt * iq_ff_limit;
	*kt_out = kt;
	return 0;
}

static int motor_compute_velocity_bandwidth_gains(const struct motor_parameters *params,
						  float bw_hz, float zeta,
						  float *kp_out, float *ki_out,
						  float *kt_out)
{
	if (params == NULL || kp_out == NULL || ki_out == NULL || kt_out == NULL) {
		return -EINVAL;
	}
	if (!isfinite(bw_hz) || !isfinite(zeta) || bw_hz <= 0.0f ||
	    zeta < OUTER_LOOP_ZETA_MIN || zeta > OUTER_LOOP_ZETA_MAX) {
		return -EINVAL;
	}

	float j = params->inertia_kgm2_active;
	float b = params->viscous_friction_nm_per_rad_s_active;
	float kt = motor_torque_gain_resolve_active(params);
	float omega = 2.0f * PI_F32 * bw_hz;

	if (!isfinite(j) || j <= 0.0f || !isfinite(kt) || kt <= 0.0f) {
		return -ERANGE;
	}
	if (!isfinite(b) || b < 0.0f) {
		b = 0.0f;
	}

	float kp = ((2.0f * zeta * omega * j) - b) / kt;
	float ki = (omega * omega * j) / kt;
	if (!isfinite(kp) || !isfinite(ki) || kp <= 0.0f || ki <= 0.0f) {
		return -ERANGE;
	}

	*kp_out = kp;
	*ki_out = ki;
	*kt_out = kt;
	return 0;
}

static int motor_estimate_velocity_bandwidth_hz(const struct motor_parameters *params,
						float *bw_hz_out)
{
	if (params == NULL || bw_hz_out == NULL) {
		return -EINVAL;
	}

	float j = params->inertia_kgm2_active;
	float ki = params->velocity_cl_ki_A_per_rad;
	float kt = motor_torque_gain_resolve_active(params);
	if (!isfinite(j) || j <= 0.0f || !isfinite(kt) || kt <= 0.0f ||
	    !isfinite(ki) || ki <= 0.0f) {
		return -ERANGE;
	}

	float omega = sqrtf((kt * ki) / j);
	if (!isfinite(omega) || omega <= 0.0f) {
		return -ERANGE;
	}

	*bw_hz_out = omega / (2.0f * PI_F32);
	return 0;
}

/*============================================================================
 * Shell Command Implementations
 *============================================================================*/

/* motor params get <name> */
static int cmd_motor_params_get(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor params get <name>");
		return -EINVAL;
	}

	const char *name = argv[1];
	float value;
	
	if (motor_api_get_param(name, &value) == 0) {
		shell_print(sh, "%s = %.6f", name, (double)value);
		return 0;
	} else {
		shell_error(sh, "Parameter '%s' not found", name);
		return -ENOENT;
	}
}

/* motor params set <name> <value> */
static int cmd_motor_params_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor params set <name> <value>");
		return -EINVAL;
	}

	const char *name = argv[1];
	float value = 0.0f;
	if (!shell_parse_finite_float(argv[2], &value)) {
		shell_error(sh, "value must be a finite number");
		return -EINVAL;
	}
	
	if (motor_api_set_param(name, value) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Set %s = %.6f", name, (double)value);
		return 0;
	} else {
		shell_error(sh, "Parameter '%s' not found or read-only", name);
		return -ENOENT;
	}
}

/* motor params list */
static int cmd_motor_params_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	
	shell_print(sh, "Available motor parameters:");
	shell_print(sh, "%-25s %s", "Name", "Value");
	shell_print(sh, "%-25s %s", "----", "-----");
	
	size_t count = motor_api_get_param_count();
	for (size_t i = 0; i < count; i++) {
		const char *name = motor_api_get_param_name(i);
		float value;
		if (motor_api_get_param_by_index(i, &value) == 0) {
			shell_print(sh, "%-25s %.6f", name, (double)value);
		}
	}
	
	return 0;
}

/* motor current id <value> */
static int cmd_motor_current_id(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current id <amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float id_amps = 0.0f;
	if (!shell_parse_finite_float(argv[1], &id_amps)) {
		shell_error(sh, "id current must be a finite number");
		return -EINVAL;
	}
	if (fabsf(id_amps) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero current commands.");
		return -EACCES;
	}
	
	if (motor_api_set_param("Id_setpoint_A", id_amps) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Id setpoint = %.3f A", (double)id_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set Id current");
		return -EIO;
	}
}

/* motor current iq <value> */
static int cmd_motor_current_iq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current iq <amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float iq_amps = 0.0f;
	if (!shell_parse_finite_float(argv[1], &iq_amps)) {
		shell_error(sh, "iq current must be a finite number");
		return -EINVAL;
	}
	if (fabsf(iq_amps) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero current commands.");
		return -EACCES;
	}
	
	if (motor_api_set_param("Iq_setpoint_A", iq_amps) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Iq setpoint = %.3f A", (double)iq_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set Iq current");
		return -EIO;
	}
}

/* motor current dq <id> <iq> */
static int cmd_motor_current_dq(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor current dq <id_amps> <iq_amps>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	float id_amps = 0.0f;
	float iq_amps = 0.0f;
	if (!shell_parse_finite_float(argv[1], &id_amps) ||
	    !shell_parse_finite_float(argv[2], &iq_amps)) {
		shell_error(sh, "id/iq currents must be finite numbers");
		return -EINVAL;
	}
	if ((fabsf(id_amps) > 1e-6f || fabsf(iq_amps) > 1e-6f) &&
	    !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero current commands.");
		return -EACCES;
	}
	
	if (motor_api_set_currents(id_amps, iq_amps) == 0) {
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Id = %.3f A, Iq = %.3f A", (double)id_amps, (double)iq_amps);
		return 0;
	} else {
		shell_error(sh, "Failed to set DQ currents");
		return -EIO;
	}
}

/* motor current gain get <id|iq> */
static int cmd_motor_current_gain_get(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor current gain get <id|iq>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const char *controller = argv[1];
	float kp = 0.0f;
	float ki = 0.0f;

	if (strcmp(controller, "id") == 0) {
		kp = g_motor_params->pi_Id.kp;
		ki = g_motor_params->pi_Id.ki;
	} else if (strcmp(controller, "iq") == 0) {
		kp = g_motor_params->pi_Iq.kp;
		ki = g_motor_params->pi_Iq.ki;
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}

	shell_print(sh, "Current PI %s: Kp=%.6f, Ki=%.6f",
		    controller, (double)kp, (double)ki);
	return 0;
}

/* motor current gain set <id|iq> <kp> <ki> */
static int cmd_motor_current_gain_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 4) {
		shell_error(sh, "Usage: motor current gain set <id|iq> <kp> <ki>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing current PI gains.");
		return -EACCES;
	}

	const char *controller = argv[1];
	float kp = 0.0f;
	float ki = 0.0f;
	if (!shell_parse_finite_float(argv[2], &kp) ||
	    !shell_parse_finite_float(argv[3], &ki)) {
		shell_error(sh, "kp/ki must be finite numbers");
		return -EINVAL;
	}
	if (kp <= 0.0f || ki < 0.0f) {
		shell_error(sh, "kp must be > 0 and ki must be >= 0");
		return -EINVAL;
	}

	if (strcmp(controller, "id") == 0) {
		pi_set_gains(&g_motor_params->pi_Id, kp, ki);
	} else if (strcmp(controller, "iq") == 0) {
		pi_set_gains(&g_motor_params->pi_Iq, kp, ki);
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Current PI %s gains set: Kp=%.6f, Ki=%.6f",
		    controller, (double)kp, (double)ki);
	return 0;
}

/* motor current gain bandwidth <id|iq> <hz> */
static int cmd_motor_current_gain_bandwidth(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: motor current gain bandwidth <id|iq> <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before tuning current PI bandwidth.");
		return -EACCES;
	}

	const char *controller = argv[1];
	float bw_hz = 0.0f;
	if (!shell_parse_finite_float(argv[2], &bw_hz)) {
		shell_error(sh, "Bandwidth must be a finite number.");
		return -EINVAL;
	}
	if (bw_hz <= 0.0f || bw_hz > CONTROL_LOOP_FREQUENCY_HZ / 2.0f) {
		shell_error(sh, "Bandwidth must be positive and below Nyquist (%.1f Hz)",
			    (double)(CONTROL_LOOP_FREQUENCY_HZ / 2.0f));
		return -EINVAL;
	}

	float32_t bw_rps = 2.0f * PI_F32 * bw_hz;
	float32_t t_sample = 1.0f / CONTROL_LOOP_FREQUENCY_HZ;
	float32_t kp = 0.0f;
	float32_t ki = 0.0f;

	if (strcmp(controller, "id") == 0) {
		if (MOTOR_INDUCTANCE_D_H <= 0.0f) {
			shell_error(sh, "Invalid D-axis inductance in configuration");
			return -ERANGE;
		}
		kp = MOTOR_INDUCTANCE_D_H * bw_rps;
		ki = (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_D_H) * t_sample;
		pi_set_gains(&g_motor_params->pi_Id, kp, ki);
	} else if (strcmp(controller, "iq") == 0) {
		if (MOTOR_INDUCTANCE_Q_H <= 0.0f) {
			shell_error(sh, "Invalid Q-axis inductance in configuration");
			return -ERANGE;
		}
		kp = MOTOR_INDUCTANCE_Q_H * bw_rps;
		ki = (MOTOR_RESISTANCE_OHM / MOTOR_INDUCTANCE_Q_H) * t_sample;
		pi_set_gains(&g_motor_params->pi_Iq, kp, ki);
	} else {
		shell_error(sh, "Controller must be 'id' or 'iq'");
		return -EINVAL;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Current PI %s tuned to %.1f Hz: Kp=%.6f, Ki=%.6f",
		    controller, (double)bw_hz, (double)kp, (double)ki);
	return 0;
}

/* motor velocity target <hz> */
static int cmd_motor_velocity_target(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor velocity target <hz>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct smf_state *mode = g_motor_params->state_for_isr;
	if (!motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_ENCODER)) {
		shell_error(sh,
			    "Velocity target requires velocity_generated or velocity_encoder mode.");
		return -EACCES;
	}

	float target_hz = 0.0f;
	if (!shell_parse_finite_float(argv[1], &target_hz)) {
		shell_error(sh, "target must be a finite number");
		return -EINVAL;
	}
	if (fabsf(target_hz) > 1e-6f && !motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Control is disarmed; run 'motor arm' before non-zero velocity commands.");
		return -EACCES;
	}

	/* Convert Hz to mechanical rad/s for trajectory */
	float target_rad_s = target_hz * 2.0f * PI_F32;

	/* Respect currently configured profile velocity limits. */
	float32_t target_clamped = clampf(target_rad_s,
					  -g_motor_params->profile_max_velocity_rad_s,
					  g_motor_params->profile_max_velocity_rad_s);

	/* Set trajectory target (thread-safe access) */
	traj_set_target_value(&g_motor_params->traj_velocity, target_clamped);
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Velocity target set to %.2f Hz", (double)(target_clamped / (2.0f * PI_F32)));
	return 0;
}

/* motor velocity decimation <ticks> */
static int cmd_motor_velocity_decimation(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor velocity decimation <ticks>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing velocity decimation.");
		return -EACCES;
	}

	uint32_t decimation = 0U;
	if (!shell_parse_u32(argv[1], &decimation)) {
		shell_error(sh, "ticks must be an integer");
		return -EINVAL;
	}
	if (decimation < OUTER_LOOP_DECIMATION_MIN || decimation > OUTER_LOOP_DECIMATION_MAX) {
		shell_error(sh, "ticks must be in [%u, %u]",
			    OUTER_LOOP_DECIMATION_MIN, OUTER_LOOP_DECIMATION_MAX);
		return -EINVAL;
	}

	int ret = motor_api_set_param("velocity_loop_decimation", (float)decimation);
	if (ret != 0) {
		shell_error(sh, "Failed to set velocity decimation (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Velocity decimation set to %u tick(s): %.3f ms, %.1f Hz update",
		    decimation,
		    (double)(1000.0f * (float32_t)decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)decimation));
	return 0;
}

/* motor velocity status */
static int cmd_motor_velocity_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	const struct smf_state *mode = g_motor_params->state_for_isr;
	if (!motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_GENERATED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_POSITION_GENERATED) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) &&
	    !motor_state_ptr_is_mode(mode, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		shell_print(sh, "Velocity controller: INACTIVE");
		return 0;
	}

	/* Get values from trajectory/controller (in rad/s) and convert to Hz for display */
	float target_rad_s = traj_get_target_value(&g_motor_params->traj_velocity);
	float ref_rad_s = g_motor_params->live.velocity_ref_rad_s;
	float meas_rad_s = g_motor_params->live.velocity_rad_s;
	float target_hz = target_rad_s / (2.0f * PI_F32);
	float ref_hz = ref_rad_s / (2.0f * PI_F32);
	float meas_hz = meas_rad_s / (2.0f * PI_F32);
	float error_hz = ref_hz - meas_hz;
	bool traj_at_target = traj_is_at_target(&g_motor_params->traj_velocity);
	uint8_t quality_flags = g_motor_params->live.position_quality_flags;
	bool feedback_valid =
		((quality_flags & MOTOR_FEEDBACK_QUALITY_VALID) != 0U) &&
		((quality_flags & MOTOR_FEEDBACK_QUALITY_ERROR) == 0U);
	bool speed_tracking_ok = fabsf(error_hz) <= VELOCITY_STATUS_TRACK_TOL_HZ;
	bool at_target = traj_at_target && feedback_valid && speed_tracking_ok;
	uint32_t velocity_decimation =
		MAX(OUTER_LOOP_DECIMATION_MIN, g_motor_params->velocity_loop_decimation);

	/* Determine motion state */
	const char *motion_str;
	if (!feedback_valid) {
		motion_str = "HOLD_QUALITY";
	} else if (fabsf(meas_rad_s) < 0.1f && fabsf(ref_rad_s) < 0.1f) {
		motion_str = "STOPPED";
	} else if (at_target) {
		motion_str = "AT_SPEED";
	} else if (fabsf(ref_rad_s) > fabsf(meas_rad_s)) {
		motion_str = "ACCELERATING";
	} else {
		motion_str = "DECELERATING";
	}

	shell_print(sh, "Velocity Controller Status:");
	shell_print(sh, "  Mode:       %s", motor_state_to_string(motor_api_get_state()));
	shell_print(sh, "  Target:     %.2f Hz", (double)target_hz);
	shell_print(sh, "  Ref:        %.2f Hz", (double)ref_hz);
	shell_print(sh, "  Measured:   %.2f Hz", (double)meas_hz);
	shell_print(sh, "  Error:      %.2f Hz", (double)error_hz);
	shell_print(sh, "  Loop dt:    %.3f ms (%u tick, %.1f Hz)",
		    (double)(1000.0f * (float32_t)velocity_decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    velocity_decimation,
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)velocity_decimation));
	shell_print(sh, "  At Target:  %s", at_target ? "YES" : "NO");
	shell_print(sh, "  Feedback:   %s (flags=0x%02X)",
		    feedback_valid ? "VALID" : "DEGRADED",
		    quality_flags);
	shell_print(sh, "  Motion:     %s", motion_str);
	shell_print(sh, "  Outer loop: %s",
		    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ?
			    "MPR" :
			    "PI");
	shell_print(sh, "  Kt active:  %.6f Nm/A",
		    (double)motor_torque_gain_resolve_active(g_motor_params));
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_print(sh, "  MPR q/r:    %.4f / %.4f",
			    (double)g_motor_params->velocity_mpr_cfg.q_speed,
			    (double)g_motor_params->velocity_mpr_cfg.r_delta_iq);
		shell_print(sh, "  Horizon:    %u", g_motor_params->velocity_mpr_cfg.horizon);
		shell_print(sh, "  dIq max:    %.4f A/sample",
			    (double)g_motor_params->velocity_mpr_cfg.max_delta_iq_a);
		shell_print(sh, "  Dist KI:    %.4f Nm/(rad/s)",
			    (double)g_motor_params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s);
		shell_print(sh, "  Iq cmd:     %.5f A",
			    (double)g_motor_params->velocity_mpr_state.iq_cmd_a);
	} else {
		shell_print(sh, "  Kp:         %.5f A/(rad/s)",
			    (double)g_motor_params->velocity_cl_kp_A_per_rad_s);
		shell_print(sh, "  Ki:         %.5f A/rad",
			    (double)g_motor_params->velocity_cl_ki_A_per_rad);
		shell_print(sh, "  I term:     %.5f A",
			    (double)g_motor_params->velocity_cl_i_term_A);
	}
	shell_print(sh, "  DOB:        %s",
		    g_motor_params->velocity_dob_cfg.enabled ? "ENABLED" : "DISABLED");
	if (g_motor_params->velocity_dob_cfg.enabled) {
		shell_print(sh, "  DOB gain:   %.5f Nm/(rad/s)",
			    (double)g_motor_params->velocity_dob_cfg.observer_gain_nm_per_rad_s);
		shell_print(sh, "  DOB lim:    %.5f Nm, %.5f A",
			    (double)g_motor_params->velocity_dob_cfg.torque_limit_nm,
			    (double)g_motor_params->velocity_dob_cfg.iq_ff_limit_a);
		shell_print(sh, "  DOB est:    %.5f Nm, iq_ff=%.5f A, res=%.5f rad/s",
			    (double)g_motor_params->live.velocity_dob_disturbance_nm,
			    (double)g_motor_params->live.velocity_dob_iq_ff_a,
			    (double)g_motor_params->live.velocity_dob_residual_rad_s);
	}
	shell_print(sh, "  Detent FF:  %s gain=%.3f limit=%.4f A live=%.5f A",
		    g_motor_params->detent_map_cfg.enabled ? "ENABLED" : "DISABLED",
		    (double)g_motor_params->detent_map_cfg.gain,
		    (double)g_motor_params->detent_map_cfg.iq_ff_limit_a,
		    (double)g_motor_params->live.detent_iq_ff_a);
	shell_print(sh, "  Iq limit:   %.3f A", (double)g_motor_params->velocity_cl_iq_limit_A);

	return 0;
}

/* motor velocity gains set <kp_a_per_rad_s> <ki_a_per_rad> <iq_limit_a>
 * motor velocity gains defaults <safe|nominal>
 * motor velocity gains bandwidth <hz> [zeta]
 */
static int cmd_motor_velocity_gains(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3 || argc > 5) {
		shell_error(sh, "Usage: motor velocity gains set <kp> <ki> <iq_limit> | "
			    "motor velocity gains defaults <safe|nominal> | "
			    "motor velocity gains bandwidth <hz> [zeta]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_warn(sh,
			   "outer_loop_mode=MPR; velocity PI gains are inactive unless outer_loop_mode is set to 0");
	}

	if (strcmp(argv[1], "defaults") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor velocity gains defaults <safe|nominal>");
			return -EINVAL;
		}

		enum motor_gains_profile profile;
		if (motor_parse_gains_profile(argv[2], &profile) != 0) {
			shell_error(sh, "Profile must be 'safe' or 'nominal'");
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		float iq_limit = 0.0f;
		float pos_kp_dummy = 0.0f;
		float pos_ki_dummy = 0.0f;
		if (profile == MOTOR_GAINS_PROFILE_SAFE) {
			motor_compute_safe_outer_gains(g_motor_params, &kp, &ki, &iq_limit,
						      &pos_kp_dummy, &pos_ki_dummy);
		} else {
			motor_compute_nominal_outer_gains(g_motor_params, &kp, &ki, &iq_limit,
							 &pos_kp_dummy, &pos_ki_dummy);
		}

		int ret = motor_apply_velocity_gains(kp, ki, iq_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to apply velocity defaults (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity %s defaults applied: Kp=%.5f A/(rad/s), Ki=%.5f A/rad, Iq limit=%.3f A",
			    (profile == MOTOR_GAINS_PROFILE_SAFE) ? "safe" : "nominal",
			    (double)kp, (double)ki, (double)iq_limit);
		return 0;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 5) {
			shell_error(sh, "Usage: motor velocity gains set <kp> <ki> <iq_limit>");
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		float iq_limit = 0.0f;
		if (!shell_parse_finite_float(argv[2], &kp) ||
		    !shell_parse_finite_float(argv[3], &ki) ||
		    !shell_parse_finite_float(argv[4], &iq_limit)) {
			shell_error(sh, "kp/ki/iq_limit must be finite numbers");
			return -EINVAL;
		}

		int ret = motor_apply_velocity_gains(kp, ki, iq_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to update velocity gains (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Velocity gains set: Kp=%.5f A/(rad/s), Ki=%.5f A/rad, Iq limit=%.3f A",
			    (double)kp, (double)ki, (double)iq_limit);
		return 0;
	}

	if (strcmp(argv[1], "bandwidth") == 0) {
		if (argc != 3 && argc != 4) {
			shell_error(sh, "Usage: motor velocity gains bandwidth <hz> [zeta]");
			return -EINVAL;
		}

		float bw_hz = 0.0f;
		float zeta = OUTER_LOOP_ZETA_DEFAULT;
		if (!shell_parse_finite_float(argv[2], &bw_hz) ||
		    (argc == 4 && !shell_parse_finite_float(argv[3], &zeta))) {
			shell_error(sh, "Bandwidth/zeta must be finite numbers");
			return -EINVAL;
		}
		if (bw_hz <= 0.0f || bw_hz > (CONTROL_LOOP_FREQUENCY_HZ * 0.25f)) {
			shell_error(sh, "Bandwidth must be in (0, %.1f] Hz",
				    (double)(CONTROL_LOOP_FREQUENCY_HZ * 0.25f));
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		float kt = 0.0f;
		int ret = motor_compute_velocity_bandwidth_gains(g_motor_params, bw_hz, zeta,
								 &kp, &ki, &kt);
		if (ret != 0) {
			if (ret == -ERANGE) {
				shell_error(sh,
					    "Need valid active commissioning params (J, torque_gain)");
			} else {
				shell_error(sh, "Invalid bandwidth/zeta; zeta range is %.1f..%.1f",
					    (double)OUTER_LOOP_ZETA_MIN, (double)OUTER_LOOP_ZETA_MAX);
			}
			return ret;
		}

		ret = motor_apply_velocity_gains(kp, ki, g_motor_params->velocity_cl_iq_limit_A);
		if (ret != 0) {
			shell_error(sh, "Failed to apply velocity bandwidth gains (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity bandwidth tuned: bw=%.2f Hz zeta=%.2f -> Kp=%.5f A/(rad/s), Ki=%.5f A/rad (Kt=%.6f Nm/A)",
			    (double)bw_hz, (double)zeta, (double)kp, (double)ki, (double)kt);
		return 0;
	}

	shell_error(sh, "Usage: motor velocity gains set <kp> <ki> <iq_limit> | "
		    "motor velocity gains defaults <safe|nominal> | "
		    "motor velocity gains bandwidth <hz> [zeta]");
	return -EINVAL;
}

/* motor velocity dob status
 * motor velocity dob defaults <safe|nominal>
 * motor velocity dob enable <0|1>
 * motor velocity dob gain <observer_gain_nm_per_rad_s>
 * motor velocity dob torque_limit <nm>
 * motor velocity dob iq_limit <a>
 */
static int cmd_motor_velocity_dob(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2 || argc > 3) {
		shell_error(sh, "Usage: motor velocity dob status | "
			    "motor velocity dob defaults <safe|nominal> | "
			    "motor velocity dob enable <0|1> | "
			    "motor velocity dob gain <nm_per_rad_s> | "
			    "motor velocity dob torque_limit <nm> | "
			    "motor velocity dob iq_limit <a>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (strcmp(argv[1], "status") == 0) {
		if (argc != 2) {
			shell_error(sh, "Usage: motor velocity dob status");
			return -EINVAL;
		}
		shell_print(sh, "Velocity DOB:");
		shell_print(sh, "  Enabled:     %s",
			    g_motor_params->velocity_dob_cfg.enabled ? "YES" : "NO");
		shell_print(sh, "  Gain:        %.6f Nm/(rad/s)",
			    (double)g_motor_params->velocity_dob_cfg.observer_gain_nm_per_rad_s);
		shell_print(sh, "  Torque limit %.6f Nm",
			    (double)g_motor_params->velocity_dob_cfg.torque_limit_nm);
		shell_print(sh, "  Iq FF limit: %.6f A",
			    (double)g_motor_params->velocity_dob_cfg.iq_ff_limit_a);
		shell_print(sh, "  State:       d=%.6f Nm, ff=%.6f A, res=%.6f rad/s",
			    (double)g_motor_params->live.velocity_dob_disturbance_nm,
			    (double)g_motor_params->live.velocity_dob_iq_ff_a,
			    (double)g_motor_params->live.velocity_dob_residual_rad_s);
		return 0;
	}

	if (strcmp(argv[1], "defaults") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor velocity dob defaults <safe|nominal>");
			return -EINVAL;
		}

		enum motor_gains_profile profile;
		if (motor_parse_gains_profile(argv[2], &profile) != 0) {
			shell_error(sh, "Profile must be 'safe' or 'nominal'");
			return -EINVAL;
		}

		float gain = 0.0f;
		float torque_limit = 0.0f;
		float iq_ff_limit = 0.0f;
		float kt = 0.0f;
		int ret = motor_compute_velocity_dob_defaults(g_motor_params, profile,
							      &gain, &torque_limit,
							      &iq_ff_limit, &kt);
		if (ret != 0) {
			shell_error(sh,
				    "Cannot compute DOB defaults; need valid active torque_gain and velocity current limit");
			return ret;
		}

		ret = motor_api_set_param("velocity_dob_enable", 1.0f);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_enable (err %d)", ret);
			return ret;
		}
		ret = motor_api_set_param("velocity_dob_observer_gain_nm_per_rad_s", gain);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_observer_gain_nm_per_rad_s (err %d)",
				    ret);
			return ret;
		}
		ret = motor_api_set_param("velocity_dob_torque_limit_nm", torque_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_torque_limit_nm (err %d)", ret);
			return ret;
		}
		ret = motor_api_set_param("velocity_dob_iq_ff_limit_a", iq_ff_limit);
		if (ret != 0) {
			shell_error(sh, "Failed to set velocity_dob_iq_ff_limit_a (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Velocity DOB %s defaults applied: enable=1 gain=%.6f Nm/(rad/s), torque_limit=%.6f Nm, iq_ff_limit=%.6f A (Kt=%.6f Nm/A)",
			    (profile == MOTOR_GAINS_PROFILE_SAFE) ? "safe" : "nominal",
			    (double)gain, (double)torque_limit, (double)iq_ff_limit, (double)kt);
		return 0;
	}

	if (argc != 3) {
		shell_error(sh, "Usage: motor velocity dob defaults <safe|nominal> | "
			    "motor velocity dob <enable|gain|torque_limit|iq_limit> <value>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "enable") == 0) {
		bool enabled = false;
		if (!shell_parse_bool01(argv[2], &enabled)) {
			shell_error(sh, "enable value must be 0 or 1");
			return -EINVAL;
		}
		int ret = motor_api_set_param("velocity_dob_enable", enabled ? 1.0f : 0.0f);
		if (ret != 0) {
			shell_error(sh, "Failed to update velocity_dob_enable (err %d)", ret);
			return ret;
		}
		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Velocity DOB %s", enabled ? "enabled" : "disabled");
		return 0;
	}

	float value = 0.0f;
	if (!shell_parse_finite_float(argv[2], &value)) {
		shell_error(sh, "value must be a finite number");
		return -EINVAL;
	}

	const char *param_name = NULL;
	const char *label = NULL;
	if (strcmp(argv[1], "gain") == 0) {
		if (value < 0.0f) {
			shell_error(sh, "gain must be >= 0");
			return -EINVAL;
		}
		param_name = "velocity_dob_observer_gain_nm_per_rad_s";
		label = "gain";
	} else if (strcmp(argv[1], "torque_limit") == 0) {
		if (value <= 0.0f) {
			shell_error(sh, "torque_limit must be > 0");
			return -EINVAL;
		}
		param_name = "velocity_dob_torque_limit_nm";
		label = "torque limit";
	} else if (strcmp(argv[1], "iq_limit") == 0) {
		if (value < 0.0f) {
			shell_error(sh, "iq_limit must be >= 0");
			return -EINVAL;
		}
		param_name = "velocity_dob_iq_ff_limit_a";
		label = "Iq FF limit";
	} else {
		shell_error(sh, "Unknown DOB field '%s' (use defaults|enable|gain|torque_limit|iq_limit)",
			    argv[1]);
		return -EINVAL;
	}

	int ret = motor_api_set_param(param_name, value);
	if (ret != 0) {
		shell_error(sh, "Failed to update %s (err %d)", param_name, ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Velocity DOB %s set to %.6f", label, (double)value);
	return 0;
}

/* motor position target <deg> */
static int cmd_motor_position_target(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor position target <deg>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		shell_error(sh,
			    "Not in position_encoder mode. Use 'motor state mode position_encoder' first.");
		return -EACCES;
	}

	float target_deg = 0.0f;
	if (!shell_parse_finite_float(argv[1], &target_deg)) {
		shell_error(sh, "target must be a finite number");
		return -EINVAL;
	}
	float target_rad = wrap_rad_2pi(target_deg * PI_F32 / 180.0f);
	g_motor_params->profile_seq.running = false;
	g_motor_params->profile_seq.tick_counter = 0U;
	motion_profile_quintic_cancel(&g_motor_params->position_profile, target_rad);
	g_motor_params->position_target_rad = target_rad;
	motor_command_feed_watchdog(g_motor_params);

	shell_print(sh, "Position target set to %.2f deg", (double)(target_rad * 180.0f / PI_F32));
	return 0;
}

/* motor position decimation <ticks> */
static int cmd_motor_position_decimation(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: motor position decimation <ticks>");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing position decimation.");
		return -EACCES;
	}

	uint32_t decimation = 0U;
	if (!shell_parse_u32(argv[1], &decimation)) {
		shell_error(sh, "ticks must be an integer");
		return -EINVAL;
	}
	if (decimation < OUTER_LOOP_DECIMATION_MIN || decimation > OUTER_LOOP_DECIMATION_MAX) {
		shell_error(sh, "ticks must be in [%u, %u]",
			    OUTER_LOOP_DECIMATION_MIN, OUTER_LOOP_DECIMATION_MAX);
		return -EINVAL;
	}

	int ret = motor_api_set_param("position_loop_decimation", (float)decimation);
	if (ret != 0) {
		shell_error(sh, "Failed to set position decimation (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Position decimation set to %u tick(s): %.3f ms, %.1f Hz update",
		    decimation,
		    (double)(1000.0f * (float32_t)decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)decimation));
	return 0;
}

/* motor position status */
static int cmd_motor_position_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (!motor_state_ptr_is_mode(g_motor_params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER)) {
		shell_print(sh, "Position controller: INACTIVE");
		return 0;
	}

	float target_rad = g_motor_params->position_target_rad;
	float meas_rad = g_motor_params->live.position_rad;
	float err_rad = wrap_rad_pi(target_rad - meas_rad);
	uint32_t position_decimation =
		MAX(OUTER_LOOP_DECIMATION_MIN, g_motor_params->position_loop_decimation);

	shell_print(sh, "Position Controller Status:");
	shell_print(sh, "  Target:     %.2f deg", (double)(target_rad * 180.0f / PI_F32));
	shell_print(sh, "  Measured:   %.2f deg", (double)(meas_rad * 180.0f / PI_F32));
	shell_print(sh, "  Error:      %.2f deg", (double)(err_rad * 180.0f / PI_F32));
	shell_print(sh, "  Loop dt:    %.3f ms (%u tick, %.1f Hz)",
		    (double)(1000.0f * (float32_t)position_decimation / CONTROL_LOOP_FREQUENCY_HZ),
		    position_decimation,
		    (double)(CONTROL_LOOP_FREQUENCY_HZ / (float32_t)position_decimation));
	shell_print(sh, "  Profile:    %s",
		    motion_profile_quintic_is_active(&g_motor_params->position_profile) ?
			    "ACTIVE" :
			    (g_motor_params->position_profile.valid ? "COMPLETE" : "OFF"));
	shell_print(sh, "  Outer loop: %s",
		    g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR ?
			    "MPR" :
			    "PI");
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_print(sh, "  MPR q_pos:  %.4f", (double)g_motor_params->position_mpr_cfg.q_position);
		shell_print(sh, "  MPR q_vel:  %.4f", (double)g_motor_params->position_mpr_cfg.q_velocity_ff);
		shell_print(sh, "  MPR r:      %.4f",
			    (double)g_motor_params->position_mpr_cfg.r_delta_velocity);
		shell_print(sh, "  Horizon:    %u", g_motor_params->position_mpr_cfg.horizon);
		shell_print(sh, "  dVel max:   %.4f rad/s/sample",
			    (double)g_motor_params->position_mpr_cfg.max_delta_velocity_rad_s);
		shell_print(sh, "  Vel cmd:    %.4f rad/s",
			    (double)g_motor_params->position_mpr_state.velocity_cmd_rad_s);
	} else {
		shell_print(sh, "  Kp:         %.5f (rad/s)/rad",
			    (double)g_motor_params->position_cl_kp_rad_s_per_rad);
		shell_print(sh, "  Ki:         %.5f (rad/s^2)/rad",
			    (double)g_motor_params->position_cl_ki_rad_s2_per_rad);
		shell_print(sh, "  I term:     %.5f rad/s",
			    (double)g_motor_params->position_cl_i_term_rad_s);
	}
	return 0;
}

/* motor position gains set <kp_rad_s_per_rad> <ki_rad_s2_per_rad>
 * motor position gains defaults <safe|nominal>
 * motor position gains bandwidth <hz> [zeta]
 */
static int cmd_motor_position_gains(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3 || argc > 4) {
		shell_error(sh, "Usage: motor position gains set <kp> <ki> | "
			    "motor position gains defaults <safe|nominal> | "
			    "motor position gains bandwidth <hz> [zeta]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}
	if (g_motor_params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR) {
		shell_warn(sh,
			   "outer_loop_mode=MPR; position PI gains are inactive unless outer_loop_mode is set to 0");
	}

	if (strcmp(argv[1], "defaults") == 0) {
		if (argc != 3) {
			shell_error(sh, "Usage: motor position gains defaults <safe|nominal>");
			return -EINVAL;
		}

		enum motor_gains_profile profile;
		if (motor_parse_gains_profile(argv[2], &profile) != 0) {
			shell_error(sh, "Profile must be 'safe' or 'nominal'");
			return -EINVAL;
		}

		float vel_kp_dummy = 0.0f;
		float vel_ki_dummy = 0.0f;
		float vel_iq_dummy = 0.0f;
		float kp = 0.0f;
		float ki = 0.0f;
		if (profile == MOTOR_GAINS_PROFILE_SAFE) {
			motor_compute_safe_outer_gains(g_motor_params, &vel_kp_dummy, &vel_ki_dummy,
						      &vel_iq_dummy, &kp, &ki);
		} else {
			motor_compute_nominal_outer_gains(g_motor_params, &vel_kp_dummy, &vel_ki_dummy,
							 &vel_iq_dummy, &kp, &ki);
		}

		int ret = motor_apply_position_gains(kp, ki);
		if (ret != 0) {
			shell_error(sh, "Failed to apply position defaults (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Position %s defaults applied: Kp=%.5f (rad/s)/rad, Ki=%.5f (rad/s^2)/rad",
			    (profile == MOTOR_GAINS_PROFILE_SAFE) ? "safe" : "nominal",
			    (double)kp, (double)ki);
		return 0;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 4) {
			shell_error(sh, "Usage: motor position gains set <kp> <ki>");
			return -EINVAL;
		}

		float kp = 0.0f;
		float ki = 0.0f;
		if (!shell_parse_finite_float(argv[2], &kp) ||
		    !shell_parse_finite_float(argv[3], &ki)) {
			shell_error(sh, "kp/ki must be finite numbers");
			return -EINVAL;
		}

		int ret = motor_apply_position_gains(kp, ki);
		if (ret != 0) {
			shell_error(sh, "Failed to update position gains (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh, "Position gains set: Kp=%.5f (rad/s)/rad, Ki=%.5f (rad/s^2)/rad",
			    (double)kp, (double)ki);
		return 0;
	}

	if (strcmp(argv[1], "bandwidth") == 0) {
		if (argc != 3 && argc != 4) {
			shell_error(sh, "Usage: motor position gains bandwidth <hz> [zeta]");
			return -EINVAL;
		}

		float bw_hz = 0.0f;
		float zeta = OUTER_LOOP_ZETA_DEFAULT;
			if (!shell_parse_finite_float(argv[2], &bw_hz) ||
			    (argc == 4 && !shell_parse_finite_float(argv[3], &zeta))) {
				shell_error(sh, "Bandwidth/zeta must be finite numbers");
				return -EINVAL;
			}
			if (bw_hz <= 0.0f || bw_hz > (CONTROL_LOOP_FREQUENCY_HZ * 0.25f) ||
			    zeta < OUTER_LOOP_ZETA_MIN || zeta > OUTER_LOOP_ZETA_MAX) {
				shell_error(sh, "Invalid bandwidth/zeta; bw in (0, %.1f] Hz, zeta in %.1f..%.1f",
					    (double)(CONTROL_LOOP_FREQUENCY_HZ * 0.25f),
					    (double)OUTER_LOOP_ZETA_MIN, (double)OUTER_LOOP_ZETA_MAX);
				return -EINVAL;
			}

		float vel_bw_hz = 0.0f;
		int ret_bw = motor_estimate_velocity_bandwidth_hz(g_motor_params, &vel_bw_hz);
		if (ret_bw == 0) {
			float max_pos_bw_hz = vel_bw_hz * POSITION_TO_VELOCITY_BW_RATIO_MAX;
			if (bw_hz > max_pos_bw_hz) {
				shell_warn(sh,
					   "Requested position BW %.2f Hz exceeds %.2f Hz (velocity BW/5); clamping",
					   (double)bw_hz, (double)max_pos_bw_hz);
				bw_hz = max_pos_bw_hz;
			}
		} else {
			shell_warn(sh, "Velocity BW estimate unavailable; skipping BW/5 cascade guard");
		}

		float omega = 2.0f * PI_F32 * bw_hz;
		float kp = 2.0f * zeta * omega;
		float ki = omega * omega;
		int ret = motor_apply_position_gains(kp, ki);
		if (ret != 0) {
			shell_error(sh, "Failed to apply position bandwidth gains (err %d)", ret);
			return ret;
		}

		motor_command_feed_watchdog(g_motor_params);
		shell_print(sh,
			    "Position bandwidth tuned: bw=%.2f Hz zeta=%.2f -> Kp=%.5f (rad/s)/rad, Ki=%.5f (rad/s^2)/rad",
			    (double)bw_hz, (double)zeta, (double)kp, (double)ki);
		return 0;
	}

	shell_error(sh, "Usage: motor position gains set <kp> <ki> | "
		    "motor position gains defaults <safe|nominal> | "
		    "motor position gains bandwidth <hz> [zeta]");
	return -EINVAL;
}

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
/* motor rls status */
static int cmd_motor_rls_status(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	shell_print(sh, "RLS Estimator Status:");
	shell_print(sh, "");
	
	/* D-axis status */
	bool d_converged = rls_motor_est_is_converged(&g_motor_params->rls.d);
	float32_t d_residual = rls_motor_est_get_residual(&g_motor_params->rls.d);
	shell_print(sh, "D-axis:");
	shell_print(sh, "  Converged:     %s", d_converged ? "YES" : "NO");
	shell_print(sh, "  Residual:      %.6f V", (double)d_residual);
	shell_print(sh, "  Update count:  %u", g_motor_params->rls.d.num_updates);
	
	/* Q-axis status */
	bool q_converged = rls_motor_est_is_converged(&g_motor_params->rls.q);
	float32_t q_residual = rls_motor_est_get_residual(&g_motor_params->rls.q);
	shell_print(sh, "");
	shell_print(sh, "Q-axis:");
	shell_print(sh, "  Converged:     %s", q_converged ? "YES" : "NO");
	shell_print(sh, "  Residual:      %.6f V", (double)q_residual);
	shell_print(sh, "  Update count:  %u", g_motor_params->rls.q.num_updates);
	
	return 0;
}

/* motor rls params */
static int cmd_motor_rls_params(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	/* D-axis estimates */
	float32_t Rs_d = rls_motor_est_get_Rs(&g_motor_params->rls.d);
	float32_t Ld = rls_motor_est_get_L(&g_motor_params->rls.d);
	float32_t Vbias_d = rls_motor_est_get_Vbias(&g_motor_params->rls.d);
	float32_t Vdt_d = rls_motor_est_get_Vdt_sign(&g_motor_params->rls.d);
	
	/* Q-axis estimates */
	float32_t Rs_q = rls_motor_est_get_Rs(&g_motor_params->rls.q);
	float32_t Lq = rls_motor_est_get_L(&g_motor_params->rls.q);
	float32_t Vbias_q = rls_motor_est_get_Vbias(&g_motor_params->rls.q);
	float32_t Vdt_q = rls_motor_est_get_Vdt_sign(&g_motor_params->rls.q);
	
	/* Averaged/synthesized values */
	float32_t Rs_avg = g_motor_params->Rs_measured_ohm;
	
	shell_print(sh, "RLS Parameter Estimates:");
	shell_print(sh, "");
	shell_print(sh, "Resistance:");
	shell_print(sh, "  Rs (d-axis):   %.6f Ω", (double)Rs_d);
	shell_print(sh, "  Rs (q-axis):   %.6f Ω", (double)Rs_q);
	shell_print(sh, "  Rs (averaged): %.6f Ω", (double)Rs_avg);
	shell_print(sh, "");
	shell_print(sh, "Inductance:");
	shell_print(sh, "  Ld:            %.6f H (%.3f mH)", (double)Ld, (double)(Ld * 1000.0f));
	shell_print(sh, "  Lq:            %.6f H (%.3f mH)", (double)Lq, (double)(Lq * 1000.0f));
	shell_print(sh, "  Saliency:      %.3f", (double)(Lq / Ld));
	shell_print(sh, "");
	shell_print(sh, "Nonlinearities:");
	shell_print(sh, "  Vbias (d):     %.6f V", (double)Vbias_d);
	shell_print(sh, "  Vbias (q):     %.6f V", (double)Vbias_q);
	shell_print(sh, "  Vdt*sign (d):  %.6f V", (double)Vdt_d);
	shell_print(sh, "  Vdt*sign (q):  %.6f V", (double)Vdt_q);
	
	return 0;
}

/* motor rls temp */
static int cmd_motor_rls_temp(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	float32_t T_rls = g_motor_params->thermal.t_rls_c;
	float32_t T_thermal = thermal_model_get_temperature(&g_motor_params->thermal.model);
	float32_t P_loss = thermal_model_get_power_loss(&g_motor_params->thermal.model);
	float32_t T_delta = T_rls - T_thermal;
	
	shell_print(sh, "Temperature Estimates:");
	shell_print(sh, "");
	shell_print(sh, "  T_rls (from Rs):       %.2f °C", (double)T_rls);
	shell_print(sh, "  T_thermal (model):     %.2f °C", (double)T_thermal);
	shell_print(sh, "  Delta:                 %.2f °C", (double)T_delta);
	shell_print(sh, "");
	shell_print(sh, "Thermal Model:");
	shell_print(sh, "  Power loss:            %.3f W", (double)P_loss);
	shell_print(sh, "  Ambient temp:          %.2f °C", (double)g_motor_params->thermal.model.T_ambient);
	shell_print(sh, "  R_th:                  %.3f °C/W", (double)g_motor_params->thermal.model.R_th);
	shell_print(sh, "  C_th:                  %.1f J/°C", (double)g_motor_params->thermal.model.C_th);
	
	return 0;
}

/* motor rls gating */
static int cmd_motor_rls_gating(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	shell_print(sh, "RLS Gating Conditions:");
	shell_print(sh, "");
	shell_print(sh, "Thresholds:");
	shell_print(sh, "  Min current:     %.3f A", (double)g_motor_params->rls.min_current_a);
	/* Convert rad/s to Hz for display */
	float32_t min_speed_hz = g_motor_params->rls.min_speed_rad_s / (2.0f * PI_F32);
	shell_print(sh, "  Min speed:       %.2f Hz", (double)min_speed_hz);
	shell_print(sh, "  Max voltage:     %.2f V", (double)g_motor_params->rls.max_voltage_v);
	shell_print(sh, "  Max residual:    %.4f V", (double)g_motor_params->rls.max_residual_v);
	shell_print(sh, "");
	shell_print(sh, "Current Status:");
	float32_t Id_abs = fabsf(g_motor_params->live.Id_A);
	float32_t Iq_abs = fabsf(g_motor_params->live.Iq_A);
	float32_t omega_hz = fabsf(g_motor_params->live.velocity_rad_s / (2.0f * PI_F32));
	shell_print(sh, "  |Id|:             %.3f A %s", (double)Id_abs,
	            Id_abs > g_motor_params->rls.min_current_a ? "[OK]" : "[LOW]");
	shell_print(sh, "  |Iq|:             %.3f A %s", (double)Iq_abs,
	            Iq_abs > g_motor_params->rls.min_current_a ? "[OK]" : "[LOW]");
	shell_print(sh, "  Speed:           %.2f Hz %s", (double)omega_hz,
	            omega_hz > min_speed_hz ? "[OK]" : "[LOW]");
	shell_print(sh, "  D residual:      %.6f V %s", (double)g_motor_params->rls.d.residual,
	            fabsf(g_motor_params->rls.d.residual) < g_motor_params->rls.max_residual_v ? "[OK]" : "[HIGH]");
	shell_print(sh, "  Q residual:      %.6f V %s", (double)g_motor_params->rls.q.residual,
	            fabsf(g_motor_params->rls.q.residual) < g_motor_params->rls.max_residual_v ? "[OK]" : "[HIGH]");
	
	return 0;
}

/* motor rls reset */
static int cmd_motor_rls_reset(const struct shell *sh, size_t argc, char **argv)
{
	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -EINVAL;
	}
	
	/* Reset both RLS estimators */
	rls_motor_est_reset(&g_motor_params->rls.d);
	rls_motor_est_reset(&g_motor_params->rls.q);
	prbs_reset(&g_motor_params->rls.prbs_gen);

	/* Reset estimator side-state used by ISR update gating/derivative timing. */
	g_motor_params->rls.id_prev_a = 0.0f;
	g_motor_params->rls.iq_prev_a = 0.0f;
	g_motor_params->rls.d_prev_cycle = g_motor_params->control_loop_count;
	g_motor_params->rls.q_prev_cycle = g_motor_params->control_loop_count;
	g_motor_params->rls.d_prev_valid = 0u;
	g_motor_params->rls.q_prev_valid = 0u;

	/* Restore synthesized values to reset estimator baselines. */
	g_motor_params->rls.ld_est_h = rls_motor_est_get_L(&g_motor_params->rls.d);
	g_motor_params->rls.lq_est_h = rls_motor_est_get_L(&g_motor_params->rls.q);
	g_motor_params->Rs_measured_ohm = rls_motor_est_get_Rs(&g_motor_params->rls.d);
	g_motor_params->thermal.t_rls_c = THERMAL_T_AMBIENT;

	shell_print(sh, "RLS estimators fully reset (state, covariance, PRBS, side-state)");

	return 0;
}
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

/*============================================================================
 * Shell Command Tree
 *============================================================================*/

/* motor params subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_params,
	SHELL_CMD(get, NULL, "Get parameter value", cmd_motor_params_get),
	SHELL_CMD(set, NULL, "Set parameter value", cmd_motor_params_set),
	SHELL_CMD(list, NULL, "List all parameters", cmd_motor_params_list),
	SHELL_SUBCMD_SET_END
);

/* motor current subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_current_gain,
	SHELL_CMD(get, NULL, "Get current-loop PI gains <id|iq>", cmd_motor_current_gain_get),
	SHELL_CMD(set, NULL, "Set current-loop PI gains <id|iq> <kp> <ki>", cmd_motor_current_gain_set),
	SHELL_CMD(bandwidth, NULL, "Set current-loop PI bandwidth <id|iq> <hz>", cmd_motor_current_gain_bandwidth),
	SHELL_SUBCMD_SET_END
);

/* motor current subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_current,
	SHELL_CMD(id, NULL, "Set Id current (A)", cmd_motor_current_id),
	SHELL_CMD(iq, NULL, "Set Iq current (A)", cmd_motor_current_iq),
	SHELL_CMD(dq, NULL, "Set Id and Iq currents", cmd_motor_current_dq),
	SHELL_CMD(gain, &sub_motor_current_gain, "Current-loop PI gain tuning", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor state mode subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state_mode,
	SHELL_CMD(current_encoder, NULL, "Encoder-commutated direct Id/Iq current mode",
		  cmd_motor_state_mode_current_encoder),
	SHELL_CMD(velocity_generated, NULL, "Generated-angle velocity mode",
		  cmd_motor_state_mode_velocity_generated),
	SHELL_CMD(position_generated, NULL, "Generated-angle position/profile mode",
		  cmd_motor_state_mode_position_generated),
	SHELL_CMD(velocity_encoder, NULL, "Encoder-feedback velocity mode",
		  cmd_motor_state_mode_velocity_encoder),
	SHELL_CMD(position_encoder, NULL, "Encoder-feedback position/profile mode",
		  cmd_motor_state_mode_position_encoder),
	SHELL_SUBCMD_SET_END
);

/* motor state subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_state,
	SHELL_CMD(idle, NULL, "Transition to IDLE state", cmd_motor_state_idle),
	SHELL_CMD(prepare, NULL, "Run prepare/calibration path before ONLINE",
		  cmd_motor_state_prepare_online),
	SHELL_CMD(online, NULL, "Transition to ONLINE state", cmd_motor_state_online),
	SHELL_CMD(calibrate, NULL, "Run fast boot calibration (current offsets only)",
		  cmd_motor_state_calibrate),
	SHELL_CMD(commission, NULL, "Run electrical commissioning (offset + R/L + Rs)",
		  cmd_motor_state_commission),
	SHELL_CMD(clear_error, NULL, "Clear error state", cmd_motor_state_clear_error),
	SHELL_CMD(status, NULL, "Show motor status", cmd_motor_state_status),
	SHELL_CMD(policy, NULL, "Show active control policy", cmd_motor_state_policy),
	SHELL_CMD(mode, &sub_motor_state_mode, "Switch control mode", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor info subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_info,
	SHELL_CMD(config, NULL, "Show motor config", cmd_motor_info_config),
	SHELL_CMD(measured, NULL, "Show measured params", cmd_motor_info_measured),
	SHELL_CMD(live, NULL, "Show live telemetry", cmd_motor_info_live),
	SHELL_CMD(stats, NULL, "Show statistics", cmd_motor_info_stats),
	SHELL_SUBCMD_SET_END
);

#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
/* motor rls subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_rls,
	SHELL_CMD(status, NULL, "Show RLS status", cmd_motor_rls_status),
	SHELL_CMD(params, NULL, "Show RLS estimates", cmd_motor_rls_params),
	SHELL_CMD(temp, NULL, "Show temperature estimates", cmd_motor_rls_temp),
	SHELL_CMD(gating, NULL, "Show gating conditions", cmd_motor_rls_gating),
	SHELL_CMD(reset, NULL, "Reset RLS estimators", cmd_motor_rls_reset),
	SHELL_SUBCMD_SET_END
);
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */

/* motor velocity subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_velocity,
	SHELL_CMD_ARG(target, NULL, "Set velocity target <hz>", cmd_motor_velocity_target, 2, 0),
	SHELL_CMD_ARG(decimation, NULL, "Set velocity loop decimation <ticks>", cmd_motor_velocity_decimation, 2, 0),
	SHELL_CMD_ARG(gains, NULL,
		      "Configure velocity PI gains: set <kp> <ki> <iq_limit> | defaults <safe|nominal> | bandwidth <hz> [zeta]",
		      cmd_motor_velocity_gains, 3, 2),
	SHELL_CMD_ARG(dob, NULL,
		      "Velocity disturbance observer: status | defaults <safe|nominal> | enable <0|1> | gain <nm_per_rad_s> | torque_limit <nm> | iq_limit <a>",
		      cmd_motor_velocity_dob, 2, 1),
	SHELL_CMD(status, NULL, "Show velocity status", cmd_motor_velocity_status),
	SHELL_SUBCMD_SET_END
);

/* motor position subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_position,
	SHELL_CMD_ARG(target, NULL, "Set position target <deg>", cmd_motor_position_target, 2, 0),
	SHELL_CMD_ARG(decimation, NULL, "Set position loop decimation <ticks>", cmd_motor_position_decimation, 2, 0),
	SHELL_CMD_ARG(gains, NULL,
		      "Configure position PI gains: set <kp> <ki> | defaults <safe|nominal> | bandwidth <hz> [zeta]",
		      cmd_motor_position_gains, 3, 1),
	SHELL_CMD(status, NULL, "Show position status", cmd_motor_position_status),
	SHELL_SUBCMD_SET_END
);

/* motor profile seq subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile_seq_trigger,
	SHELL_CMD_ARG(source, NULL, "Set trigger source <timer|external> (alias: internal)",
		      cmd_motor_profile_seq_trigger_source, 2, 0),
	SHELL_CMD_ARG(edge, NULL, "Set external edge <rising|falling|both>",
		      cmd_motor_profile_seq_trigger_edge, 2, 0),
	SHELL_CMD_ARG(channel, NULL, "Set external capture channel <0..3>",
		      cmd_motor_profile_seq_trigger_channel, 2, 0),
	SHELL_CMD_ARG(min_interval_us, NULL, "Set minimum trigger spacing <us>",
		      cmd_motor_profile_seq_trigger_min_interval, 2, 0),
	SHELL_CMD(status, NULL, "Show trigger source/edge/filter status",
		  cmd_motor_profile_seq_trigger_status),
	SHELL_CMD(fire, NULL, "Inject one software external trigger", cmd_motor_profile_seq_trigger_fire),
	SHELL_SUBCMD_SET_END
);

/* motor profile seq subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile_seq,
	SHELL_CMD(clear, NULL, "Clear sequence points", cmd_motor_profile_seq_clear),
	SHELL_CMD_ARG(add, NULL, "Add sequence point <target_deg>", cmd_motor_profile_seq_add, 2, 0),
	SHELL_CMD_ARG(period_ms, NULL, "Set trigger period <ms>", cmd_motor_profile_seq_period_ms, 2, 0),
	SHELL_CMD_ARG(move_ms, NULL, "Set move duration <ms>", cmd_motor_profile_seq_move_ms, 2, 0),
	SHELL_CMD_ARG(end_vel_hz, NULL, "Set segment end velocity <hz>",
		      cmd_motor_profile_seq_end_vel_hz, 2, 0),
	SHELL_CMD_ARG(loop, NULL, "Set loop enable <0|1>", cmd_motor_profile_seq_loop, 2, 0),
	SHELL_CMD_ARG(config, NULL,
		      "Legacy bulk set <period_ms> <move_ms> <end_vel_hz> <loop:0|1>",
		      cmd_motor_profile_seq_config, 5, 0),
	SHELL_CMD(trigger, &sub_motor_profile_seq_trigger, "Sequence trigger source config", NULL),
	SHELL_CMD(start, NULL, "Start sequence playback using configured trigger source",
		  cmd_motor_profile_seq_start),
	SHELL_CMD(stop, NULL, "Stop sequence playback", cmd_motor_profile_seq_stop),
	SHELL_CMD(status, NULL, "Show sequence status", cmd_motor_profile_seq_status),
	SHELL_CMD(list, NULL, "List sequence points", cmd_motor_profile_seq_list),
	SHELL_SUBCMD_SET_END
);

/* motor profile subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_profile,
	SHELL_CMD_ARG(set, NULL, "Set profile limits <max_hz> <max_accel_hz_s>", cmd_motor_profile_set, 3, 0),
	SHELL_CMD_ARG(move, NULL, "Plan quintic move <target_deg> <end_vel_hz> <duration_ms>", cmd_motor_profile_move, 4, 0),
	SHELL_CMD(cancel, NULL, "Cancel active motion profile", cmd_motor_profile_cancel),
	SHELL_CMD(seq, &sub_motor_profile_seq, "Sequence playback control", NULL),
	SHELL_CMD(status, NULL, "Show motion profile status", cmd_motor_profile_status),
	SHELL_SUBCMD_SET_END
);

/* motor chopper calib subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_chopper_calib,
	SHELL_CMD_ARG(start, NULL, "Capture edges <slots> <revs> <speed_hz>", cmd_motor_chopper_calib_start, 4, 0),
	SHELL_CMD(stop, NULL, "Stop active calibration capture", cmd_motor_chopper_calib_stop),
	SHELL_CMD(status, NULL, "Show calibration capture/midpoint status", cmd_motor_chopper_calib_status),
	SHELL_CMD(apply, NULL, "Apply midpoint table to profile sequence points", cmd_motor_chopper_calib_apply),
	SHELL_CMD(clear, NULL, "Clear calibration buffers and midpoint table", cmd_motor_chopper_calib_clear),
	SHELL_SUBCMD_SET_END
);

/* motor chopper subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_chopper,
	SHELL_CMD(calib, &sub_motor_chopper_calib, "Chopper midpoint calibration", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor encoder subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_capture,
	SHELL_CMD_ARG(start, NULL, "Start capture [decimation]", cmd_motor_encoder_capture_start, 1, 1),
	SHELL_CMD(stop, NULL, "Stop capture", cmd_motor_encoder_capture_stop),
	SHELL_CMD(status, NULL, "Show capture buffer status", cmd_motor_encoder_capture_status),
	SHELL_CMD(summary, NULL, "Summarize capture buffer", cmd_motor_encoder_capture_summary),
	SHELL_CMD_ARG(dump, NULL, "Dump samples [count] or <offset> <count> (max 32 rows)",
		      cmd_motor_encoder_capture_dump, 1, 2),
	SHELL_CMD_ARG(compare, NULL, "Dump encoder comparison [count] [gen|obs]",
		      cmd_motor_encoder_capture_compare, 1, 2),
	SHELL_CMD(clear, NULL, "Clear capture buffer", cmd_motor_encoder_capture_clear),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_trace,
	SHELL_CMD_ARG(start, NULL, "Start raw trace [decimation]", cmd_motor_encoder_trace_start, 1, 1),
	SHELL_CMD(stop, NULL, "Stop raw trace", cmd_motor_encoder_trace_stop),
	SHELL_CMD(status, NULL, "Show raw trace buffer status", cmd_motor_encoder_trace_status),
	SHELL_CMD(summary, NULL, "Summarize raw trace buffer", cmd_motor_encoder_trace_summary),
	SHELL_CMD_ARG(dump, NULL, "Dump raw samples [count] or <offset> <count> (max 32 rows)",
		      cmd_motor_encoder_trace_dump, 1, 2),
	SHELL_CMD(clear, NULL, "Clear raw trace buffer", cmd_motor_encoder_trace_clear),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder_protocol,
	SHELL_CMD(status, NULL, "Show AEAT-9955 protocol/config state",
		  cmd_motor_encoder_protocol_status),
	SHELL_CMD(spi4_8_volatile, NULL, "Switch AEAT to volatile SPI4-8 CRC16",
		  cmd_motor_encoder_protocol_spi4_8_volatile),
	SHELL_CMD(spi4_16_volatile, NULL, "Switch AEAT to volatile SPI4-16 parity",
		  cmd_motor_encoder_protocol_spi4_16_volatile),
	SHELL_CMD(detect, NULL, "Detect current AEAT SPI4 protocol",
		  cmd_motor_encoder_protocol_detect),
	SHELL_CMD(driver_spi4_8, NULL, "Set driver-only protocol to SPI4-8 CRC16",
		  cmd_motor_encoder_protocol_driver_spi4_8),
	SHELL_CMD(driver_spi4_16, NULL, "Set driver-only protocol to SPI4-16 parity",
		  cmd_motor_encoder_protocol_driver_spi4_16),
	SHELL_CMD_ARG(spi_mode, NULL, "Set RT SPI electrical mode <cpol 0|1> <cpha 0|1>",
		      cmd_motor_encoder_protocol_spi_mode, 3, 0),
	SHELL_CMD(raw_position, NULL, "Read one raw AEAT position frame",
		  cmd_motor_encoder_protocol_raw_position),
	SHELL_CMD_ARG(raw_reg, NULL, "Read one raw AEAT register frame <addr>",
		      cmd_motor_encoder_protocol_raw_reg, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_encoder,
	SHELL_CMD(alarm, NULL, "Read AEAT-9955 alarm byte (MHI/MLO)", cmd_motor_encoder_alarm),
	SHELL_CMD(fast, NULL, "Show fast encoder_rt driver status/counters",
		  cmd_motor_encoder_fast),
	SHELL_CMD_ARG(reg_read, NULL, "Read AEAT-9955 register <addr>",
		      cmd_motor_encoder_reg_read, 2, 0),
	SHELL_CMD_ARG(reg_write, NULL, "Write AEAT-9955 register <addr> <value>",
		      cmd_motor_encoder_reg_write, 3, 0),
	SHELL_CMD(protocol, &sub_motor_encoder_protocol, "AEAT-9955 protocol control", NULL),
	SHELL_CMD_ARG(direction, NULL, "Get/set encoder direction sign [<1|-1>]",
		      cmd_motor_encoder_direction, 1, 1),
	SHELL_CMD_ARG(trim, NULL, "Get/set electrical commutation trim [<-180..180> deg]",
		      cmd_motor_encoder_trim, 1, 1),
	SHELL_CMD(capture, &sub_motor_encoder_capture, "Encoder sample capture buffer", NULL),
	SHELL_CMD(trace, &sub_motor_encoder_trace, "Raw encoder telemetry trace buffer", NULL),
	SHELL_CMD(acquisition, NULL, "Show encoder acquisition status/counters",
		  cmd_motor_encoder_acquisition),
	SHELL_CMD(acquisition_reset, NULL, "Reset encoder acquisition counters",
		  cmd_motor_encoder_acquisition_reset),
	SHELL_CMD_ARG(acquisition_inject, NULL, "Acquisition fault inject mode [none|status|frame]",
		      cmd_motor_encoder_acquisition_inject, 1, 1),
	SHELL_CMD(control_status, NULL, "Show encoder-control readiness gate",
		  cmd_motor_encoder_control_status),
	SHELL_SUBCMD_SET_END
);

/* motor safety subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_safety,
	SHELL_CMD_ARG(timeout, NULL, "Set command timeout <ms> (0 disables)", cmd_motor_safety_timeout, 2, 0),
	SHELL_CMD(status, NULL, "Show safety interlock/timeout status", cmd_motor_safety_status),
	SHELL_CMD(pet, NULL, "Refresh command watchdog timer", cmd_motor_safety_pet),
	SHELL_SUBCMD_SET_END
);

/* motor gate-driver subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_gate,
	SHELL_CMD(status, NULL, "Show gate-driver cached fault status", cmd_motor_gate_status),
	SHELL_CMD(reset, NULL, "Pulse DRV8328 nSLEEP to clear latched faults", cmd_motor_gate_reset),
	SHELL_SUBCMD_SET_END
);

/* motor fault snapshot subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_fault_snapshot,
	SHELL_CMD_ARG(start, NULL, "Start fault snapshot capture [decimation]",
		      cmd_motor_fault_snapshot_start, 1, 1),
	SHELL_CMD(stop, NULL, "Stop fault snapshot capture", cmd_motor_fault_snapshot_stop),
	SHELL_CMD(status, NULL, "Show fault snapshot ring status", cmd_motor_fault_snapshot_status),
	SHELL_CMD_ARG(dump, NULL, "Dump latest fault snapshot rows [count]",
		      cmd_motor_fault_snapshot_dump, 1, 1),
	SHELL_CMD(clear, NULL, "Clear fault snapshot ring and latch", cmd_motor_fault_snapshot_clear),
	SHELL_SUBCMD_SET_END
);

/* motor fault subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_fault,
	SHELL_CMD(snapshot, &sub_motor_fault_snapshot, "ISR fault snapshot diagnostics", NULL),
	SHELL_SUBCMD_SET_END
);

/* motor commission flux subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_flux,
	SHELL_CMD_ARG(run, NULL,
		      "Run flux capture <min_hz> <max_hz> <steps> <settle_ms> <sample_ms> <iq_limit_a>",
		      cmd_motor_commission_flux_run, 7, 0),
	SHELL_SUBCMD_SET_END
);

/* motor commission mech subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_mech,
	SHELL_CMD_ARG(run, NULL,
		      "Run mechanical capture <base_hz> <dither_hz> <dither_period_ms> <duration_ms>",
		      cmd_motor_commission_mech_run, 5, 0),
	SHELL_SUBCMD_SET_END
);

/* motor commission motion subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_motion,
	SHELL_CMD_ARG(threshold, NULL,
		      "Find min moving current <start_a> <stop_a> <step_a> <hold_ms> [min_motion_deg]",
		      cmd_motor_commission_motion_threshold, 5, 1),
	SHELL_SUBCMD_SET_END
);

/* motor commission encoder subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_encoder,
	SHELL_CMD_ARG(run, NULL,
		      "Run generated sweep encoder mapping <current_a> <mech_hz> <cycles>",
		      cmd_motor_commission_encoder_run, 4, 0),
	SHELL_CMD(status, NULL, "Show staged encoder mapping result",
		  cmd_motor_commission_encoder_status),
	SHELL_CMD(apply, NULL, "Apply valid staged encoder mapping",
		  cmd_motor_commission_encoder_apply),
	SHELL_CMD(clear, NULL, "Clear staged encoder mapping result",
		  cmd_motor_commission_encoder_clear),
	SHELL_SUBCMD_SET_END
);

/* motor commission detent subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_detent,
	SHELL_CMD_ARG(run, NULL,
		      "Run detent feedforward capture <mech_hz> <cycles> [decimation] [iq_limit_a]",
		      cmd_motor_commission_detent_run, 3, 2),
	SHELL_CMD(status, NULL, "Show staged detent feedforward table",
		  cmd_motor_commission_detent_status),
	SHELL_CMD_ARG(apply, NULL,
		      "Apply staged detent table [enable] [gain] [limit_a]",
		      cmd_motor_commission_detent_apply, 1, 3),
	SHELL_CMD(clear, NULL, "Clear staged and runtime detent feedforward table",
		  cmd_motor_commission_detent_clear),
	SHELL_SUBCMD_SET_END
);

/* motor commission auto subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission_auto,
	SHELL_CMD_ARG(run, NULL, "Plan or run identify+tune workflow [slow|confirm] [apply]",
		      cmd_motor_commission_auto_run, 1, 1),
	SHELL_CMD(status, NULL, "Show staged auto-tune defaults and reject flags",
		  cmd_motor_commission_auto_status),
	SHELL_CMD(apply, NULL, "Apply staged auto-tune defaults to active runtime parameters",
		  cmd_motor_commission_auto_apply),
	SHELL_CMD_ARG(validate, NULL,
		      "Apply staged tune and run velocity_encoder validation [max_hz] [hold_ms]",
		      cmd_motor_commission_auto_validate, 1, 2),
	SHELL_SUBCMD_SET_END
);

/* motor commission subcommands */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor_commission,
	SHELL_CMD(status, NULL, "Show commissioning status and capture stats", cmd_motor_commission_status),
	SHELL_CMD(clear, NULL, "Clear commissioning context and captured data", cmd_motor_commission_clear),
	SHELL_CMD(abort, NULL, "Abort active commissioning run", cmd_motor_commission_abort),
	SHELL_CMD(apply, NULL, "Apply valid commissioning estimates to active runtime params", cmd_motor_commission_apply),
	SHELL_CMD(motion, &sub_motor_commission_motion, "Motion threshold commissioning", NULL),
	SHELL_CMD(flux, &sub_motor_commission_flux, "Flux-linkage commissioning", NULL),
	SHELL_CMD(mech, &sub_motor_commission_mech, "Mechanical commissioning", NULL),
	SHELL_CMD(encoder, &sub_motor_commission_encoder, "Generated-sweep encoder mapping", NULL),
	SHELL_CMD(detent, &sub_motor_commission_detent, "Detent feedforward commissioning", NULL),
	SHELL_CMD(auto, &sub_motor_commission_auto, "One-command identify+tune workflow", NULL),
	SHELL_SUBCMD_SET_END
);

/* Top-level motor command */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD(params, &sub_motor_params, "Parameter access", NULL),
	SHELL_CMD(current, &sub_motor_current, "Current control", NULL),
	SHELL_CMD(state, &sub_motor_state, "State machine control", NULL),
	SHELL_CMD(arm, NULL, "Arm torque-producing control output", cmd_motor_arm),
	SHELL_CMD(disarm, NULL, "Disarm control output and request IDLE", cmd_motor_disarm),
	SHELL_CMD(safety, &sub_motor_safety, "Safety interlock and timeout", NULL),
	SHELL_CMD(gate, &sub_motor_gate, "Gate-driver diagnostics/recovery", NULL),
	SHELL_CMD(fault, &sub_motor_fault, "Fault diagnostics", NULL),
	SHELL_CMD(info, &sub_motor_info, "Motor information", NULL),
#ifdef CONFIG_RLS_PARAMETER_ESTIMATION
	SHELL_CMD(rls, &sub_motor_rls, "RLS parameter estimation", NULL),
#endif /* CONFIG_RLS_PARAMETER_ESTIMATION */
	SHELL_CMD(velocity, &sub_motor_velocity, "Velocity control", NULL),
	SHELL_CMD(position, &sub_motor_position, "Position control", NULL),
	SHELL_CMD(profile, &sub_motor_profile, "Motion profile settings", NULL),
	SHELL_CMD(chopper, &sub_motor_chopper, "Optical chopper utilities", NULL),
	SHELL_CMD(commission, &sub_motor_commission, "Commissioning workflows", NULL),
	SHELL_CMD(encoder, &sub_motor_encoder, "Encoder diagnostics", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &sub_motor, "Motor control commands", NULL);
