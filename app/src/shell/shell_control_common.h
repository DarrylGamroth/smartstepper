#ifndef APP_SRC_SHELL_SHELL_CONTROL_COMMON_H_
#define APP_SRC_SHELL_SHELL_CONTROL_COMMON_H_

/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "shell_control.h"
#include "shell_commands_motion.h"
#include "motor_control_api.h"
#include "motor_states.h"
#include "motor_state_utils.h"
#include "config.h"
#include "motor_torque.h"
#include "motor/control/dob.h"
#include "motor/control/mpr.h"
#include "motor/math/angle_wrap.h"
#include "motor/motion/motion_planner.h"
#include "motor/runtime/commission_tune.h"
#include "shell_parse.h"
#include "shell_motor_limits.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(shell_commands, CONFIG_APP_LOG_LEVEL);

enum motor_gains_profile {
	MOTOR_GAINS_PROFILE_NOMINAL = 0,
	MOTOR_GAINS_PROFILE_SAFE,
};

#define OUTER_LOOP_ZETA_DEFAULT 1.0f
#define OUTER_LOOP_ZETA_MIN 0.2f
#define OUTER_LOOP_ZETA_MAX 2.0f
#define POSITION_TO_VELOCITY_BW_RATIO_MAX 0.2f
#define VELOCITY_STATUS_TRACK_TOL_HZ 0.2f
#define VELOCITY_DEFAULT_NOMINAL_IQ_LIMIT_A 0.120f
#define VELOCITY_DEFAULT_SAFE_IQ_LIMIT_A 0.040f
#define VELOCITY_DEFAULT_GAIN_SPEED_HZ 0.50f
#define VELOCITY_MODEL_SAFE_BW_HZ 0.25f
#define VELOCITY_MODEL_NOMINAL_BW_HZ 0.50f
#define VELOCITY_MODEL_LOW_SPEED_GAIN_HZ 0.50f
#define VELOCITY_MODEL_LOW_SPEED_KP_CURRENT_FRACTION 1.0f
#define VELOCITY_MODEL_KI_TO_KP_MAX 2.0f
#define POSITION_MODEL_SAFE_BW_RATIO 0.10f
#define POSITION_MODEL_NOMINAL_BW_RATIO 0.15f
#define POSITION_TARGET_MIN_DURATION_S 0.20f

static inline int motor_parse_gains_profile(const char *token, enum motor_gains_profile *profile)
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

static inline bool motor_velocity_dob_ready(const struct motor_parameters *params,
				     const char **reason)
{
	if (params == NULL) {
		if (reason != NULL) {
			*reason = "motor not initialized";
		}
		return false;
	}
	if (params->detent_map_cfg.enabled) {
		if (reason != NULL) {
			*reason = "detent feedforward enabled";
		}
		return false;
	}
	if (params->flux_model_source != MOTOR_MODEL_SOURCE_MEASURED ||
	    params->mech_model_source != MOTOR_MODEL_SOURCE_MEASURED) {
		if (reason != NULL) {
			*reason = "model source is not measured";
		}
		return false;
	}

	bool encoder_mode =
		motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_VELOCITY_ENCODER) ||
		motor_state_ptr_is_mode(params->state_for_isr, MOTOR_STATE_ONLINE_POSITION_ENCODER);
	struct motor_dob_readiness_result readiness = {0};
	const struct motor_dob_readiness_input input = {
		.commissioning_complete = params->calibration.complete,
		.encoder_mapping_complete = params->calibration.encoder_mapping_complete,
		.feedback_trusted = !encoder_mode ||
				    params->live.position_trust_state ==
					    MOTOR_FEEDBACK_TRUST_TRUSTED,
		.fault_active = motor_api_get_state() == MOTOR_STATE_ERROR,
		.torque_constant_nm_per_a = params->torque_gain_nm_per_a_active,
		.inertia_kgm2 = params->inertia_kgm2_active,
		.velocity_iq_limit_a = params->velocity_cl_iq_limit_A,
		.cfg = &params->velocity_dob_cfg,
	};

	if (motor_dob_readiness_check(&input, &readiness) != 0 || !readiness.ready) {
		if (reason != NULL) {
			*reason = (readiness.reason_str != NULL) ? readiness.reason_str :
								   "readiness check failed";
		}
		return false;
	}

	if (reason != NULL) {
		*reason = readiness.reason_str;
	}
	return true;
}

static inline void motor_compute_nominal_outer_gains(const struct motor_parameters *params,
					       float *vel_kp, float *vel_ki,
					       float *vel_iq_limit,
					       float *pos_kp, float *pos_ki)
{
	float move_iq_a = (params->commission.results.iq_move_valid ||
			   params->commission.results.iq_move_pos_valid) ?
				  params->commission.results.iq_move_recommended_a :
				  0.0f;
	float iq_limit = fmaxf(VELOCITY_DEFAULT_NOMINAL_IQ_LIMIT_A, 1.50f * move_iq_a);
	iq_limit = clampf(iq_limit, 0.06f,
			  fminf(MOTOR_MAX_CURRENT_A, 0.25f * MOTOR_MAX_CURRENT_A));

	float gain_speed_rad_s = 2.0f * PI_F32 * VELOCITY_DEFAULT_GAIN_SPEED_HZ;
	*vel_kp = 0.75f * iq_limit / gain_speed_rad_s;
	*vel_ki = *vel_kp;
	*vel_iq_limit = iq_limit;
	float max_vel = fmaxf(params->profile_max_velocity_rad_s, 1.0f);
	*pos_kp = max_vel / PI_F32;
	*pos_ki = 0.5f * (*pos_kp);
}

static inline void motor_compute_safe_outer_gains(const struct motor_parameters *params,
					    float *vel_kp, float *vel_ki,
					    float *vel_iq_limit,
					    float *pos_kp, float *pos_ki)
{
	motor_compute_nominal_outer_gains(params, vel_kp, vel_ki, vel_iq_limit, pos_kp, pos_ki);

	float move_iq_a = (params->commission.results.iq_move_valid ||
			   params->commission.results.iq_move_pos_valid) ?
				  params->commission.results.iq_move_recommended_a :
				  0.0f;
	float safe_limit = fmaxf(VELOCITY_DEFAULT_SAFE_IQ_LIMIT_A, move_iq_a);
	*vel_iq_limit = clampf(safe_limit, 0.04f,
			       fminf(MOTOR_MAX_CURRENT_A, 0.12f));
	float gain_speed_rad_s = 2.0f * PI_F32 * VELOCITY_DEFAULT_GAIN_SPEED_HZ;
	*vel_kp = 0.75f * (*vel_iq_limit) / gain_speed_rad_s;
	*vel_ki = 2.0f * (*vel_kp);
	*pos_kp *= 0.5f;
	*pos_ki *= 0.1f;
}

static inline int motor_compute_model_outer_gains(const struct motor_parameters *params,
					   enum motor_gains_profile profile,
					   float *vel_kp, float *vel_ki,
					   float *vel_iq_limit,
					   float *pos_kp, float *pos_ki)
{
	if (params == NULL || vel_kp == NULL || vel_ki == NULL ||
	    vel_iq_limit == NULL || pos_kp == NULL || pos_ki == NULL) {
		return -EINVAL;
	}

	const struct motor_commission_results *res = &params->commission.results;
	bool have_identified_model = res->psi_f_valid && res->mech_valid;
	bool have_applied_tune = params->commission.auto_tune_applied &&
				 params->commission.auto_tune_staged.accepted;
	if (!have_identified_model && !have_applied_tune) {
		return -ENOENT;
	}

	float j = have_identified_model ? res->inertia_kgm2 : params->inertia_kgm2_active;
	float b = have_identified_model ? res->viscous_friction_nm_per_rad_s :
					  params->viscous_friction_nm_per_rad_s_active;
	float kt = have_identified_model ?
			   motor_torque_gain_from_flux_pole_pairs(res->psi_f_wb, MOTOR_POLE_PAIRS) :
			   motor_torque_gain_resolve_active(params);
	if (!isfinite(j) || j <= 0.0f || !isfinite(b) || b < 0.0f ||
	    !isfinite(kt) || kt <= 0.0f) {
		return -ERANGE;
	}

	float move_iq_a = (res->iq_move_valid || res->iq_move_pos_valid) ?
				  res->iq_move_recommended_a :
				  0.0f;
	float iq_base = (profile == MOTOR_GAINS_PROFILE_SAFE) ?
				VELOCITY_DEFAULT_SAFE_IQ_LIMIT_A :
				VELOCITY_DEFAULT_NOMINAL_IQ_LIMIT_A;
	float iq_scale = (profile == MOTOR_GAINS_PROFILE_SAFE) ? 1.0f : 1.5f;
	float iq_limit = fmaxf(iq_base, iq_scale * move_iq_a);
	iq_limit = clampf(iq_limit, 0.04f,
			  fminf(MOTOR_MAX_CURRENT_A,
				(profile == MOTOR_GAINS_PROFILE_SAFE) ? 0.12f :
									  (0.25f * MOTOR_MAX_CURRENT_A)));

	float velocity_bw_hz = (profile == MOTOR_GAINS_PROFILE_SAFE) ?
				       VELOCITY_MODEL_SAFE_BW_HZ :
				       VELOCITY_MODEL_NOMINAL_BW_HZ;
	float zeta = OUTER_LOOP_ZETA_DEFAULT;
	float omega = 2.0f * PI_F32 * velocity_bw_hz;
	float kp_num = (2.0f * zeta * omega * j) - b;
	float kp_floor = (0.25f * omega * j) / kt;
	float low_speed_gain_rad_s = 2.0f * PI_F32 * VELOCITY_MODEL_LOW_SPEED_GAIN_HZ;
	float kp_authority =
		(VELOCITY_MODEL_LOW_SPEED_KP_CURRENT_FRACTION * iq_limit) /
		low_speed_gain_rad_s;
	float kp = fmaxf(fmaxf(kp_num / kt, kp_floor), kp_authority);
	float ki_model = (omega * omega * j) / kt;
	float ki = fminf(ki_model, VELOCITY_MODEL_KI_TO_KP_MAX * kp);
	if (!isfinite(kp) || !isfinite(ki) || kp <= 0.0f || ki <= 0.0f) {
		return -ERANGE;
	}

	float pos_ratio = (profile == MOTOR_GAINS_PROFILE_SAFE) ?
				  POSITION_MODEL_SAFE_BW_RATIO :
				  POSITION_MODEL_NOMINAL_BW_RATIO;
	float pos_bw_hz = velocity_bw_hz * pos_ratio;
	float pos_omega = 2.0f * PI_F32 * pos_bw_hz;

	*vel_kp = kp;
	*vel_ki = ki;
	*vel_iq_limit = iq_limit;
	*pos_kp = 2.0f * zeta * pos_omega;
	*pos_ki = pos_omega * pos_omega;
	return 0;
}

static inline int motor_apply_velocity_gains(float kp, float ki, float iq_limit)
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
		motor_velocity_regulator_reset(&g_motor_params->velocity_reg_state, 0.0f);
	}

	return 0;
}

static inline int motor_apply_position_gains(float kp, float ki)
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
		motor_position_regulator_reset(&g_motor_params->position_reg_state, 0.0f);
	}

	return 0;
}

static inline int motor_compute_velocity_dob_defaults(const struct motor_parameters *params,
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

static inline int motor_compute_velocity_bandwidth_gains(const struct motor_parameters *params,
						  float bw_hz, float zeta,
						  float iq_limit,
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

	float kp_num = (2.0f * zeta * omega * j) - b;
	float kp_floor = (0.25f * omega * j) / kt;
	if (!isfinite(iq_limit) || iq_limit <= 0.0f) {
		return -ERANGE;
	}
	float low_speed_gain_rad_s = 2.0f * PI_F32 * VELOCITY_MODEL_LOW_SPEED_GAIN_HZ;
	float kp_authority =
		(VELOCITY_MODEL_LOW_SPEED_KP_CURRENT_FRACTION * iq_limit) /
		low_speed_gain_rad_s;
	float kp = fmaxf(fmaxf(kp_num / kt, kp_floor), kp_authority);
	float ki_model = (omega * omega * j) / kt;
	float ki = fminf(ki_model, VELOCITY_MODEL_KI_TO_KP_MAX * kp);
	if (!isfinite(kp) || !isfinite(ki) || kp <= 0.0f || ki <= 0.0f) {
		return -ERANGE;
	}

	*kp_out = kp;
	*ki_out = ki;
	*kt_out = kt;
	return 0;
}

static inline int motor_estimate_velocity_bandwidth_hz(const struct motor_parameters *params,
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

static inline int motor_set_param_checked(const char *name, float value)
{
	int ret = motor_api_set_param(name, value);

	if (ret != 0) {
		return ret;
	}

	return 0;
}

static inline float32_t motor_position_target_duration_s(float32_t distance_rad,
						   float32_t max_velocity_rad_s,
						   float32_t max_accel_rad_s2)
{
	float32_t dist = fabsf(distance_rad);
	if (dist <= 1.0e-6f) {
		return POSITION_TARGET_MIN_DURATION_S;
	}

	float32_t max_vel = fmaxf(max_velocity_rad_s, 0.1f);
	float32_t max_accel = fmaxf(max_accel_rad_s2, 0.1f);

	/* A zero-endpoint quintic peaks above average speed/accel. These factors
	 * are intentionally conservative for shell-commanded position moves.
	 */
	float32_t t_vel = 2.0f * dist / max_vel;
	float32_t t_accel = sqrtf(8.0f * dist / max_accel);

	return fmaxf(POSITION_TARGET_MIN_DURATION_S, fmaxf(t_vel, t_accel));
}

static inline int motor_position_target_plan_bounded(float32_t target_wrapped_rad,
					      float32_t *duration_s_out)
{
	if (g_motor_params == NULL || duration_s_out == NULL) {
		return -EINVAL;
	}

	float32_t start_pos_rad = g_motor_params->live.position_rad;
	float32_t start_vel_rad_s = g_motor_params->live.velocity_filtered_rad_s;
	if (!isfinite(start_pos_rad)) {
		start_pos_rad = g_motor_params->position_target_rad;
	}
	if (!isfinite(start_vel_rad_s)) {
		start_vel_rad_s = 0.0f;
	}

	float32_t delta_rad = wrap_rad_pi(target_wrapped_rad - start_pos_rad);
	float32_t duration_s =
		motor_position_target_duration_s(delta_rad,
						 g_motor_params->profile_max_velocity_rad_s,
						 g_motor_params->profile_max_accel_rad_s2);

	int ret = -ERANGE;
	for (uint8_t attempt = 0U; attempt < 5U; attempt++) {
		ret = motor_position_move_plan_sequence_segment(&g_motor_params->position_profile,
								start_pos_rad,
								start_vel_rad_s,
								target_wrapped_rad,
								0.0f,
								duration_s,
								g_motor_params->profile_max_velocity_rad_s,
								g_motor_params->profile_max_accel_rad_s2);
		if (ret == 0) {
			*duration_s_out = duration_s;
			return 0;
		}
		duration_s *= 1.5f;
	}

	return ret;
}

/*============================================================================
 * Shell Command Implementations
 *============================================================================*/

/* motor params get <name> */

#endif /* APP_SRC_SHELL_SHELL_CONTROL_COMMON_H_ */
