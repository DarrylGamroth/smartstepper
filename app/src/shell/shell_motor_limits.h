#ifndef APP_SRC_SHELL_MOTOR_LIMITS_H_
#define APP_SRC_SHELL_MOTOR_LIMITS_H_

#include <errno.h>
#include <math.h>

#include "config.h"
#include "motor_torque.h"
#include "motor/control/speed_limit.h"
#include "motor/math/math_constants.h"

#define MOTOR_VOLTAGE_SPEED_LIMIT_SAFETY 0.75f
#define MOTOR_SHELL_VBUS_MIN_VALID_V 0.1f

static inline const char *motor_shell_flux_source_name(const struct motor_parameters *params)
{
	if (params != NULL && params->flux_model_source == MOTOR_MODEL_SOURCE_MEASURED &&
	    isfinite(params->flux_linkage_wb_active) && params->flux_linkage_wb_active > 0.0f) {
		return "measured";
	}

	if (isfinite(MOTOR_FLUX_LINKAGE_WB) && MOTOR_FLUX_LINKAGE_WB > 0.0f) {
		return "fallback";
	}

	return "unknown";
}

static inline float motor_shell_live_or_nominal_vbus(const struct motor_parameters *params)
{
	float vbus = (params != NULL) ? params->live.dc_bus_voltage_V : 0.0f;

	if (!isfinite(vbus) || vbus < MOTOR_SHELL_VBUS_MIN_VALID_V) {
		vbus = NOMINAL_VOLTAGE_V;
	}

	return vbus;
}

static inline int motor_shell_flux_for_voltage_speed_limit(
	const struct motor_parameters *params,
	float *flux_linkage_wb)
{
	if (params == NULL || flux_linkage_wb == NULL) {
		return -EINVAL;
	}

	if (params->flux_model_source == MOTOR_MODEL_SOURCE_MEASURED &&
	    isfinite(params->flux_linkage_wb_active) && params->flux_linkage_wb_active > 0.0f) {
		*flux_linkage_wb = fabsf(params->flux_linkage_wb_active);
		return 0;
	}

	if (isfinite(MOTOR_FLUX_LINKAGE_WB) && MOTOR_FLUX_LINKAGE_WB > 0.0f) {
		*flux_linkage_wb = MOTOR_FLUX_LINKAGE_WB;
		return 0;
	}

	*flux_linkage_wb = 0.0f;
	return -ENODATA;
}

static inline int motor_shell_voltage_speed_limit(
	const struct motor_parameters *params,
	float iq_limit_a,
	struct motor_voltage_speed_limit_result *result)
{
	if (params == NULL || result == NULL) {
		return -EINVAL;
	}

	float rs = params->Rs_measured_ohm;
	if (!isfinite(rs) || rs < 0.0f) {
		rs = MOTOR_RESISTANCE_OHM;
	}

	float ls = params->Lq_measured_H;
	if (!isfinite(ls) || ls < 0.0f) {
		ls = MOTOR_INDUCTANCE_Q_H;
	}

	float psi = 0.0f;
	int ret = motor_shell_flux_for_voltage_speed_limit(params, &psi);
	if (ret != 0) {
		*result = (struct motor_voltage_speed_limit_result){0};
		return ret;
	}

	const struct motor_voltage_speed_limit_input input = {
		.vbus_v = motor_shell_live_or_nominal_vbus(params),
		.max_modulation_index = params->max_modulation_index,
		.resistance_ohm = rs,
		.inductance_h = ls,
		.flux_linkage_wb = psi,
		.current_limit_a = iq_limit_a,
		.pole_pairs = MOTOR_POLE_PAIRS,
		.safety_factor = MOTOR_VOLTAGE_SPEED_LIMIT_SAFETY,
	};

	return motor_voltage_speed_limit_compute(&input, result);
}

static inline float motor_shell_velocity_command_limit_hz(
	const struct motor_parameters *params)
{
	if (params == NULL) {
		return 0.0f;
	}

	float profile_limit_hz = params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	if (!isfinite(profile_limit_hz) || profile_limit_hz <= 0.0f) {
		profile_limit_hz = MOTOR_MAX_SPEED_HZ;
	}

	struct motor_voltage_speed_limit_result voltage = {0};
	if (motor_shell_voltage_speed_limit(params,
					    fmaxf(params->velocity_cl_iq_limit_A, 0.0f),
					    &voltage) == 0 &&
	    voltage.valid) {
		profile_limit_hz = fminf(profile_limit_hz, voltage.max_mech_hz);
	}

	return fmaxf(profile_limit_hz, 0.0f);
}

#endif /* APP_SRC_SHELL_MOTOR_LIMITS_H_ */
