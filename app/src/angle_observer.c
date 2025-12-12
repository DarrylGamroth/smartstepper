/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "angle_observer.h"
#include <zephyr/dsp/types.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(angle_observer, CONFIG_APP_LOG_LEVEL);

/* Pi constant for bandwidth to rad/s conversion */
#define PI_F32 3.14159265358979323846f

/**
 * @brief Wrap angle to (-180, 180] degrees
 *
 * Used for computing the shortest-path error when the encoder crosses the
 * ±180° boundary. For typical small per-tick angle changes, this executes
 * zero or one loop iteration.
 *
 * @param angle Input angle in degrees
 * @return Wrapped angle in (-180, 180]
 */
static inline float32_t wrap_deg_180(float32_t angle)
{
	while (angle > 180.0f) {
		angle -= 360.0f;
	}
	while (angle <= -180.0f) {
		angle += 360.0f;
	}
	return angle;
}

/**
 * @brief Wrap angle to [0, 360) degrees
 *
 * Used for normalizing output angles to a consistent range.
 *
 * @param angle Input angle in degrees
 * @return Wrapped angle in [0, 360)
 */
static inline float32_t wrap_deg_360(float32_t angle)
{
	while (angle >= 360.0f) {
		angle -= 360.0f;
	}
	while (angle < 0.0f) {
		angle += 360.0f;
	}
	return angle;
}

void angle_observer_init(struct angle_observer_state *obs,
			 float32_t sample_period_s,
			 float32_t bandwidth_hz,
			 uint32_t pole_pairs)
{
	/* Zero all state */
	obs->angle_est_deg = 0.0f;
	obs->speed_est_dps = 0.0f;

	/* Zero all outputs */
	obs->mech_angle_deg = 0.0f;
	obs->elec_angle_deg = 0.0f;
	obs->mech_speed_dps = 0.0f;
	obs->mech_angle_pred_deg = 0.0f;
	obs->elec_angle_pred_deg = 0.0f;

	/* Cache configuration */
	obs->sample_period_s = sample_period_s;
	obs->bandwidth_hz = bandwidth_hz;
	obs->pole_pairs = pole_pairs;

	/* Precalculate observer gains (critically damped tuning) */
	const float32_t wo = 2.0f * PI_F32 * bandwidth_hz;
	obs->L1Ts = 2.0f * wo * sample_period_s;  /* Position gain × Ts */
	obs->L2Ts = wo * wo * sample_period_s;    /* Velocity gain × Ts */

	/* Zero mechanical angle offset */
	obs->mech_angle_offset_deg = 0.0f;
}

void angle_observer_set_offset(struct angle_observer_state *obs,
			       float32_t offset_deg)
{
	obs->mech_angle_offset_deg = offset_deg;
}

void angle_observer_update(struct angle_observer_state *obs,
			   float32_t encoder_angle_deg)
{
	const float32_t Ts = obs->sample_period_s;

	/* 
	 * α-β Tracking Observer for Mechanical Angle and Speed
	 * 
	 * State equations (continuous time):
	 *   θ̇ = ω              (angle rate = speed)
	 *   ω̇ = 0              (constant speed model)
	 * 
	 * Observer equations:
	 *   θ̂̇ = ω̂ + L₁·(θ - θ̂)     (angle estimate + correction)
	 *   ω̂̇ = L₂·(θ - θ̂)          (speed estimate correction)
	 * 
	 * Critically damped tuning (ζ = 1):
	 *   L₁ = 2·ωₒ           (position gain)
	 *   L₂ = ωₒ²            (velocity gain)
	 *   where ωₒ = 2π·bandwidth_hz
	 * 
	 * Forward Euler discretization:
	 *   θ̂[k+1] = θ̂[k] + Ts·ω̂[k] + L₁Ts·e[k]
	 *   ω̂[k+1] = ω̂[k] + L₂Ts·e[k]
	 *   where e[k] = θ[k] - θ̂[k] (wrapped error)
	 */

	/* Compute wrapped error in (-180, 180] for shortest-path tracking */
	float32_t err_deg = wrap_deg_180(encoder_angle_deg - obs->angle_est_deg);

	/* Observer update (forward Euler discretization) */
	obs->angle_est_deg += Ts * obs->speed_est_dps + obs->L1Ts * err_deg;
	obs->speed_est_dps += obs->L2Ts * err_deg;

	/* Wrap angle estimate to [0, 360) for easier debugging */
	obs->angle_est_deg = wrap_deg_360(obs->angle_est_deg);

	/* Current-cycle outputs (wrapped for readability) */
	obs->mech_angle_deg = obs->angle_est_deg;
	obs->mech_speed_dps = obs->speed_est_dps;

	/* Apply mechanical offset and compute electrical angle */
	float32_t mech_angle_offset = obs->mech_angle_deg + obs->mech_angle_offset_deg;
	obs->elec_angle_deg = wrap_deg_360(mech_angle_offset * obs->pole_pairs);

	/* One-step prediction for next control cycle */
	float32_t mech_angle_pred = obs->angle_est_deg + Ts * obs->speed_est_dps;
	obs->mech_angle_pred_deg = wrap_deg_360(mech_angle_pred);

	/* Apply offset to predicted mechanical angle for predicted electrical angle */
	float32_t mech_angle_pred_offset = obs->mech_angle_pred_deg + obs->mech_angle_offset_deg;
	obs->elec_angle_pred_deg = wrap_deg_360(mech_angle_pred_offset * obs->pole_pairs);
}
