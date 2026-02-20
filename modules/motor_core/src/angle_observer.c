/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "angle_observer.h"
#include "angle_wrap.h"
#include <zephyr/dsp/types.h>

void angle_observer_init(struct angle_observer_state *obs,
			 float32_t sample_period_s,
			 float32_t bandwidth_hz,
			 uint32_t pole_pairs,
			 float32_t delay_samples)
{
	/* Zero all state */
	obs->angle_est_rad = 0.0f;
	obs->speed_est_rad_s = 0.0f;

	/* Zero all outputs */
	obs->mech_angle_rad = 0.0f;
	obs->elec_angle_rad = 0.0f;
	obs->mech_speed_rad_s = 0.0f;
	obs->mech_angle_pred_rad = 0.0f;
	obs->elec_angle_pred_rad = 0.0f;

	/* Cache configuration */
	obs->sample_period_s = sample_period_s;
	obs->bandwidth_hz = bandwidth_hz;
	obs->pole_pairs = pole_pairs;
	obs->delay_samples = delay_samples;

	/* Precalculate observer gains (critically damped tuning) */
	const float32_t wo = 2.0f * PI_F32 * bandwidth_hz;
	obs->L1Ts = 2.0f * wo * sample_period_s;  /* Position gain × Ts */
	obs->L2Ts = wo * wo * sample_period_s;    /* Velocity gain × Ts */

	/* Zero mechanical angle offset */
	obs->mech_angle_offset_rad = 0.0f;
}

void angle_observer_set_offset(struct angle_observer_state *obs,
			       float32_t offset_rad)
{
	obs->mech_angle_offset_rad = offset_rad;
}

void angle_observer_update(struct angle_observer_state *obs,
			   float32_t encoder_angle_rad)
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
	 * 
	 * Delay compensation (for pipelined SPI reads):
	 *   If encoder reading is N samples old, extrapolate forward:
	 *   θ_compensated = θ_encoder + N·Ts·ω̂
	 */

	/* Compensate for encoder measurement delay using current speed estimate */
	float32_t compensated_angle = encoder_angle_rad;
	if (obs->delay_samples > 0.0f) {
		const float32_t delay_compensation = obs->delay_samples * Ts * obs->speed_est_rad_s;
		compensated_angle = wrap_rad_2pi(encoder_angle_rad + delay_compensation);
	}

	/* Compute wrapped error in (-π, π] for shortest-path tracking */
	float32_t err_rad = wrap_rad_pi(compensated_angle - obs->angle_est_rad);

	/* Observer update (forward Euler discretization) */
	obs->angle_est_rad += Ts * obs->speed_est_rad_s + obs->L1Ts * err_rad;
	obs->speed_est_rad_s += obs->L2Ts * err_rad;

	/* Wrap angle estimate to [0, 2π) for easier debugging */
	obs->angle_est_rad = wrap_rad_2pi(obs->angle_est_rad);

	/* Current-cycle outputs (wrapped for readability) */
	obs->mech_angle_rad = obs->angle_est_rad;
	obs->mech_speed_rad_s = obs->speed_est_rad_s;

	/* Apply mechanical offset and compute electrical angle */
	float32_t mech_angle_offset = obs->mech_angle_rad + obs->mech_angle_offset_rad;
	obs->elec_angle_rad = wrap_rad_2pi(mech_angle_offset * obs->pole_pairs);

	/* One-step prediction for next control cycle */
	float32_t mech_angle_pred = obs->angle_est_rad + Ts * obs->speed_est_rad_s;
	obs->mech_angle_pred_rad = wrap_rad_2pi(mech_angle_pred);

	/* Apply offset to predicted mechanical angle for predicted electrical angle */
	float32_t mech_angle_pred_offset = obs->mech_angle_pred_rad + obs->mech_angle_offset_rad;
	obs->elec_angle_pred_rad = wrap_rad_2pi(mech_angle_pred_offset * obs->pole_pairs);
}
