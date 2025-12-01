/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHOPPER_PI_H_
#define CHOPPER_PI_H_

#include <zephyr/sys/util.h>
#include <zephyr/dsp/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PI controller object
 */
struct pi_f32 {
	/** Proportional gain for the PI controller */
	float32_t kp;
	/** Integral gain for the PI controller */
	float32_t ki;
	/** Integrator state value */
	float32_t ui;
	/** Reference input value */
	float32_t ref_value;
	/** Feedback input value */
	float32_t fback_value;
	/** Feedforward input value */
	float32_t ffwd_value;
	/** Minimum output value allowed for the PI controller */
	float32_t out_min;
	/** Maximum output value allowed for the PI controller */
	float32_t out_max;
};

/**
 * @brief Get the feedback value in the PI controller
 *
 * @param pi PI controller instance
 * @return Feedback value
 */
static inline float32_t pi_get_fback_value(const struct pi_f32 *pi)
{
	return pi->fback_value;
}

/**
 * @brief Get the feedforward value in the PI controller
 *
 * @param pi PI controller instance
 * @return Feedforward value
 */
static inline float32_t pi_get_ffwd_value(const struct pi_f32 *pi)
{
	return pi->ffwd_value;
}

/**
 * @brief Get the gains in the PI controller
 *
 * @param pi PI controller instance
 * @param kp Pointer to store proportional gain value
 * @param ki Pointer to store integral gain value
 */
static inline void pi_get_gains(const struct pi_f32 *pi, float32_t *kp, float32_t *ki)
{
	*kp = pi->kp;
	*ki = pi->ki;
}

/**
 * @brief Get the integral gain in the PI controller
 *
 * @param pi PI controller instance
 * @return Integral gain
 */
static inline float32_t pi_get_ki(const struct pi_f32 *pi)
{
	return pi->ki;
}

/**
 * @brief Get the proportional gain in the PI controller
 *
 * @param pi PI controller instance
 * @return Proportional gain
 */
static inline float32_t pi_get_kp(const struct pi_f32 *pi)
{
	return pi->kp;
}

/**
 * @brief Get the minimum and maximum output values allowed in the PI controller
 *
 * @param pi PI controller instance
 * @param out_min Pointer to store minimum output value
 * @param out_max Pointer to store maximum output value
 */
static inline void pi_get_min_fmaxf(const struct pi_f32 *pi, float32_t *out_min,
				  float32_t *out_max)
{
	*out_min = pi->out_min;
	*out_max = pi->out_max;
}

/**
 * @brief Get the maximum output value allowed in the PI controller
 *
 * @param pi PI controller instance
 * @return Maximum output value
 */
static inline float32_t pi_get_out_fmaxf(const struct pi_f32 *pi)
{
	return pi->out_max;
}

/**
 * @brief Get the minimum output value allowed in the PI controller
 *
 * @param pi PI controller instance
 * @return Minimum output value
 */
static inline float32_t pi_get_out_fminf(const struct pi_f32 *pi)
{
	return pi->out_min;
}

/**
 * @brief Get the reference value in the PI controller
 *
 * @param pi PI controller instance
 * @return Reference value
 */
static inline float32_t pi_get_ref_value(const struct pi_f32 *pi)
{
	return pi->ref_value;
}

/**
 * @brief Get the integrator state value in the PI controller
 *
 * @param pi PI controller instance
 * @return Integrator state value
 */
static inline float32_t pi_get_ui(const struct pi_f32 *pi)
{
	return pi->ui;
}

/**
 * @brief Initialize the PI controller
 *
 * @param pi PI controller instance to initialize
 */
static inline void pi_init(struct pi_f32 *pi)
{
	memset(pi, 0, sizeof(struct pi_f32));
}

/**
 * @brief Set the feedback value in the PI controller
 *
 * @param pi PI controller instance
 * @param fback_value Feedback value
 */
static inline void pi_set_fback_value(struct pi_f32 *pi, float32_t fback_value)
{
	pi->fback_value = fback_value;
}

/**
 * @brief Set the feedforward value in the PI controller
 *
 * @param pi PI controller instance
 * @param ffwd_value Feedforward value
 */
static inline void pi_set_ffwd_value(struct pi_f32 *pi, float32_t ffwd_value)
{
	pi->ffwd_value = ffwd_value;
}

/**
 * @brief Set the gains in the PI controller
 *
 * @param pi PI controller instance
 * @param kp Proportional gain
 * @param ki Integral gain
 */
static inline void pi_set_gains(struct pi_f32 *pi, float32_t kp, float32_t ki)
{
	pi->kp = kp;
	pi->ki = ki;
}

/**
 * @brief Set the integral gain in the PI controller
 *
 * @param pi PI controller instance
 * @param ki Integral gain
 */
static inline void pi_set_ki(struct pi_f32 *pi, float32_t ki)
{
	pi->ki = ki;
}

/**
 * @brief Set the proportional gain in the PI controller
 *
 * @param pi PI controller instance
 * @param kp Proportional gain
 */
static inline void pi_set_kp(struct pi_f32 *pi, float32_t kp)
{
	pi->kp = kp;
}

/**
 * @brief Set the minimum and maximum output values allowed in the PI controller
 *
 * @param pi PI controller instance
 * @param out_min Minimum output value
 * @param out_max Maximum output value
 */
static inline void pi_set_min_fmaxf(struct pi_f32 *pi, float32_t out_min, float32_t out_max)
{
	pi->out_min = out_min;
	pi->out_max = out_max;
}

/**
 * @brief Set the maximum output value allowed in the PI controller
 *
 * @param pi PI controller instance
 * @param out_max Maximum output value
 */
static inline void pi_set_out_fmaxf(struct pi_f32 *pi, float32_t out_max)
{
	pi->out_max = out_max;
}

/**
 * @brief Set the minimum output value allowed in the PI controller
 *
 * @param pi PI controller instance
 * @param out_min Minimum output value
 */
static inline void pi_set_out_fminf(struct pi_f32 *pi, float32_t out_min)
{
	pi->out_min = out_min;
}

/**
 * @brief Set the reference value in the PI controller
 *
 * @param pi PI controller instance
 * @param ref_value Reference value
 */
static inline void pi_set_ref_value(struct pi_f32 *pi, float32_t ref_value)
{
	pi->ref_value = ref_value;
}

/**
 * @brief Set the integrator state value in the PI controller
 *
 * @param pi PI controller instance
 * @param ui Integrator state value
 */
static inline void pi_set_ui(struct pi_f32 *pi, float32_t ui)
{
	pi->ui = ui;
}

/**
 * @brief Run the parallel form of the PI controller
 *
 * In parallel form: output = Kp*error + Ki*∫error + feedforward
 * Uses dynamic integrator clamping to prevent windup while allowing
 * the integrator to continue accumulating in the direction that helps
 * bring the output back into range.
 *
 * @param pi PI controller instance
 * @param ref_value Reference value to the controller
 * @param fback_value Feedback value to the controller
 * @param ffwd_value Feedforward value to the controller
 * @param out_value Pointer to store controller output value
 */
static inline void pi_run_parallel(struct pi_f32 *pi, float32_t ref_value, float32_t fback_value,
				   float32_t ffwd_value, float32_t *out_value)
{
	float32_t error;
	float32_t up;
	float32_t p_out;
	float32_t ui_min;
	float32_t ui_max;

	error = ref_value - fback_value;
	up = pi->kp * error;
	p_out = up + ffwd_value;

	/* Dynamic integrator clamping:
	 * Imax = fmaxf(H - Pout, 0)
	 * Imin = fminf(L - Pout, 0)
	 * This allows the integrator to accumulate in the direction that
	 * helps bring the output back into range, preventing windup.
	 */
	ui_max = fmaxf(pi->out_max - p_out, 0.0f);
	ui_min = fminf(pi->out_min - p_out, 0.0f);

	pi->ui = CLAMP(pi->ui + (pi->ki * error), ui_min, ui_max);

	pi->ref_value = ref_value;
	pi->fback_value = fback_value;
	pi->ffwd_value = ffwd_value;

	*out_value = CLAMP(p_out + pi->ui, pi->out_min, pi->out_max);
}

/**
 * @brief Run the series form of the PI controller
 *
 * In series form: output = Kp*error + Ki*∫(Kp*error) + feedforward
 * Uses dynamic integrator clamping to prevent windup while allowing
 * the integrator to continue accumulating in the direction that helps
 * bring the output back into range.
 *
 * @param pi PI controller instance
 * @param ref_value Reference value to the controller
 * @param fback_value Feedback value to the controller
 * @param ffwd_value Feedforward value to the controller
 * @param out_value Pointer to store controller output value
 */
static inline void pi_run_series(struct pi_f32 *pi, float32_t ref_value, float32_t fback_value,
				 float32_t ffwd_value, float32_t *out_value)
{
	float32_t error;
	float32_t up;
	float32_t p_out;
	float32_t ui_min;
	float32_t ui_max;

	error = ref_value - fback_value;
	up = pi->kp * error;
	p_out = up + ffwd_value;

	/* Dynamic integrator clamping:
	 * Imax = fmaxf(H - Pout, 0)
	 * Imin = fminf(L - Pout, 0)
	 * This allows the integrator to accumulate in the direction that
	 * helps bring the output back into range, preventing windup.
	 */
	ui_max = fmaxf(pi->out_max - p_out, 0.0f);
	ui_min = fminf(pi->out_min - p_out, 0.0f);

	pi->ui = CLAMP(pi->ui + (pi->ki * up), ui_min, ui_max);

	pi->ref_value = ref_value;
	pi->fback_value = fback_value;
	pi->ffwd_value = ffwd_value;

	*out_value = CLAMP(p_out + pi->ui, pi->out_min, pi->out_max);
}

/**
 * @brief Run the PI controller (series form without feedforward)
 *
 * Basic PI controller: output = Kp*error + Ki*∫(Kp*error)
 * Uses dynamic integrator clamping to prevent windup while allowing
 * the integrator to continue accumulating in the direction that helps
 * bring the output back into range.
 *
 * @param pi PI controller instance
 * @param ref_value Reference value to the controller
 * @param fback_value Feedback value to the controller
 * @param out_value Pointer to store controller output value
 */
static inline void pi_run(struct pi_f32 *pi, float32_t ref_value, float32_t fback_value,
			  float32_t *out_value)
{
	float32_t error;
	float32_t up;
	float32_t ui_min;
	float32_t ui_max;

	error = ref_value - fback_value;
	up = pi->kp * error;

	/* Dynamic integrator clamping:
	 * Imax = fmaxf(H - Pout, 0)
	 * Imin = fminf(L - Pout, 0)
	 * This allows the integrator to accumulate in the direction that
	 * helps bring the output back into range, preventing windup.
	 */
	ui_max = fmaxf(pi->out_max - up, 0.0f);
	ui_min = fminf(pi->out_min - up, 0.0f);

	pi->ui = CLAMP(pi->ui + (pi->ki * up), ui_min, ui_max);

	*out_value = CLAMP(up + pi->ui, pi->out_min, pi->out_max);
}

#ifdef __cplusplus
}
#endif

#endif /* CHOPPER_PI_H_ */
