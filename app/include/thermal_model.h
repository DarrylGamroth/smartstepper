/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THERMAL_MODEL_H_
#define THERMAL_MODEL_H_

#include <zephyr/dsp/utils.h>
#include <stdbool.h>

/**
 * @file thermal_model.h
 * @brief First-order thermal model for motor winding temperature estimation
 *
 * Models motor winding temperature based on I²R losses with thermal
 * resistance and capacitance. Provides validation for RLS Rs estimates.
 */

/**
 * @brief First-order thermal model state
 *
 * Simple RC thermal model:
 *   dT/dt = (P_loss - (T_winding - T_ambient) / R_th) / C_th
 *
 * Where:
 *   P_loss = I²R (ohmic losses in winding)
 *   R_th = thermal resistance (°C/W)
 *   C_th = thermal capacitance (J/°C)
 */
struct thermal_model {
	/* Model parameters */
	float32_t R_th;         /* Thermal resistance (°C/W) */
	float32_t C_th;         /* Thermal capacitance (J/°C) */
	float32_t T_ambient;    /* Ambient temperature (°C) */
	
	/* State variables */
	float32_t T_winding;    /* Winding temperature (°C) */
	float32_t P_loss;       /* Current power loss (W) */
	
	/* Update rate */
	float32_t dt;           /* Time step (s) */
};

/**
 * @brief Initialize thermal model
 *
 * @param model Thermal model state
 * @param R_th Thermal resistance in °C/W
 * @param C_th Thermal capacitance in J/°C
 * @param T_ambient Ambient temperature in °C
 * @param update_freq Update frequency in Hz
 */
void thermal_model_init(struct thermal_model *model,
                        float32_t R_th,
                        float32_t C_th,
                        float32_t T_ambient,
                        float32_t update_freq);

/**
 * @brief Update thermal model with new current measurements
 *
 * @param model Thermal model state
 * @param Id D-axis current (A)
 * @param Iq Q-axis current (A)
 * @param Rs Winding resistance (ohms)
 */
void thermal_model_update(struct thermal_model *model,
                          float32_t Id,
                          float32_t Iq,
                          float32_t Rs);

/**
 * @brief Reset thermal model to ambient temperature
 *
 * @param model Thermal model state
 */
void thermal_model_reset(struct thermal_model *model);

/* Accessor functions */

static inline float32_t thermal_model_get_temperature(const struct thermal_model *model)
{
	return model->T_winding;
}

static inline float32_t thermal_model_get_power_loss(const struct thermal_model *model)
{
	return model->P_loss;
}

/**
 * @brief Convert Rs estimate to temperature using temperature coefficient
 *
 * @param Rs_measured Measured resistance at T_measured (ohms)
 * @param Rs_ref Reference resistance at T_ref (ohms)
 * @param T_ref Reference temperature (°C)
 * @param alpha Temperature coefficient (1/°C), typical 0.00393 for copper
 * @return Estimated temperature (°C)
 */
static inline float32_t thermal_Rs_to_temperature(float32_t Rs_measured,
                                                   float32_t Rs_ref,
                                                   float32_t T_ref,
                                                   float32_t alpha)
{
	/* Rs(T) = Rs_ref * (1 + alpha*(T - T_ref))
	 * Solving for T:
	 *   T = T_ref + (Rs/Rs_ref - 1) / alpha
	 */
	return T_ref + ((Rs_measured / Rs_ref) - 1.0f) / alpha;
}

#endif /* THERMAL_MODEL_H_ */
