/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "thermal_model.h"
#include "math_constants.h"
#include <math.h>

void thermal_model_init(struct thermal_model *model, float32_t R_th, float32_t C_th,
			float32_t T_ambient, float32_t update_freq)
{
	model->R_th = R_th;
	model->C_th = C_th;
	model->T_ambient = T_ambient;
	model->T_winding = T_ambient;
	model->P_loss = 0.0f;
	model->dt = 1.0f / update_freq;
}

void thermal_model_update(struct thermal_model *model, float32_t Id, float32_t Iq, float32_t Rs)
{
	/* Calculate I²R losses (both axes contribute) */
	float32_t I_sq = Id * Id + Iq * Iq;
	model->P_loss = I_sq * Rs;

	/* First-order thermal model: dT/dt = (P_loss - (T - T_amb)/R_th) / C_th
	 *
	 * Heat flow:
	 *   - Heat in: P_loss (from copper losses)
	 *   - Heat out: (T_winding - T_ambient) / R_th
	 *   - Stored energy: C_th * dT
	 *
	 * Forward Euler integration (suitable for slow thermal dynamics):
	 *   T[k+1] = T[k] + dt * dT/dt
	 */
	float32_t heat_dissipation = (model->T_winding - model->T_ambient) / model->R_th;
	float32_t dT_dt = (model->P_loss - heat_dissipation) / model->C_th;

	model->T_winding += model->dt * dT_dt;

	/* Sanity bounds (prevent unrealistic temperatures) */
	model->T_winding = clampf(model->T_winding, model->T_ambient, 200.0f);
}

void thermal_model_reset(struct thermal_model *model)
{
	model->T_winding = model->T_ambient;
	model->P_loss = 0.0f;
}
