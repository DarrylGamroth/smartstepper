/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/telemetry/thermal_model.h"
#include "motor/math/math_constants.h"
#include <math.h>

void thermal_model_init(struct thermal_model *model, float32_t R_th, float32_t C_th,
			float32_t T_ambient, float32_t update_freq)
{
	if (model == NULL) {
		return;
	}

	if (!isfinite(R_th) || R_th <= 0.0f || !isfinite(C_th) || C_th <= 0.0f ||
	    !isfinite(T_ambient) || !isfinite(update_freq) || update_freq <= 0.0f) {
		model->R_th = 0.0f;
		model->C_th = 0.0f;
		model->T_ambient = isfinite(T_ambient) ? T_ambient : 0.0f;
		model->T_winding = model->T_ambient;
		model->P_loss = 0.0f;
		model->dt = 0.0f;
		return;
	}

	model->R_th = R_th;
	model->C_th = C_th;
	model->T_ambient = T_ambient;
	model->T_winding = T_ambient;
	model->P_loss = 0.0f;
	model->dt = 1.0f / update_freq;
}

void thermal_model_update(struct thermal_model *model, float32_t Id, float32_t Iq, float32_t Rs)
{
	if (model == NULL) {
		return;
	}
	if (!isfinite(model->R_th) || model->R_th <= 0.0f ||
	    !isfinite(model->C_th) || model->C_th <= 0.0f ||
	    !isfinite(model->dt) || model->dt <= 0.0f ||
	    !isfinite(model->T_winding) || !isfinite(model->T_ambient) ||
	    !isfinite(Id) || !isfinite(Iq) || !isfinite(Rs)) {
		return;
	}

	/* Calculate I²R losses (both axes contribute) */
	float32_t I_sq = Id * Id + Iq * Iq;
	model->P_loss = I_sq * Rs;
	if (!isfinite(model->P_loss)) {
		return;
	}

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
	if (!isfinite(dT_dt)) {
		return;
	}

	float32_t next_temp = model->T_winding + model->dt * dT_dt;
	if (!isfinite(next_temp)) {
		return;
	}
	model->T_winding = next_temp;

	/* Sanity bounds (prevent unrealistic temperatures) */
	model->T_winding = clampf(model->T_winding, model->T_ambient, 200.0f);
}

void thermal_model_reset(struct thermal_model *model)
{
	if (model == NULL) {
		return;
	}

	model->T_winding = model->T_ambient;
	model->P_loss = 0.0f;
}
