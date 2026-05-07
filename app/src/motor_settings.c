/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_settings.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/atomic.h>

#include "config.h"
#include "motor/control/dob.h"
#include "motor/control/mpr.h"
#include "motor/compensation/detent_map.h"
#include "motor/filters/pi.h"
#include "motor/math/math_constants.h"
#include "motor/observers/angle_observer.h"
#include "motor/control/position_regulator.h"
#include "motor/control/velocity_regulator.h"
#include "motor_state_utils.h"

#define MOTOR_SETTINGS_ROOT "motor"
#define KEY_META_SCHEMA "motor/meta/schema_version"
#define KEY_META_GENERATION "motor/meta/generation"
#define KEY_META_VALID_GROUPS "motor/meta/valid_groups"

#define KEY_ENCODER_DIRECTION "motor/encoder/direction_sign"
#define KEY_ENCODER_OFFSET "motor/encoder/commutation_offset_mech_rad"
#define KEY_ENCODER_TRIM "motor/encoder/trim_elec_rad"
#define KEY_ENCODER_CORRELATION "motor/encoder/mapping_correlation"
#define KEY_ENCODER_RESIDUAL "motor/encoder/mapping_residual_rad"

#define KEY_MODEL_RS "motor/model/rs_ohm"
#define KEY_MODEL_LD "motor/model/ld_h"
#define KEY_MODEL_LQ "motor/model/lq_h"
#define KEY_MODEL_FLUX "motor/model/flux_linkage_wb"
#define KEY_MODEL_KT "motor/model/kt_nm_per_a"
#define KEY_MODEL_J "motor/model/inertia_kgm2"
#define KEY_MODEL_B "motor/model/viscous_friction_nm_per_rad_s"
#define KEY_MODEL_TC "motor/model/coulomb_friction_nm"

#define KEY_CTRL_VEL_KP "motor/controllers/velocity_kp_a_per_rad_s"
#define KEY_CTRL_VEL_KI "motor/controllers/velocity_ki_a_per_rad"
#define KEY_CTRL_VEL_LIMIT "motor/controllers/velocity_iq_limit_a"
#define KEY_CTRL_POS_KP "motor/controllers/position_kp_rad_s_per_rad"
#define KEY_CTRL_POS_KI "motor/controllers/position_ki_rad_s2_per_rad"
#define KEY_CTRL_VMPR_Q "motor/controllers/velocity_mpr_q_speed"
#define KEY_CTRL_VMPR_R "motor/controllers/velocity_mpr_r_delta_iq"
#define KEY_CTRL_VMPR_DIQ "motor/controllers/velocity_mpr_max_delta_iq_a"
#define KEY_CTRL_VMPR_DIST_KI "motor/controllers/velocity_mpr_disturbance_ki_nm_per_rad_s"
#define KEY_CTRL_VMPR_HORIZON "motor/controllers/velocity_mpr_horizon"
#define KEY_CTRL_PMPR_QP "motor/controllers/position_mpr_q_position"
#define KEY_CTRL_PMPR_QV "motor/controllers/position_mpr_q_velocity_ff"
#define KEY_CTRL_PMPR_R "motor/controllers/position_mpr_r_delta_velocity"
#define KEY_CTRL_PMPR_DVEL "motor/controllers/position_mpr_max_delta_velocity_rad_s"
#define KEY_CTRL_PMPR_HORIZON "motor/controllers/position_mpr_horizon"
#define KEY_CTRL_DOB_ENABLE "motor/controllers/velocity_dob_enabled"
#define KEY_CTRL_DOB_GAIN "motor/controllers/velocity_dob_observer_gain_nm_per_rad_s"
#define KEY_CTRL_DOB_TORQUE_LIMIT "motor/controllers/velocity_dob_torque_limit_nm"
#define KEY_CTRL_DOB_IQ_LIMIT "motor/controllers/velocity_dob_iq_ff_limit_a"

#define KEY_DETENT_ENABLE "motor/detent/enabled"
#define KEY_DETENT_BINS "motor/detent/bins"
#define KEY_DETENT_PHASE "motor/detent/phase_advance_bins"
#define KEY_DETENT_GAIN "motor/detent/gain"
#define KEY_DETENT_LIMIT "motor/detent/iq_ff_limit_a"
#define KEY_DETENT_TABLE_CRC "motor/detent/table_crc32"

struct motor_settings_read_ctx {
	struct motor_settings_snapshot *snapshot;
	uint32_t present_groups;
	uint32_t present_meta;
	int error;
};

static bool finite_positive(float32_t value)
{
	return isfinite(value) && value > 0.0f;
}

static bool finite_nonnegative(float32_t value)
{
	return isfinite(value) && value >= 0.0f;
}

static bool group_enabled(uint32_t groups, uint32_t group)
{
	return (groups & group) != 0U;
}

static int read_exact(settings_read_cb read_cb, void *cb_arg, void *dst, size_t expected)
{
	ssize_t rc = read_cb(cb_arg, dst, expected);

	if (rc < 0) {
		return (int)rc;
	}
	return (rc == (ssize_t)expected) ? 0 : -EINVAL;
}

#define LOAD_FIELD(key_lit, field, group_bit) \
	do { \
		if (strcmp(key, (key_lit)) == 0) { \
			ctx->error = read_exact(read_cb, cb_arg, &ctx->snapshot->field, sizeof(ctx->snapshot->field)); \
			if (ctx->error == 0) { \
				ctx->present_groups |= (group_bit); \
			} \
			return ctx->error == 0 ? 0 : 1; \
		} \
	} while (false)

static int settings_read_cb_direct(const char *key, size_t len,
				   settings_read_cb read_cb, void *cb_arg, void *param)
{
	ARG_UNUSED(len);
	struct motor_settings_read_ctx *ctx = param;

	if (ctx == NULL || ctx->snapshot == NULL) {
		return 1;
	}

	LOAD_FIELD("meta/schema_version", schema_version, 0U);
	LOAD_FIELD("meta/generation", generation, 0U);
	LOAD_FIELD("meta/valid_groups", valid_groups, 0U);
	LOAD_FIELD("encoder/direction_sign", encoder_direction_sign, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/commutation_offset_mech_rad", encoder_commutation_offset_mech_rad, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/trim_elec_rad", encoder_trim_elec_rad, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/mapping_correlation", encoder_mapping_correlation, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/mapping_residual_rad", encoder_mapping_residual_rad, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("model/rs_ohm", model_rs_ohm, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/ld_h", model_ld_h, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/lq_h", model_lq_h, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/flux_linkage_wb", model_flux_linkage_wb, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/kt_nm_per_a", model_kt_nm_per_a, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/inertia_kgm2", model_inertia_kgm2, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/viscous_friction_nm_per_rad_s", model_viscous_friction_nm_per_rad_s, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("model/coulomb_friction_nm", model_coulomb_friction_nm, MOTOR_SETTINGS_GROUP_MODEL);
	LOAD_FIELD("controllers/velocity_kp_a_per_rad_s", ctrl_velocity_kp_a_per_rad_s, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_ki_a_per_rad", ctrl_velocity_ki_a_per_rad, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_iq_limit_a", ctrl_velocity_iq_limit_a, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_kp_rad_s_per_rad", ctrl_position_kp_rad_s_per_rad, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_ki_rad_s2_per_rad", ctrl_position_ki_rad_s2_per_rad, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_mpr_q_speed", ctrl_velocity_mpr_q_speed, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_mpr_r_delta_iq", ctrl_velocity_mpr_r_delta_iq, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_mpr_max_delta_iq_a", ctrl_velocity_mpr_max_delta_iq_a, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_mpr_disturbance_ki_nm_per_rad_s", ctrl_velocity_mpr_disturbance_ki_nm_per_rad_s, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_mpr_horizon", ctrl_velocity_mpr_horizon, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_mpr_q_position", ctrl_position_mpr_q_position, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_mpr_q_velocity_ff", ctrl_position_mpr_q_velocity_ff, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_mpr_r_delta_velocity", ctrl_position_mpr_r_delta_velocity, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_mpr_max_delta_velocity_rad_s", ctrl_position_mpr_max_delta_velocity_rad_s, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/position_mpr_horizon", ctrl_position_mpr_horizon, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_dob_enabled", ctrl_velocity_dob_enabled, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_dob_observer_gain_nm_per_rad_s", ctrl_velocity_dob_observer_gain_nm_per_rad_s, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_dob_torque_limit_nm", ctrl_velocity_dob_torque_limit_nm, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("controllers/velocity_dob_iq_ff_limit_a", ctrl_velocity_dob_iq_ff_limit_a, MOTOR_SETTINGS_GROUP_CONTROLLERS);
	LOAD_FIELD("detent/enabled", detent_enabled, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/bins", detent_bins, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/phase_advance_bins", detent_phase_advance_bins, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/gain", detent_gain, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/iq_ff_limit_a", detent_iq_ff_limit_a, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/table_crc32", detent_table_crc32, MOTOR_SETTINGS_GROUP_DETENT);

	return 0;
}

#undef LOAD_FIELD

int motor_settings_read(struct motor_settings_snapshot *snapshot, uint32_t *present_groups)
{
	if (snapshot == NULL) {
		return -EINVAL;
	}

	memset(snapshot, 0, sizeof(*snapshot));
	struct motor_settings_read_ctx ctx = {
		.snapshot = snapshot,
	};
	int ret = settings_load_subtree_direct(MOTOR_SETTINGS_ROOT,
						 settings_read_cb_direct, &ctx);
	if (ret != 0) {
		return ret;
	}
	if (ctx.error != 0) {
		return ctx.error;
	}

	if (present_groups != NULL) {
		*present_groups = ctx.present_groups;
	}
	return 0;
}

static bool settings_mutation_allowed(const struct motor_parameters *params)
{
	return params != NULL &&
	       atomic_get(&params->control_armed) == 0 &&
	       !motor_state_ptr_is_online_control_state(params->state_for_isr) &&
	       !params->calibration.running;
}

static uint32_t next_generation(void)
{
	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;

	if (motor_settings_read(&snap, &present) == 0 &&
	    snap.schema_version == MOTOR_SETTINGS_SCHEMA_VERSION) {
		return snap.generation + 1U;
	}
	return 1U;
}

static int save_one(const char *key, const void *value, size_t len)
{
	return settings_save_one(key, value, len);
}

#define SAVE_SCALAR(key, value) \
	do { \
		ret = save_one((key), &(value), sizeof(value)); \
		if (ret != 0) { \
			return ret; \
		} \
	} while (false)

static int save_encoder_group(const struct motor_parameters *params)
{
	if (!params->calibration.encoder_mapping_complete) {
		return -ENOENT;
	}
	if (params->encoder_direction_sign != -1 && params->encoder_direction_sign != 1) {
		return -ERANGE;
	}
	if (!isfinite(params->observer_alignment_offset_rad) ||
	    !isfinite(params->observer_elec_trim_rad)) {
		return -ERANGE;
	}

	int ret;
	int8_t direction = params->encoder_direction_sign;
	float32_t correlation = 0.0f;
	float32_t residual = 0.0f;
	SAVE_SCALAR(KEY_ENCODER_DIRECTION, direction);
	SAVE_SCALAR(KEY_ENCODER_OFFSET, params->observer_alignment_offset_rad);
	SAVE_SCALAR(KEY_ENCODER_TRIM, params->observer_elec_trim_rad);
	SAVE_SCALAR(KEY_ENCODER_CORRELATION, correlation);
	SAVE_SCALAR(KEY_ENCODER_RESIDUAL, residual);
	return 0;
}

static int save_model_group(const struct motor_parameters *params)
{
	if (!finite_positive(params->Rs_measured_ohm) ||
	    !finite_positive(params->Ld_measured_H) ||
	    !finite_positive(params->Lq_measured_H) ||
	    !finite_positive(params->flux_linkage_wb_active) ||
	    !finite_positive(params->torque_gain_nm_per_a_active) ||
	    !finite_positive(params->inertia_kgm2_active) ||
	    !finite_nonnegative(params->viscous_friction_nm_per_rad_s_active) ||
	    !finite_nonnegative(params->coulomb_friction_nm_active)) {
		return -ERANGE;
	}

	int ret;
	SAVE_SCALAR(KEY_MODEL_RS, params->Rs_measured_ohm);
	SAVE_SCALAR(KEY_MODEL_LD, params->Ld_measured_H);
	SAVE_SCALAR(KEY_MODEL_LQ, params->Lq_measured_H);
	SAVE_SCALAR(KEY_MODEL_FLUX, params->flux_linkage_wb_active);
	SAVE_SCALAR(KEY_MODEL_KT, params->torque_gain_nm_per_a_active);
	SAVE_SCALAR(KEY_MODEL_J, params->inertia_kgm2_active);
	SAVE_SCALAR(KEY_MODEL_B, params->viscous_friction_nm_per_rad_s_active);
	SAVE_SCALAR(KEY_MODEL_TC, params->coulomb_friction_nm_active);
	return 0;
}

static int save_controllers_group(const struct motor_parameters *params)
{
	if (!finite_positive(params->velocity_cl_kp_A_per_rad_s) ||
	    !finite_positive(params->velocity_cl_ki_A_per_rad) ||
	    !finite_positive(params->velocity_cl_iq_limit_A) ||
	    !finite_positive(params->position_cl_kp_rad_s_per_rad) ||
	    !finite_nonnegative(params->position_cl_ki_rad_s2_per_rad) ||
	    !finite_positive(params->velocity_mpr_cfg.q_speed) ||
	    !finite_positive(params->velocity_mpr_cfg.r_delta_iq) ||
	    !finite_nonnegative(params->velocity_mpr_cfg.max_delta_iq_a) ||
	    !finite_nonnegative(params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s) ||
	    params->velocity_mpr_cfg.horizon == 0U ||
	    !finite_nonnegative(params->position_mpr_cfg.q_position) ||
	    !finite_nonnegative(params->position_mpr_cfg.q_velocity_ff) ||
	    !finite_positive(params->position_mpr_cfg.r_delta_velocity) ||
	    !finite_nonnegative(params->position_mpr_cfg.max_delta_velocity_rad_s) ||
	    params->position_mpr_cfg.horizon == 0U ||
	    !finite_nonnegative(params->velocity_dob_cfg.observer_gain_nm_per_rad_s) ||
	    !finite_positive(params->velocity_dob_cfg.torque_limit_nm) ||
	    !finite_nonnegative(params->velocity_dob_cfg.iq_ff_limit_a)) {
		return -ERANGE;
	}

	int ret;
	SAVE_SCALAR(KEY_CTRL_VEL_KP, params->velocity_cl_kp_A_per_rad_s);
	SAVE_SCALAR(KEY_CTRL_VEL_KI, params->velocity_cl_ki_A_per_rad);
	SAVE_SCALAR(KEY_CTRL_VEL_LIMIT, params->velocity_cl_iq_limit_A);
	SAVE_SCALAR(KEY_CTRL_POS_KP, params->position_cl_kp_rad_s_per_rad);
	SAVE_SCALAR(KEY_CTRL_POS_KI, params->position_cl_ki_rad_s2_per_rad);
	SAVE_SCALAR(KEY_CTRL_VMPR_Q, params->velocity_mpr_cfg.q_speed);
	SAVE_SCALAR(KEY_CTRL_VMPR_R, params->velocity_mpr_cfg.r_delta_iq);
	SAVE_SCALAR(KEY_CTRL_VMPR_DIQ, params->velocity_mpr_cfg.max_delta_iq_a);
	SAVE_SCALAR(KEY_CTRL_VMPR_DIST_KI, params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s);
	SAVE_SCALAR(KEY_CTRL_VMPR_HORIZON, params->velocity_mpr_cfg.horizon);
	SAVE_SCALAR(KEY_CTRL_PMPR_QP, params->position_mpr_cfg.q_position);
	SAVE_SCALAR(KEY_CTRL_PMPR_QV, params->position_mpr_cfg.q_velocity_ff);
	SAVE_SCALAR(KEY_CTRL_PMPR_R, params->position_mpr_cfg.r_delta_velocity);
	SAVE_SCALAR(KEY_CTRL_PMPR_DVEL, params->position_mpr_cfg.max_delta_velocity_rad_s);
	SAVE_SCALAR(KEY_CTRL_PMPR_HORIZON, params->position_mpr_cfg.horizon);
	SAVE_SCALAR(KEY_CTRL_DOB_ENABLE, params->velocity_dob_cfg.enabled);
	SAVE_SCALAR(KEY_CTRL_DOB_GAIN, params->velocity_dob_cfg.observer_gain_nm_per_rad_s);
	SAVE_SCALAR(KEY_CTRL_DOB_TORQUE_LIMIT, params->velocity_dob_cfg.torque_limit_nm);
	SAVE_SCALAR(KEY_CTRL_DOB_IQ_LIMIT, params->velocity_dob_cfg.iq_ff_limit_a);
	return 0;
}

static uint32_t detent_table_crc32(const struct motor_parameters *params)
{
	if (params == NULL || params->detent_map_cfg.table_iq_a == NULL ||
	    params->detent_map_cfg.table_len == 0U) {
		return 0U;
	}

	return crc32_ieee((const uint8_t *)params->detent_map_cfg.table_iq_a,
			   params->detent_map_cfg.table_len * sizeof(params->detent_map_cfg.table_iq_a[0]));
}

static int save_detent_group(const struct motor_parameters *params)
{
	if (params->detent_map_cfg.table_iq_a == NULL ||
	    params->detent_map_cfg.table_len == 0U ||
	    !finite_nonnegative(params->detent_map_cfg.gain) ||
	    !finite_nonnegative(params->detent_map_cfg.iq_ff_limit_a)) {
		return -ERANGE;
	}

	int ret;
	uint16_t bins = params->detent_map_cfg.table_len;
	uint32_t crc = detent_table_crc32(params);
	SAVE_SCALAR(KEY_DETENT_ENABLE, params->detent_map_cfg.enabled);
	SAVE_SCALAR(KEY_DETENT_BINS, bins);
	SAVE_SCALAR(KEY_DETENT_PHASE, params->detent_map_cfg.phase_advance_bins);
	SAVE_SCALAR(KEY_DETENT_GAIN, params->detent_map_cfg.gain);
	SAVE_SCALAR(KEY_DETENT_LIMIT, params->detent_map_cfg.iq_ff_limit_a);
	SAVE_SCALAR(KEY_DETENT_TABLE_CRC, crc);
	return 0;
}

int motor_settings_save(const struct motor_parameters *params, uint32_t groups,
			uint32_t *saved_groups)
{
	if (!settings_mutation_allowed(params) || groups == 0U) {
		return params == NULL ? -EINVAL : -EBUSY;
	}

	groups &= MOTOR_SETTINGS_GROUP_ALL;
	uint32_t written = 0U;
	int ret;

	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_ENCODER)) {
		ret = save_encoder_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_ENCODER;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_MODEL)) {
		ret = save_model_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_MODEL;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CONTROLLERS)) {
		ret = save_controllers_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_CONTROLLERS;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_DETENT)) {
		ret = save_detent_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_DETENT;
	}

	struct motor_settings_snapshot existing = {0};
	uint32_t present = 0U;
	uint32_t old_valid = 0U;
	if (motor_settings_read(&existing, &present) == 0 &&
	    existing.schema_version == MOTOR_SETTINGS_SCHEMA_VERSION) {
		old_valid = existing.valid_groups & MOTOR_SETTINGS_GROUP_ALL;
	}
	uint32_t schema = MOTOR_SETTINGS_SCHEMA_VERSION;
	uint32_t generation = next_generation();
	uint32_t valid_groups = (old_valid & ~groups) | written;
	SAVE_SCALAR(KEY_META_SCHEMA, schema);
	SAVE_SCALAR(KEY_META_GENERATION, generation);
	SAVE_SCALAR(KEY_META_VALID_GROUPS, valid_groups);

	if (saved_groups != NULL) {
		*saved_groups = written;
	}
	return 0;
}

#undef SAVE_SCALAR

static bool snapshot_group_available(const struct motor_settings_snapshot *snapshot,
					     uint32_t present_groups, uint32_t group)
{
	return snapshot != NULL &&
	       snapshot->schema_version == MOTOR_SETTINGS_SCHEMA_VERSION &&
	       group_enabled(snapshot->valid_groups, group) &&
	       group_enabled(present_groups, group);
}

static int apply_encoder_group(struct motor_parameters *params,
			       const struct motor_settings_snapshot *snapshot)
{
	if (snapshot->encoder_direction_sign != -1 && snapshot->encoder_direction_sign != 1) {
		return -ERANGE;
	}
	if (!isfinite(snapshot->encoder_commutation_offset_mech_rad) ||
	    !isfinite(snapshot->encoder_trim_elec_rad)) {
		return -ERANGE;
	}

	params->encoder_direction_sign = snapshot->encoder_direction_sign;
	params->observer_alignment_offset_rad = snapshot->encoder_commutation_offset_mech_rad;
	params->observer_elec_trim_rad = snapshot->encoder_trim_elec_rad;
	float32_t mech_trim_rad = snapshot->encoder_trim_elec_rad / (float32_t)MOTOR_POLE_PAIRS;
	angle_observer_set_offset(&params->observer,
				  params->observer_alignment_offset_rad + mech_trim_rad);
	angle_observer_reset_tracking(&params->observer, params->live.observer_mech_rad,
				      params->live.velocity_rad_s);
	params->calibration.encoder_mapping_complete = true;
	params->live.position_quality_flags = 0U;
	params->live.position_trust_state = MOTOR_FEEDBACK_TRUST_FAULT;
	params->live.position_stale_count = 0U;
	params->live.position_stale_events = 0U;
	params->live.position_glitch_count = 0U;
	params->live.position_jitter_count = 0U;
	return 0;
}

static int apply_model_group(struct motor_parameters *params,
			     const struct motor_settings_snapshot *snapshot)
{
	if (!finite_positive(snapshot->model_rs_ohm) ||
	    !finite_positive(snapshot->model_ld_h) ||
	    !finite_positive(snapshot->model_lq_h) ||
	    !finite_positive(snapshot->model_flux_linkage_wb) ||
	    !finite_positive(snapshot->model_kt_nm_per_a) ||
	    !finite_positive(snapshot->model_inertia_kgm2) ||
	    !finite_nonnegative(snapshot->model_viscous_friction_nm_per_rad_s) ||
	    !finite_nonnegative(snapshot->model_coulomb_friction_nm)) {
		return -ERANGE;
	}

	params->Rs_measured_ohm = snapshot->model_rs_ohm;
	params->Ld_measured_H = snapshot->model_ld_h;
	params->Lq_measured_H = snapshot->model_lq_h;
	params->Ls_measured_H = 0.5f * (params->Ld_measured_H + params->Lq_measured_H);
	params->R_over_L_measured = params->Rs_measured_ohm / params->Ls_measured_H;
	params->flux_linkage_wb_active = snapshot->model_flux_linkage_wb;
	params->torque_gain_nm_per_a_active = snapshot->model_kt_nm_per_a;
	params->inertia_kgm2_active = snapshot->model_inertia_kgm2;
	params->viscous_friction_nm_per_rad_s_active =
		snapshot->model_viscous_friction_nm_per_rad_s;
	params->coulomb_friction_nm_active = snapshot->model_coulomb_friction_nm;
	params->flux_model_source = MOTOR_MODEL_SOURCE_MEASURED;
	params->mech_model_source = MOTOR_MODEL_SOURCE_MEASURED;
	params->thermal.rs_ref_ohm = params->Rs_measured_ohm;
	return 0;
}

static int apply_controllers_group(struct motor_parameters *params,
				   const struct motor_settings_snapshot *snapshot)
{
	if (!finite_positive(snapshot->ctrl_velocity_kp_a_per_rad_s) ||
	    !finite_positive(snapshot->ctrl_velocity_ki_a_per_rad) ||
	    !finite_positive(snapshot->ctrl_velocity_iq_limit_a) ||
	    !finite_positive(snapshot->ctrl_position_kp_rad_s_per_rad) ||
	    !finite_nonnegative(snapshot->ctrl_position_ki_rad_s2_per_rad) ||
	    snapshot->ctrl_velocity_mpr_horizon == 0U ||
	    snapshot->ctrl_position_mpr_horizon == 0U) {
		return -ERANGE;
	}

	params->velocity_cl_kp_A_per_rad_s = snapshot->ctrl_velocity_kp_a_per_rad_s;
	params->velocity_cl_ki_A_per_rad = snapshot->ctrl_velocity_ki_a_per_rad;
	params->velocity_cl_iq_limit_A = snapshot->ctrl_velocity_iq_limit_a;
	params->velocity_cl_i_term_A = 0.0f;
	params->position_cl_kp_rad_s_per_rad = snapshot->ctrl_position_kp_rad_s_per_rad;
	params->position_cl_ki_rad_s2_per_rad = snapshot->ctrl_position_ki_rad_s2_per_rad;
	params->position_cl_i_term_rad_s = 0.0f;
	params->velocity_mpr_cfg.q_speed = snapshot->ctrl_velocity_mpr_q_speed;
	params->velocity_mpr_cfg.r_delta_iq = snapshot->ctrl_velocity_mpr_r_delta_iq;
	params->velocity_mpr_cfg.max_delta_iq_a = snapshot->ctrl_velocity_mpr_max_delta_iq_a;
	params->velocity_mpr_cfg.disturbance_ki_nm_per_rad_s =
		snapshot->ctrl_velocity_mpr_disturbance_ki_nm_per_rad_s;
	params->velocity_mpr_cfg.horizon = snapshot->ctrl_velocity_mpr_horizon;
	params->velocity_mpr_cfg.iq_limit_a = params->velocity_cl_iq_limit_A;
	params->position_mpr_cfg.q_position = snapshot->ctrl_position_mpr_q_position;
	params->position_mpr_cfg.q_velocity_ff = snapshot->ctrl_position_mpr_q_velocity_ff;
	params->position_mpr_cfg.r_delta_velocity = snapshot->ctrl_position_mpr_r_delta_velocity;
	params->position_mpr_cfg.max_delta_velocity_rad_s =
		snapshot->ctrl_position_mpr_max_delta_velocity_rad_s;
	params->position_mpr_cfg.horizon = snapshot->ctrl_position_mpr_horizon;
	params->velocity_dob_cfg.enabled = snapshot->ctrl_velocity_dob_enabled;
	params->velocity_dob_cfg.observer_gain_nm_per_rad_s =
		snapshot->ctrl_velocity_dob_observer_gain_nm_per_rad_s;
	params->velocity_dob_cfg.torque_limit_nm = snapshot->ctrl_velocity_dob_torque_limit_nm;
	params->velocity_dob_cfg.iq_ff_limit_a = snapshot->ctrl_velocity_dob_iq_ff_limit_a;

	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);
	motor_velocity_regulator_reset(&params->velocity_reg_state, params->live.velocity_rad_s);
	motor_position_regulator_reset(&params->position_reg_state, 0.0f);
	motor_mpr_velocity_reset(&params->velocity_mpr_state, params->live.velocity_rad_s, 0.0f);
	motor_mpr_position_reset(&params->position_mpr_state, params->live.velocity_ref_rad_s);
	motor_dob_reset(&params->velocity_dob_state, params->live.velocity_rad_s);
	params->live.velocity_dob_iq_ff_a = 0.0f;
	params->live.velocity_dob_disturbance_nm = 0.0f;
	params->live.velocity_dob_residual_rad_s = 0.0f;
	return 0;
}

static int apply_detent_group(struct motor_parameters *params,
			      const struct motor_settings_snapshot *snapshot)
{
	if (snapshot->detent_bins != params->detent_map_cfg.table_len ||
	    !finite_nonnegative(snapshot->detent_gain) ||
	    !finite_nonnegative(snapshot->detent_iq_ff_limit_a)) {
		return -ERANGE;
	}

	uint32_t current_crc = detent_table_crc32(params);
	params->detent_map_cfg.gain = snapshot->detent_gain;
	params->detent_map_cfg.iq_ff_limit_a = snapshot->detent_iq_ff_limit_a;
	params->detent_map_cfg.phase_advance_bins = snapshot->detent_phase_advance_bins;
	/* V1 stores only table metadata; do not enable unless the volatile table matches. */
	params->detent_map_cfg.enabled = snapshot->detent_enabled &&
				       snapshot->detent_table_crc32 != 0U &&
				       snapshot->detent_table_crc32 == current_crc;
	motor_detent_map_reset(&params->detent_map_state);
	params->live.detent_iq_ff_a = 0.0f;
	return 0;
}

int motor_settings_load(struct motor_parameters *params, uint32_t groups,
			uint32_t *loaded_groups)
{
	if (!settings_mutation_allowed(params) || groups == 0U) {
		return params == NULL ? -EINVAL : -EBUSY;
	}

	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;
	int ret = motor_settings_read(&snap, &present);
	if (ret != 0) {
		return ret;
	}
	if (snap.schema_version != MOTOR_SETTINGS_SCHEMA_VERSION) {
		return -ENOENT;
	}

	groups &= MOTOR_SETTINGS_GROUP_ALL;
	uint32_t loaded = 0U;
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_ENCODER)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_ENCODER)) {
			return -ENOENT;
		}
		ret = apply_encoder_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_ENCODER;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_MODEL)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_MODEL)) {
			return -ENOENT;
		}
		ret = apply_model_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_MODEL;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CONTROLLERS)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_CONTROLLERS)) {
			return -ENOENT;
		}
		ret = apply_controllers_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_CONTROLLERS;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_DETENT)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_DETENT)) {
			return -ENOENT;
		}
		ret = apply_detent_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_DETENT;
	}

	config_init_runtime_adapters(params);
	params->calibration.commissioning_complete =
		group_enabled(loaded, MOTOR_SETTINGS_GROUP_MODEL) ||
		params->calibration.commissioning_complete;
	if (loaded_groups != NULL) {
		*loaded_groups = loaded;
	}
	return 0;
}

static int delete_key(const char *key)
{
	int ret = settings_delete(key);
	return (ret == -ENOENT) ? 0 : ret;
}

static int clear_encoder_keys(void)
{
	int ret = delete_key(KEY_ENCODER_DIRECTION);
	if (ret != 0) { return ret; }
	ret = delete_key(KEY_ENCODER_OFFSET);
	if (ret != 0) { return ret; }
	ret = delete_key(KEY_ENCODER_TRIM);
	if (ret != 0) { return ret; }
	ret = delete_key(KEY_ENCODER_CORRELATION);
	if (ret != 0) { return ret; }
	return delete_key(KEY_ENCODER_RESIDUAL);
}

static int clear_model_keys(void)
{
	const char *keys[] = { KEY_MODEL_RS, KEY_MODEL_LD, KEY_MODEL_LQ, KEY_MODEL_FLUX,
		KEY_MODEL_KT, KEY_MODEL_J, KEY_MODEL_B, KEY_MODEL_TC };
	for (size_t i = 0U; i < ARRAY_SIZE(keys); i++) {
		int ret = delete_key(keys[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int clear_controller_keys(void)
{
	const char *keys[] = { KEY_CTRL_VEL_KP, KEY_CTRL_VEL_KI, KEY_CTRL_VEL_LIMIT,
		KEY_CTRL_POS_KP, KEY_CTRL_POS_KI, KEY_CTRL_VMPR_Q, KEY_CTRL_VMPR_R,
		KEY_CTRL_VMPR_DIQ, KEY_CTRL_VMPR_DIST_KI, KEY_CTRL_VMPR_HORIZON,
		KEY_CTRL_PMPR_QP, KEY_CTRL_PMPR_QV, KEY_CTRL_PMPR_R, KEY_CTRL_PMPR_DVEL,
		KEY_CTRL_PMPR_HORIZON, KEY_CTRL_DOB_ENABLE, KEY_CTRL_DOB_GAIN,
		KEY_CTRL_DOB_TORQUE_LIMIT, KEY_CTRL_DOB_IQ_LIMIT };
	for (size_t i = 0U; i < ARRAY_SIZE(keys); i++) {
		int ret = delete_key(keys[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int clear_detent_keys(void)
{
	const char *keys[] = { KEY_DETENT_ENABLE, KEY_DETENT_BINS, KEY_DETENT_PHASE,
		KEY_DETENT_GAIN, KEY_DETENT_LIMIT, KEY_DETENT_TABLE_CRC };
	for (size_t i = 0U; i < ARRAY_SIZE(keys); i++) {
		int ret = delete_key(keys[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

int motor_settings_clear(uint32_t groups)
{
	groups &= MOTOR_SETTINGS_GROUP_ALL;
	if (groups == 0U) {
		return -EINVAL;
	}

	struct motor_settings_snapshot existing = {0};
	uint32_t present = 0U;
	uint32_t old_valid = 0U;
	if (motor_settings_read(&existing, &present) == 0 &&
	    existing.schema_version == MOTOR_SETTINGS_SCHEMA_VERSION) {
		old_valid = existing.valid_groups & MOTOR_SETTINGS_GROUP_ALL;
	}

	int ret;
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_ENCODER)) {
		ret = clear_encoder_keys();
		if (ret != 0) { return ret; }
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_MODEL)) {
		ret = clear_model_keys();
		if (ret != 0) { return ret; }
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CONTROLLERS)) {
		ret = clear_controller_keys();
		if (ret != 0) { return ret; }
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_DETENT)) {
		ret = clear_detent_keys();
		if (ret != 0) { return ret; }
	}

	uint32_t schema = MOTOR_SETTINGS_SCHEMA_VERSION;
	uint32_t generation = next_generation();
	uint32_t valid_groups = old_valid & ~groups;
	ret = save_one(KEY_META_SCHEMA, &schema, sizeof(schema));
	if (ret != 0) { return ret; }
	ret = save_one(KEY_META_GENERATION, &generation, sizeof(generation));
	if (ret != 0) { return ret; }
	return save_one(KEY_META_VALID_GROUPS, &valid_groups, sizeof(valid_groups));
}

int motor_settings_clear_all(void)
{
	int ret = motor_settings_clear(MOTOR_SETTINGS_GROUP_ALL);
	if (ret != 0) {
		return ret;
	}
	ret = delete_key(KEY_META_SCHEMA);
	if (ret != 0) { return ret; }
	ret = delete_key(KEY_META_GENERATION);
	if (ret != 0) { return ret; }
	return delete_key(KEY_META_VALID_GROUPS);
}

bool motor_settings_autoload_enabled(void)
{
	return false;
}

const char *motor_settings_key_root(void)
{
	return MOTOR_SETTINGS_ROOT;
}
