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
#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"
#include "motor/observers/angle_observer.h"
#include "motor/control/position_regulator.h"
#include "motor/control/velocity_regulator.h"
#include "motor_state_utils.h"
#include "motor_torque.h"

#define MOTOR_SETTINGS_ROOT "motor"
#define KEY_META_SCHEMA "motor/meta/schema_version"
#define KEY_META_GENERATION "motor/meta/generation"
#define KEY_META_VALID_GROUPS "motor/meta/valid_groups"
#define KEY_AUTOLOAD_ENABLED "motor/autoload/enabled"
#define KEY_AUTOLOAD_GROUPS "motor/autoload/groups"

#define KEY_ENCODER_DIRECTION "motor/encoder/direction_sign"
#define KEY_ENCODER_OFFSET "motor/encoder/commutation_offset_mech_rad"
#define KEY_ENCODER_TRIM "motor/encoder/trim_elec_rad"
#define KEY_ENCODER_CORRELATION "motor/encoder/mapping_correlation"
#define KEY_ENCODER_RESIDUAL "motor/encoder/mapping_residual_rad"

#define KEY_IDENTITY_POLE_PAIRS "motor/identity/pole_pairs"

#define KEY_MODEL_ELEC_RS "motor/model/electrical/rs_ohm"
#define KEY_MODEL_ELEC_LD "motor/model/electrical/ld_h"
#define KEY_MODEL_ELEC_LQ "motor/model/electrical/lq_h"
#define KEY_MODEL_ELEC_FLUX "motor/model/electrical/flux_linkage_wb"

#define OLD_KEY_MODEL_RS "motor/model/rs_ohm"
#define OLD_KEY_MODEL_LD "motor/model/ld_h"
#define OLD_KEY_MODEL_LQ "motor/model/lq_h"
#define OLD_KEY_MODEL_FLUX "motor/model/flux_linkage_wb"
#define OLD_KEY_MODEL_KT "motor/model/kt_nm_per_a"
#define OLD_KEY_MODEL_J "motor/model/inertia_kgm2"
#define OLD_KEY_MODEL_B "motor/model/viscous_friction_nm_per_rad_s"
#define OLD_KEY_MODEL_TC "motor/model/coulomb_friction_nm"
#define OLD_KEY_MODEL_ELEC_KT "motor/model/electrical/kt_nm_per_a"

#define KEY_CTRL_OUTER_MODE "motor/controllers/outer_loop_mode"
#define KEY_CTRL_VEL_BW "motor/controllers/velocity_bandwidth_hz"
#define KEY_CTRL_POS_BW "motor/controllers/position_bandwidth_hz"
#define KEY_CTRL_DAMPING "motor/controllers/damping_ratio"
#define KEY_CTRL_VEL_LIMIT "motor/controllers/velocity_iq_limit_a"
#define KEY_CTRL_DOB_ENABLE "motor/controllers/velocity_dob_enabled"
#define KEY_CTRL_DOB_GAIN_SCALE "motor/controllers/velocity_dob_gain_scale"

#define OLD_KEY_CTRL_VEL_KP "motor/controllers/velocity_kp_a_per_rad_s"
#define OLD_KEY_CTRL_VEL_KI "motor/controllers/velocity_ki_a_per_rad"
#define OLD_KEY_CTRL_POS_KP "motor/controllers/position_kp_rad_s_per_rad"
#define OLD_KEY_CTRL_POS_KI "motor/controllers/position_ki_rad_s2_per_rad"
#define OLD_KEY_CTRL_VMPR_Q "motor/controllers/velocity_mpr_q_speed"
#define OLD_KEY_CTRL_VMPR_R "motor/controllers/velocity_mpr_r_delta_iq"
#define OLD_KEY_CTRL_VMPR_DIQ "motor/controllers/velocity_mpr_max_delta_iq_a"
#define OLD_KEY_CTRL_VMPR_DIST_KI "motor/controllers/velocity_mpr_disturbance_ki_nm_per_rad_s"
#define OLD_KEY_CTRL_VMPR_HORIZON "motor/controllers/velocity_mpr_horizon"
#define OLD_KEY_CTRL_PMPR_QP "motor/controllers/position_mpr_q_position"
#define OLD_KEY_CTRL_PMPR_QV "motor/controllers/position_mpr_q_velocity_ff"
#define OLD_KEY_CTRL_PMPR_R "motor/controllers/position_mpr_r_delta_velocity"
#define OLD_KEY_CTRL_PMPR_DVEL "motor/controllers/position_mpr_max_delta_velocity_rad_s"
#define OLD_KEY_CTRL_PMPR_HORIZON "motor/controllers/position_mpr_horizon"
#define OLD_KEY_CTRL_DOB_GAIN "motor/controllers/velocity_dob_observer_gain_nm_per_rad_s"
#define OLD_KEY_CTRL_DOB_TORQUE_LIMIT "motor/controllers/velocity_dob_torque_limit_nm"
#define OLD_KEY_CTRL_DOB_IQ_LIMIT "motor/controllers/velocity_dob_iq_ff_limit_a"

#define KEY_DETENT_ENABLE "motor/detent/enabled"
#define KEY_DETENT_BINS "motor/detent/bins"
#define KEY_DETENT_PHASE "motor/detent/phase_advance_bins"
#define KEY_DETENT_GAIN "motor/detent/gain"
#define KEY_DETENT_LIMIT "motor/detent/iq_ff_limit_a"
#define KEY_DETENT_TABLE_CRC "motor/detent/table_crc32"

#define KEY_CHOPPER_SLOTS "motor/chopper/slots"
#define KEY_CHOPPER_TEETH "motor/chopper/teeth"
#define KEY_CHOPPER_CENTER_COUNT "motor/chopper/center_count"
#define KEY_CHOPPER_CENTERS "motor/chopper/centers_rad"
#define KEY_CHOPPER_CENTER_KIND "motor/chopper/center_kind"

#define KEY_LIMITS_NOMINAL_VOLTAGE "motor/limits/nominal_voltage_v"
#define KEY_LIMITS_MAX_CURRENT "motor/limits/max_current_a"
#define KEY_LIMITS_BRAKE_CURRENT "motor/limits/brake_current_a"
#define KEY_LIMITS_MAX_VELOCITY "motor/limits/max_velocity_hz"
#define KEY_LIMITS_MAX_ACCEL "motor/limits/max_accel_hz_s"
#define KEY_LIMITS_COMMAND_TIMEOUT "motor/limits/command_timeout_ms"

#define MOTOR_SETTINGS_ZETA_MIN 0.2f
#define MOTOR_SETTINGS_ZETA_MAX 2.0f
#define MOTOR_SETTINGS_POSITION_TO_VELOCITY_BW_RATIO_MAX 0.20f
#define MOTOR_SETTINGS_VELOCITY_KI_TO_KP_MAX 2.0f
#define MOTOR_SETTINGS_CTRL_FIELD_OUTER_MODE BIT(0)
#define MOTOR_SETTINGS_CTRL_FIELD_VEL_BW BIT(1)
#define MOTOR_SETTINGS_CTRL_FIELD_POS_BW BIT(2)
#define MOTOR_SETTINGS_CTRL_FIELD_DAMPING BIT(3)
#define MOTOR_SETTINGS_CTRL_FIELD_VEL_LIMIT BIT(4)
#define MOTOR_SETTINGS_CTRL_FIELD_DOB_ENABLE BIT(5)
#define MOTOR_SETTINGS_CTRL_FIELD_DOB_GAIN_SCALE BIT(6)
#define MOTOR_SETTINGS_CTRL_FIELDS_ALL \
	(MOTOR_SETTINGS_CTRL_FIELD_OUTER_MODE | MOTOR_SETTINGS_CTRL_FIELD_VEL_BW | \
	 MOTOR_SETTINGS_CTRL_FIELD_POS_BW | MOTOR_SETTINGS_CTRL_FIELD_DAMPING | \
	 MOTOR_SETTINGS_CTRL_FIELD_VEL_LIMIT | MOTOR_SETTINGS_CTRL_FIELD_DOB_ENABLE | \
	 MOTOR_SETTINGS_CTRL_FIELD_DOB_GAIN_SCALE)
#define MOTOR_SETTINGS_IDENTITY_FIELD_POLE_PAIRS BIT(0)
#define MOTOR_SETTINGS_IDENTITY_FIELDS_ALL MOTOR_SETTINGS_IDENTITY_FIELD_POLE_PAIRS
#define MOTOR_SETTINGS_LIMIT_FIELD_NOMINAL_VOLTAGE BIT(0)
#define MOTOR_SETTINGS_LIMIT_FIELD_MAX_CURRENT BIT(1)
#define MOTOR_SETTINGS_LIMIT_FIELD_BRAKE_CURRENT BIT(2)
#define MOTOR_SETTINGS_LIMIT_FIELD_MAX_VELOCITY BIT(3)
#define MOTOR_SETTINGS_LIMIT_FIELD_MAX_ACCEL BIT(4)
#define MOTOR_SETTINGS_LIMIT_FIELD_COMMAND_TIMEOUT BIT(5)
#define MOTOR_SETTINGS_LIMIT_FIELDS_ALL \
	(MOTOR_SETTINGS_LIMIT_FIELD_NOMINAL_VOLTAGE | MOTOR_SETTINGS_LIMIT_FIELD_MAX_CURRENT | \
	 MOTOR_SETTINGS_LIMIT_FIELD_BRAKE_CURRENT | MOTOR_SETTINGS_LIMIT_FIELD_MAX_VELOCITY | \
	 MOTOR_SETTINGS_LIMIT_FIELD_MAX_ACCEL | MOTOR_SETTINGS_LIMIT_FIELD_COMMAND_TIMEOUT)
#define MOTOR_SETTINGS_MODEL_ELEC_FIELD_RS BIT(0)
#define MOTOR_SETTINGS_MODEL_ELEC_FIELD_LD BIT(1)
#define MOTOR_SETTINGS_MODEL_ELEC_FIELD_LQ BIT(2)
#define MOTOR_SETTINGS_MODEL_ELEC_FIELD_FLUX BIT(3)
#define MOTOR_SETTINGS_MODEL_ELEC_FIELDS_ALL \
	(MOTOR_SETTINGS_MODEL_ELEC_FIELD_RS | MOTOR_SETTINGS_MODEL_ELEC_FIELD_LD | \
	 MOTOR_SETTINGS_MODEL_ELEC_FIELD_LQ)
#define MOTOR_SETTINGS_CHOPPER_FIELD_SLOTS BIT(0)
#define MOTOR_SETTINGS_CHOPPER_FIELD_TEETH BIT(1)
#define MOTOR_SETTINGS_CHOPPER_FIELD_CENTER_COUNT BIT(2)
#define MOTOR_SETTINGS_CHOPPER_FIELD_CENTERS BIT(3)
#define MOTOR_SETTINGS_CHOPPER_FIELD_KIND BIT(4)
#define MOTOR_SETTINGS_CHOPPER_FIELDS_ALL \
	(MOTOR_SETTINGS_CHOPPER_FIELD_SLOTS | MOTOR_SETTINGS_CHOPPER_FIELD_TEETH | \
	 MOTOR_SETTINGS_CHOPPER_FIELD_CENTER_COUNT | MOTOR_SETTINGS_CHOPPER_FIELD_CENTERS | \
	 MOTOR_SETTINGS_CHOPPER_FIELD_KIND)
#define MOTOR_SETTINGS_AUTOLOAD_ALLOWED_GROUPS \
	(MOTOR_SETTINGS_GROUP_BASELINE | MOTOR_SETTINGS_GROUP_CHOPPER)

BUILD_ASSERT(MOTOR_SETTINGS_CHOPPER_MAX_CENTERS == MOTOR_PROFILE_SEQUENCE_MAX_POINTS,
	     "Settings chopper table size must match profile sequence size");

struct motor_settings_read_ctx {
	struct motor_settings_snapshot *snapshot;
	uint32_t present_groups;
	uint32_t identity_fields;
	uint32_t model_electrical_fields;
	uint32_t controller_fields;
	uint32_t limit_fields;
	uint32_t chopper_fields;
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

#define LOAD_CONTROLLER_FIELD(key_lit, field, field_bit) \
	do { \
		if (strcmp(key, (key_lit)) == 0) { \
			ctx->error = read_exact(read_cb, cb_arg, &ctx->snapshot->field, sizeof(ctx->snapshot->field)); \
			if (ctx->error == 0) { \
				ctx->present_groups |= MOTOR_SETTINGS_GROUP_CONTROLLERS; \
				ctx->controller_fields |= (field_bit); \
			} \
			return ctx->error == 0 ? 0 : 1; \
		} \
	} while (false)

#define LOAD_IDENTITY_FIELD(key_lit, field, field_bit) \
	do { \
		if (strcmp(key, (key_lit)) == 0) { \
			ctx->error = read_exact(read_cb, cb_arg, &ctx->snapshot->field, sizeof(ctx->snapshot->field)); \
			if (ctx->error == 0) { \
				ctx->present_groups |= MOTOR_SETTINGS_GROUP_IDENTITY; \
				ctx->identity_fields |= (field_bit); \
			} \
			return ctx->error == 0 ? 0 : 1; \
		} \
	} while (false)

#define LOAD_LIMIT_FIELD(key_lit, field, field_bit) \
	do { \
		if (strcmp(key, (key_lit)) == 0) { \
			ctx->error = read_exact(read_cb, cb_arg, &ctx->snapshot->field, sizeof(ctx->snapshot->field)); \
			if (ctx->error == 0) { \
				ctx->present_groups |= MOTOR_SETTINGS_GROUP_LIMITS; \
				ctx->limit_fields |= (field_bit); \
			} \
			return ctx->error == 0 ? 0 : 1; \
		} \
	} while (false)

#define LOAD_MODEL_ELECTRICAL_FIELD(key_lit, field, field_bit) \
	do { \
		if (strcmp(key, (key_lit)) == 0) { \
			ctx->error = read_exact(read_cb, cb_arg, &ctx->snapshot->field, sizeof(ctx->snapshot->field)); \
			if (ctx->error == 0) { \
				ctx->present_groups |= MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL; \
				ctx->model_electrical_fields |= (field_bit); \
			} \
			return ctx->error == 0 ? 0 : 1; \
		} \
	} while (false)

#define LOAD_CHOPPER_FIELD(key_lit, field, field_bit) \
	do { \
		if (strcmp(key, (key_lit)) == 0) { \
			ctx->error = read_exact(read_cb, cb_arg, &ctx->snapshot->field, sizeof(ctx->snapshot->field)); \
			if (ctx->error == 0) { \
				ctx->present_groups |= MOTOR_SETTINGS_GROUP_CHOPPER; \
				ctx->chopper_fields |= (field_bit); \
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
	LOAD_FIELD("autoload/enabled", autoload_enabled, 0U);
	LOAD_FIELD("autoload/groups", autoload_groups, 0U);
	LOAD_FIELD("encoder/direction_sign", encoder_direction_sign, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/commutation_offset_mech_rad", encoder_commutation_offset_mech_rad, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/trim_elec_rad", encoder_trim_elec_rad, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/mapping_correlation", encoder_mapping_correlation, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_FIELD("encoder/mapping_residual_rad", encoder_mapping_residual_rad, MOTOR_SETTINGS_GROUP_ENCODER);
	LOAD_IDENTITY_FIELD("identity/pole_pairs", identity_pole_pairs,
			    MOTOR_SETTINGS_IDENTITY_FIELD_POLE_PAIRS);
	LOAD_MODEL_ELECTRICAL_FIELD("model/electrical/rs_ohm", model_rs_ohm,
				    MOTOR_SETTINGS_MODEL_ELEC_FIELD_RS);
	LOAD_MODEL_ELECTRICAL_FIELD("model/electrical/ld_h", model_ld_h,
				    MOTOR_SETTINGS_MODEL_ELEC_FIELD_LD);
	LOAD_MODEL_ELECTRICAL_FIELD("model/electrical/lq_h", model_lq_h,
				    MOTOR_SETTINGS_MODEL_ELEC_FIELD_LQ);
	LOAD_MODEL_ELECTRICAL_FIELD("model/electrical/flux_linkage_wb",
				    model_flux_linkage_wb,
				    MOTOR_SETTINGS_MODEL_ELEC_FIELD_FLUX);
	LOAD_CONTROLLER_FIELD("controllers/outer_loop_mode", ctrl_outer_loop_mode,
			      MOTOR_SETTINGS_CTRL_FIELD_OUTER_MODE);
	LOAD_CONTROLLER_FIELD("controllers/velocity_bandwidth_hz", ctrl_velocity_bandwidth_hz,
			      MOTOR_SETTINGS_CTRL_FIELD_VEL_BW);
	LOAD_CONTROLLER_FIELD("controllers/position_bandwidth_hz", ctrl_position_bandwidth_hz,
			      MOTOR_SETTINGS_CTRL_FIELD_POS_BW);
	LOAD_CONTROLLER_FIELD("controllers/damping_ratio", ctrl_damping_ratio,
			      MOTOR_SETTINGS_CTRL_FIELD_DAMPING);
	LOAD_CONTROLLER_FIELD("controllers/velocity_iq_limit_a", ctrl_velocity_iq_limit_a,
			      MOTOR_SETTINGS_CTRL_FIELD_VEL_LIMIT);
	LOAD_CONTROLLER_FIELD("controllers/velocity_dob_enabled", ctrl_velocity_dob_enabled,
			      MOTOR_SETTINGS_CTRL_FIELD_DOB_ENABLE);
	LOAD_CONTROLLER_FIELD("controllers/velocity_dob_gain_scale", ctrl_velocity_dob_gain_scale,
			      MOTOR_SETTINGS_CTRL_FIELD_DOB_GAIN_SCALE);
	LOAD_FIELD("detent/enabled", detent_enabled, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/bins", detent_bins, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/phase_advance_bins", detent_phase_advance_bins, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/gain", detent_gain, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/iq_ff_limit_a", detent_iq_ff_limit_a, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_FIELD("detent/table_crc32", detent_table_crc32, MOTOR_SETTINGS_GROUP_DETENT);
	LOAD_CHOPPER_FIELD("chopper/slots", chopper_slots,
			   MOTOR_SETTINGS_CHOPPER_FIELD_SLOTS);
	LOAD_CHOPPER_FIELD("chopper/teeth", chopper_teeth,
			   MOTOR_SETTINGS_CHOPPER_FIELD_TEETH);
	LOAD_CHOPPER_FIELD("chopper/center_count", chopper_center_count,
			   MOTOR_SETTINGS_CHOPPER_FIELD_CENTER_COUNT);
	LOAD_CHOPPER_FIELD("chopper/centers_rad", chopper_centers_rad,
			   MOTOR_SETTINGS_CHOPPER_FIELD_CENTERS);
	LOAD_CHOPPER_FIELD("chopper/center_kind", chopper_center_kind,
			   MOTOR_SETTINGS_CHOPPER_FIELD_KIND);
	LOAD_LIMIT_FIELD("limits/nominal_voltage_v", limits_nominal_voltage_v,
			 MOTOR_SETTINGS_LIMIT_FIELD_NOMINAL_VOLTAGE);
	LOAD_LIMIT_FIELD("limits/max_current_a", limits_max_current_a,
			 MOTOR_SETTINGS_LIMIT_FIELD_MAX_CURRENT);
	LOAD_LIMIT_FIELD("limits/brake_current_a", limits_brake_current_a,
			 MOTOR_SETTINGS_LIMIT_FIELD_BRAKE_CURRENT);
	LOAD_LIMIT_FIELD("limits/max_velocity_hz", limits_max_velocity_hz,
			 MOTOR_SETTINGS_LIMIT_FIELD_MAX_VELOCITY);
	LOAD_LIMIT_FIELD("limits/max_accel_hz_s", limits_max_accel_hz_s,
			 MOTOR_SETTINGS_LIMIT_FIELD_MAX_ACCEL);
	LOAD_LIMIT_FIELD("limits/command_timeout_ms", limits_command_timeout_ms,
			 MOTOR_SETTINGS_LIMIT_FIELD_COMMAND_TIMEOUT);

	return 0;
}

#undef LOAD_FIELD
#undef LOAD_CONTROLLER_FIELD
#undef LOAD_IDENTITY_FIELD
#undef LOAD_LIMIT_FIELD
#undef LOAD_MODEL_ELECTRICAL_FIELD
#undef LOAD_CHOPPER_FIELD

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
	if ((ctx.identity_fields & MOTOR_SETTINGS_IDENTITY_FIELDS_ALL) !=
	    MOTOR_SETTINGS_IDENTITY_FIELDS_ALL) {
		ctx.present_groups &= ~MOTOR_SETTINGS_GROUP_IDENTITY;
	}
	if ((ctx.model_electrical_fields & MOTOR_SETTINGS_MODEL_ELEC_FIELDS_ALL) !=
	    MOTOR_SETTINGS_MODEL_ELEC_FIELDS_ALL) {
		ctx.present_groups &= ~MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL;
	}
	if ((ctx.controller_fields & MOTOR_SETTINGS_CTRL_FIELDS_ALL) !=
	    MOTOR_SETTINGS_CTRL_FIELDS_ALL) {
		ctx.present_groups &= ~MOTOR_SETTINGS_GROUP_CONTROLLERS;
	}
	if ((ctx.limit_fields & MOTOR_SETTINGS_LIMIT_FIELDS_ALL) !=
	    MOTOR_SETTINGS_LIMIT_FIELDS_ALL) {
		ctx.present_groups &= ~MOTOR_SETTINGS_GROUP_LIMITS;
	}
	if ((ctx.chopper_fields & MOTOR_SETTINGS_CHOPPER_FIELDS_ALL) !=
	    MOTOR_SETTINGS_CHOPPER_FIELDS_ALL) {
		ctx.present_groups &= ~MOTOR_SETTINGS_GROUP_CHOPPER;
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

static bool settings_groups_available(const struct motor_settings_snapshot *snapshot,
				      uint32_t present_groups, uint32_t groups)
{
	return snapshot != NULL &&
	       snapshot->schema_version == MOTOR_SETTINGS_SCHEMA_VERSION &&
	       (groups & ~MOTOR_SETTINGS_GROUP_ALL) == 0U &&
	       (snapshot->valid_groups & groups) == groups &&
	       (present_groups & groups) == groups;
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

static int delete_key(const char *key);

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

static int save_identity_group(const struct motor_parameters *params)
{
	ARG_UNUSED(params);

	if (MOTOR_POLE_PAIRS <= 0) {
		return -ERANGE;
	}

	int ret;
	uint16_t pole_pairs = (uint16_t)MOTOR_POLE_PAIRS;
	SAVE_SCALAR(KEY_IDENTITY_POLE_PAIRS, pole_pairs);
	return 0;
}

static int clear_legacy_model_keys(void)
{
	const char *keys[] = { OLD_KEY_MODEL_RS, OLD_KEY_MODEL_LD, OLD_KEY_MODEL_LQ,
		OLD_KEY_MODEL_FLUX, OLD_KEY_MODEL_KT, OLD_KEY_MODEL_J, OLD_KEY_MODEL_B,
		OLD_KEY_MODEL_TC, OLD_KEY_MODEL_ELEC_KT };

	for (size_t i = 0U; i < ARRAY_SIZE(keys); i++) {
		int ret = delete_key(keys[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int save_model_electrical_group(const struct motor_parameters *params)
{
	if (!finite_positive(params->Rs_measured_ohm) ||
	    !finite_positive(params->Ld_measured_H) ||
	    !finite_positive(params->Lq_measured_H)) {
		return -ERANGE;
	}

	int ret;
	SAVE_SCALAR(KEY_MODEL_ELEC_RS, params->Rs_measured_ohm);
	SAVE_SCALAR(KEY_MODEL_ELEC_LD, params->Ld_measured_H);
	SAVE_SCALAR(KEY_MODEL_ELEC_LQ, params->Lq_measured_H);
	if (finite_positive(params->flux_linkage_wb_active) &&
	    params->flux_model_source == MOTOR_MODEL_SOURCE_MEASURED) {
		SAVE_SCALAR(KEY_MODEL_ELEC_FLUX, params->flux_linkage_wb_active);
	} else {
		ret = delete_key(KEY_MODEL_ELEC_FLUX);
		if (ret != 0) {
			return ret;
		}
	}
	return clear_legacy_model_keys();
}

static int save_limits_group(const struct motor_parameters *params)
{
	if (!finite_positive(NOMINAL_VOLTAGE_V) ||
	    !finite_positive(MOTOR_MAX_CURRENT_A) ||
	    !finite_nonnegative(BRAKE_CURRENT_A) ||
	    !finite_positive(params->profile_max_velocity_rad_s) ||
	    !finite_positive(params->profile_max_accel_rad_s2)) {
		return -ERANGE;
	}

	int ret;
	float32_t nominal_voltage_v = NOMINAL_VOLTAGE_V;
	float32_t max_current_a = MOTOR_MAX_CURRENT_A;
	float32_t brake_current_a = BRAKE_CURRENT_A;
	float32_t max_velocity_hz = params->profile_max_velocity_rad_s / (2.0f * PI_F32);
	float32_t max_accel_hz_s = params->profile_max_accel_rad_s2 / (2.0f * PI_F32);
	uint32_t command_timeout_ms = params->command_timeout_ms;
	SAVE_SCALAR(KEY_LIMITS_NOMINAL_VOLTAGE, nominal_voltage_v);
	SAVE_SCALAR(KEY_LIMITS_MAX_CURRENT, max_current_a);
	SAVE_SCALAR(KEY_LIMITS_BRAKE_CURRENT, brake_current_a);
	SAVE_SCALAR(KEY_LIMITS_MAX_VELOCITY, max_velocity_hz);
	SAVE_SCALAR(KEY_LIMITS_MAX_ACCEL, max_accel_hz_s);
	SAVE_SCALAR(KEY_LIMITS_COMMAND_TIMEOUT, command_timeout_ms);
	return 0;
}

static float32_t torque_gain_active(const struct motor_parameters *params)
{
	if (params == NULL) {
		return 0.0f;
	}
	if (finite_positive(params->torque_gain_nm_per_a_active)) {
		return params->torque_gain_nm_per_a_active;
	}
	if (finite_positive(params->flux_linkage_wb_active)) {
		return motor_torque_gain_from_flux(params->flux_linkage_wb_active);
	}
	return motor_torque_gain_from_flux(MOTOR_FLUX_LINKAGE_WB);
}

static int estimate_velocity_bandwidth_hz(const struct motor_parameters *params,
					  float32_t *bw_hz_out)
{
	if (params == NULL || bw_hz_out == NULL) {
		return -EINVAL;
	}

	float32_t kt = torque_gain_active(params);
	float32_t j = params->inertia_kgm2_active;
	if (!finite_positive(kt) || !finite_positive(j)) {
		return -ERANGE;
	}

	if (params->outer_loop_mode == MOTOR_OUTER_LOOP_MODE_MPR &&
	    finite_positive(params->velocity_mpr_cfg.q_speed)) {
		float32_t iq_limit = params->velocity_mpr_cfg.iq_limit_a;
		if (!finite_positive(iq_limit)) {
			return -ERANGE;
		}
		*bw_hz_out = (params->velocity_mpr_cfg.q_speed * kt * iq_limit) /
			     (2.0f * PI_F32 * j * MOTOR_MPR_VELOCITY_BW_MODEL_SCALE);
		return finite_positive(*bw_hz_out) ? 0 : -ERANGE;
	}

	float32_t ki = params->velocity_cl_ki_A_per_rad;
	if (!finite_positive(ki)) {
		return -ERANGE;
	}

	float32_t omega = sqrtf((kt * ki) / j);
	*bw_hz_out = omega / (2.0f * PI_F32);
	return finite_positive(*bw_hz_out) ? 0 : -ERANGE;
}

static float32_t estimate_damping_ratio(const struct motor_parameters *params,
					float32_t velocity_bw_hz)
{
	float32_t kt = torque_gain_active(params);
	float32_t j = params->inertia_kgm2_active;
	float32_t b = params->viscous_friction_nm_per_rad_s_active;
	float32_t kp = params->velocity_cl_kp_A_per_rad_s;
	float32_t omega = 2.0f * PI_F32 * velocity_bw_hz;

	if (!finite_positive(kt) || !finite_positive(j) || !finite_positive(kp) ||
	    !finite_positive(omega)) {
		return 1.0f;
	}
	if (!finite_nonnegative(b)) {
		b = 0.0f;
	}

	float32_t zeta = ((kp * kt) + b) / (2.0f * omega * j);
	if (!isfinite(zeta)) {
		zeta = 1.0f;
	}
	return clampf(zeta, MOTOR_SETTINGS_ZETA_MIN, MOTOR_SETTINGS_ZETA_MAX);
}

static float32_t estimate_position_bandwidth_hz(const struct motor_parameters *params,
						float32_t zeta,
						float32_t velocity_bw_hz)
{
	float32_t bw_hz = 0.0f;

	if (finite_positive(params->position_cl_ki_rad_s2_per_rad)) {
		bw_hz = sqrtf(params->position_cl_ki_rad_s2_per_rad) / (2.0f * PI_F32);
	} else if (finite_positive(params->position_cl_kp_rad_s_per_rad) &&
		   finite_positive(zeta)) {
		bw_hz = params->position_cl_kp_rad_s_per_rad / (4.0f * PI_F32 * zeta);
	}

	if (!finite_positive(bw_hz)) {
		bw_hz = fmaxf(0.05f, velocity_bw_hz * 0.10f);
	}

	float32_t max_position_bw_hz =
		fmaxf(0.01f, velocity_bw_hz * MOTOR_SETTINGS_POSITION_TO_VELOCITY_BW_RATIO_MAX);
	return clampf(bw_hz, 0.01f, max_position_bw_hz);
}

static float32_t default_dob_gain_for_bandwidth(float32_t velocity_bw_hz)
{
	if (!finite_positive(velocity_bw_hz)) {
		velocity_bw_hz = 0.50f;
	}
	return clampf(0.01f + (0.001f * velocity_bw_hz), 0.01f, 0.05f);
}

static float32_t default_dob_iq_limit(float32_t velocity_iq_limit_a)
{
	if (!finite_positive(velocity_iq_limit_a)) {
		return 0.0f;
	}
	if (velocity_iq_limit_a < 0.05f) {
		return 0.50f * velocity_iq_limit_a;
	}
	return clampf(0.40f * velocity_iq_limit_a, 0.05f, velocity_iq_limit_a);
}

static int save_controllers_group(const struct motor_parameters *params)
{
	if (!finite_positive(params->velocity_cl_iq_limit_A) ||
	    params->outer_loop_mode > MOTOR_OUTER_LOOP_MODE_MPR) {
		return -ERANGE;
	}

	int ret;
	float32_t velocity_bw_hz = 0.0f;
	ret = estimate_velocity_bandwidth_hz(params, &velocity_bw_hz);
	if (ret != 0) {
		return ret;
	}
	float32_t damping = estimate_damping_ratio(params, velocity_bw_hz);
	float32_t position_bw_hz = estimate_position_bandwidth_hz(params, damping, velocity_bw_hz);
	float32_t default_dob_gain = default_dob_gain_for_bandwidth(velocity_bw_hz);
	float32_t dob_gain_scale = 1.0f;
	if (finite_positive(params->velocity_dob_cfg.observer_gain_nm_per_rad_s) &&
	    finite_positive(default_dob_gain)) {
		dob_gain_scale =
			params->velocity_dob_cfg.observer_gain_nm_per_rad_s / default_dob_gain;
	}
	if (!finite_positive(dob_gain_scale)) {
		dob_gain_scale = 1.0f;
	}
	dob_gain_scale = clampf(dob_gain_scale, 0.10f, 10.0f);
	uint8_t outer_loop_mode = params->outer_loop_mode;

	SAVE_SCALAR(KEY_CTRL_OUTER_MODE, outer_loop_mode);
	SAVE_SCALAR(KEY_CTRL_VEL_BW, velocity_bw_hz);
	SAVE_SCALAR(KEY_CTRL_POS_BW, position_bw_hz);
	SAVE_SCALAR(KEY_CTRL_DAMPING, damping);
	SAVE_SCALAR(KEY_CTRL_VEL_LIMIT, params->velocity_cl_iq_limit_A);
	SAVE_SCALAR(KEY_CTRL_DOB_ENABLE, params->velocity_dob_cfg.enabled);
	SAVE_SCALAR(KEY_CTRL_DOB_GAIN_SCALE, dob_gain_scale);
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

static int save_chopper_group(const struct motor_parameters *params)
{
	if (params->chopper_cal.slots == 0U ||
	    params->chopper_cal.teeth == 0U ||
	    params->chopper_cal.midpoint_count == 0U ||
	    params->chopper_cal.midpoint_count > MOTOR_SETTINGS_CHOPPER_MAX_CENTERS ||
	    params->chopper_cal.midpoint_count !=
		    (uint16_t)(params->chopper_cal.slots + params->chopper_cal.teeth) ||
	    !params->chopper_cal.valid) {
		return -ERANGE;
	}

	for (uint16_t i = 0U; i < params->chopper_cal.midpoint_count; i++) {
		if (!isfinite(params->chopper_cal.blade_midpoints_rad[i])) {
			return -ERANGE;
		}
	}

	int ret;
	uint16_t slots = params->chopper_cal.slots;
	uint16_t teeth = params->chopper_cal.teeth;
	uint16_t center_count = params->chopper_cal.midpoint_count;
	float32_t centers[MOTOR_SETTINGS_CHOPPER_MAX_CENTERS] = {0};
	uint8_t kinds[MOTOR_SETTINGS_CHOPPER_MAX_CENTERS] = {0};

	for (uint16_t i = 0U; i < center_count; i++) {
		centers[i] = params->chopper_cal.blade_midpoints_rad[i];
		kinds[i] = params->chopper_cal.midpoint_kind[i];
	}

	SAVE_SCALAR(KEY_CHOPPER_SLOTS, slots);
	SAVE_SCALAR(KEY_CHOPPER_TEETH, teeth);
	SAVE_SCALAR(KEY_CHOPPER_CENTER_COUNT, center_count);
	ret = save_one(KEY_CHOPPER_CENTERS, centers, sizeof(centers));
	if (ret != 0) {
		return ret;
	}
	return save_one(KEY_CHOPPER_CENTER_KIND, kinds, sizeof(kinds));
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
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_IDENTITY)) {
		ret = save_identity_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_IDENTITY;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL)) {
		ret = save_model_electrical_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CONTROLLERS)) {
		ret = save_controllers_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_CONTROLLERS;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_LIMITS)) {
		ret = save_limits_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_LIMITS;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_DETENT)) {
		ret = save_detent_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_DETENT;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CHOPPER)) {
		ret = save_chopper_group(params);
		if (ret != 0) {
			return ret;
		}
		written |= MOTOR_SETTINGS_GROUP_CHOPPER;
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
	return settings_groups_available(snapshot, present_groups, group);
}

static int apply_identity_group(struct motor_parameters *params,
				const struct motor_settings_snapshot *snapshot)
{
	ARG_UNUSED(params);

	if (snapshot->identity_pole_pairs == 0U) {
		return -ERANGE;
	}

	/*
	 * Pole pairs are persisted so a stored record describes the motor identity,
	 * but this firmware still uses MOTOR_POLE_PAIRS in several ISR/control
	 * paths. Reject mismatches instead of partially applying an inconsistent
	 * identity.
	 */
	return snapshot->identity_pole_pairs == (uint16_t)MOTOR_POLE_PAIRS ? 0 : -ENOTSUP;
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

static int apply_model_electrical_group(struct motor_parameters *params,
					const struct motor_settings_snapshot *snapshot)
{
	if (!finite_positive(snapshot->model_rs_ohm) ||
	    !finite_positive(snapshot->model_ld_h) ||
	    !finite_positive(snapshot->model_lq_h)) {
		return -ERANGE;
	}

	params->Rs_measured_ohm = snapshot->model_rs_ohm;
	params->Ld_measured_H = snapshot->model_ld_h;
	params->Lq_measured_H = snapshot->model_lq_h;
	params->Ls_measured_H = 0.5f * (params->Ld_measured_H + params->Lq_measured_H);
	params->R_over_L_measured = params->Rs_measured_ohm / params->Ls_measured_H;
	params->electrical_model_source = MOTOR_ELECTRICAL_MODEL_SOURCE_SETTINGS;
	if (finite_positive(snapshot->model_flux_linkage_wb)) {
		params->flux_linkage_wb_active = snapshot->model_flux_linkage_wb;
		params->torque_gain_nm_per_a_active =
			motor_torque_gain_from_flux(snapshot->model_flux_linkage_wb);
		params->flux_model_source = MOTOR_MODEL_SOURCE_MEASURED;
	} else {
		params->flux_linkage_wb_active = MOTOR_FLUX_LINKAGE_WB;
		params->torque_gain_nm_per_a_active =
			motor_torque_gain_from_flux(MOTOR_FLUX_LINKAGE_WB);
		params->flux_model_source = MOTOR_MODEL_SOURCE_FALLBACK;
	}
	params->thermal.rs_ref_ohm = params->Rs_measured_ohm;

	float32_t kp_d = params->Ld_measured_H * CURRENT_LOOP_BANDWIDTH_RPS;
	float32_t kp_q = params->Lq_measured_H * CURRENT_LOOP_BANDWIDTH_RPS;
	float32_t ki_d = (params->Rs_measured_ohm / params->Ld_measured_H) /
			 CONTROL_LOOP_FREQUENCY_HZ;
	float32_t ki_q = (params->Rs_measured_ohm / params->Lq_measured_H) /
			 CONTROL_LOOP_FREQUENCY_HZ;
	if (!finite_positive(kp_d) || !finite_positive(kp_q) ||
	    !finite_positive(ki_d) || !finite_positive(ki_q)) {
		return -ERANGE;
	}

	pi_set_gains(&params->pi_Id, kp_d, ki_d);
	pi_set_gains(&params->pi_Iq, kp_q, ki_q);
	pi_set_ui(&params->pi_Id, 0.0f);
	pi_set_ui(&params->pi_Iq, 0.0f);
	return 0;
}

static int apply_limits_group(struct motor_parameters *params,
			      const struct motor_settings_snapshot *snapshot)
{
	if (!finite_positive(snapshot->limits_nominal_voltage_v) ||
	    !finite_positive(snapshot->limits_max_current_a) ||
	    !finite_nonnegative(snapshot->limits_brake_current_a) ||
	    !finite_positive(snapshot->limits_max_velocity_hz) ||
	    !finite_positive(snapshot->limits_max_accel_hz_s)) {
		return -ERANGE;
	}

	/*
	 * These board/motor safety limits are currently devicetree defaults in
	 * multiple hot paths. Accept only values compatible with the image until
	 * those paths are converted to runtime limit fields.
	 */
	if (fabsf(snapshot->limits_nominal_voltage_v - NOMINAL_VOLTAGE_V) > 0.001f ||
	    snapshot->limits_max_current_a > (MOTOR_MAX_CURRENT_A + 0.001f) ||
	    snapshot->limits_brake_current_a > (MOTOR_MAX_CURRENT_A + 0.001f)) {
		return -ENOTSUP;
	}

	params->profile_max_velocity_rad_s = snapshot->limits_max_velocity_hz * 2.0f * PI_F32;
	params->profile_max_accel_rad_s2 = snapshot->limits_max_accel_hz_s * 2.0f * PI_F32;
	params->command_timeout_ms = snapshot->limits_command_timeout_ms;

	params->position_mpr_cfg.velocity_limit_rad_s = params->profile_max_velocity_rad_s;
	params->position_mpr_cfg.max_delta_velocity_rad_s =
		params->profile_max_accel_rad_s2 * params->position_mpr_cfg.dt_s;

	float32_t iq_limit = fminf(params->velocity_cl_iq_limit_A,
				  snapshot->limits_max_current_a);
	if (finite_positive(iq_limit)) {
		params->velocity_cl_iq_limit_A = iq_limit;
		params->velocity_mpr_cfg.iq_limit_a = iq_limit;
	}
	return 0;
}

static int apply_controllers_group(struct motor_parameters *params,
				   const struct motor_settings_snapshot *snapshot)
{
	if (snapshot->ctrl_outer_loop_mode > MOTOR_OUTER_LOOP_MODE_MPR ||
	    !finite_positive(snapshot->ctrl_velocity_bandwidth_hz) ||
	    !finite_positive(snapshot->ctrl_position_bandwidth_hz) ||
	    !finite_positive(snapshot->ctrl_damping_ratio) ||
	    snapshot->ctrl_damping_ratio < MOTOR_SETTINGS_ZETA_MIN ||
	    snapshot->ctrl_damping_ratio > MOTOR_SETTINGS_ZETA_MAX ||
	    !finite_positive(snapshot->ctrl_velocity_iq_limit_a) ||
	    !finite_positive(snapshot->ctrl_velocity_dob_gain_scale)) {
		return -ERANGE;
	}

	params->velocity_cl_iq_limit_A = snapshot->ctrl_velocity_iq_limit_a;
	params->outer_loop_mode = snapshot->ctrl_outer_loop_mode;

	float32_t kt = torque_gain_active(params);
	float32_t j = params->inertia_kgm2_active;
	float32_t b = params->viscous_friction_nm_per_rad_s_active;
	if (!finite_positive(kt) || !finite_positive(j)) {
		return -ERANGE;
	}
	if (!finite_nonnegative(b)) {
		b = 0.0f;
	}

	float32_t velocity_omega = 2.0f * PI_F32 * snapshot->ctrl_velocity_bandwidth_hz;
	float32_t kp_num = (2.0f * snapshot->ctrl_damping_ratio * velocity_omega * j) - b;
	float32_t kp_floor = (0.25f * velocity_omega * j) / kt;
	float32_t velocity_kp = fmaxf(kp_num / kt, kp_floor);
	float32_t velocity_ki_model = (velocity_omega * velocity_omega * j) / kt;
	float32_t velocity_ki =
		fminf(velocity_ki_model, MOTOR_SETTINGS_VELOCITY_KI_TO_KP_MAX * velocity_kp);
	if (!finite_positive(velocity_kp) || !finite_positive(velocity_ki)) {
		return -ERANGE;
	}
	params->velocity_cl_kp_A_per_rad_s = velocity_kp;
	params->velocity_cl_ki_A_per_rad = velocity_ki;
	params->velocity_cl_i_term_A = 0.0f;

	float32_t position_bw_hz = snapshot->ctrl_position_bandwidth_hz;
	float32_t max_position_bw_hz =
		fmaxf(0.01f, snapshot->ctrl_velocity_bandwidth_hz *
				     MOTOR_SETTINGS_POSITION_TO_VELOCITY_BW_RATIO_MAX);
	position_bw_hz = clampf(position_bw_hz, 0.01f, max_position_bw_hz);
	float32_t position_omega = 2.0f * PI_F32 * position_bw_hz;
	params->position_cl_kp_rad_s_per_rad =
		2.0f * snapshot->ctrl_damping_ratio * position_omega;
	params->position_cl_ki_rad_s2_per_rad = position_omega * position_omega;
	params->position_cl_i_term_rad_s = 0.0f;

	struct motor_mpr_bandwidth_result result = {0};
	struct motor_mpr_velocity_config velocity_mpr_cfg = params->velocity_mpr_cfg;
	const struct motor_mpr_velocity_bandwidth_input velocity_mpr_input = {
		.bandwidth_hz = snapshot->ctrl_velocity_bandwidth_hz,
		.inertia_kgm2 = j,
		.torque_constant_nm_per_a = kt,
		.iq_limit_a = params->velocity_cl_iq_limit_A,
		.dt_s = params->velocity_mpr_cfg.dt_s,
	};
	int ret = motor_mpr_velocity_config_from_bandwidth(&velocity_mpr_input,
							   &velocity_mpr_cfg, &result);
	if (ret != 0) {
		return ret;
	}
	params->velocity_mpr_cfg = velocity_mpr_cfg;

	struct motor_mpr_position_config position_mpr_cfg = params->position_mpr_cfg;
	const struct motor_mpr_position_bandwidth_input position_mpr_input = {
		.bandwidth_hz = position_bw_hz,
		.velocity_limit_rad_s = params->profile_max_velocity_rad_s,
		.accel_limit_rad_s2 = params->profile_max_accel_rad_s2,
		.dt_s = params->position_mpr_cfg.dt_s,
	};
	ret = motor_mpr_position_config_from_bandwidth(&position_mpr_input,
						       &position_mpr_cfg, &result);
	if (ret != 0) {
		return ret;
	}
	params->position_mpr_cfg = position_mpr_cfg;

	params->velocity_dob_cfg.enabled = snapshot->ctrl_velocity_dob_enabled;
	params->velocity_dob_cfg.observer_gain_nm_per_rad_s =
		default_dob_gain_for_bandwidth(snapshot->ctrl_velocity_bandwidth_hz) *
		clampf(snapshot->ctrl_velocity_dob_gain_scale, 0.10f, 10.0f);
	params->velocity_dob_cfg.iq_ff_limit_a =
		default_dob_iq_limit(params->velocity_cl_iq_limit_A);
	params->velocity_dob_cfg.torque_limit_nm =
		kt * params->velocity_dob_cfg.iq_ff_limit_a;

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
	/* Store only table metadata; do not enable unless the volatile table matches. */
	params->detent_map_cfg.enabled = snapshot->detent_enabled &&
				       snapshot->detent_table_crc32 != 0U &&
				       snapshot->detent_table_crc32 == current_crc;
	motor_detent_map_reset(&params->detent_map_state);
	params->live.detent_iq_ff_a = 0.0f;
	return 0;
}

static int apply_chopper_group(struct motor_parameters *params,
			       const struct motor_settings_snapshot *snapshot)
{
	uint32_t center_count = snapshot->chopper_center_count;
	if (snapshot->chopper_slots == 0U ||
	    snapshot->chopper_teeth == 0U ||
	    center_count == 0U ||
	    center_count > CHOPPER_CAL_MAX_SLOTS ||
	    center_count != ((uint32_t)snapshot->chopper_slots + snapshot->chopper_teeth)) {
		return -ERANGE;
	}

	for (uint16_t i = 0U; i < center_count; i++) {
		if (!isfinite(snapshot->chopper_centers_rad[i])) {
			return -ERANGE;
		}
	}

	params->chopper_cal.active = false;
	params->chopper_cal.complete = true;
	params->chopper_cal.valid = true;
	params->chopper_cal.slots = snapshot->chopper_slots;
	params->chopper_cal.teeth = snapshot->chopper_teeth;
	params->chopper_cal.revs_target = 0U;
	params->chopper_cal.samples_per_edge = 0U;
	params->chopper_cal.midpoint_count = snapshot->chopper_center_count;
	params->chopper_cal.total_edges_target = 0U;
	params->chopper_cal.total_edges_captured = 0U;
	params->chopper_cal.discarded_edges = 0U;
	params->chopper_cal.spacing_min_rad = 0.0f;
	params->chopper_cal.spacing_max_rad = 0.0f;
	params->chopper_cal.spacing_mean_rad = 0.0f;
	params->chopper_cal.spacing_max_error_rad = 0.0f;
	params->chopper_cal.speed_target_rad_s = 0.0f;
	params->chopper_cal.edge_min_step_rad = 0.0f;

	for (uint16_t i = 0U; i < CHOPPER_CAL_MAX_EDGES; i++) {
		params->chopper_cal.edge_sum_rad[i] = 0.0f;
		params->chopper_cal.edge_count[i] = 0U;
		params->chopper_cal.edge_status[i] = 0U;
	}
	for (uint16_t i = 0U; i < CHOPPER_CAL_MAX_SLOTS; i++) {
		params->chopper_cal.blade_midpoints_rad[i] = 0.0f;
		params->chopper_cal.midpoint_kind[i] = CHOPPER_REGION_KIND_UNKNOWN;
	}
	for (uint16_t i = 0U; i < center_count; i++) {
		params->chopper_cal.blade_midpoints_rad[i] =
			wrap_rad_2pi(snapshot->chopper_centers_rad[i]);
		params->chopper_cal.midpoint_kind[i] = snapshot->chopper_center_kind[i];
	}
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
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_IDENTITY)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_IDENTITY)) {
			return -ENOENT;
		}
		ret = apply_identity_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_IDENTITY;
	}
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
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL)) {
		if (!snapshot_group_available(&snap, present,
					      MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL)) {
			return -ENOENT;
		}
		ret = apply_model_electrical_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL;
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_LIMITS)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_LIMITS)) {
			return -ENOENT;
		}
		ret = apply_limits_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_LIMITS;
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
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CHOPPER)) {
		if (!snapshot_group_available(&snap, present, MOTOR_SETTINGS_GROUP_CHOPPER)) {
			return -ENOENT;
		}
		ret = apply_chopper_group(params, &snap);
		if (ret != 0) {
			return ret;
		}
		loaded |= MOTOR_SETTINGS_GROUP_CHOPPER;
	}

	config_init_runtime_adapters(params);
	params->calibration.commissioning_complete =
		group_enabled(loaded, MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL) ||
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

static int clear_identity_keys(void)
{
	return delete_key(KEY_IDENTITY_POLE_PAIRS);
}

static int clear_model_electrical_keys(void)
{
	const char *keys[] = { KEY_MODEL_ELEC_RS, KEY_MODEL_ELEC_LD, KEY_MODEL_ELEC_LQ,
		KEY_MODEL_ELEC_FLUX };
	for (size_t i = 0U; i < ARRAY_SIZE(keys); i++) {
		int ret = delete_key(keys[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return clear_legacy_model_keys();
}

static int clear_limits_keys(void)
{
	const char *keys[] = {
		KEY_LIMITS_NOMINAL_VOLTAGE, KEY_LIMITS_MAX_CURRENT,
		KEY_LIMITS_BRAKE_CURRENT, KEY_LIMITS_MAX_VELOCITY,
		KEY_LIMITS_MAX_ACCEL, KEY_LIMITS_COMMAND_TIMEOUT,
	};

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
	const char *keys[] = {
		KEY_CTRL_OUTER_MODE, KEY_CTRL_VEL_BW, KEY_CTRL_POS_BW,
		KEY_CTRL_DAMPING, KEY_CTRL_VEL_LIMIT, KEY_CTRL_DOB_ENABLE,
		KEY_CTRL_DOB_GAIN_SCALE,
		OLD_KEY_CTRL_VEL_KP, OLD_KEY_CTRL_VEL_KI, OLD_KEY_CTRL_POS_KP,
		OLD_KEY_CTRL_POS_KI, OLD_KEY_CTRL_VMPR_Q, OLD_KEY_CTRL_VMPR_R,
		OLD_KEY_CTRL_VMPR_DIQ, OLD_KEY_CTRL_VMPR_DIST_KI,
		OLD_KEY_CTRL_VMPR_HORIZON, OLD_KEY_CTRL_PMPR_QP,
		OLD_KEY_CTRL_PMPR_QV, OLD_KEY_CTRL_PMPR_R, OLD_KEY_CTRL_PMPR_DVEL,
		OLD_KEY_CTRL_PMPR_HORIZON, OLD_KEY_CTRL_DOB_GAIN,
		OLD_KEY_CTRL_DOB_TORQUE_LIMIT, OLD_KEY_CTRL_DOB_IQ_LIMIT,
	};
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

static int clear_chopper_keys(void)
{
	const char *keys[] = { KEY_CHOPPER_SLOTS, KEY_CHOPPER_TEETH,
		KEY_CHOPPER_CENTER_COUNT, KEY_CHOPPER_CENTERS, KEY_CHOPPER_CENTER_KIND };
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
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_IDENTITY)) {
		ret = clear_identity_keys();
		if (ret != 0) { return ret; }
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_MODEL_ELECTRICAL)) {
		ret = clear_model_electrical_keys();
		if (ret != 0) { return ret; }
	}
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_LIMITS)) {
		ret = clear_limits_keys();
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
	if (group_enabled(groups, MOTOR_SETTINGS_GROUP_CHOPPER)) {
		ret = clear_chopper_keys();
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
	ret = delete_key(KEY_META_VALID_GROUPS);
	if (ret != 0) { return ret; }
	ret = delete_key(KEY_AUTOLOAD_ENABLED);
	if (ret != 0) { return ret; }
	return delete_key(KEY_AUTOLOAD_GROUPS);
}

bool motor_settings_autoload_enabled(void)
{
	bool enabled = false;
	uint32_t groups = 0U;

	return motor_settings_autoload_read(&enabled, &groups) == 0 && enabled &&
	       groups != 0U;
}

int motor_settings_autoload_read(bool *enabled, uint32_t *groups)
{
	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;
	int ret = motor_settings_read(&snap, &present);
	if (ret != 0) {
		return ret;
	}

	uint32_t autoload_groups =
		snap.autoload_groups & MOTOR_SETTINGS_AUTOLOAD_ALLOWED_GROUPS;
	bool autoload_enabled = snap.autoload_enabled && autoload_groups != 0U &&
				settings_groups_available(&snap, present, autoload_groups);

	if (enabled != NULL) {
		*enabled = autoload_enabled;
	}
	if (groups != NULL) {
		*groups = autoload_enabled ? autoload_groups : 0U;
	}
	return 0;
}

int motor_settings_autoload_set(bool enabled, uint32_t groups)
{
	groups &= MOTOR_SETTINGS_GROUP_ALL;

	if (!enabled) {
		bool disabled = false;
		uint32_t no_groups = 0U;
		int ret = save_one(KEY_AUTOLOAD_ENABLED, &disabled, sizeof(disabled));
		if (ret != 0) {
			return ret;
		}
		return save_one(KEY_AUTOLOAD_GROUPS, &no_groups, sizeof(no_groups));
	}

	if (groups == 0U || (groups & ~MOTOR_SETTINGS_AUTOLOAD_ALLOWED_GROUPS) != 0U) {
		return -ENOTSUP;
	}

	struct motor_settings_snapshot snap = {0};
	uint32_t present = 0U;
	int ret = motor_settings_read(&snap, &present);
	if (ret != 0) {
		return ret;
	}
	if (!settings_groups_available(&snap, present, groups)) {
		return -ENOENT;
	}

	bool enabled_value = true;
	ret = save_one(KEY_AUTOLOAD_GROUPS, &groups, sizeof(groups));
	if (ret != 0) {
		return ret;
	}
	return save_one(KEY_AUTOLOAD_ENABLED, &enabled_value, sizeof(enabled_value));
}

int motor_settings_autoload_apply(struct motor_parameters *params,
				  uint32_t *loaded_groups)
{
	bool enabled = false;
	uint32_t groups = 0U;
	int ret = motor_settings_autoload_read(&enabled, &groups);
	if (ret != 0) {
		return ret;
	}
	if (!enabled || groups == 0U) {
		if (loaded_groups != NULL) {
			*loaded_groups = 0U;
		}
		return 0;
	}
	return motor_settings_load(params, groups, loaded_groups);
}

const char *motor_settings_key_root(void)
{
	return MOTOR_SETTINGS_ROOT;
}
