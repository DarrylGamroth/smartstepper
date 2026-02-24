/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/ztest.h>

#include "motor/math/angle_wrap.h"
#include "motor/math/math_constants.h"
#include "motor_dob.h"
#include "motor/motion/motor_motion_modules.h"
#include "motor_mpr.h"
#include "motor/observers/motor_position_convert.h"

static void position_convert_update_sample(struct motor_position_convert_state *state,
					   const struct motor_position_convert_config *cfg,
					   bool sample_valid,
					   bool sample_fresh,
					   float32_t measurement_wrapped_rad)
{
	struct motor_position_convert_input input = {
		.sample_valid = sample_valid,
		.sample_fresh = sample_fresh,
		.source_generated = false,
		.warning = false,
		.error = false,
		.measurement_wrapped_rad = measurement_wrapped_rad,
		.latency_samples = 0.0f,
	};

	zassert_ok(motor_position_convert_update(state, cfg, &input), NULL);
}

ZTEST(motor_core_contracts, test_motion_segment_handoff_has_no_boundary_discontinuity)
{
	struct motion_profile_quintic profile = {0};
	float32_t measured_pos_rad = 0.30f;
	float32_t target_wrapped_rad = 0.0f;
	float32_t position_error_rad = 0.0f;
	float32_t velocity_ff_rad_s = 0.0f;
	bool completed = false;

	motion_profile_quintic_init(&profile, 0.001f);

	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
							      measured_pos_rad,
							      0.0f,
							      1.20f,
							      0.0f,
							      0.050f,
							      100.0f,
							      3000.0f),
		   NULL);

	for (int i = 0; i < 200; i++) {
		zassert_true(motor_position_move_resolve(&profile, true, measured_pos_rad,
							 &target_wrapped_rad,
							 &position_error_rad,
							 &velocity_ff_rad_s),
			     NULL);

		if (!motion_profile_quintic_is_active(&profile)) {
			completed = true;
			zassert_within(velocity_ff_rad_s, 0.0f, 1e-6f, NULL);
			break;
		}

		measured_pos_rad = target_wrapped_rad;
	}

	zassert_true(completed, NULL);

	float32_t first_segment_end_wrapped = target_wrapped_rad;
	float32_t first_segment_end_pos = profile.position_rad;
	float32_t first_segment_end_vel = profile.velocity_rad_s;

	zassert_ok(motor_position_move_plan_sequence_segment(&profile,
							      first_segment_end_pos,
							      first_segment_end_vel,
							      1.80f,
							      0.0f,
							      0.040f,
							      100.0f,
							      3000.0f),
		   NULL);

	zassert_true(motor_position_move_resolve(&profile, false, measured_pos_rad,
						 &target_wrapped_rad,
						 &position_error_rad,
						 &velocity_ff_rad_s),
		     NULL);
	zassert_within(wrap_rad_pi(target_wrapped_rad - first_segment_end_wrapped), 0.0f, 1e-5f,
		       NULL);

	zassert_true(motor_position_move_resolve(&profile, true, measured_pos_rad,
						 &target_wrapped_rad,
						 &position_error_rad,
						 &velocity_ff_rad_s),
		     NULL);
	zassert_true(wrap_rad_pi(target_wrapped_rad - first_segment_end_wrapped) >= -1e-5f, NULL);
}

ZTEST(motor_core_contracts, test_position_convert_mpr_dob_stale_transition_contract)
{
	const float32_t dt_s = 0.001f;
	const float32_t omega_true_rad_s = 20.0f;

	struct motor_position_convert_config pos_cfg = {
		.dt_s = dt_s,
		.velocity_lpf_hz = 500.0f,
		.accel_lpf_hz = 300.0f,
		.max_step_rad = 0.60f,
		.latency_samples_default = 0.0f,
		.jitter_threshold_rad = 0.30f,
		.stale_threshold_samples = 3U,
	};
	struct motor_position_convert_state pos_state = {0};

	struct motor_mpr_position_config pos_mpr_cfg = {
		.dt_s = dt_s,
		.horizon = 12U,
		.q_position = 8.0f,
		.q_velocity_ff = 0.2f,
		.r_delta_velocity = 0.2f,
		.velocity_limit_rad_s = 60.0f,
		.max_delta_velocity_rad_s = 2.0f,
	};
	struct motor_mpr_position_state pos_mpr_state = {0};

	struct motor_mpr_velocity_config vel_mpr_cfg = {
		.dt_s = dt_s,
		.horizon = 10U,
		.q_speed = 2.0f,
		.r_delta_iq = 0.08f,
		.iq_limit_a = 3.0f,
		.max_delta_iq_a = 0.20f,
		.disturbance_ki_nm_per_rad_s = 0.02f,
	};
	struct motor_mpr_velocity_model vel_model = {
		.inertia_kgm2 = 0.002f,
		.viscous_friction_nm_per_rad_s = 0.010f,
		.coulomb_friction_nm = 0.0f,
		.torque_constant_nm_per_a = 0.15f,
	};
	struct motor_mpr_velocity_state vel_mpr_state = {0};

	struct motor_dob_config dob_cfg = {
		.enabled = true,
		.dt_s = dt_s,
		.observer_gain_nm_per_rad_s = 0.05f,
		.torque_limit_nm = 0.8f,
		.iq_ff_limit_a = 1.5f,
	};
	struct motor_dob_model dob_model = {
		.inertia_kgm2 = vel_model.inertia_kgm2,
		.viscous_friction_nm_per_rad_s = vel_model.viscous_friction_nm_per_rad_s,
		.coulomb_friction_nm = vel_model.coulomb_friction_nm,
		.torque_constant_nm_per_a = vel_model.torque_constant_nm_per_a,
	};
	struct motor_dob_state dob_state = {0};

	float32_t angle_unwrapped_rad = 0.0f;
	float32_t velocity_cmd_rad_s = 0.0f;
	float32_t iq_cmd_a = 0.0f;
	float32_t iq_ff_a = 0.0f;
	const float32_t position_target_rad = 1.0f;

	motor_position_convert_init(&pos_state, &pos_cfg, 0.0f);

	/* Fresh phase: valid measurements and controller updates. */
	for (int i = 0; i < 160; i++) {
		angle_unwrapped_rad += omega_true_rad_s * dt_s;
		position_convert_update_sample(&pos_state, &pos_cfg, true, true,
					       wrap_rad_2pi(angle_unwrapped_rad));

		zassert_true((pos_state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_FRESH) != 0U, NULL);
		zassert_true((pos_state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) == 0U, NULL);

		float32_t position_error_rad =
			wrap_rad_pi(position_target_rad - pos_state.position_wrapped_rad);
		zassert_ok(motor_mpr_position_step(&pos_mpr_cfg, &pos_mpr_state, position_error_rad,
						   0.0f, &velocity_cmd_rad_s),
			   NULL);
		zassert_ok(motor_mpr_velocity_step(&vel_mpr_cfg, &vel_model, &vel_mpr_state,
						   pos_state.velocity_rad_s, velocity_cmd_rad_s,
						   &iq_cmd_a),
			   NULL);
		zassert_ok(motor_dob_step(&dob_cfg, &dob_model, &dob_state, pos_state.velocity_rad_s,
					  iq_cmd_a, &iq_ff_a),
			   NULL);

		zassert_true(isfinite(velocity_cmd_rad_s), NULL);
		zassert_true(isfinite(iq_cmd_a), NULL);
		zassert_true(isfinite(iq_ff_a), NULL);
		zassert_true(fabsf(iq_cmd_a) <= vel_mpr_cfg.iq_limit_a + 1e-5f, NULL);
		zassert_true(fabsf(iq_ff_a) <= dob_cfg.iq_ff_limit_a + 1e-5f, NULL);
	}

	/* Stale phase: sensor dropouts should latch stale while preserving finite controller IO. */
	for (int i = 0; i < 5; i++) {
		position_convert_update_sample(&pos_state, &pos_cfg, false, false, 0.0f);

		float32_t position_error_rad =
			wrap_rad_pi(position_target_rad - pos_state.position_wrapped_rad);
		zassert_ok(motor_mpr_position_step(&pos_mpr_cfg, &pos_mpr_state, position_error_rad,
						   0.0f, &velocity_cmd_rad_s),
			   NULL);
		zassert_ok(motor_mpr_velocity_step(&vel_mpr_cfg, &vel_model, &vel_mpr_state,
						   pos_state.velocity_rad_s, velocity_cmd_rad_s,
						   &iq_cmd_a),
			   NULL);
		zassert_ok(motor_dob_step(&dob_cfg, &dob_model, &dob_state, pos_state.velocity_rad_s,
					  iq_cmd_a, &iq_ff_a),
			   NULL);

		zassert_true(isfinite(pos_state.velocity_rad_s), NULL);
		zassert_true(isfinite(velocity_cmd_rad_s), NULL);
		zassert_true(isfinite(iq_cmd_a), NULL);
		zassert_true(isfinite(iq_ff_a), NULL);
	}

	zassert_true((pos_state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) != 0U, NULL);
	zassert_equal(pos_state.stale_event_count, 1U, NULL);

	/* Recovery phase: fresh samples clear stale flag and controller outputs remain valid. */
	for (int i = 0; i < 20; i++) {
		angle_unwrapped_rad += omega_true_rad_s * dt_s;
		position_convert_update_sample(&pos_state, &pos_cfg, true, true,
					       wrap_rad_2pi(angle_unwrapped_rad));

		float32_t position_error_rad =
			wrap_rad_pi(position_target_rad - pos_state.position_wrapped_rad);
		zassert_ok(motor_mpr_position_step(&pos_mpr_cfg, &pos_mpr_state, position_error_rad,
						   0.0f, &velocity_cmd_rad_s),
			   NULL);
		zassert_ok(motor_mpr_velocity_step(&vel_mpr_cfg, &vel_model, &vel_mpr_state,
						   pos_state.velocity_rad_s, velocity_cmd_rad_s,
						   &iq_cmd_a),
			   NULL);
		zassert_ok(motor_dob_step(&dob_cfg, &dob_model, &dob_state, pos_state.velocity_rad_s,
					  iq_cmd_a, &iq_ff_a),
			   NULL);
	}

	zassert_true((pos_state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_STALE) == 0U, NULL);
	zassert_true((pos_state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_FRESH) != 0U, NULL);
	zassert_true((pos_state.quality_flags & MOTOR_POSITION_CONVERT_QUALITY_VALID) != 0U, NULL);
}

ZTEST_SUITE(motor_core_contracts, NULL, NULL, NULL, NULL, NULL);
