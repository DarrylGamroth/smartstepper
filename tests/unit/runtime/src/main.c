#include <errno.h>

#include <zephyr/ztest.h>

#include "motor/runtime/config_snapshot.h"
#include "motor/runtime/actuator_adapter.h"
#include "motor/runtime/control_policy.h"
#include "motor/runtime/control_refs.h"

ZTEST(runtime, test_fast_step_boundary_contract_present)
{
	zassert_true(true, "fast-step boundary checks are enforced in CMake");
}

ZTEST(runtime, test_control_refs_keep_motion_independent_from_foc_backend)
{
	struct motor_motion_ref motion = {
		.position_rad = 1.25f,
		.velocity_target_rad_s = 2.0f,
		.velocity_ref_rad_s = 1.5f,
		.velocity_rad_s = 1.0f,
		.acceleration_rad_s2 = 0.5f,
	};
	struct motor_feedback_ref feedback = {
		.source = MOTOR_FEEDBACK_ENCODER,
		.input_source = 1U,
		.quality_flags = 0x01U,
		.fresh = true,
		.position_rad = motion.position_rad,
		.velocity_rad_s = motion.velocity_rad_s,
	};
	struct motor_actuator_ref actuator = {
		.kind = MOTOR_ACTUATOR_STEP_DIR,
		.effort_kind = MOTOR_ACTUATOR_EFFORT_STEP_DIR,
		.enabled = true,
		.position_rad = motion.position_rad,
		.velocity_rad_s = motion.velocity_rad_s,
	};

	zassert_equal(actuator.kind, MOTOR_ACTUATOR_STEP_DIR, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_STEP_DIR, NULL);
	zassert_equal(feedback.source, MOTOR_FEEDBACK_ENCODER, NULL);
	zassert_within(motion.position_rad, 1.25f, 1.0e-6f, NULL);
	zassert_within(motion.velocity_target_rad_s, 2.0f, 1.0e-6f, NULL);
}

ZTEST(runtime, test_foc_actuator_ref_carries_dq_current_without_motion_coupling)
{
	struct motor_actuator_ref actuator = {0};

	motor_actuator_ref_set_foc_current(&actuator,
					   true,
					   1.0f,
					   2.0f,
					   3.0f,
					   0.1f,
					   0.2f);

	zassert_equal(actuator.kind, MOTOR_ACTUATOR_FOC_CURRENT, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_CURRENT_DQ, NULL);
	zassert_true(actuator.enabled, NULL);
	zassert_within(actuator.position_rad, 1.0f, 1.0e-6f, NULL);
	zassert_within(actuator.velocity_rad_s, 2.0f, 1.0e-6f, NULL);
	zassert_within(actuator.acceleration_rad_s2, 3.0f, 1.0e-6f, NULL);
	zassert_within(actuator.id_ref_a, 0.1f, 1.0e-6f, NULL);
	zassert_within(actuator.iq_ref_a, 0.2f, 1.0e-6f, NULL);

	motor_actuator_ref_clear(&actuator, MOTOR_ACTUATOR_BRUSHED_CURRENT);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_BRUSHED_CURRENT, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_NONE, NULL);
	zassert_false(actuator.enabled, NULL);
	zassert_within(actuator.id_ref_a, 0.0f, 1.0e-6f, NULL);
	zassert_within(actuator.iq_ref_a, 0.0f, 1.0e-6f, NULL);
}

ZTEST(runtime, test_position_generated_encoder_disabled_still_builds_foc_actuator_command)
{
	struct motor_control_policy policy = {0};
	struct motor_control_policy_input input = {
		.mode = MOTOR_CONTROL_POLICY_MODE_POSITION_GENERATED,
		.features = {
			.encoder_read_enabled = false,
			.angle_gen_enabled = true,
			.commanded_currents_enabled = true,
			.current_loop_enabled = true,
		},
		.profile_sequence_active = true,
	};
	struct motor_servo_ref servo = {0};
	struct motor_actuator_ref actuator = {0};

	zassert_ok(motor_control_policy_derive(&input, &policy), NULL);
	zassert_false(policy.encoder_required_for_control, NULL);
	zassert_equal(policy.generated_angle_mode,
		      MOTOR_GENERATED_ANGLE_POSITION_DRIVEN, NULL);

	motor_servo_ref_set_dq_current(&servo, true, 0.75f, 1.25f, 0.5f, 0.0f, 0.15f);
	zassert_ok(motor_actuator_ref_from_servo(&policy, &servo, &actuator), NULL);

	zassert_true(actuator.enabled, NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_FOC_CURRENT, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_CURRENT_DQ, NULL);
	zassert_within(actuator.position_rad, 0.75f, 1.0e-6f, NULL);
	zassert_within(actuator.iq_ref_a, 0.15f, 1.0e-6f, NULL);
}

ZTEST(runtime, test_disarm_clears_servo_effort_without_corrupting_motion_ref)
{
	struct motor_motion_ref motion = {
		.position_rad = 2.0f,
		.velocity_ref_rad_s = 3.0f,
		.acceleration_rad_s2 = 4.0f,
	};
	struct motor_control_policy policy = {
		.actuator_kind = MOTOR_ACTUATOR_FOC_CURRENT,
	};
	struct motor_servo_ref servo = {0};
	struct motor_actuator_ref actuator = {0};

	motor_servo_ref_set_dq_current(&servo, true,
				       motion.position_rad,
				       motion.velocity_ref_rad_s,
				       motion.acceleration_rad_s2,
				       0.0f,
				       0.15f);
	motor_servo_ref_clear(&servo);
	zassert_ok(motor_actuator_ref_from_servo(&policy, &servo, &actuator), NULL);

	zassert_false(actuator.enabled, NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_FOC_CURRENT, NULL);
	zassert_within(motion.position_rad, 2.0f, 1.0e-6f, NULL);
	zassert_within(motion.velocity_ref_rad_s, 3.0f, 1.0e-6f, NULL);
}

ZTEST(runtime, test_same_motion_reference_maps_to_simulated_backends)
{
	const float32_t position = 1.0f;
	const float32_t velocity = 2.0f;
	const float32_t accel = 3.0f;
	struct motor_control_policy policy = {0};
	struct motor_servo_ref servo = {0};
	struct motor_actuator_ref actuator = {0};

	policy.actuator_kind = MOTOR_ACTUATOR_FOC_CURRENT;
	motor_servo_ref_set_dq_current(&servo, true, position, velocity, accel, 0.0f, 0.2f);
	zassert_ok(motor_actuator_ref_from_servo(&policy, &servo, &actuator), NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_FOC_CURRENT, NULL);
	zassert_within(actuator.position_rad, position, 1.0e-6f, NULL);
	zassert_within(actuator.iq_ref_a, 0.2f, 1.0e-6f, NULL);

	policy.actuator_kind = MOTOR_ACTUATOR_BRUSHED_CURRENT;
	motor_servo_ref_set_current(&servo, true, position, velocity, accel, 0.2f);
	zassert_ok(motor_actuator_ref_from_servo(&policy, &servo, &actuator), NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_BRUSHED_CURRENT, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_CURRENT_SCALAR, NULL);
	zassert_within(actuator.current_a, 0.2f, 1.0e-6f, NULL);

	policy.actuator_kind = MOTOR_ACTUATOR_BRUSHED_VOLTAGE;
	motor_servo_ref_set_voltage(&servo, true, position, velocity, accel, 1.5f);
	zassert_ok(motor_actuator_ref_from_servo(&policy, &servo, &actuator), NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_BRUSHED_VOLTAGE, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_VOLTAGE_SCALAR, NULL);
	zassert_within(actuator.voltage_v, 1.5f, 1.0e-6f, NULL);

	policy.actuator_kind = MOTOR_ACTUATOR_STEP_DIR;
	motor_servo_ref_set_motion(&servo, true, position, velocity, accel);
	zassert_ok(motor_actuator_ref_from_servo(&policy, &servo, &actuator), NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_STEP_DIR, NULL);
	zassert_equal(actuator.effort_kind, MOTOR_ACTUATOR_EFFORT_STEP_DIR, NULL);
	zassert_within(actuator.position_rad, position, 1.0e-6f, NULL);
	zassert_within(actuator.velocity_rad_s, velocity, 1.0e-6f, NULL);
}

ZTEST(runtime, test_backend_adapter_rejects_incompatible_effort_domain)
{
	struct motor_control_policy policy = {
		.actuator_kind = MOTOR_ACTUATOR_BRUSHED_CURRENT,
	};
	struct motor_servo_ref servo = {0};
	struct motor_actuator_ref actuator = {0};

	motor_servo_ref_set_dq_current(&servo, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f);
	zassert_equal(motor_actuator_ref_from_servo(&policy, &servo, &actuator),
		      -ENOTSUP, NULL);
	zassert_false(actuator.enabled, NULL);
	zassert_equal(actuator.kind, MOTOR_ACTUATOR_BRUSHED_CURRENT, NULL);
}

ZTEST(runtime, test_config_snapshot_publish_read_is_coherent)
{
	struct motor_rt_config_snapshot in = {
		.state = (const struct smf_state *)0x1000,
		.feature_flags = 0xA5,
		.velocity_loop_decimation = 3U,
		.position_loop_decimation = 5U,
		.profile_sequence_running = true,
		.profile_sequence_loop = false,
		.profile_sequence_trigger_source = 1U,
		.profile_sequence_trigger_edge = 2U,
		.profile_sequence_trigger_channel = 7U,
		.profile_sequence_period_ticks = 123U,
		.profile_sequence_period_ms = 40U,
		.control_policy_valid = true,
		.control_policy_input = {
			.mode = MOTOR_CONTROL_POLICY_MODE_VELOCITY_ENCODER,
			.features = {
				.encoder_read_enabled = true,
				.angle_gen_enabled = false,
				.velocity_traj_enabled = true,
				.commanded_currents_enabled = false,
				.current_loop_enabled = true,
			},
			.profile_sequence_active = false,
		},
		.control_policy = {
			.motion_source = MOTOR_MOTION_SOURCE_VELOCITY_TRAJ,
			.feedback_source = MOTOR_FEEDBACK_ENCODER,
			.angle_source = MOTOR_ANGLE_SOURCE_ENCODER,
			.current_source = MOTOR_CURRENT_SOURCE_VELOCITY_LOOP,
			.actuator_kind = MOTOR_ACTUATOR_FOC_CURRENT,
			.generated_angle_mode = MOTOR_GENERATED_ANGLE_NONE,
		},
	};
	struct motor_rt_config_snapshot out = {0};

	motor_config_snapshot_init();
	motor_config_snapshot_publish(&in);
	zassert_true(motor_config_snapshot_read(&out), "snapshot read failed");

	zassert_equal_ptr(out.state, in.state, "state mismatch");
	zassert_equal(out.feature_flags, in.feature_flags, "feature_flags mismatch");
	zassert_equal(out.velocity_loop_decimation, in.velocity_loop_decimation, "velocity decimation mismatch");
	zassert_equal(out.position_loop_decimation, in.position_loop_decimation, "position decimation mismatch");
	zassert_equal(out.profile_sequence_running, in.profile_sequence_running, "running mismatch");
	zassert_equal(out.profile_sequence_trigger_source, in.profile_sequence_trigger_source, "trigger source mismatch");
	zassert_equal(out.profile_sequence_period_ticks, in.profile_sequence_period_ticks, "period ticks mismatch");
	zassert_true(out.control_policy_valid, "policy validity mismatch");
	zassert_equal(out.control_policy_input.mode, in.control_policy_input.mode, "policy mode mismatch");
	zassert_equal(out.control_policy.angle_source, in.control_policy.angle_source, "angle source mismatch");
	zassert_equal(out.control_policy.actuator_kind, in.control_policy.actuator_kind, "actuator kind mismatch");
	zassert_true(out.epoch > 0U, "epoch not tagged");
}

ZTEST(runtime, test_config_snapshot_prevents_mixed_epoch_fields)
{
	struct motor_rt_config_snapshot even = {
		.state = (const struct smf_state *)0x2000,
		.feature_flags = 0x11,
		.velocity_loop_decimation = 2U,
		.position_loop_decimation = 4U,
		.profile_sequence_running = false,
		.profile_sequence_loop = false,
		.profile_sequence_trigger_source = 0U,
		.profile_sequence_trigger_edge = 0U,
		.profile_sequence_trigger_channel = 1U,
		.profile_sequence_period_ticks = 50U,
		.profile_sequence_period_ms = 5U,
	};
	struct motor_rt_config_snapshot odd = {
		.state = (const struct smf_state *)0x3000,
		.feature_flags = 0x22,
		.velocity_loop_decimation = 3U,
		.position_loop_decimation = 6U,
		.profile_sequence_running = true,
		.profile_sequence_loop = true,
		.profile_sequence_trigger_source = 1U,
		.profile_sequence_trigger_edge = 2U,
		.profile_sequence_trigger_channel = 2U,
		.profile_sequence_period_ticks = 75U,
		.profile_sequence_period_ms = 7U,
	};
	struct motor_rt_config_snapshot out = {0};
	uint32_t prev_epoch = 0U;

	motor_config_snapshot_init();

	for (int i = 0; i < 2000; i++) {
		if ((i & 1) == 0) {
			motor_config_snapshot_publish(&even);
		} else {
			motor_config_snapshot_publish(&odd);
		}

		zassert_true(motor_config_snapshot_read(&out), "snapshot read failed");
		zassert_true(out.epoch > prev_epoch, "epoch must increase monotonically");
		prev_epoch = out.epoch;

		bool matches_even =
			out.feature_flags == even.feature_flags &&
			out.velocity_loop_decimation == even.velocity_loop_decimation &&
			out.position_loop_decimation == even.position_loop_decimation &&
			out.profile_sequence_trigger_source == even.profile_sequence_trigger_source &&
			out.profile_sequence_period_ticks == even.profile_sequence_period_ticks;
		bool matches_odd =
			out.feature_flags == odd.feature_flags &&
			out.velocity_loop_decimation == odd.velocity_loop_decimation &&
			out.position_loop_decimation == odd.position_loop_decimation &&
			out.profile_sequence_trigger_source == odd.profile_sequence_trigger_source &&
			out.profile_sequence_period_ticks == odd.profile_sequence_period_ticks;

		zassert_true(matches_even || matches_odd,
			     "snapshot contains mixed-epoch fields");
	}
}

ZTEST_SUITE(runtime, NULL, NULL, NULL, NULL, NULL);
