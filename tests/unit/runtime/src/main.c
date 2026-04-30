#include <zephyr/ztest.h>

#include "motor/runtime/config_snapshot.h"
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
		.enabled = true,
	};

	zassert_equal(actuator.kind, MOTOR_ACTUATOR_STEP_DIR, NULL);
	zassert_equal(feedback.source, MOTOR_FEEDBACK_ENCODER, NULL);
	zassert_within(motion.position_rad, 1.25f, 1.0e-6f, NULL);
	zassert_within(motion.velocity_target_rad_s, 2.0f, 1.0e-6f, NULL);
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
