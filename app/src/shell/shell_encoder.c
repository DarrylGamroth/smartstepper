#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

int cmd_motor_encoder_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	shell_print(sh, "Encoder Status:");
	shell_print(sh, "  Acquisition:  %s, %s",
		    motor_encoder_acquisition_is_enabled() ? "enabled" : "disabled",
		    motor_encoder_acquisition_is_busy() ? "busy" : "idle");
	shell_print(sh, "  Control use:  trust=%s source=%s fresh=%s",
		    motor_shell_feedback_trust_name(g_motor_params->live.position_trust_state),
		    motor_encoder_input_source_to_string(g_motor_params->live.encoder_input_source),
		    g_motor_params->live.encoder_sample_fresh ? "yes" : "no");
	shell_print(sh, "  Raw angle:    %.3f deg",
		    (double)g_motor_params->live.encoder_raw_deg);
	shell_print(sh, "  Used angle:   %.3f deg",
		    motor_shell_rad_to_deg(g_motor_params->live.encoder_observer_input_rad));
	shell_print(sh, "  Direction:    %d",
		    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1);
	shell_print(sh, "  Last flags:   status=0x%02X warn=%s error=%s",
		    g_motor_params->live.encoder_last_status,
		    g_motor_params->live.encoder_sample_warning ? "SET" : "clear",
		    g_motor_params->live.encoder_sample_error ? "SET" : "clear");
	shell_print(sh, "  Counters:     warnings=%u errors=%u faults=%u",
		    g_motor_params->encoder_warning_count,
		    g_motor_params->encoder_error_count,
		    g_motor_params->encoder_fault_counter);
	shell_print(sh, "  Diagnostics:  use 'motor encoder acquisition', 'motor encoder protocol status', or traces for transport details");

	return 0;
}


/* motor encoder acquisition */
int cmd_motor_encoder_acquisition(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct motor_encoder_acquisition_stats stats = {0};
	motor_encoder_acquisition_get_stats(&stats);
	enum motor_encoder_test_inject_mode inject_mode =
		motor_encoder_acquisition_get_test_inject_mode();
	const char *inject_label = "none";
	if (inject_mode == MOTOR_ENCODER_TEST_INJECT_STATUS) {
		inject_label = "status";
	} else if (inject_mode == MOTOR_ENCODER_TEST_INJECT_FRAME) {
		inject_label = "frame";
	}

	shell_print(sh, "Encoder acquisition:");
	shell_print(sh, "  State:    %s, %s",
		    motor_encoder_acquisition_is_enabled() ? "enabled" : "disabled",
		    motor_encoder_acquisition_is_busy() ? "busy" : "idle");
	shell_print(sh, "  Inject:   %s", inject_label);
	shell_print(sh, "  Request:  ok=%u busy=%u disabled=%u error=%u",
		    stats.request_ok, stats.request_busy,
		    stats.request_disabled, stats.request_error);
	shell_print(sh, "  Collect:  ok=%u pending=%u empty=%u error=%u",
		    stats.collect_ok, stats.collect_pending,
		    stats.collect_empty, stats.collect_error);
	shell_print(sh, "  Errors:   transport=%u frame=%u parity=%u crc=%u status=%u glitch=%u",
		    stats.collect_transport_error,
		    stats.collect_frame_error,
		    stats.collect_frame_parity_error,
		    stats.collect_frame_crc_error,
		    stats.collect_frame_status_error,
		    stats.collect_frame_glitch_error);

	return 0;
}

/* motor encoder acquisition_reset */
int cmd_motor_encoder_acquisition_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_encoder_acquisition_reset_stats();
	if (g_motor_params) {
		g_motor_params->encoder_fault_counter = 0U;
		g_motor_params->recovery_status.encoder_recovery_done = true;
		g_motor_params->recovery_status.safe_idle_ready =
			(!g_motor_params->recovery_status.gate_reset_required ||
			 g_motor_params->recovery_status.gate_reset_done) &&
			(!g_motor_params->recovery_status.encoder_recovery_required ||
			 g_motor_params->recovery_status.encoder_recovery_done);
	}
	shell_print(sh, "Encoder acquisition counters reset");
	return 0;
}

/* motor encoder recover */
int cmd_motor_encoder_recover(const struct shell *sh, size_t argc, char **argv)
{
	int ret = cmd_motor_encoder_acquisition_reset(sh, argc, argv);
	if (ret == 0) {
		shell_print(sh, "Encoder recovery complete");
	}
	return ret;
}

/* motor encoder acquisition_inject [none|status|frame] */
int cmd_motor_encoder_acquisition_inject(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 2U) {
		shell_error(sh, "Usage: motor encoder acquisition_inject [none|status|frame]");
		return -EINVAL;
	}

	enum motor_encoder_test_inject_mode mode = motor_encoder_acquisition_get_test_inject_mode();

	if (argc == 1U) {
		const char *label = "none";

		if (mode == MOTOR_ENCODER_TEST_INJECT_STATUS) {
			label = "status";
		} else if (mode == MOTOR_ENCODER_TEST_INJECT_FRAME) {
			label = "frame";
		}
		shell_print(sh, "Encoder acquisition inject mode: %s", label);
		return 0;
	}

	if (strcmp(argv[1], "none") == 0) {
		mode = MOTOR_ENCODER_TEST_INJECT_NONE;
	} else if (strcmp(argv[1], "status") == 0) {
		mode = MOTOR_ENCODER_TEST_INJECT_STATUS;
	} else if (strcmp(argv[1], "frame") == 0) {
		mode = MOTOR_ENCODER_TEST_INJECT_FRAME;
	} else {
		shell_error(sh, "Invalid mode '%s' (expected none|status|frame)", argv[1]);
		return -EINVAL;
	}

	motor_encoder_acquisition_set_test_inject_mode(mode);
	shell_print(sh, "Encoder acquisition inject mode set: %s", argv[1]);
	return 0;
}

/* motor encoder control_status */
int cmd_motor_encoder_control_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	struct motor_encoder_control_status status = {0};
	char reason[96] = {0};
	int ret = motor_encoder_control_get_status(g_motor_params, true, &status,
						   reason, sizeof(reason));
	if (ret != 0) {
		shell_error(sh, "Failed to evaluate encoder-control readiness (err %d)", ret);
		return ret;
	}

	shell_print(sh, "Encoder Control Readiness:");
	shell_print(sh, "  Ready:           %s", status.ready ? "YES" : "NO");
	shell_print(sh, "  Reason:          %s", reason);
	shell_print(sh, "  Device ready:    %s", status.device_ready ? "YES" : "NO");
	shell_print(sh, "  Mapping applied: %s", status.mapping_complete ? "YES" : "NO");
	shell_print(sh, "  Acquisition idle:   %s", status.acquisition_idle ? "YES" : "NO");
	shell_print(sh, "  Injection off:   %s", status.injection_disabled ? "YES" : "NO");
	shell_print(sh, "  Protocol ok:     %s", status.protocol_ok ? "YES" : "NO");
	if (status.protocol_checked) {
		shell_print(sh, "  AEAT Config0:    0x%02X", status.config0);
		shell_print(sh, "  AEAT SPI4/UVW:   0x%02X", status.config7);
		shell_print(sh, "  AEAT PSEL:       0x%02X", status.config9);
	}
	if (status.protocol_error != 0) {
		shell_print(sh, "  Protocol error:  %d", status.protocol_error);
	}
	shell_print(sh, "  Acquisition req:    ok=%u busy=%u disabled=%u error=%u",
		    status.acquisition_stats.request_ok,
		    status.acquisition_stats.request_busy,
		    status.acquisition_stats.request_disabled,
		    status.acquisition_stats.request_error);
	shell_print(sh, "  Acquisition collect: ok=%u pending=%u empty=%u error=%u",
		    status.acquisition_stats.collect_ok,
		    status.acquisition_stats.collect_pending,
		    status.acquisition_stats.collect_empty,
		    status.acquisition_stats.collect_error);
	shell_print(sh, "  Acquisition errors: transport=%u parity=%u crc=%u glitch=%u status=%u",
		    status.acquisition_stats.collect_transport_error,
		    status.acquisition_stats.collect_frame_parity_error,
		    status.acquisition_stats.collect_frame_crc_error,
		    status.acquisition_stats.collect_frame_glitch_error,
		    status.acquisition_stats.collect_frame_status_error);

	return 0;
}

/* motor encoder direction [sign] */
int cmd_motor_encoder_direction(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder direction [<1|-1>]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	if (argc == 1U) {
		shell_print(sh, "Encoder direction sign: %d (devicetree default: %d)",
			    (g_motor_params->encoder_direction_sign >= 0) ? 1 : -1,
			    ENCODER_DIRECTION_SIGN);
		return 0;
	}

	if (motor_control_is_armed(g_motor_params)) {
		shell_error(sh, "Disarm control before changing encoder direction sign");
		return -EPERM;
	}

	int state = motor_api_get_state();
	if (state == MOTOR_STATE_ONLINE || motor_state_is_online_submode(state)) {
		shell_error(sh, "Set encoder direction sign while in ONLINE state");
		return -EPERM;
	}

	float parsed = 0.0f;
	if (!shell_parse_finite_float(argv[1], &parsed)) {
		shell_error(sh, "direction sign must be numeric (-1 or 1)");
		return -EINVAL;
	}

	int sign = 0;
	if (fabsf(parsed - 1.0f) < 1.0e-3f) {
		sign = 1;
	} else if (fabsf(parsed + 1.0f) < 1.0e-3f) {
		sign = -1;
	} else {
		shell_error(sh, "direction sign must be -1 or 1");
		return -EINVAL;
	}

	int ret = motor_api_update_param("encoder_direction_sign", (float)sign);
	if (ret != 0) {
		shell_error(sh, "Failed to update encoder direction sign (err %d)", ret);
		return ret;
	}

	shell_print(sh, "Encoder direction sign update posted (%d)", sign);
	return 0;
}

/* motor encoder trim [deg] */
int cmd_motor_encoder_trim(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 1U && argc != 2U) {
		shell_error(sh, "Usage: motor encoder trim [<-180.0..180.0>]");
		return -EINVAL;
	}

	if (!g_motor_params) {
		shell_error(sh, "Motor not initialized");
		return -ENODEV;
	}

	double trim_deg = motor_shell_rad_to_deg(g_motor_params->observer_elec_trim_rad);
	double base_mech_offset_deg =
		motor_shell_rad_to_deg(g_motor_params->observer_alignment_offset_rad);
	double active_mech_offset_deg =
		motor_shell_rad_to_deg(g_motor_params->observer.mech_angle_offset_rad);
	if (argc == 1U) {
		shell_print(sh, "Encoder electrical trim: %.3f deg (mechanical equivalent: %.4f deg)",
			    trim_deg,
			    trim_deg / (double)MOTOR_POLE_PAIRS);
		shell_print(sh, "Observer base offset: %.3f deg mechanical",
			    base_mech_offset_deg);
		shell_print(sh, "Observer active offset: %.3f deg mechanical",
			    active_mech_offset_deg);
		shell_print(sh, "Observer raw: align=0x%08X active=0x%08X trim=0x%08X pos=0x%08X",
		    motor_shell_f32_bits(g_motor_params->observer_alignment_offset_rad),
		    motor_shell_f32_bits(g_motor_params->observer.mech_angle_offset_rad),
		    motor_shell_f32_bits(g_motor_params->observer_elec_trim_rad),
		    motor_shell_f32_bits(g_motor_params->calibration.align_mech_angle_rad));
		return 0;
	}

	float requested_trim_deg = 0.0f;
	if (!shell_parse_finite_float(argv[1], &requested_trim_deg)) {
		shell_error(sh, "trim must be a finite number of electrical degrees");
		return -EINVAL;
	}
	if (requested_trim_deg < -180.0f || requested_trim_deg > 180.0f) {
		shell_error(sh, "trim must be within [-180.0, 180.0] electrical degrees");
		return -EINVAL;
	}

	int ret = motor_api_update_param("observer_elec_trim_deg", requested_trim_deg);
	if (ret != 0) {
		shell_error(sh, "Failed to update observer electrical trim (err %d)", ret);
		return ret;
	}

	motor_command_feed_watchdog(g_motor_params);
	shell_print(sh, "Observer electrical trim update posted: %.3f deg",
		    (double)requested_trim_deg);
	return 0;
}

/* motor encoder alarm */
int cmd_motor_encoder_alarm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955
	shell_error(sh, "encoder1 is not AEAT-9955 on this build");
	return -ENOTSUP;
#else
	uint8_t status = 0U;
	bool mhi = false;
	bool mlo = false;
	int ret = motor_encoder_read_aeat_alarm(&status, &mhi, &mlo);
	if (ret < 0) {
		shell_error(sh, "Failed to read AEAT alarm status (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 alarm/error status:");
	shell_print(sh, "  Raw status: 0x%02X", status);
	shell_print(sh, "  MHI:        %s", mhi ? "SET" : "CLEAR");
	shell_print(sh, "  MLO:        %s", mlo ? "SET" : "CLEAR");

	return 0;
#endif
}

/* motor encoder fast */
int cmd_motor_encoder_fast(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_FAST
	shell_error(sh, "encoder1 is not a fast encoder_rt device on this build");
	return -ENOTSUP;
#else
	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}

	struct encoder_rt_stats stats = {0};
	encoder_rt_get_stats(encoder1, &stats);
#if MOTOR_ENCODER_IS_AEAT9955_FAST
	enum aeat9955_fast_spi4_mode spi4_mode = AEAT9955_FAST_SPI4_16_PARITY;
	(void)aeat9955_fast_get_spi4_mode(encoder1, &spi4_mode);
#endif

	shell_print(sh, "Fast Encoder:");
	shell_print(sh, "  Device:          %s", encoder1->name);
#if MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_print(sh, "  SPI4 mode:       %s",
		    (spi4_mode == AEAT9955_FAST_SPI4_8_CRC16) ?
			    "spi4-8-crc16" : "spi4-16-parity");
#endif
	shell_print(sh, "  Transport delay:  %u samples", encoder_rt_get_pipeline_delay(encoder1));
	shell_print(sh, "  Request:         ok=%u busy=%u disabled=%u error=%u",
		    stats.request_count, stats.busy_count,
		    stats.disabled_count, stats.request_error_count);
	shell_print(sh, "  Collect:         ok=%u pending=%u empty=%u error=%u",
		    stats.collect_count, stats.pending_count,
		    stats.empty_count, stats.collect_error_count);
	shell_print(sh, "  Errors:          transport=%u frame=%u parity=%u crc=%u status=%u warning=%u",
		    stats.transport_error_count,
		    stats.frame_error_count,
		    stats.frame_parity_error_count,
		    stats.frame_crc_error_count,
		    stats.frame_status_error_count,
		    stats.warning_count);

	return 0;
#endif
}

#if MOTOR_ENCODER_IS_AEAT9955_FAST
static int motor_encoder_parse_u8_arg(const char *arg, uint8_t *value)
{
	if (arg == NULL || value == NULL) {
		return -EINVAL;
	}

	errno = 0;
	char *endp = NULL;
	unsigned long parsed = strtoul(arg, &endp, 0);
	if (endp == arg || *endp != '\0' || errno == ERANGE || parsed > UINT8_MAX) {
		return -EINVAL;
	}

	*value = (uint8_t)parsed;
	return 0;
}
#endif

/* motor encoder reg_read <addr> */
int cmd_motor_encoder_reg_read(const struct shell *sh, size_t argc, char **argv)
{
#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "register reads require the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (argc != 2U) {
		shell_error(sh, "Usage: motor encoder reg_read <addr>");
		return -EINVAL;
	}
	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before register reads");
		return -EBUSY;
	}

	uint8_t reg = 0U;
	if (motor_encoder_parse_u8_arg(argv[1], &reg) != 0) {
		shell_error(sh, "addr must be an 8-bit register address");
		return -EINVAL;
	}

	uint8_t value = 0U;
	int ret = aeat9955_fast_read_register(encoder1, reg, &value);
	if (ret != 0) {
		shell_error(sh, "Failed to read AEAT register 0x%02X (err %d)",
			    (unsigned int)reg, ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 register 0x%02X = 0x%02X",
		    (unsigned int)reg, value);
	return 0;
#endif
}

/* motor encoder reg_write <addr> <value> */
int cmd_motor_encoder_reg_write(const struct shell *sh, size_t argc, char **argv)
{
#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "register writes require the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (argc != 3U) {
		shell_error(sh, "Usage: motor encoder reg_write <addr> <value>");
		return -EINVAL;
	}
	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before register writes");
		return -EBUSY;
	}

	uint8_t reg = 0U;
	uint8_t value = 0U;
	if (motor_encoder_parse_u8_arg(argv[1], &reg) != 0 ||
	    motor_encoder_parse_u8_arg(argv[2], &value) != 0) {
		shell_error(sh, "addr and value must be 8-bit values");
		return -EINVAL;
	}

	int ret = aeat9955_fast_write_register(encoder1, reg, value);
	if (ret != 0) {
		shell_error(sh, "Failed to write AEAT register 0x%02X (err %d)",
			    (unsigned int)reg, ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 register 0x%02X <= 0x%02X",
		    (unsigned int)reg, value);
	return 0;
#endif
}


