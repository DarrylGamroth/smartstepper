#include "shell_state_common.h"

/* Domain implementation split from shell_commands_state.c. */

/* motor encoder protocol status */
int cmd_motor_encoder_protocol_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT protocol control requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	enum aeat9955_fast_spi4_mode mode = AEAT9955_FAST_SPI4_16_PARITY;
	uint8_t reg0 = 0U;
	uint8_t reg7 = 0U;
	uint8_t reg9 = 0U;
	int ret;

	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}
	ret = aeat9955_fast_get_spi4_mode(encoder1, &mode);
	if (ret != 0) {
		shell_error(sh, "Failed to get AEAT driver mode (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 Protocol:");
	shell_print(sh, "  Driver mode: %s",
		    (mode == AEAT9955_FAST_SPI4_8_CRC16) ?
			    "spi4-8-crc16" : "spi4-16-parity");
#if MOTOR_ENCODER_HAS_RTSPI
	if (device_is_ready(encoder_rtspi)) {
		struct rt_spi_config spi_cfg = {0};

		rt_spi_get_config(encoder_rtspi, &spi_cfg);
		shell_print(sh, "  SPI mode:    CPOL=%u CPHA=%u",
			    spi_cfg.cpol ? 1U : 0U, spi_cfg.cpha ? 1U : 0U);
	} else {
		shell_print(sh, "  SPI mode:    encoder RT SPI transport not ready");
	}
#endif

	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_print(sh, "  Registers:   unavailable while realtime sampling is active");
		return 0;
	}

	ret = aeat9955_fast_read_register(encoder1, AEAT9955_FAST_REG_CONFIG0, &reg0);
	if (ret == 0) {
		ret = aeat9955_fast_read_register(encoder1, AEAT9955_FAST_REG_CONFIG0_SPI4,
						  &reg7);
	}
	if (ret == 0) {
		ret = aeat9955_fast_read_register(encoder1, AEAT9955_FAST_REG_CONFIG1_PSEL,
						  &reg9);
	}
	if (ret != 0) {
		shell_print(sh, "  Registers:   read failed (err %d)", ret);
		return 0;
	}

	shell_print(sh, "  Config0:     0x%02X safety=%s crc=%s init=%u",
		    reg0,
		    (reg0 & AEAT9955_FAST_CONFIG0_SAFETY_BIT) ? "on" : "off",
		    (reg0 & AEAT9955_FAST_CONFIG0_CRC_SELECT) ? "crc16" : "crc8",
		    (unsigned int)((reg0 & AEAT9955_FAST_CONFIG0_CRC_INIT_MASK) >> 4));
	shell_print(sh, "  SPI4/UVW:    0x%02X spi4=%u",
		    reg7,
		    (unsigned int)((reg7 & AEAT9955_FAST_CONFIG0_SPI4_MODE_MASK) >> 6));
	shell_print(sh, "  PSEL:        0x%02X psel=%u",
		    reg9,
		    (reg9 & AEAT9955_FAST_CONFIG1_PSEL_BIT) ? 1U : 0U);

	return 0;
#endif
}

int cmd_motor_encoder_protocol_spi_mode(const struct shell *sh, size_t argc, char **argv)
{
#if !MOTOR_ENCODER_HAS_RTSPI
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "runtime SPI mode control requires encoder RT SPI transport");
	return -ENOTSUP;
#else
	if (argc != 3U) {
		shell_error(sh, "Usage: motor encoder protocol spi_mode <cpol 0|1> <cpha 0|1>");
		return -EINVAL;
	}
	if (!device_is_ready(encoder_rtspi)) {
		shell_error(sh, "encoder RT SPI transport is not ready");
		return -ENODEV;
	}
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before changing SPI mode");
		return -EBUSY;
	}

	bool cpol = false;
	bool cpha = false;
	if (!shell_parse_bool01(argv[1], &cpol) || !shell_parse_bool01(argv[2], &cpha)) {
		shell_error(sh, "cpol and cpha must be 0 or 1");
		return -EINVAL;
	}

	const struct rt_spi_config spi_cfg = {
		.cpol = cpol,
		.cpha = cpha,
	};
	int ret = rt_spi_configure(encoder_rtspi, &spi_cfg);
	if (ret != 0) {
		shell_error(sh, "Failed to set RT SPI mode (err %d)", ret);
		return ret;
	}

	shell_print(sh, "RT SPI mode set to CPOL=%u CPHA=%u",
		    cpol ? 1U : 0U, cpha ? 1U : 0U);
	return 0;
#endif
}

int cmd_motor_encoder_protocol_detect(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT protocol detection requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (!device_is_ready(encoder1)) {
		shell_error(sh, "encoder1 is not ready");
		return -ENODEV;
	}
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before protocol detection");
		return -EBUSY;
	}

	enum aeat9955_fast_spi4_mode mode = AEAT9955_FAST_SPI4_16_PARITY;
	int ret = aeat9955_fast_detect_spi4_mode(encoder1, &mode);
	if (ret != 0) {
		shell_error(sh, "Failed to detect AEAT SPI4 mode (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 detected protocol: %s",
		    (mode == AEAT9955_FAST_SPI4_8_CRC16) ?
			    "spi4-8-crc16" : "spi4-16-parity");
	return 0;
#endif
}

int cmd_motor_encoder_protocol_spi4_8_volatile(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT protocol control requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before changing AEAT protocol");
		return -EBUSY;
	}

	int ret = aeat9955_fast_configure_spi4_8_crc16_volatile(encoder1);
	if (ret != 0) {
		shell_error(sh, "Failed to switch AEAT to volatile SPI4-8 CRC16 (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 volatile protocol set to SPI4-8 CRC16");
	return 0;
#endif
}

int cmd_motor_encoder_protocol_spi4_16_volatile(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT protocol control requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before changing AEAT protocol");
		return -EBUSY;
	}

	int ret = aeat9955_fast_configure_spi4_16_parity_volatile(encoder1);
	if (ret != 0) {
		shell_error(sh, "Failed to switch AEAT to volatile SPI4-16 parity (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 volatile protocol set to SPI4-16 parity");
	return 0;
#endif
}

int cmd_motor_encoder_protocol_driver_spi4_8(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT protocol control requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before changing driver mode");
		return -EBUSY;
	}

	int ret = aeat9955_fast_set_spi4_mode_runtime(encoder1, AEAT9955_FAST_SPI4_8_CRC16);
	if (ret != 0) {
		shell_error(sh, "Failed to force driver SPI4-8 mode (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 driver-only protocol set to SPI4-8 CRC16");
	return 0;
#endif
}

int cmd_motor_encoder_protocol_driver_spi4_16(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT protocol control requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before changing driver mode");
		return -EBUSY;
	}

	int ret = aeat9955_fast_set_spi4_mode_runtime(encoder1, AEAT9955_FAST_SPI4_16_PARITY);
	if (ret != 0) {
		shell_error(sh, "Failed to force driver SPI4-16 mode (err %d)", ret);
		return ret;
	}

	shell_print(sh, "AEAT-9955 driver-only protocol set to SPI4-16 parity");
	return 0;
#endif
}

int cmd_motor_encoder_protocol_raw_position(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	shell_error(sh, "AEAT raw position requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before raw position read");
		return -EBUSY;
	}

	uint8_t raw[RT_SPI_MAX_FRAME_BYTES] = {0};
	uint8_t len = 0U;
	int ret = aeat9955_fast_read_position_raw(encoder1, raw, sizeof(raw), &len);
	if (ret != 0) {
		shell_error(sh, "Failed to read raw AEAT position frame (err %d)", ret);
		return ret;
	}

	enum aeat9955_fast_spi4_mode mode = AEAT9955_FAST_SPI4_16_PARITY;
	(void)aeat9955_fast_get_spi4_mode(encoder1, &mode);
	shell_print(sh, "AEAT-9955 raw position frame:");
	shell_print(sh, "  Mode: %s",
		    (mode == AEAT9955_FAST_SPI4_8_CRC16) ?
			    "spi4-8-crc16" : "spi4-16-parity");
	shell_print(sh, "  Len:  %u", len);
	shell_fprintf(sh, SHELL_NORMAL, "  Raw: ");
	for (uint8_t i = 0U; i < len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02X%s", raw[i],
			      (i + 1U == len) ? "" : " ");
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	return 0;
#endif
}

int cmd_motor_encoder_protocol_raw_reg(const struct shell *sh, size_t argc, char **argv)
{
#if !MOTOR_ENCODER_IS_AEAT9955_FAST
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_error(sh, "AEAT raw register read requires the AEAT-9955 fast encoder driver");
	return -ENOTSUP;
#else
	if (argc != 2U) {
		shell_error(sh, "Usage: motor encoder protocol raw_reg <addr>");
		return -EINVAL;
	}
	if (motor_encoder_acquisition_is_enabled() || motor_encoder_acquisition_is_busy()) {
		shell_error(sh, "disable realtime encoder sampling before raw register read");
		return -EBUSY;
	}

	uint8_t reg = 0U;
	if (motor_encoder_parse_u8_arg(argv[1], &reg) != 0) {
		shell_error(sh, "addr must be an 8-bit register address");
		return -EINVAL;
	}

	uint8_t raw[RT_SPI_MAX_FRAME_BYTES] = {0};
	uint8_t len = 0U;
	int ret = aeat9955_fast_read_register_raw(encoder1, reg, raw, sizeof(raw), &len);
	if (ret != 0) {
		shell_error(sh, "Failed to read raw AEAT register frame (err %d)", ret);
		return ret;
	}

	enum aeat9955_fast_spi4_mode mode = AEAT9955_FAST_SPI4_16_PARITY;
	(void)aeat9955_fast_get_spi4_mode(encoder1, &mode);
	shell_print(sh, "AEAT-9955 raw register frame:");
	shell_print(sh, "  Mode: %s",
		    (mode == AEAT9955_FAST_SPI4_8_CRC16) ?
			    "spi4-8-crc16" : "spi4-16-parity");
	shell_print(sh, "  Reg:  0x%02X", reg);
	shell_print(sh, "  Len:  %u", len);
	shell_fprintf(sh, SHELL_NORMAL, "  Raw: ");
	for (uint8_t i = 0U; i < len; i++) {
		shell_fprintf(sh, SHELL_NORMAL, "%02X%s", raw[i],
			      (i + 1U == len) ? "" : " ");
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	return 0;
#endif
}


