/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private definitions for MT6835 driver implementation
 */

#ifndef MAGNTEK_MT6835_PRIV_H_
#define MAGNTEK_MT6835_PRIV_H_

#include <errno.h>
#include <zephyr/sys/crc.h>
#include <drivers/sensor/magntek_mt6835.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device tree compatibility string */
#define DT_DRV_COMPAT magntek_mt6835

/* MT6835 SPI Command definitions */
#define MT6835_CMD_READ  (0x03U << 4)
#define MT6835_CMD_WRITE (0x06U << 4)
#define MT6835_CMD_PROG  (0x0BU << 4)
#define MT6835_CMD_ZERO  (0x05U << 4)
#define MT6835_CMD_ANGLE (0x0AU << 4)

/* MT6835 Register addresses */
#define MT6835_REG_USER_ID      0x0000U
#define MT6835_REG_ANGLE_H      0x0003U
#define MT6835_REG_ANGLE_M      0x0004U
#define MT6835_REG_ANGLE_L      0x0005U
#define MT6835_REG_STATUS       0x0005U
#define MT6835_REG_CRC          0x0006U
#define MT6835_REG_ABZ_RES1     0x0007U
#define MT6835_REG_ABZ_RES2     0x0008U
#define MT6835_REG_ZERO_POS_H   0x0009U
#define MT6835_REG_ZERO_POS_L   0x000AU
#define MT6835_REG_Z_PHASE_UVW  0x000BU
#define MT6835_REG_PWM_NLC      0x000CU
#define MT6835_REG_ROT_HYST     0x000DU
#define MT6835_REG_GPIO_AUTOCAL 0x000EU
#define MT6835_REG_OPTS5        0x0011U
#define MT6835_REG_CAL_STATUS   0x0113U

/* Register bit masks */
#define MT6835_AUTOCAL_FREQ_MASK 0x07  /**< Bits 2:0: Auto-calibration rotation speed range */
#define MT6835_STATUS_MASK       0x07  /**< STATUS register mask */
#define MT6835_CAL_STATUS_MASK   0xC0  /**< Calibration status bits [7:6] */
#define MT6835_ROT_DIR_MASK      0x80  /**< Bit 7: Rotation direction */
#define MT6835_HYST_MASK         0x07  /**< Bits 2:0: Hysteresis */

/* Conversion constants */
#define MT6835_FULL_ANGLE     360
#define MT6835_PULSES_PER_REV 2097152
#define MT6835_MILLION_UNIT   1000000
#define MT6835_MAX_COUNT      (1U << MT6835_RESOLUTION_BITS)
#define MT6835_FRAME_DATA_OFFSET 2U
#define MT6835_FRAME_DATA_LEN    3U
#define MT6835_FRAME_CRC_OFFSET  5U

/**
 * @brief Decode MT6835 position and verify frame CRC8
 *
 * @param raw_buf Raw SPI response buffer (must be at least 6 bytes)
 * @param position Output: 21-bit position value
 * @param crc_error Output: CRC mismatch flag
 * @return 0 on success, -EIO on CRC mismatch
 */
static inline int mt6835_decode_position(const uint8_t *raw_buf, uint32_t *position, bool *crc_error)
{
	uint8_t expected_crc =
		crc8_ccitt(0x00U, &raw_buf[MT6835_FRAME_DATA_OFFSET], MT6835_FRAME_DATA_LEN);
	uint8_t received_crc = raw_buf[MT6835_FRAME_CRC_OFFSET];
	bool frame_crc_error = (expected_crc != received_crc);

	if (crc_error != NULL) {
		*crc_error = frame_crc_error;
	}

	if (frame_crc_error) {
		return -EIO;
	}

	*position = (sys_get_be24(&raw_buf[MT6835_FRAME_DATA_OFFSET]) >> 3) &
		    (MT6835_MAX_COUNT - 1U);

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* MAGNTEK_MT6835_PRIV_H_ */
