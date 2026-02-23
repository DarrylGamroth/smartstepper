/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private definitions for AEAT9955 driver implementation
 */

#ifndef BRCM_AEAT9955_PRIV_H_
#define BRCM_AEAT9955_PRIV_H_

#include <drivers/sensor/brcm_aeat9955.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device tree compatibility string */
#define DT_DRV_COMPAT brcm_aeat_9955

/* AEAT9955 SPI Command definitions */
/* SPI4-16 (16-Bit Parity) format - used for initialization only */
#define AEAT9955_CMD_READ_SPI16  0x40U /* RW=1 for read */
#define AEAT9955_CMD_WRITE_SPI16 0x00U /* RW=0 for write */

/* SPI4-8 (8-Bit Safety) format - operational mode */
#define AEAT9955_CMD_READ_SPI8     0x81U /* Register read operation code */
#define AEAT9955_CMD_WRITE_SPI8    0xCFU /* Register write operation code */
#define AEAT9955_CMD_POS_READ_SPI8 0xA6U /* Position read operation code */

/* SPI4 Mode selection values for register 0x07 bits [7:6] */
#define AEAT9955_SPI4_MODE_16BIT 0x00U /* SPI4-16 (16-bit parity) */
#define AEAT9955_SPI4_MODE_24A   0x01U /* SPI4-24a (24-bit parallel CRC) */
#define AEAT9955_SPI4_MODE_24B   0x02U /* SPI4-24b (24-bit serial CRC) */
#define AEAT9955_SPI4_MODE_8BIT  0x03U /* SPI4-8 (8-bit safety) */

/* Customer Configuration 0 Registers (Table 2) */
#define AEAT9955_REG_CONFIG0           0x00U /* Safety, CRC, SC Initialize */
#define AEAT9955_REG_CONFIG0_MAG       0x01U /* Alarm Latch, SPI Output High-Z */
#define AEAT9955_REG_CONFIG0_MAG_FIELD 0x02U /* Magnetic High/Low limits */
#define AEAT9955_REG_CONFIG0_MULTI_IDX 0x05U /* Multi-Index pulses per rev */
#define AEAT9955_REG_CONFIG0_SENSING   0x06U /* Auto-cal hardware, sensing axis, VH selection */
#define AEAT9955_REG_CONFIG0_SPI4_UVW  0x07U /* SPI4 mode, UVW resolution */
#define AEAT9955_REG_CONFIG0_PWM_IDX   0x08U /* PWM resolution, Index state/width */

/* Customer Configuration 1 Registers (Table 3) */
#define AEAT9955_REG_CONFIG1_PSEL     0x09U /* Protocol mode selection */
#define AEAT9955_REG_CONFIG1_INCR_RES 0x0AU /* Incremental resolution [13:8] */

/* Customer Configuration 2 Registers (Table 4) */
#define AEAT9955_REG_CONFIG2_HYST_DIR 0x0BU /* Hysteresis, Direction, Single turn resolution */

/* Customer Single-Turn Reset (Table 5) */
#define AEAT9955_REG_ZERO_RESET_2 0x0CU /* Zero Reset 2 (MSB bit-17 to bit-10) */
#define AEAT9955_REG_ZERO_RESET_1 0x0DU /* Zero Reset 1 (Bit-9 to bit-2) */
#define AEAT9955_REG_ZERO_RESET_0 0x0EU /* Zero Reset 0 (LSB bit-1 to bit-0) */

/* EEPROM Programming registers */
#define AEAT9955_REG_UNLOCK         0x10U /* Memory unlock register */
#define AEAT9955_REG_CALIBRATION    0x12U /* Calibration/zero reset control */
#define AEAT9955_REG_EEPROM_PROGRAM 0x14U /* EEPROM program trigger */
#define AEAT9955_REG_EEPROM_PAGE    0x16U /* EEPROM page selection */

/* EEPROM Passcode registers (default h00-h00-h00-h00-h00-h00-h00) */
#define AEAT9955_REG_PASSCODE_0 0x18U /* Passcode byte 0 */
#define AEAT9955_REG_PASSCODE_1 0x19U /* Passcode byte 1 */
#define AEAT9955_REG_PASSCODE_2 0x1AU /* Passcode byte 2 */
#define AEAT9955_REG_PASSCODE_3 0x1BU /* Passcode byte 3 */
#define AEAT9955_REG_PASSCODE_4 0x1CU /* Passcode byte 4 */
#define AEAT9955_REG_PASSCODE_5 0x1DU /* Passcode byte 5 */
#define AEAT9955_REG_PASSCODE_6 0x1EU /* Passcode byte 6 */

/* Volatile Status registers */
#define AEAT9955_REG_ERROR_STATUS 0x29U /* Error and alarm status register (Chip Status) */
#define AEAT9955_REG_STATUS       0x2AU /* Status register: Memory Busy, OTP, Calibration status */
#define AEAT9955_REG_CHIP_ID      0x2BU /* Chip ID register */

#define AEAT9955_CHIP_ID_EXPECTED 0x32U /* Expected Chip ID value for AEAT-9955 */

/* Position register */
#define AEAT9955_REG_POS             0x3FU /* Position register for fast angle read */
#define AEAT9955_POS_STATUS_PARITY_BIT 0x80U /* Status/parity bit in response byte 0 */
#define AEAT9955_POS_STATUS_ERROR_BIT  0x40U /* Device error bit in response byte 0 */

/* Calibration control commands */
#define AEAT9955_CAL_CMD_ACCURACY   0x02U /* Trigger accuracy calibration */
#define AEAT9955_CAL_CMD_ZERO_RESET 0x08U /* Trigger zero reset calibration */
#define AEAT9955_CAL_CMD_EXIT       0x00U /* Exit calibration mode */

/* Error status bits in register 0x29 */
#define AEAT9955_ERROR_RDY_BIT  0xC0U /* Ready bits (bits 7:6): RDY[1] and RDY[0] */
#define AEAT9955_ERROR_RDY1_BIT 0x80U /* Ready bit 1 (bit 7): 1=ready */
#define AEAT9955_ERROR_RDY0_BIT 0x40U /* Ready bit 0 (bit 6):  */
#define AEAT9955_ERROR_MHI_BIT  0x20U /* Magnet High error (bit 5): 1=error */
#define AEAT9955_ERROR_MLO_BIT  0x10U /* Magnet Low error (bit 4): 1=error */
#define AEAT9955_ERROR_OV_BIT   0x08U /* Overvoltage error (bit 3): 1=error */
#define AEAT9955_ERROR_UV_BIT   0x04U /* Undervoltage error (bit 2): 1=error */
#define AEAT9955_ERROR_MEM_BIT  0x02U /* Memory error (bit 1): 1=error */
#define AEAT9955_ERROR_TRK_BIT  0x01U /* Tracker error (bit 0): 1=error */

/* Status register (0x2A) bit definitions */
#define AEAT9955_STATUS_MEM_BUSY_BIT 0x80U /* Memory busy bit [7]: 0=completed, 1=in progress */
#define AEAT9955_STATUS_OTP_UNLOCKED 0x40U /* OTP unlocked bit [6]: 0=locked, 1=unlocked */
#define AEAT9955_STATUS_ZERO_MASK    0x0CU /* Zero reset status [3:2] */
#define AEAT9955_STATUS_ZERO_PASS    0x08U /* Zero reset pass (10b) */
#define AEAT9955_STATUS_ZERO_FAIL    0x0CU /* Zero reset fail (11b) */
#define AEAT9955_STATUS_CAL_MASK     0x03U /* Auto calibration status [1:0] */
#define AEAT9955_STATUS_CAL_PASS     0x02U /* Auto calibration pass (10b) */
#define AEAT9955_STATUS_CAL_FAIL     0x03U /* Auto calibration fail (11b) */

/* EEPROM unlock and programming commands */
#define AEAT9955_UNLOCK_LEVEL1       0xABU /* Unlock Level 1 memory access */
#define AEAT9955_EEPROM_PROGRAM_CMD  0xA1U /* Program shadow registers to EEPROM */
#define AEAT9955_EEPROM_PAGE_CONFIG  0x00U /* Load Customer Configuration page */
#define AEAT9955_EEPROM_PAGE_RESERVE 0x01U /* Load Customer Reserve page */

/* Register bit masks and fields */
/* Config 0 register (0x00) */
#define AEAT9955_CONFIG0_SAFETY_BIT     0x80U /* Safety bit enable (bit 7) */
#define AEAT9955_CONFIG0_CRC_SELECT     0x40U /* CRC select (bit 6): 1=16b CRC, 0=8b CRC */
#define AEAT9955_CONFIG0_CRC_INIT_MASK  0x30U /* CRC initialize mask (bits 5:4) */
#define AEAT9955_CONFIG0_CRC_INIT_SHIFT 4     /* CRC initialize shift */
#define AEAT9955_CONFIG0_SC_INIT_MASK   0x0FU /* Sequence Counter initialize mask (bits 3:0) */

/* Config 0 Mag register (0x01) */
#define AEAT9955_CONFIG0_ALARM_LATCH      0x80U /* Alarm latch (bit 7) */
#define AEAT9955_CONFIG0_SPI_HIGHZ        0x40U /* SPI Output High-Z (bit 6) */
#define AEAT9955_CONFIG0_ABS_HYST_OFF     0x10U /* Absolute hysteresis off (bit 4) */
#define AEAT9955_CONFIG0_INC_SAFE_OFF     0x08U /* Incremental safety off (bit 3) */
#define AEAT9955_CONFIG0_ACC_CAL_SKIP     0x04U /* Accuracy calibration skip (bit 2) */
#define AEAT9955_CONFIG0_AUTO_CAL_HW_TIME 0x02U /* Auto-calibration hardware hold time (bit 1) */
#define AEAT9955_CONFIG0_MULTI_AUTO_CAL   0x01U /* Multiple auto-calibration hardware (bit 0) */

/* Config 0 Magnetic Field register (0x02) */
#define AEAT9955_CONFIG0_MAG_HIGH_MASK  0xF0U /* Magnetic High limit (bits 7:4) */
#define AEAT9955_CONFIG0_MAG_HIGH_SHIFT 4     /* Magnetic High shift */
#define AEAT9955_CONFIG0_MAG_LOW_MASK   0x0FU /* Magnetic Low limit (bits 3:0) */

/* Config 0 Multi-Index register (0x05) */
#define AEAT9955_CONFIG0_MULTI_IDX_MASK  0xE0U /* Multi-index mask (bits 7:5) */
#define AEAT9955_CONFIG0_MULTI_IDX_SHIFT 5     /* Multi-index shift */
#define AEAT9955_CONFIG0_ACCEL_CFG_MASK  0x0CU /* Acceleration configuration mask (bits 3:2) */
#define AEAT9955_CONFIG0_ACCEL_CFG_SHIFT 2     /* Acceleration configuration shift */

/* Config 0 Sensing register (0x06) */
#define AEAT9955_CONFIG0_AUTO_CAL_HW   0x80U /* Auto-calibration hardware enable (bit 7) */
#define AEAT9955_CONFIG0_SENSING_MASK  0x70U /* Sensing axis mask (bits 6:4) */
#define AEAT9955_CONFIG0_SENSING_SHIFT 4     /* Sensing axis shift */
#define AEAT9955_CONFIG0_VH_SEL_MASK   0x0FU /* Vertical Hall selection mask (bits 3:0) */

/* Config 0 SPI4/UVW register (0x07) */
#define AEAT9955_CONFIG0_SPI4_MODE_MASK  0xC0U /* SPI4 mode mask (bits 7:6) */
#define AEAT9955_CONFIG0_SPI4_MODE_SHIFT 6     /* SPI4 mode shift */
#define AEAT9955_CONFIG0_UVW_MASK        0x3FU /* UVW resolution mask (bits 5:0) */

/* Config 0 PWM/Index register (0x08) */
#define AEAT9955_CONFIG0_PWM_RES_MASK    0xF0U /* PWM resolution mask (bits 7:4) */
#define AEAT9955_CONFIG0_PWM_RES_SHIFT   4     /* PWM resolution shift */
#define AEAT9955_CONFIG0_IDX_STATE_MASK  0x0CU /* Index state mask (bits 3:2) */
#define AEAT9955_CONFIG0_IDX_STATE_SHIFT 2     /* Index state shift */
#define AEAT9955_CONFIG0_IDX_WIDTH_MASK  0x03U /* Index width mask (bits 1:0) */

/* Config 1 PSEL register (0x09) */
#define AEAT9955_CONFIG1_PSEL_BIT          0x80U /* Protocol mode selection (bit 7) */
#define AEAT9955_CONFIG1_INCR_RES_LOW_MASK 0x7FU /* Incremental resolution low 7 bits */

/* Config 2 register (0x0B) */
#define AEAT9955_CONFIG2_HYST_MASK  0xE0U /* Hysteresis mask (bits 7:5) */
#define AEAT9955_CONFIG2_HYST_SHIFT 5     /* Hysteresis bit shift */
#define AEAT9955_CONFIG2_DIR_BIT    0x10U /* Direction bit (bit 4) */
#define AEAT9955_CONFIG2_STR_MASK   0x0FU /* Single turn resolution mask (bits 3:0) */

/* Zero Reset register (0x0E) - LSB 2 bits */
#define AEAT9955_ZERO_RESET_0_MASK 0xC0U /* Zero reset bits [1:0] in upper 2 bits (bits 7:6) */

/* Status byte bits (received during angle read in byte 1) */
#define AEAT9955_STATUS_PARITY_BIT 0x80U /* Parity bit (bit 7) */
#define AEAT9955_STATUS_ERROR_BIT  0x40U /* Error flag bit (bit 6) */

/* Conversion constants */
#define AEAT9955_FULL_ANGLE      360
#define AEAT9955_MILLION_UNIT    1000000
#define AEAT9955_RESOLUTION_BITS 18
#define AEAT9955_MAX_COUNT       (1U << AEAT9955_RESOLUTION_BITS)

/**
 * @brief Decode AEAT-9955 position and status from raw SPI response
 *
 * @param raw_buf Raw SPI response buffer (must be at least 3 bytes)
 * @param position Output: 18-bit position value
 * @param status_error Output: device error/status bit
 * @param parity_error Output: parity check error flag
 * @return 0 on success, -EIO on error
 */
static inline int aeat9955_decode_position(const uint8_t *raw_buf, uint32_t *position,
					   bool *status_error, bool *parity_error)
{
	uint8_t status0 = raw_buf[0];
	uint32_t raw24 = sys_get_be24(raw_buf) & 0x00FFFFFFU;

	/* Extract 18-bit position (bits 0-17) */
	*position = (raw24 >> 4) & (AEAT9955_MAX_COUNT - 1);
	bool status_bit_error = (status0 & AEAT9955_POS_STATUS_ERROR_BIT) != 0U;
	/* SPI4-16 response parity covers the full 24-bit encoder frame. */
	bool frame_parity_error = (POPCOUNT(raw24) & 1U) != 0U;

	if (status_error != NULL) {
		*status_error = status_bit_error;
	}
	if (parity_error != NULL) {
		*parity_error = frame_parity_error;
	}

	if (status_bit_error || frame_parity_error) {
		return -EIO;
	}

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* BRCM_AEAT9955_PRIV_H_ */
