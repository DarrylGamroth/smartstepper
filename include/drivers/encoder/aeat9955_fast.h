/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ENCODER_AEAT9955_FAST_H_
#define ZEPHYR_INCLUDE_DRIVERS_ENCODER_AEAT9955_FAST_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AEAT9955_FAST_RESOLUTION_BITS 18U
#define AEAT9955_FAST_MAX_COUNT       (1U << AEAT9955_FAST_RESOLUTION_BITS)
#define AEAT9955_FAST_COUNTS_TO_DEGREES \
	(360.0f / (float)AEAT9955_FAST_MAX_COUNT)

#define AEAT9955_FAST_CMD_READ_SPI16  0x40U
#define AEAT9955_FAST_CMD_WRITE_SPI16 0x00U
#define AEAT9955_FAST_CMD_READ_SPI8   0x81U
#define AEAT9955_FAST_CMD_WRITE_SPI8  0xCFU
#define AEAT9955_FAST_CMD_POS_SPI8    0xA6U

enum aeat9955_fast_spi4_mode {
	AEAT9955_FAST_SPI4_16_PARITY = 0,
	AEAT9955_FAST_SPI4_8_CRC16 = 1,
};

#define AEAT9955_FAST_REG_CONFIG0      0x00U
#define AEAT9955_FAST_REG_CONFIG0_SPI4 0x07U
#define AEAT9955_FAST_REG_CONFIG1_PSEL 0x09U
#define AEAT9955_FAST_REG_UNLOCK       0x10U
#define AEAT9955_FAST_REG_POS          0x3FU
#define AEAT9955_FAST_REG_ERROR_STATUS 0x29U

#define AEAT9955_FAST_CONFIG0_SAFETY_BIT     0x80U
#define AEAT9955_FAST_CONFIG0_CRC_SELECT     0x40U
#define AEAT9955_FAST_CONFIG0_CRC_INIT_MASK  0x30U
#define AEAT9955_FAST_CONFIG0_CRC_INIT_FFFF  0x30U
#define AEAT9955_FAST_CONFIG0_SPI4_MODE_MASK 0xC0U
#define AEAT9955_FAST_CONFIG0_SPI4_MODE_16   0x00U
#define AEAT9955_FAST_CONFIG0_SPI4_MODE_8    0xC0U
#define AEAT9955_FAST_CONFIG1_PSEL_BIT       0x80U
#define AEAT9955_FAST_UNLOCK_LEVEL1          0xABU

#define AEAT9955_FAST_POS_STATUS_PARITY_BIT 0x80U
#define AEAT9955_FAST_POS_STATUS_ERROR_BIT  0x40U
#define AEAT9955_FAST_FRAME_STATUS_MASK \
	(AEAT9955_FAST_POS_STATUS_PARITY_BIT | AEAT9955_FAST_POS_STATUS_ERROR_BIT)

#define AEAT9955_FAST_ERROR_MHI_BIT 0x20U
#define AEAT9955_FAST_ERROR_MLO_BIT 0x10U

int aeat9955_fast_read_register(const struct device *dev, uint8_t reg, uint8_t *value);
int aeat9955_fast_write_register(const struct device *dev, uint8_t reg, uint8_t value);
int aeat9955_fast_get_spi4_mode(const struct device *dev,
				enum aeat9955_fast_spi4_mode *mode);
int aeat9955_fast_set_spi4_mode_runtime(const struct device *dev,
					enum aeat9955_fast_spi4_mode mode);
int aeat9955_fast_configure_spi4_8_crc16_volatile(const struct device *dev);
int aeat9955_fast_configure_spi4_16_parity_volatile(const struct device *dev);
int aeat9955_fast_read_position_raw(const struct device *dev, uint8_t *raw,
				    uint8_t raw_len, uint8_t *frame_len);
int aeat9955_fast_read_register_raw(const struct device *dev, uint8_t reg,
				    uint8_t *raw, uint8_t raw_len,
				    uint8_t *frame_len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ENCODER_AEAT9955_FAST_H_ */
