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

#define AEAT9955_FAST_REG_POS          0x3FU
#define AEAT9955_FAST_REG_ERROR_STATUS 0x29U

#define AEAT9955_FAST_POS_STATUS_PARITY_BIT 0x80U
#define AEAT9955_FAST_POS_STATUS_ERROR_BIT  0x40U
#define AEAT9955_FAST_FRAME_STATUS_MASK \
	(AEAT9955_FAST_POS_STATUS_PARITY_BIT | AEAT9955_FAST_POS_STATUS_ERROR_BIT)

#define AEAT9955_FAST_ERROR_MHI_BIT 0x20U
#define AEAT9955_FAST_ERROR_MLO_BIT 0x10U

int aeat9955_fast_read_register(const struct device *dev, uint8_t reg, uint8_t *value);
int aeat9955_fast_write_register(const struct device *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ENCODER_AEAT9955_FAST_H_ */
