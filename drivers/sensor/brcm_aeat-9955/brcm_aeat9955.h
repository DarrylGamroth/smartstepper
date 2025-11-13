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
#define AEAT9955_CMD_READ_BIT  0x40U
#define AEAT9955_CMD_WRITE_BIT 0x00U
#define AEAT9955_CMD_READ      (0x03U << 4)
#define AEAT9955_CMD_WRITE     (0x06U << 4)
#define AEAT9955_CMD_PROG      (0x0BU << 4)
#define AEAT9955_CMD_ZERO      (0x05U << 4)
#define AEAT9955_CMD_ANGLE     (0x0AU << 4)

/* AEAT9955 Register addresses */
#define AEAT9955_REG_POS      0x3FU
#define AEAT9955_REG_USER_ID  0x00U
#define AEAT9955_REG_ANGLE_H  0x03U
#define AEAT9955_REG_ANGLE_M  0x04U
#define AEAT9955_REG_ANGLE_L  0x05U
#define AEAT9955_REG_CRC      0x06U
#define AEAT9955_REG_ABZ_RES1 0x07U
#define AEAT9955_REG_ABZ_RES2 0x08U
#define AEAT9955_REG_ZERO1    0x09U
#define AEAT9955_REG_ZERO2    0x0AU
#define AEAT9955_REG_OPTS0    0x0AU
#define AEAT9955_REG_OPTS1    0x0BU
#define AEAT9955_REG_OPTS2    0x0CU
#define AEAT9955_REG_OPTS3    0x0DU
#define AEAT9955_REG_OPTS4    0x0EU
#define AEAT9955_REG_OPTS5    0x11U

/* Conversion constants */
#define AEAT9955_FULL_ANGLE     360
#define AEAT9955_PULSES_PER_REV 262144
#define AEAT9955_MILLION_UNIT   1000000

/* AEAT9955 has 18-bit resolution (2^18 = 262144 counts per revolution) */
#define AEAT9955_RESOLUTION_BITS 18
#define AEAT9955_MAX_COUNT       (1U << AEAT9955_RESOLUTION_BITS)

/**
 * @brief Get the sensor decoder API for AEAT9955
 * 
 * @param dev AEAT9955 device instance
 * @param decoder Pointer to store the decoder API
 * @return 0 on success, negative error code on failure
 */
int aeat9955_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

#ifdef __cplusplus
}
#endif

#endif /* BRCM_AEAT9955_PRIV_H_ */
