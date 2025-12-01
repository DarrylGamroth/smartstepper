/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Broadcom AEAT9955 magnetic encoder sensor driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_BRCM_AEAT9955_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_BRCM_AEAT9955_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/dsp/dsp.h>
#include <zephyr/sys/util.h>

#if !IS_ENABLED(CONFIG_SENSOR)
#error "CONFIG_SENSOR must be enabled to use the AEAT9955 driver"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AEAT9955 Q31 sensor reading
 */
struct aeat9955_q31_reading {
	uint32_t timestamp_delta; /**< Time since base timestamp in nanoseconds */
	q31_t value;              /**< Position in Q31 format for arm_sin_cos_q31 */
};

/**
 * @brief AEAT9955 specific sensor attributes
 */
enum aeat9955_sensor_attribute {
	/** ABZ Resolution setting (0-3 for 1024, 2048, 4096, 8192 PPR) */
	AEAT9955_ATTR_ABZ_RESOLUTION = SENSOR_ATTR_PRIV_START,
	
	/** Zero position setting (16-bit value: val1=high byte, val2=low byte) */
	AEAT9955_ATTR_ZERO_POSITION,
	
	/** Trigger calibration (set val1=1 to start calibration) */
	AEAT9955_ATTR_CALIBRATION,
	
	/** Output options configuration (val1=OPTS0, val2=OPTS1) */
	AEAT9955_ATTR_OUTPUT_OPTIONS,
	
	/** Auto-calibration rotation speed range (0-7 for different RPM ranges) */
	AEAT9955_ATTR_AUTOCAL_FREQ,
	
	/** Calibration status (read-only: bits [7:6] from register 0x113) */
	AEAT9955_ATTR_CAL_STATUS,
	
	/** Rotation direction (0=clockwise, 1=counter-clockwise) */
	AEAT9955_ATTR_ROTATION_DIRECTION,
	
	/** Hysteresis setting (0-7 for different hysteresis levels) */
	AEAT9955_ATTR_HYSTERESIS,
};

/**
 * @brief AEAT9955 encoder resolution
 */
#define AEAT9955_RESOLUTION_BITS 18  /**< 18-bit encoder (262144 counts per revolution) */

/**
 * @brief AEAT9955 angle conversion constants
 */
#define AEAT9955_COUNTS_TO_DEGREES (360.0f / (float)(1U << AEAT9955_RESOLUTION_BITS))  /**< Multiply factor: counts to degrees */

/**
 * @brief ABZ Resolution values
 */
#define AEAT9955_ABZ_RES_1024_PPR   0  /**< 1024 pulses per revolution */
#define AEAT9955_ABZ_RES_2048_PPR   1  /**< 2048 pulses per revolution */
#define AEAT9955_ABZ_RES_4096_PPR   2  /**< 4096 pulses per revolution */
#define AEAT9955_ABZ_RES_8192_PPR   3  /**< 8192 pulses per revolution */

/**
 * @brief Auto-calibration rotation speed range values
 * Based on AEAT9955 datasheet Table: User Auto-Calibration Rotation Speed Register (EEPROM)
 */
#define AEAT9955_AUTOCAL_SPEED_3200_6400_RPM   0  /**< 3200 ≤ Speed < 6400 RPM */
#define AEAT9955_AUTOCAL_SPEED_1600_3200_RPM   1  /**< 1600 ≤ Speed < 3200 RPM */
#define AEAT9955_AUTOCAL_SPEED_800_1600_RPM    2  /**< 800 ≤ Speed < 1600 RPM */
#define AEAT9955_AUTOCAL_SPEED_400_800_RPM     3  /**< 400 ≤ Speed < 800 RPM */
#define AEAT9955_AUTOCAL_SPEED_200_400_RPM     4  /**< 200 ≤ Speed < 400 RPM */
#define AEAT9955_AUTOCAL_SPEED_100_200_RPM     5  /**< 100 ≤ Speed < 200 RPM */
#define AEAT9955_AUTOCAL_SPEED_50_100_RPM      6  /**< 50 ≤ Speed < 100 RPM */
#define AEAT9955_AUTOCAL_SPEED_25_50_RPM       7  /**< 25 ≤ Speed < 50 RPM */

/**
 * @brief STATUS[2:0] values from register 0x0005
 * Based on AEAT9955 datasheet - these indicate chip warnings/status
 */
#define AEAT9955_STATUS_BIT0_ROTATION_OVERSPEED  0x01  /**< Bit 0: Rotation Over Speed Warning */
#define AEAT9955_STATUS_BIT1_WEAK_MAGNETIC       0x02  /**< Bit 1: Weak Magnetic Field Warning */
#define AEAT9955_STATUS_BIT2_UNDER_VOLTAGE       0x04  /**< Bit 2: Under Voltage Warning */

/**
 * @brief Calibration status values from register 0x113 bits [7:6]
 * Used with AEAT9955_ATTR_CAL_STATUS attribute
 */
#define AEAT9955_CAL_STATUS_NONE      0x00  /**< No calibration */
#define AEAT9955_CAL_STATUS_RUNNING   0x40  /**< Running auto calibration */
#define AEAT9955_CAL_STATUS_FAILED    0x80  /**< Calibration failed */
#define AEAT9955_CAL_STATUS_SUCCESS   0xC0  /**< Calibration successful */

/**
 * @brief Rotation direction values
 * Used with AEAT9955_ATTR_ROTATION_DIRECTION attribute
 */
#define AEAT9955_ROTATION_CLOCKWISE       0  /**< Clockwise rotation */
#define AEAT9955_ROTATION_COUNTER_CLOCKWISE  1  /**< Counter-clockwise rotation */

/**
 * @brief Hysteresis values (0-7)
 * Used with AEAT9955_ATTR_HYSTERESIS attribute
 */
#define AEAT9955_HYSTERESIS_MIN  0  /**< Minimum hysteresis */
#define AEAT9955_HYSTERESIS_MAX  7  /**< Maximum hysteresis */

/** Raw SPI frame size returned by the encoder. */
#define AEAT9955_RAW_FRAME_SIZE 5U

/** Status bit indicating a parity failure in the sensor response. */
#define AEAT9955_STATUS_PARITY_BIT 0x80U
/** Status bit indicating a sensor error in the response. */
#define AEAT9955_STATUS_ERROR_BIT  0x40U

/**
 * @brief Metadata recorded for each AEAT-9955 RTIO submission.
 */
struct aeat9955_sample_header {
	uint64_t timestamp_ns; /**< Capture timestamp in nanoseconds. */
};

/**
 * @brief Buffer layout produced by the AEAT-9955 RTIO driver.
 */
struct aeat9955_sample {
	struct aeat9955_sample_header header;
	uint8_t raw[AEAT9955_RAW_FRAME_SIZE]; /**< Raw SPI response bytes. */
};

/**
 * @brief Get the sensor decoder API for AEAT9955
 * 
 * @param dev AEAT9955 device instance
 * @param decoder Pointer to store the decoder API
 * @return 0 on success, negative error code on failure
 */
int aeat9955_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

/**
 * @brief Fast inline position decode for ISR use (Q31 format)
 *
 * Optimized for zero-latency ISR - bypasses all validation and API overhead.
 * Decodes raw AEAT9955 RTIO buffer directly to Q31 angle format.
 *
 * @warning This function assumes:
 * - buffer points to valid aeat9955_sample structure
 * - No status/parity error checking (use only for trusted data paths)
 * - Caller handles error detection separately if needed
 *
 * @param buffer Pointer to aeat9955_sample from RTIO completion
 * @return Position in Q31 format [-2^31, 2^31-1] for arm_sin_cos_q31
 *         Maps 0° to 360° encoder range to -180° to +180° Q31 range
 */
static inline q31_t aeat9955_decode_position_q31(const uint8_t *buffer)
{
	const struct aeat9955_sample *sample = (const struct aeat9955_sample *)buffer;
	/* Extract 18-bit position from bytes [2:4], bits [17:0] after status bits */
	uint32_t position = sys_get_be24(&sample->raw[2]) >> 6;
	/* Convert to Q31 centered at 0: maps [0, 262143] to [-2^31, 2^31-1] */
	return (q31_t)((position << (32 - AEAT9955_RESOLUTION_BITS)) - 0x80000000UL);
}

/**
 * @brief Fast inline position decode for ISR use (float32 degrees)
 *
 * Optimized for zero-latency ISR - bypasses all validation and API overhead.
 * Decodes raw AEAT9955 RTIO buffer directly to float angle in degrees.
 *
 * @warning This function assumes:
 * - buffer points to valid aeat9955_sample structure
 * - No status/parity error checking (use only for trusted data paths)
 * - Caller handles error detection separately if needed
 *
 * @param buffer Pointer to aeat9955_sample from RTIO completion
 * @return Position in degrees [-180.0f, +180.0f)
 *         Maps 0° to 360° encoder range to -180° to +180° degrees
 */
static inline float aeat9955_decode_position_f32(const uint8_t *buffer)
{
	const struct aeat9955_sample *sample = (const struct aeat9955_sample *)buffer;
	/* Extract 18-bit position from bytes [2:4], bits [17:0] after status bits */
	int32_t position = (int32_t)(sys_get_be24(&sample->raw[2]) >> 6);
	/* Convert to degrees centered at 0: [0, 262143] -> [-180.0, +180.0) */
	return (float)(position - (1 << (AEAT9955_RESOLUTION_BITS - 1))) * AEAT9955_COUNTS_TO_DEGREES;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_BRCM_AEAT9955_H_ */
