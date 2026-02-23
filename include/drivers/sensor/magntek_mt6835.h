/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Magntek MT6835 magnetic encoder sensor driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAGNTEK_MT6835_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAGNTEK_MT6835_H_

#include <errno.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/dsp/dsp.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MT6835 Q31 sensor reading
 */
struct mt6835_q31_reading {
	uint32_t timestamp_delta; /**< Time since base timestamp in nanoseconds */
	q31_t value;              /**< Position in Q31 format for arm_sin_cos_q31 */
};

/**
 * @brief MT6835 specific sensor attributes
 */
enum mt6835_sensor_attribute {
	/** ABZ Resolution setting (0-3 for 1024, 2048, 4096, 8192 PPR) */
	MT6835_ATTR_ABZ_RESOLUTION = SENSOR_ATTR_PRIV_START,
	
	/** Zero position setting (16-bit value: val1=high byte, val2=low byte) */
	MT6835_ATTR_ZERO_POSITION,
	
	/** Trigger calibration (set val1=1 to start calibration) */
	MT6835_ATTR_CALIBRATION,
	
	/** Output options configuration (val1=OPTS0, val2=OPTS1) */
	MT6835_ATTR_OUTPUT_OPTIONS,
	
	/** Auto-calibration rotation speed range (0-7 for different RPM ranges) */
	MT6835_ATTR_AUTOCAL_FREQ,
	
	/** Calibration status (read-only: bits [7:6] from register 0x113) */
	MT6835_ATTR_CAL_STATUS,
	
	/** Rotation direction (0=clockwise, 1=counter-clockwise) */
	MT6835_ATTR_ROTATION_DIRECTION,
	
	/** Hysteresis setting (0-7 for different hysteresis levels) */
	MT6835_ATTR_HYSTERESIS,
};

/**
 * @brief ABZ Resolution values
 */
#define MT6835_ABZ_RES_1024_PPR   0  /**< 1024 pulses per revolution */
#define MT6835_ABZ_RES_2048_PPR   1  /**< 2048 pulses per revolution */
#define MT6835_ABZ_RES_4096_PPR   2  /**< 4096 pulses per revolution */
#define MT6835_ABZ_RES_8192_PPR   3  /**< 8192 pulses per revolution */

/**
 * @brief MT6835 encoder resolution
 */
#define MT6835_RESOLUTION_BITS 21  /**< 21-bit encoder (2097152 counts per revolution) */
#define MT6835_MAX_COUNT       (1U << MT6835_RESOLUTION_BITS)

/**
 * @brief MT6835 angle conversion constants
 */
#define MT6835_COUNTS_TO_DEGREES (360.0f / (float)(1U << MT6835_RESOLUTION_BITS))  /**< Multiply factor: counts to degrees */

/**
 * @brief Auto-calibration rotation speed range values
 * Based on MT6835 datasheet Table: User Auto-Calibration Rotation Speed Register (EEPROM)
 */
#define MT6835_AUTOCAL_SPEED_3200_6400_RPM   0  /**< 3200 ≤ Speed < 6400 RPM */
#define MT6835_AUTOCAL_SPEED_1600_3200_RPM   1  /**< 1600 ≤ Speed < 3200 RPM */
#define MT6835_AUTOCAL_SPEED_800_1600_RPM    2  /**< 800 ≤ Speed < 1600 RPM */
#define MT6835_AUTOCAL_SPEED_400_800_RPM     3  /**< 400 ≤ Speed < 800 RPM */
#define MT6835_AUTOCAL_SPEED_200_400_RPM     4  /**< 200 ≤ Speed < 400 RPM */
#define MT6835_AUTOCAL_SPEED_100_200_RPM     5  /**< 100 ≤ Speed < 200 RPM */
#define MT6835_AUTOCAL_SPEED_50_100_RPM      6  /**< 50 ≤ Speed < 100 RPM */
#define MT6835_AUTOCAL_SPEED_25_50_RPM       7  /**< 25 ≤ Speed < 50 RPM */

/**
 * @brief STATUS[2:0] values from register 0x0005
 * Based on MT6835 datasheet - these indicate chip warnings/status
 */
#define MT6835_STATUS_BIT0_ROTATION_OVERSPEED  0x01  /**< Bit 0: Rotation Over Speed Warning */
#define MT6835_STATUS_BIT1_WEAK_MAGNETIC       0x02  /**< Bit 1: Weak Magnetic Field Warning */
#define MT6835_STATUS_BIT2_UNDER_VOLTAGE       0x04  /**< Bit 2: Under Voltage Warning */
#define MT6835_STATUS_MASK                      0x07  /**< STATUS bits [2:0] in fast angle frame */

/**
 * @brief Calibration status values from register 0x113 bits [7:6]
 * Used with MT6835_ATTR_CAL_STATUS attribute
 */
#define MT6835_CAL_STATUS_NONE      0x00  /**< No calibration */
#define MT6835_CAL_STATUS_RUNNING   0x40  /**< Running auto calibration */
#define MT6835_CAL_STATUS_FAILED    0x80  /**< Calibration failed */
#define MT6835_CAL_STATUS_SUCCESS   0xC0  /**< Calibration successful */

/**
 * @brief Rotation direction values
 * Used with MT6835_ATTR_ROTATION_DIRECTION attribute
 */
#define MT6835_ROTATION_CLOCKWISE       0  /**< Clockwise rotation */
#define MT6835_ROTATION_COUNTER_CLOCKWISE  1  /**< Counter-clockwise rotation */

/**
 * @brief Hysteresis values (0-7)
 * Used with MT6835_ATTR_HYSTERESIS attribute
 */
#define MT6835_HYSTERESIS_MIN  0  /**< Minimum hysteresis */
#define MT6835_HYSTERESIS_MAX  7  /**< Maximum hysteresis */

/** Raw SPI frame size returned by the encoder. */
#define MT6835_RAW_FRAME_SIZE 6U
#define MT6835_FRAME_DATA_OFFSET 2U
#define MT6835_FRAME_DATA_LEN    3U
#define MT6835_FRAME_CRC_OFFSET  5U

/**
 * @brief Metadata recorded for each MT6835 RTIO submission.
 */
struct mt6835_sample_header {
	uint64_t timestamp_ns; /**< Capture timestamp in nanoseconds. */
};

/**
 * @brief Buffer layout produced by the MT6835 RTIO driver.
 */
struct mt6835_sample {
	struct mt6835_sample_header header;
	uint8_t raw[MT6835_RAW_FRAME_SIZE]; /**< Raw SPI response bytes. */
};
/**
 * @brief Get the sensor decoder API for MT6835
 * 
 * @param dev MT6835 device instance
 * @param decoder Pointer to store the decoder API
 * @return 0 on success, negative error code on failure
 */
int mt6835_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

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

	if (position != NULL) {
		*position = (sys_get_be24(&raw_buf[MT6835_FRAME_DATA_OFFSET]) >> 3) &
			    (MT6835_MAX_COUNT - 1U);
	}

	return 0;
}

/**
 * @brief Decode full MT6835 RTIO sample into angle and frame status flags
 *
 * @param buffer Pointer to `struct mt6835_sample`
 * @param angle_deg Output: decoded angle in degrees, centered [-180, 180)
 * @param status Output: STATUS bits [2:0] from angle frame
 * @param warning Output: warning flag (kept false for now; see status output)
 * @param error Output: true on CRC failure
 * @param status_error_out Output: status-error flag (always false for MT6835)
 * @param parity_error_out Output: frame-check error flag (mapped from CRC)
 * @return 0 on success, -EIO on CRC mismatch
 */
static inline int mt6835_decode_sample_f32(const uint8_t *buffer, float *angle_deg, uint8_t *status,
					   bool *warning, bool *error, bool *status_error_out,
					   bool *parity_error_out)
{
	const struct mt6835_sample *sample = (const struct mt6835_sample *)buffer;
	uint32_t position = 0U;
	bool crc_error = false;
	int ret = mt6835_decode_position(sample->raw, &position, &crc_error);

	if (angle_deg != NULL) {
		*angle_deg = (float)((int32_t)position - (1 << (MT6835_RESOLUTION_BITS - 1))) *
			     MT6835_COUNTS_TO_DEGREES;
	}
	if (status != NULL) {
		*status = sample->raw[4] & MT6835_STATUS_MASK;
	}
	if (warning != NULL) {
		/* Keep advisory status bits non-faulting in control-loop path for now. */
		*warning = false;
	}
	if (error != NULL) {
		*error = crc_error;
	}
	if (status_error_out != NULL) {
		*status_error_out = false;
	}
	if (parity_error_out != NULL) {
		/* Preserve pipeline stats/counter naming for existing tooling. */
		*parity_error_out = crc_error;
	}

	return ret;
}

/**
 * @brief Fast inline position decode for ISR use (Q31 format)
 *
 * Optimized for zero-latency ISR - bypasses all validation and API overhead.
 * Decodes raw MT6835 RTIO buffer directly to Q31 angle format.
 *
 * @warning This function assumes:
 * - buffer points to valid mt6835_sample structure
 * - No status/parity error checking (use only for trusted data paths)
 * - Caller handles error detection separately if needed
 *
 * @param buffer Pointer to mt6835_sample from RTIO completion
 * @return Position in Q31 format [-2^31, 2^31-1] for arm_sin_cos_q31
 *         Maps 0° to 360° encoder range to -180° to +180° Q31 range
 */
static inline q31_t mt6835_decode_position_q31(const uint8_t *buffer)
{
	const struct mt6835_sample *sample = (const struct mt6835_sample *)buffer;
	/* Extract 21-bit position from bytes [2:4], bits [20:0] after status bits */
	uint32_t position = sys_get_be24(&sample->raw[2]) >> 3;
	/* Convert to Q31 centered at 0: maps [0, 2097151] to [-2^31, 2^31-1] */
	return (q31_t)((position << (32 - MT6835_RESOLUTION_BITS)) - 0x80000000UL);
}

/**
 * @brief Fast inline position decode for ISR use (float32 degrees)
 *
 * Optimized for zero-latency ISR - bypasses all validation and API overhead.
 * Decodes raw MT6835 RTIO buffer directly to float angle in degrees.
 *
 * @warning This function assumes:
 * - buffer points to valid mt6835_sample structure
 * - No status/parity error checking (use only for trusted data paths)
 * - Caller handles error detection separately if needed
 *
 * @param buffer Pointer to mt6835_sample from RTIO completion
 * @return Position in degrees [-180.0f, +180.0f)
 *         Maps 0° to 360° encoder range to -180° to +180° degrees
 */
static inline float mt6835_decode_position_f32(const uint8_t *buffer)
{
	const struct mt6835_sample *sample = (const struct mt6835_sample *)buffer;
	/* Extract 21-bit position from bytes [2:4], bits [20:0] after status bits */
	int32_t position = (int32_t)(sys_get_be24(&sample->raw[2]) >> 3);
	/* Convert to degrees centered at 0: [0, 2097151] -> [-180.0, +180.0) */
	return (float)(position - (1 << (MT6835_RESOLUTION_BITS - 1))) * MT6835_COUNTS_TO_DEGREES;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAGNTEK_MT6835_H_ */
