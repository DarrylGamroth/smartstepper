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

/* AEAT-9955 encoder specifications */
#define AEAT9955_RESOLUTION_BITS 18 /**< 18-bit absolute position resolution */
#define AEAT9955_MAX_COUNT       (1U << AEAT9955_RESOLUTION_BITS) /**< Maximum count value (262144) */
#define AEAT9955_COUNTS_TO_DEGREES (360.0f / (float)AEAT9955_MAX_COUNT) /**< Conversion factor */

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

	/** Zero position setting (18-bit value: val1=MSB bits[17:10], val2=bits[9:2]) */
	AEAT9955_ATTR_ZERO_POSITION,

	/** Trigger accuracy angle calibration (set val1=1 to start, motor must rotate 10-2000 RPM) */
	AEAT9955_ATTR_AUTO_CALIBRATION,

	/** Trigger zero reset calibration (set val1=1, encoder must be stationary at desired zero) */
	AEAT9955_ATTR_ZERO_RESET,

	/** Calibration status (read-only: val1=accuracy cal status [1:0], val2=zero reset status [3:2]) */
	AEAT9955_ATTR_CAL_STATUS,

	/** Multi-index pulses per revolution (0=1, 1=2, 2=4, ..., 7=128 pulses) */
	AEAT9955_ATTR_MULTI_INDEX,

	/** Sensing axis configuration (0=On-Axis, 1=Off-Axis radial, 2=Off-Axis axial, 3=Off-Axis side shaft) */
	AEAT9955_ATTR_SENSING_AXIS,

	/** Incremental resolution (15-bit value for CPR: 0=OFF, 1=1 CPR, ..., 20000=20,000 CPR) */
	AEAT9955_ATTR_INCREMENTAL_RESOLUTION,

	/** Rotation direction (0=clockwise, 1=counter-clockwise) */
	AEAT9955_ATTR_DIRECTION,

	/** Hysteresis setting (0-7 for different hysteresis levels) */
	AEAT9955_ATTR_HYSTERESIS,

	/** UVW output resolution (6-bit value: 0=OFF, 1=1 pole pair, ..., 31=31 pole pairs, 32=32 pole pairs) */
	AEAT9955_ATTR_UVW_RESOLUTION,

	/** Single turn resolution (4-bit value: 0=18bit, 1=17bit, ..., 8=10bit minimum) */
	AEAT9955_ATTR_SINGLE_TURN_RESOLUTION,

	/** Program current configuration to EEPROM (set val1=1 to save to non-volatile memory) */
	AEAT9955_ATTR_EEPROM_PROGRAM,

	/** Alarm latch configuration (0=triggered alarm resets once error recovered, 1=alarm remains until user clears or power-cycles) */
	AEAT9955_ATTR_ALARM_LATCH,

	/** SPI output high-impedance mode (0=enable high-z, 1=disable high-z for multi-slave or bus connection) */
	AEAT9955_ATTR_SPI_HIGHZ,

	/** Magnetic field high limit (val1=0-15, 0=lowest, 15=highest, default=1001b=10) */
	AEAT9955_ATTR_MAGNETIC_HIGH,

	/** Magnetic field low limit (val1=0-15, 0=lowest, 15=highest, default=0101b=5) */
	AEAT9955_ATTR_MAGNETIC_LOW,

	/** Vertical Hall selection for off-axis configurations (4-bit value, see datasheet) */
	AEAT9955_ATTR_VERTICAL_HALL_SEL,

	/** PWM resolution/frequency (0-15, see datasheet for fixed period or clock settings) */
	AEAT9955_ATTR_PWM_RESOLUTION,

	/** Index state configuration (0=A low B low, 1=A low B high, 2=A high B high, 3=A high B low) */
	AEAT9955_ATTR_INDEX_STATE,

	/** Index width configuration (0=90°, 1=180e°, 2=270e°, 3=360e°) */
	AEAT9955_ATTR_INDEX_WIDTH,

	/** Protocol mode selection PSEL (0=SSI3a/SSI2a/All SPI4 modes, 1=SSI3b/SSI2b/PWM) */
	AEAT9955_ATTR_PROTOCOL_MODE,

	/** Auto-calibration hardware enable via M1 pin (0=disable, 1=enable calibration on M1 pin) */
	AEAT9955_ATTR_AUTO_CAL_HARDWARE,

	/** Raw alarm/error status byte from register 0x29 (Chip Status) */
	AEAT9955_ATTR_ERROR_STATUS,

	/** Magnet high alarm status from register 0x29 bit 5 (0=OK, 1=alarm) */
	AEAT9955_ATTR_ALARM_MAGNET_HIGH,

	/** Magnet low alarm status from register 0x29 bit 4 (0=OK, 1=alarm) */
	AEAT9955_ATTR_ALARM_MAGNET_LOW,
};

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
	uint8_t raw[3]; /**< Raw 3-byte SPI frame from encoder */
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
	uint32_t position = (sys_get_be24(sample->raw) >> 4) & (AEAT9955_MAX_COUNT - 1);
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
	int32_t position = (int32_t)((sys_get_be24(sample->raw) >> 4) &
					 (AEAT9955_MAX_COUNT - 1));
	/* Convert to degrees centered at 0: [0, 262143] -> [-180.0, +180.0) */
	return (float)(position - (1 << (AEAT9955_RESOLUTION_BITS - 1))) * AEAT9955_COUNTS_TO_DEGREES;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_BRCM_AEAT9955_H_ */
