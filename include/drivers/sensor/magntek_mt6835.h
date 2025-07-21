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

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAGNTEK_MT6835_H_ */
