/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief TI DRV8328 gate driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GATE_DRIVER_TI_DRV8328_H_
#define ZEPHYR_INCLUDE_DRIVERS_GATE_DRIVER_TI_DRV8328_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief User fault callback function pointer type
 * 
 * @param dev DRV8328 device instance
 * @param user_data User data passed during callback registration
 */
typedef void (*drv8328_fault_cb_t)(const struct device *dev, void *user_data);

/**
 * @brief Set DRV8328 sleep mode
 * 
 * @param dev DRV8328 device instance
 * @param sleep true to enter sleep mode, false for active mode
 * @return 0 on success, negative errno on failure
 */
int drv8328_set_sleep_mode(const struct device *dev, bool sleep);

/**
 * @brief Reset DRV8328 fault condition
 * 
 * Generates a precise 1.1μs pulse on the sleep pin to reset latched faults
 * as per DRV8328 datasheet section 8.4.3
 * 
 * @param dev DRV8328 device instance
 * @return 0 on success, negative errno on failure
 */
int drv8328_reset_fault(const struct device *dev);

/**
 * @brief Get current fault status
 * 
 * @param dev DRV8328 device instance
 * @param fault Pointer to store fault status (true if fault detected)
 * @return 0 on success, negative errno on failure
 */
int drv8328_get_fault_status(const struct device *dev, bool *fault);

/**
 * @brief Enable/disable gate drivers (for DRV8328C/D variants)
 * 
 * @param dev DRV8328 device instance
 * @param enable true to enable gate drivers, false to disable
 * @return 0 on success, negative errno on failure
 */
int drv8328_set_gate_drivers(const struct device *dev, bool enable);

/**
 * @brief Set INLx pin state
 * 
 * @param dev DRV8328 device instance
 * @param state true for high, false for low
 * @return 0 on success, negative errno on failure
 */
int drv8328_set_inlx_state(const struct device *dev, bool state);

/**
 * @brief Force INLx pins high (convenience function)
 * 
 * @param dev DRV8328 device instance
 * @return 0 on success, negative errno on failure
 */
int drv8328_force_inlx_high(const struct device *dev);

/**
 * @brief Register user fault callback
 * 
 * Registers a callback function that will be called when a fault interrupt
 * is detected on the fault GPIO pin
 * 
 * @param dev DRV8328 device instance
 * @param callback Fault callback function (NULL to unregister)
 * @param user_data User data to pass to callback
 * @return 0 on success, negative errno on failure
 */
int drv8328_register_fault_callback(const struct device *dev, 
                                   drv8328_fault_cb_t callback, 
                                   void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GATE_DRIVER_TI_DRV8328_H_ */
