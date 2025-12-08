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
 * @brief Enable a specific gate driver channel
 * 
 * In 3x mode: Sets INL GPIO high, then enables PWM on INH
 * In 6x mode: Enables PWM with complementary mode on INH/INL pair
 * 
 * @param dev DRV8328 device instance
 * @param channel Channel index (0-2)
 * @return 0 on success, negative errno on failure
 */
int drv8328_enable_channel(const struct device *dev, uint8_t channel);

/**
 * @brief Disable a specific gate driver channel
 * 
 * In 3x mode: Disables PWM on INH, sets INL GPIO low
 * In 6x mode: Disables PWM with complementary mode
 * 
 * @param dev DRV8328 device instance
 * @param channel Channel index (0-2)
 * @return 0 on success, negative errno on failure
 */
int drv8328_disable_channel(const struct device *dev, uint8_t channel);

/**
 * @brief Disable all channels immediately
 * 
 * Disables all gate driver channels.
 * 
 * @param dev DRV8328 device instance
 * @return 0 on success, negative errno on failure
 */
int drv8328_disable_all_channels(const struct device *dev);

/**
 * @brief Get PWM device pointer for a channel
 * 
 * Returns the PWM device pointer for direct ISR access.
 * Used to cache PWM pointers in motor control structs for
 * high-performance ISR duty cycle updates.
 * 
 * @param dev DRV8328 device instance
 * @param channel Channel index (0-2)
 * @return PWM device pointer, or NULL on failure
 */
const struct device *drv8328_get_pwm_device(const struct device *dev, uint8_t channel);

/**
 * @brief Get PWM channel number for a channel
 * 
 * Returns the PWM channel number for direct ISR access.
 * Used with drv8328_get_pwm_device() to cache channel info
 * for high-performance ISR duty cycle updates.
 * 
 * @param dev DRV8328 device instance
 * @param channel Channel index (0-2)
 * @param pwm_channel Pointer to store PWM channel number
 * @return 0 on success, negative errno on failure
 */
int drv8328_get_pwm_channel(const struct device *dev, uint8_t channel, uint32_t *pwm_channel);

/**
 * @brief Set DRV8328 sleep mode
 * 
 * Controls the nSLEEP pin to enter/exit sleep mode.
 * When entering sleep, automatically disables all channels first.
 * When exiting sleep, waits tWAKE before returning.
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
 * as per DRV8328 datasheet section 8.4.3. Waits tWAKE (100μs) before returning.
 * Clears internal fault state.
 * 
 * @param dev DRV8328 device instance
 * @return 0 on success, negative errno on failure
 */
int drv8328_reset_fault(const struct device *dev);

/**
 * @brief Get current fault status
 * 
 * Returns the current fault state from the last nFAULT interrupt.
 * 
 * @param dev DRV8328 device instance
 * @param fault Pointer to store fault status (true if fault detected)
 * @return 0 on success, negative errno on failure
 */
int drv8328_get_fault_status(const struct device *dev, bool *fault);

/**
 * @brief Enable/disable gate drivers (for DRV8328C/D variants)
 * 
 * Controls the DRVOFF pin to enable/disable all gate drivers.
 * Independent of PWM state - provides additional safety layer.
 * 
 * @param dev DRV8328 device instance
 * @param enable true to enable gate drivers, false to disable
 * @return 0 on success, negative errno on failure
 */
int drv8328_set_gate_drivers(const struct device *dev, bool enable);

/**
 * @brief Register user fault callback
 * 
 * Registers a callback function that will be called when a fault interrupt
 * is detected on the nFAULT GPIO pin. The callback is invoked AFTER
 * automatic emergency_stop.
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
