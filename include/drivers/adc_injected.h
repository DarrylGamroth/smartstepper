/*
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public Injected Channel ADC Driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_INJECTED_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_INJECTED_H_

/**
 * @brief Injected Channel ADC Interface
 * @defgroup adc_injected_interface Injected Channel ADC Interface
 * @since 1.0
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dsp/types.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Injected ADC conversion complete callback signature.
 *
 * @warning This callback is invoked directly from a zero-latency ISR context.
 *          NO kernel calls are allowed (no k_*, LOG_*, device APIs, etc.).
 *          Only direct hardware register writes and simple data updates permitted.
 *
 * Invoked immediately after hardware-triggered conversion sequence completes.
 * Typical use: Read ADC values and update PWM duty cycles for motor control.
 *
 * @param dev ADC device instance that generated the event.
 * @param values Array of Q31 fixed-point ADC values (up to 4 channels).
 *               Values are in Q31 format where 0x7FFFFFFF represents full-scale.
 * @param count Number of valid values in the array.
 * @param user_data Pointer supplied during callback registration.
 */
typedef void (*adc_injected_callback_t)(const struct device *dev,
					const q31_t *values,
					uint8_t count,
					void *user_data);

/** @name Injected ADC Error Flags
 * @{
 */
/** Queue overflow / overrun error - conversion completed before previous was read */
#define ADC_INJ_ERROR_OVERRUN    BIT(0)
/** @} */

/**
 * @brief Injected ADC error callback signature.
 *
 * @warning This callback is invoked directly from a zero-latency ISR context.
 *          NO kernel calls are allowed (no k_*, LOG_*, device APIs, etc.).
 *
 * Invoked when an error condition is detected (e.g., conversion overrun).
 * This indicates a serious timing issue - conversions are completing faster
 * than they can be processed.
 *
 * @param dev ADC device instance that generated the error.
 * @param error_flags Bitfield of ADC_INJ_ERROR_* flags.
 * @param user_data Pointer supplied during callback registration.
 */
typedef void (*adc_injected_error_callback_t)(const struct device *dev,
					      uint32_t error_flags,
					      void *user_data);

/**
 * @brief Container for injected ADC information specified in devicetree.
 *
 * @see ADC_INJECTED_DT_SPEC_GET
 */
struct adc_injected_dt_spec {
	/** ADC device instance. */
	const struct device *dev;
	/** Number of channels configured. */
	uint8_t num_channels;
};

/**
 * @brief Static initializer for a struct adc_injected_dt_spec
 *
 * Example devicetree fragment:
 *
 * @code{.dts}
 *    &adc1 {
 *        compatible = "st,stm32-adc-injected";
 *        status = "okay";
 *        st,adc-trigger-source = "tim1_cc4";
 *        st,adc-trigger-edge = "rising";
 *        
 *        channel@3 {
 *            reg = <3>;
 *            zephyr,acquisition-time = <ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 2)>;
 *        };
 *        
 *        channel@10 {
 *            reg = <10>;
 *        };
 *    };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *    static const struct adc_injected_dt_spec adc_spec =
 *        ADC_INJECTED_DT_SPEC_GET(DT_NODELABEL(adc1));
 * @endcode
 *
 * @param node_id Devicetree node identifier.
 *
 * @return Static initializer for an adc_injected_dt_spec structure.
 */
#define ADC_INJECTED_DT_SPEC_GET(node_id)				\
	{								\
		.dev = DEVICE_DT_GET(node_id),				\
		.num_channels = DT_PROP(node_id, st_adc_num_channels),	\
	}

/**
 * @brief Static initializer for a struct adc_injected_dt_spec from a DT_DRV_COMPAT instance.
 *
 * @param inst DT_DRV_COMPAT instance number
 *
 * @return Static initializer for an adc_injected_dt_spec structure.
 *
 * @see ADC_INJECTED_DT_SPEC_GET
 */
#define ADC_INJECTED_DT_SPEC_INST_GET(inst)				\
	ADC_INJECTED_DT_SPEC_GET(DT_DRV_INST(inst))

/** @cond INTERNAL_HIDDEN */
/**
 * @brief Injected ADC driver API call to set callback.
 * @see adc_injected_set_callback() for argument description.
 */
typedef int (*adc_injected_set_callback_t)(const struct device *dev,
					   adc_injected_callback_t cb,
					   void *user_data);

/**
 * @brief Injected ADC driver API call to set error callback.
 * @see adc_injected_set_error_callback() for argument description.
 */
typedef int (*adc_injected_set_error_callback_t)(const struct device *dev,
							 adc_injected_error_callback_t cb,
							 void *user_data);

/**
 * @brief Injected ADC driver API call to enable conversions.
 * @see adc_injected_enable() for argument description.
 */
typedef int (*adc_injected_enable_t)(const struct device *dev);

/**
 * @brief Injected ADC driver API call to disable conversions.
 * @see adc_injected_disable() for argument description.
 */
typedef int (*adc_injected_disable_t)(const struct device *dev);

/** @brief Injected ADC driver API definition. */
__subsystem struct adc_injected_driver_api {
	adc_injected_set_callback_t set_callback;
	adc_injected_set_error_callback_t set_error_callback;
	adc_injected_enable_t enable;
	adc_injected_disable_t disable;
};
/** @endcond */

/**
 * @brief Set callback for conversion complete notification.
 *
 * Registers a callback to be invoked directly from zero-latency ISR context
 * after each hardware-triggered conversion sequence completes.
 *
 * @warning The callback runs in ISR context with NO kernel calls allowed.
 *          Do NOT use LOG_*, k_*, or device APIs in the callback.
 *          Only direct hardware register writes and data updates permitted.
 *
 * @param dev Injected ADC device instance.
 * @param cb Callback function, or NULL to clear.
 * @param user_data Pointer passed to callback when invoked.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int adc_injected_set_callback(const struct device *dev,
					adc_injected_callback_t cb,
					void *user_data);

static inline int z_impl_adc_injected_set_callback(const struct device *dev,
						    adc_injected_callback_t cb,
						    void *user_data)
{
	return DEVICE_API_GET(adc_injected, dev)->set_callback(dev, cb, user_data);
}

/**
 * @brief Enable injected ADC conversions.
 *
 * Enables hardware-triggered injected channel conversions. The ADC will
 * begin converting on each trigger event from the configured source
 * (e.g., TIM1_CC4).
 *
 * @param dev Injected ADC device instance.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int adc_injected_enable(const struct device *dev);

static inline int z_impl_adc_injected_enable(const struct device *dev)
{
	return DEVICE_API_GET(adc_injected, dev)->enable(dev);
}

/**
 * @brief Disable injected ADC conversions.
 *
 * Stops hardware-triggered injected channel conversions. The ADC will
 * no longer respond to trigger events.
 *
 * @param dev Injected ADC device instance.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int adc_injected_disable(const struct device *dev);

static inline int z_impl_adc_injected_disable(const struct device *dev)
{
	return DEVICE_API_GET(adc_injected, dev)->disable(dev);
}

/**
 * @brief Set error callback for fault notification.
 *
 * Registers a callback to be invoked from zero-latency ISR context when
 * an error condition is detected (e.g., conversion overrun).
 *
 * @warning The callback runs in ISR context with NO kernel calls allowed.
 *
 * @param dev Injected ADC device instance.
 * @param cb Error callback function, or NULL to clear.
 * @param user_data Pointer passed to callback when invoked.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int adc_injected_set_error_callback(const struct device *dev,
					      adc_injected_error_callback_t cb,
					      void *user_data);

static inline int z_impl_adc_injected_set_error_callback(const struct device *dev,
							  adc_injected_error_callback_t cb,
							  void *user_data)
{
	const struct adc_injected_driver_api *api = DEVICE_API_GET(adc_injected, dev);
	
	if (api->set_error_callback == NULL) {
		return -ENOSYS;
	}
	
	return api->set_error_callback(dev, cb, user_data);
}

/**
 * @brief Set callback from a struct adc_injected_dt_spec.
 *
 * This is equivalent to:
 *
 *     adc_injected_set_callback(spec->dev, cb, user_data)
 *
 * @param spec Injected ADC specification from devicetree.
 * @param cb Callback function, or NULL to clear.
 * @param user_data Pointer passed to callback when invoked.
 *
 * @return A value from adc_injected_set_callback().
 */
static inline int adc_injected_set_callback_dt(const struct adc_injected_dt_spec *spec,
					       adc_injected_callback_t cb,
					       void *user_data)
{
	return adc_injected_set_callback(spec->dev, cb, user_data);
}

/**
 * @brief Enable injected ADC from a struct adc_injected_dt_spec.
 *
 * This is equivalent to:
 *
 *     adc_injected_enable(spec->dev)
 *
 * @param spec Injected ADC specification from devicetree.
 *
 * @return A value from adc_injected_enable().
 */
static inline int adc_injected_enable_dt(const struct adc_injected_dt_spec *spec)
{
	return adc_injected_enable(spec->dev);
}

/**
 * @brief Disable injected ADC from a struct adc_injected_dt_spec.
 *
 * This is equivalent to:
 *
 *     adc_injected_disable(spec->dev)
 *
 * @param spec Injected ADC specification from devicetree.
 *
 * @return A value from adc_injected_disable().
 */
static inline int adc_injected_disable_dt(const struct adc_injected_dt_spec *spec)
{
	return adc_injected_disable(spec->dev);
}

/**
 * @brief Validate that the injected ADC device is ready.
 *
 * @param spec Injected ADC specification from devicetree
 *
 * @retval true If the injected ADC device is ready for use
 * @retval false If the injected ADC device is not ready for use
 */
static inline bool adc_injected_is_ready_dt(const struct adc_injected_dt_spec *spec)
{
	return device_is_ready(spec->dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <zephyr/syscalls/adc_injected.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_INJECTED_H_ */
