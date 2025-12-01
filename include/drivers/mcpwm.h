/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2020-2021 Vestas Wind Systems A/S
 * Copyright (c) 2025 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public PWM Driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MCPWM_H_
#define ZEPHYR_INCLUDE_DRIVERS_MCPWM_H_

/**
 * @brief PWM Interface
 * @defgroup mcpwm_interface PWM Interface
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
#include <zephyr/sys_clock.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/toolchain.h>

#include <zephyr/dt-bindings/pwm/pwm.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @} */

/**
 * @brief Provides a type to hold PWM configuration flags.
 *
 * The lower 8 bits are used for standard PWM flags from pwm.h:
 * - PWM_POLARITY_NORMAL: Active-high pulse
 * - PWM_POLARITY_INVERTED: Active-low pulse
 *
 * The upper 8 bits are reserved for SoC specific flags.
 *
 * @see zephyr/dt-bindings/pwm/pwm.h
 * @see zephyr/dt-bindings/pwm/stm32_pwm.h
 */
typedef uint16_t mcpwm_flags_t;

/**
 * @brief MCPWM compare event callback signature.
 *
 * Invoked from ISR context whenever a channel raises an output compare event.
 *
 * @param dev PWM device instance that generated the event.
 * @param channel Channel index [1..N] associated with the event.
 * @param user_data Pointer supplied during callback registration.
 */
typedef void (*mcpwm_compare_cb_t)(const struct device *dev, uint32_t channel, void *user_data);

/**
 * @brief MCPWM break fault callback signature.
 *
 * Called from ISR context when a break or break2 fault is latched.
 *
 * @param dev PWM device instance that detected the break.
 * @param user_data Pointer supplied during callback registration.
 */
typedef void (*mcpwm_break_cb_t)(const struct device *dev, void *user_data);

/**
 * @brief Container for PWM information specified in devicetree.
 *
 * This type contains a pointer to a PWM device, channel number (controlled by
 * the PWM device) and the flags applicable to the channel.
 * Note that not all PWM drivers support flags.
 * In such case, flags will be set to 0.
 *
 * @see MCPWM_DT_SPEC_GET_BY_NAME
 * @see MCPWM_DT_SPEC_GET_BY_NAME_OR
 * @see MCPWM_DT_SPEC_GET_BY_IDX
 * @see MCPWM_DT_SPEC_GET_BY_IDX_OR
 * @see MCPWM_DT_SPEC_GET
 * @see MCPWM_DT_SPEC_GET_OR
 */
struct mcpwm_dt_spec {
	/** PWM device instance. */
	const struct device *dev;
	/** Channel number. */
	uint32_t channel;
	/** Flags. */
	mcpwm_flags_t flags;
};

/**
 * @brief Static initializer for a struct mcpwm_dt_spec
 *
 * This returns a static initializer for a struct mcpwm_dt_spec given a devicetree
 * node identifier and an index.
 *
 * Example devicetree fragment:
 *
 * @code{.dts}
 *    n: node {
 *        pwms = <&pwm1 1 PWM_POLARITY_NORMAL>,
 *               <&pwm2 3 PWM_POLARITY_INVERTED>;
 *        pwm-names = "alpha", "beta";
 *    };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *    const struct mcpwm_dt_spec spec =
 *        MCPWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(n), alpha);
 *
 *    // Initializes 'spec' to:
 *    // {
 *    //         .dev = DEVICE_DT_GET(DT_NODELABEL(pwm1)),
 *    //         .channel = 1,
 *    //         .flags = PWM_POLARITY_NORMAL,
 *    // }
 * @endcode
 *
 * The device (dev) must still be checked for readiness, e.g. using
 * device_is_ready(). It is an error to use this macro unless the node exists,
 * has the 'pwms' property, and that 'pwms' property specifies a PWM controller,
 * a channel, a period in nanoseconds and optionally flags.
 *
 * @param node_id Devicetree node identifier.
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_INST_GET_BY_NAME
 */
#define MCPWM_DT_SPEC_GET_BY_NAME(node_id, name)				       \
	{								       \
		.dev = DEVICE_DT_GET(DT_PWMS_CTLR_BY_NAME(node_id, name)),     \
		.channel = DT_PWMS_CHANNEL_BY_NAME(node_id, name),	       \
		.flags = DT_PWMS_FLAGS_BY_NAME(node_id, name),		       \
	}

/**
 * @brief Static initializer for a struct mcpwm_dt_spec from a DT_DRV_COMPAT
 *        instance.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_GET_BY_NAME
 */
#define MCPWM_DT_SPEC_INST_GET_BY_NAME(inst, name)			       \
	MCPWM_DT_SPEC_GET_BY_NAME(DT_DRV_INST(inst), name)

/**
 * @brief Like MCPWM_DT_SPEC_GET_BY_NAME(), with a fallback to a default value.
 *
 * If the devicetree node identifier 'node_id' refers to a node with a property
 * 'pwms', this expands to <tt>MCPWM_DT_SPEC_GET_BY_NAME(node_id, name)</tt>. The
 * @p default_value parameter is not expanded in this case. Otherwise, this
 * expands to @p default_value.
 *
 * @param node_id Devicetree node identifier.
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see MCPWM_DT_SPEC_INST_GET_BY_NAME_OR
 */
#define MCPWM_DT_SPEC_GET_BY_NAME_OR(node_id, name, default_value)	       \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pwms),			       \
		    (MCPWM_DT_SPEC_GET_BY_NAME(node_id, name)),		       \
		    (default_value))

/**
 * @brief Like MCPWM_DT_SPEC_INST_GET_BY_NAME(), with a fallback to a default
 *        value.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see MCPWM_DT_SPEC_GET_BY_NAME_OR
 */
#define MCPWM_DT_SPEC_INST_GET_BY_NAME_OR(inst, name, default_value)	       \
	MCPWM_DT_SPEC_GET_BY_NAME_OR(DT_DRV_INST(inst), name, default_value)

/**
 * @brief Static initializer for a struct mcpwm_dt_spec
 *
 * This returns a static initializer for a struct mcpwm_dt_spec given a devicetree
 * node identifier and an index.
 *
 * Example devicetree fragment:
 *
 * @code{.dts}
 *    n: node {
 *        pwms = <&pwm1 1 PWM_POLARITY_NORMAL>,
 *               <&pwm2 3 PWM_POLARITY_INVERTED>;
 *    };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *    const struct mcpwm_dt_spec spec =
 *        MCPWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(n), 1);
 *
 *    // Initializes 'spec' to:
 *    // {
 *    //         .dev = DEVICE_DT_GET(DT_NODELABEL(pwm2)),
 *    //         .channel = 3,
 *    //         .flags = PWM_POLARITY_INVERTED,
 *    // }
 * @endcode
 *
 * The device (dev) must still be checked for readiness, e.g. using
 * device_is_ready(). It is an error to use this macro unless the node exists,
 * has the 'pwms' property, and that 'pwms' property specifies a PWM controller,
 * a channel, a period in nanoseconds and optionally flags.
 *
 * @param node_id Devicetree node identifier.
 * @param idx Logical index into 'pwms' property.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_INST_GET_BY_IDX
 */
#define MCPWM_DT_SPEC_GET_BY_IDX(node_id, idx)				       \
	{								       \
		.dev = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(node_id, idx)),       \
		.channel = DT_PWMS_CHANNEL_BY_IDX(node_id, idx),	       \
		.flags = DT_PWMS_FLAGS_BY_IDX(node_id, idx),		       \
	}

/**
 * @brief Static initializer for a struct mcpwm_dt_spec from a DT_DRV_COMPAT
 *        instance.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param idx Logical index into 'pwms' property.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_GET_BY_IDX
 */
#define MCPWM_DT_SPEC_INST_GET_BY_IDX(inst, idx)				       \
	MCPWM_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), idx)

/**
 * @brief Like MCPWM_DT_SPEC_GET_BY_IDX(), with a fallback to a default value.
 *
 * If the devicetree node identifier 'node_id' refers to a node with a property
 * 'pwms', this expands to <tt>MCPWM_DT_SPEC_GET_BY_IDX(node_id, idx)</tt>. The
 * @p default_value parameter is not expanded in this case. Otherwise, this
 * expands to @p default_value.
 *
 * @param node_id Devicetree node identifier.
 * @param idx Logical index into 'pwms' property.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see MCPWM_DT_SPEC_INST_GET_BY_IDX_OR
 */
#define MCPWM_DT_SPEC_GET_BY_IDX_OR(node_id, idx, default_value)		       \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pwms),			       \
		    (MCPWM_DT_SPEC_GET_BY_IDX(node_id, idx)),		       \
		    (default_value))

/**
 * @brief Like MCPWM_DT_SPEC_INST_GET_BY_IDX(), with a fallback to a default
 *        value.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param idx Logical index into 'pwms' property.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see MCPWM_DT_SPEC_GET_BY_IDX_OR
 */
#define MCPWM_DT_SPEC_INST_GET_BY_IDX_OR(inst, idx, default_value)	       \
	MCPWM_DT_SPEC_GET_BY_IDX_OR(DT_DRV_INST(inst), idx, default_value)

/**
 * @brief Equivalent to <tt>MCPWM_DT_SPEC_GET_BY_IDX(node_id, 0)</tt>.
 *
 * @param node_id Devicetree node identifier.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_GET_BY_IDX
 * @see MCPWM_DT_SPEC_INST_GET
 */
#define MCPWM_DT_SPEC_GET(node_id) MCPWM_DT_SPEC_GET_BY_IDX(node_id, 0)

/**
 * @brief Equivalent to <tt>MCPWM_DT_SPEC_INST_GET_BY_IDX(inst, 0)</tt>.
 *
 * @param inst DT_DRV_COMPAT instance number
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_INST_GET_BY_IDX
 * @see MCPWM_DT_SPEC_GET
 */
#define MCPWM_DT_SPEC_INST_GET(inst) MCPWM_DT_SPEC_GET(DT_DRV_INST(inst))

/**
 * @brief Equivalent to
 *        <tt>MCPWM_DT_SPEC_GET_BY_IDX_OR(node_id, 0, default_value)</tt>.
 *
 * @param node_id Devicetree node identifier.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_GET_BY_IDX_OR
 * @see MCPWM_DT_SPEC_INST_GET_OR
 */
#define MCPWM_DT_SPEC_GET_OR(node_id, default_value)			       \
	MCPWM_DT_SPEC_GET_BY_IDX_OR(node_id, 0, default_value)

/**
 * @brief Equivalent to
 *        <tt>MCPWM_DT_SPEC_INST_GET_BY_IDX_OR(inst, 0, default_value)</tt>.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct mcpwm_dt_spec for the property.
 *
 * @see MCPWM_DT_SPEC_INST_GET_BY_IDX_OR
 * @see MCPWM_DT_SPEC_GET_OR
 */
#define MCPWM_DT_SPEC_INST_GET_OR(inst, default_value)			       \
	MCPWM_DT_SPEC_GET_OR(DT_DRV_INST(inst), default_value)

/** @cond INTERNAL_HIDDEN */
/**
 * @brief PWM driver API call to configure PWM pin.
 * @see mcpwm_configure() for argument description.
 */
typedef int (*mcpwm_configure_t)(const struct device *dev, uint32_t channel,
				mcpwm_flags_t flags);

/**
 * @brief PWM driver API call to enable PWM channel.
 * @see mcpwm_enable() for argument description.
 */
typedef int (*mcpwm_enable_t)(const struct device *dev, uint32_t channel);

/**
 * @brief PWM driver API call to disable PWM channel.
 * @see mcpwm_disable() for argument description.
 */
typedef int (*mcpwm_disable_t)(const struct device *dev, uint32_t channel);

/**
 * @brief PWM driver API call to start PWM timer.
 * @see mcpwm_start() for argument description.
 */
typedef int (*mcpwm_start_t)(const struct device *dev);

/**
 * @brief PWM driver API call to stop PWM timer.
 * @see mcpwm_stop() for argument description.
 */
typedef int (*mcpwm_stop_t)(const struct device *dev);

/**
 * @brief PWM driver API call to set duty cycle.
 * @see mcpwm_set_duty_cycle() for argument description.
 */
typedef int (*mcpwm_set_duty_cycle_t)(const struct device *dev,
					uint32_t channel, q31_t duty_cycle);

typedef int (*mcpwm_set_compare_callback_t)(const struct device *dev, uint32_t channel,
					mcpwm_compare_cb_t cb, void *user_data);

typedef int (*mcpwm_set_break_callback_t)(const struct device *dev, mcpwm_break_cb_t cb,
					void *user_data);

/** @brief PWM driver API definition. */
__subsystem struct mcpwm_driver_api {
	mcpwm_configure_t configure;
	mcpwm_enable_t enable;
	mcpwm_disable_t disable;
	mcpwm_start_t start;
	mcpwm_stop_t stop;
	mcpwm_set_duty_cycle_t set_duty_cycle;
	mcpwm_set_compare_callback_t set_compare_callback;
	mcpwm_set_break_callback_t set_break_callback;
};
/** @endcond */

/**
 * @brief Configure a PWM channel.
 *
 * @param[in] dev PWM device instance.
 * @param channel PWM channel.
 * @param flags PWM configuration flags.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int mcpwm_configure(const struct device *dev, uint32_t channel,
			     mcpwm_flags_t flags);

static inline int z_impl_mcpwm_configure(const struct device *dev,
					 uint32_t channel, mcpwm_flags_t flags)
{
	return DEVICE_API_GET(mcpwm, dev)->configure(dev, channel, flags);
}

/**
 * @brief Enable a PWM channel.
 *
 * @param[in] dev PWM device instance.
 * @param channel PWM channel.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int mcpwm_enable(const struct device *dev, uint32_t channel);

static inline int z_impl_mcpwm_enable(const struct device *dev,
				      uint32_t channel)
{
	return DEVICE_API_GET(mcpwm, dev)->enable(dev, channel);
}

/**
 * @brief Disable a PWM channel.
 *
 * @param[in] dev PWM device instance.
 * @param channel PWM channel.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int mcpwm_disable(const struct device *dev, uint32_t channel);

static inline int z_impl_mcpwm_disable(const struct device *dev,
				       uint32_t channel)
{
	return DEVICE_API_GET(mcpwm, dev)->disable(dev, channel);
}

/**
 * @brief Set the duty cycle for a single PWM output.
 *
 * The PWM duty cycle will synchronously be set to the new values
 * without glitches in the PWM signal, but the call will not block for the
 * change to take effect.
 *
 * @note Not all PWM controllers support synchronous, glitch-free updates of the
 * PWM duty cycle. Depending on the hardware, changing the PWM duty cycle may
 * cause a glitch in the generated PWM signal.
 *
 * @param[in] dev PWM device instance.
 * @param channel PWM channel.
 * @param duty_cycle Fixed-point q31_t format duty cycle:
 *                   - 0x00000000 =   0% duty cycle
 *                   - 0x40000000 =  50% duty cycle
 *                   - 0x7FFFFFFF = ~100% duty cycle
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int mcpwm_set_duty_cycle(const struct device *dev, uint32_t channel,
			     q31_t duty_cycle);

static inline int z_impl_mcpwm_set_duty_cycle(const struct device *dev,
					uint32_t channel, q31_t duty_cycle)
{
	return DEVICE_API_GET(mcpwm, dev)->set_duty_cycle(dev, channel, duty_cycle);
}

/**
 * @brief Start MCPWM timer(s)
 *
 * Start the MCPWM timer(s) for synchronized operation. This is useful
 * for controlling timer startup timing in multi-timer configurations.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code if failure.
 */
__syscall int mcpwm_start(const struct device *dev);

static inline int z_impl_mcpwm_start(const struct device *dev)
{
	return DEVICE_API_GET(mcpwm, dev)->start(dev);
}

/**
 * @brief Stop MCPWM timer(s)
 *
 * Stop the MCPWM timer(s). This is useful for controlling timer
 * stop timing in multi-timer configurations and debugging.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code if failure.
 */
__syscall int mcpwm_stop(const struct device *dev);

static inline int z_impl_mcpwm_stop(const struct device *dev)
{
	return DEVICE_API_GET(mcpwm, dev)->stop(dev);
}

/**
 * @brief Set the duty cycle for a single PWM output using floating point.
 *
 * This is a convenience function that converts a floating point duty cycle
 * to the fixed-point Q31 format used by the underlying driver.
 *
 * @param[in] dev PWM device instance.
 * @param channel PWM channel.
 * @param duty_cycle Floating point duty cycle in range [0.0, 1.0]:
 *                   - 0.0 =   0% duty cycle
 *                   - 0.5 =  50% duty cycle
 *                   - 1.0 = 100% duty cycle
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
static inline int mcpwm_set_duty_cycle_f32(const struct device *dev, uint32_t channel,
					    float duty_cycle)
{
	q31_t q31_duty;

	/* Convert to Q31 format with saturation at edges */
	if (duty_cycle >= 1.0f) {
		q31_duty = INT32_MAX;
	} else if (duty_cycle <= 0.0f) {
		q31_duty = 0;
	} else {
		q31_duty = (q31_t)(duty_cycle * 2147483648.0f);
	}

	return mcpwm_set_duty_cycle(dev, channel, q31_duty);
}

/**
 * @brief Set duty cycle from a struct mcpwm_dt_spec.
 *
 * This is equivalent to:
 *
 *     mcpwm_set_duty_cycle(spec->dev, spec->channel, duty_cycle)
 *
 * @param[in] spec PWM specification from devicetree.
 * @param duty_cycle Fixed-point format [0, 1) duty-cycle set to the PWM.
 *
 * @return A value from mcpwm_set_duty_cycle().
 *
 * @see mcpwm_set_duty_cycle_dt()
 */
static inline int mcpwm_set_duty_cycle_dt(const struct mcpwm_dt_spec *spec,
				   q31_t duty_cycle)
{
	return mcpwm_set_duty_cycle(spec->dev, spec->channel, duty_cycle);
}

/**
 * @brief Set duty cycle from a struct mcpwm_dt_spec using floating point.
 *
 * This is equivalent to:
 *
 *     mcpwm_set_duty_cycle_f32(spec->dev, spec->channel, duty_cycle)
 *
 * @param[in] spec PWM specification from devicetree.
 * @param duty_cycle Floating point duty cycle in range [0.0, 1.0]:
 *                   - 0.0 =   0% duty cycle
 *                   - 0.5 =  50% duty cycle
 *                   - 1.0 = 100% duty cycle
 *
 * @return A value from mcpwm_set_duty_cycle_f32().
 *
 * @see mcpwm_set_duty_cycle_f32()
 */
static inline int mcpwm_set_duty_cycle_f32_dt(const struct mcpwm_dt_spec *spec,
					       float duty_cycle)
{
	return mcpwm_set_duty_cycle_f32(spec->dev, spec->channel, duty_cycle);
}

/**
 * @brief Configure PWM from a struct mcpwm_dt_spec.
 *
 * This is equivalent to:
 *
 *     mcpwm_configure(spec->dev, spec->channel, spec->flags)
 *
 * @param[in] spec PWM specification from devicetree.
 *
 * @return A value from mcpwm_configure().
 */
static inline int mcpwm_configure_dt(const struct mcpwm_dt_spec *spec)
{
	return mcpwm_configure(spec->dev, spec->channel, spec->flags);
}

/**
 * @brief Enable PWM from a struct mcpwm_dt_spec.
 *
 * This is equivalent to:
 *
 *     mcpwm_enable(spec->dev, spec->channel)
 *
 * @param[in] spec PWM specification from devicetree.
 *
 * @return A value from mcpwm_enable().
 */
static inline int mcpwm_enable_dt(const struct mcpwm_dt_spec *spec)
{
	return mcpwm_enable(spec->dev, spec->channel);
}

/**
 * @brief Disable PWM from a struct mcpwm_dt_spec.
 *
 * This is equivalent to:
 *
 *     mcpwm_disable(spec->dev, spec->channel)
 *
 * @param[in] spec PWM specification from devicetree.
 *
 * @return A value from mcpwm_disable().
 */
static inline int mcpwm_disable_dt(const struct mcpwm_dt_spec *spec)
{
	return mcpwm_disable(spec->dev, spec->channel);
}

/**
 * @brief Start MCPWM timer from a struct mcpwm_dt_spec.
 *
 * This is equivalent to:
 *
 *     mcpwm_start(spec->dev)
 *
 * @param[in] spec PWM specification from devicetree.
 *
 * @return A value from mcpwm_start().
 */
static inline int mcpwm_start_dt(const struct mcpwm_dt_spec *spec)
{
	return mcpwm_start(spec->dev);
}

/**
 * @brief Register or clear a callback for a channel compare event.
 *
 * @param dev PWM device instance.
 * @param channel Channel index to associate with the callback.
 * @param cb Function to invoke from ISR context, or NULL to unregister.
 * @param user_data Pointer passed to the callback when invoked.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP If the driver does not implement compare callbacks.
 * @retval -errno Negative errno code on other failures.
 */
static inline int mcpwm_set_compare_callback(const struct device *dev, uint32_t channel,
					mcpwm_compare_cb_t cb, void *user_data)
{
	const struct mcpwm_driver_api *api = DEVICE_API_GET(mcpwm, dev);

	if (api->set_compare_callback == NULL) {
		return -ENOTSUP;
	}

	return api->set_compare_callback(dev, channel, cb, user_data);
}

/**
 * @brief Register or clear the break fault callback.
 *
 * @param dev PWM device instance.
 * @param cb Function to invoke from ISR context when a break fault occurs, or NULL.
 * @param user_data Pointer passed to the callback when invoked.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP If the driver does not implement break callbacks.
 * @retval -errno Negative errno code on other failures.
 */
static inline int mcpwm_set_break_callback(const struct device *dev, mcpwm_break_cb_t cb,
					void *user_data)
{
	const struct mcpwm_driver_api *api = DEVICE_API_GET(mcpwm, dev);

	if (api->set_break_callback == NULL) {
		return -ENOTSUP;
	}

	return api->set_break_callback(dev, cb, user_data);
}

/**
 * @brief Stop MCPWM timer from a struct mcpwm_dt_spec.
 *
 * This is equivalent to:
 *
 *     mcpwm_stop(spec->dev)
 *
 * @param[in] spec PWM specification from devicetree.
 *
 * @return A value from mcpwm_stop().
 */
static inline int mcpwm_stop_dt(const struct mcpwm_dt_spec *spec)
{
	return mcpwm_stop(spec->dev);
}

/**
 * @brief Validate that the PWM device is ready.
 *
 * @param spec PWM specification from devicetree
 *
 * @retval true If the PWM device is ready for use
 * @retval false If the PWM device is not ready for use
 */
static inline bool mcpwm_is_ready_dt(const struct mcpwm_dt_spec *spec)
{
	return device_is_ready(spec->dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <zephyr/syscalls/mcpwm.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_MCPWM_H_ */
