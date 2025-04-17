/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2020-2021 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public Timer Driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_TIMER_IC_H_
#define ZEPHYR_INCLUDE_DRIVERS_TIMER_IC_H_

/**
 * @brief Timer Interface
 * @defgroup timer_ic_interface Timer Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/toolchain.h>

#include <zephyr/dt-bindings/pwm/pwm.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Timer capture configuration flags
 * @anchor TIMER_IC_CAPTURE_FLAGS
 * @{
 */

/** @cond INTERNAL_HIDDEN */
/* Bit 0 is used for TIMER_IC_POLARITY_NORMAL/TIMER_IC_POLARITY_INVERTED */
#define TIMER_IC_CAPTURE_EDGE_SHIFT		1U
#define TIMER_IC_CAPTURE_EDGE_MASK		(3U << TIMER_IC_CAPTURE_EDGE_SHIFT)
#define TIMER_IC_CAPTURE_MODE_SHIFT		3U
#define TIMER_IC_CAPTURE_MODE_MASK		(1U << TIMER_IC_CAPTURE_MODE_SHIFT)
/** @endcond */

/** Timer pin capture on rising edge */
#define TIMER_IC_CAPTURE_EDGE_RISING		(1U << TIMER_IC_CAPTURE_EDGE_SHIFT)

/** Timer pin capture on falling edge */
#define TIMER_IC_CAPTURE_EDGE_FALLING		(2U << TIMER_IC_CAPTURE_EDGE_SHIFT)

/** Timer pin capture on both edges */
#define TIMER_IC_CAPTURE_EDGE_BOTH			(TIMER_IC_CAPTURE_EDGE_RISING | TIMER_IC_CAPTURE_EDGE_FALLING)

/** Timer pin capture captures a single period/pulse width. */
#define TIMER_IC_CAPTURE_MODE_SINGLE		(0U << TIMER_IC_CAPTURE_MODE_SHIFT)

/** Timer pin capture captures period/pulse width continuously. */
#define TIMER_IC_CAPTURE_MODE_CONTINUOUS	(1U << TIMER_IC_CAPTURE_MODE_SHIFT)

/** @} */

/**
 * @brief Provides a type to hold Timer configuration flags.
 *
 * The lower 8 bits are used for standard flags.
 * The upper 8 bits are reserved for SoC specific flags.
 *
 * @see @ref TIMER_IC_CAPTURE_FLAGS.
 */

typedef uint16_t timer_ic_flags_t;

/**
 * @brief Timer capture callback handler function signature
 *
 * @note The callback handler will be called in interrupt context.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.

 * @param cycles Captured timer value (in clock cycles). HW specific.
 * @param status Status for the Timer capture (0 if no error, negative errno
 *               otherwise. See timer_ic_capture_cycles() return value
 *               descriptions for details).
 * @param user_data User data passed to timer_ic_configure_capture()
 */
typedef void (*timer_ic_capture_callback_handler_t)(const struct device *dev,
					       uint32_t channel,
					       uint32_t cycles,
					       int status, void *user_data);

/** @cond INTERNAL_HIDDEN */
/**
 * @brief Timer driver API call to obtain the Timer cycles per second (frequency).
 * @see timer_ic_get_cycles_per_sec() for argument description
 */
typedef int (*timer_ic_get_cycles_per_sec_t)(const struct device *dev,
					uint32_t channel, uint64_t *cycles);

/**
 * @brief Timer driver API call to configure Timer capture.
 * @see timer_ic_configure_capture() for argument description.
 */
typedef int (*timer_ic_configure_capture_t)(const struct device *dev,
				       uint32_t channel, timer_ic_flags_t flags,
				       timer_ic_capture_callback_handler_t cb,
				       void *user_data);

/**
 * @brief Timer driver API call to enable Timer capture.
 * @see timer_ic_enable_capture() for argument description.
 */
typedef int (*timer_ic_enable_capture_t)(const struct device *dev, uint32_t channel);

/**
 * @brief Timer driver API call to disable Timer capture.
 * @see timer_ic_disable_capture() for argument description
 */
typedef int (*timer_ic_disable_capture_t)(const struct device *dev,
				     uint32_t channel);

/** @brief Timer driver API definition. */
__subsystem struct timer_ic_driver_api {
	timer_ic_get_cycles_per_sec_t get_cycles_per_sec;
	timer_ic_configure_capture_t configure_capture;
	timer_ic_enable_capture_t enable_capture;
	timer_ic_disable_capture_t disable_capture;
};
/** @endcond */

/**
 * @brief Get the clock rate (cycles per second) for a single Timer output.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 * @param[out] cycles Pointer to the memory to store clock rate (cycles per
 *                    sec). HW specific.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
__syscall int timer_ic_get_cycles_per_sec(const struct device *dev, uint32_t channel,
				     uint64_t *cycles);

static inline int z_impl_timer_ic_get_cycles_per_sec(const struct device *dev,
						uint32_t channel,
						uint64_t *cycles)
{
	const struct timer_ic_driver_api *api =
		(const struct timer_ic_driver_api *)dev->api;

	return api->get_cycles_per_sec(dev, channel, cycles);
}

/**
 * @brief Convert from Timer cycles to microseconds.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 * @param cycles Cycles to be converted.
 * @param[out] usec Pointer to the memory to store calculated usec.
 *
 * @retval 0 If successful.
 * @retval -ERANGE If result is too large.
 * @retval -errno Other negative errno code on failure.
 */
static inline int timer_ic_cycles_to_usec(const struct device *dev, uint32_t channel,
				     uint32_t cycles, uint64_t *usec)
{
	int err;
	uint64_t temp;
	uint64_t cycles_per_sec;

	err = timer_ic_get_cycles_per_sec(dev, channel, &cycles_per_sec);
	if (err < 0) {
		return err;
	}

	if (u64_mul_overflow(cycles, (uint64_t)USEC_PER_SEC, &temp)) {
		return -ERANGE;
	}

	*usec = temp / cycles_per_sec;

	return 0;
}

/**
 * @brief Convert from Timer cycles to nanoseconds.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 * @param cycles Cycles to be converted.
 * @param[out] nsec Pointer to the memory to store the calculated nsec.
 *
 * @retval 0 If successful.
 * @retval -ERANGE If result is too large.
 * @retval -errno Other negative errno code on failure.
 */
static inline int timer_ic_cycles_to_nsec(const struct device *dev, uint32_t channel,
				     uint32_t cycles, uint64_t *nsec)
{
	int err;
	uint64_t temp;
	uint64_t cycles_per_sec;

	err = timer_ic_get_cycles_per_sec(dev, channel, &cycles_per_sec);
	if (err < 0) {
		return err;
	}

	if (u64_mul_overflow(cycles, (uint64_t)NSEC_PER_SEC, &temp)) {
		return -ERANGE;
	}

	*nsec = temp / cycles_per_sec;

	return 0;
}

/**
 * @brief Configure Timer capture for a single capture input.
 *
 * After configuring timer capture using this function, the capture can be
 * enabled/disabled using timer_ic_enable_capture() and
 * timer_ic_disable_capture().
 *
 * @note This API function cannot be invoked from user space due to the use of a
 * function callback. In user space, one of the simpler API functions
 * (timer_ic_capture_cycles(), timer_ic_capture_usec(), or
 * timer_ic_capture_nsec()) can be used instead.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 * @param flags Timer capture flags
 * @param[in] cb Application callback handler function to be called upon capture
 * @param[in] user_data User data to pass to the application callback handler
 *                      function
 *
 * @retval -EINVAL if invalid function parameters were given
 * @retval -ENOSYS if Timer capture is not supported or the given flags are not
 *                  supported
 * @retval -EIO if IO error occurred while configuring
 * @retval -EBUSY if Timer capture is already in progress
 */
static inline int timer_ic_configure_capture(const struct device *dev,
					uint32_t channel, timer_ic_flags_t flags,
					timer_ic_capture_callback_handler_t cb,
					void *user_data)
{
	const struct timer_ic_driver_api *api =
		(const struct timer_ic_driver_api *)dev->api;

	if (api->configure_capture == NULL) {
		return -ENOSYS;
	}

	return api->configure_capture(dev, channel, flags, cb,
					      user_data);
}

/**
 * @brief Enable Timer period/pulse width capture for a single capture input.
 *
 * The Timer pin must be configured using timer_ic_configure_capture() prior to
 * calling this function.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 *
 * @retval 0 If successful.
 * @retval -EINVAL if invalid function parameters were given
 * @retval -ENOSYS if Timer capture is not supported
 * @retval -EIO if IO error occurred while enabling Timer capture
 * @retval -EBUSY if Timer capture is already in progress
 */
__syscall int timer_ic_enable_capture(const struct device *dev, uint32_t channel);

static inline int z_impl_timer_ic_enable_capture(const struct device *dev,
					    uint32_t channel)
{
	const struct timer_ic_driver_api *api =
		(const struct timer_ic_driver_api *)dev->api;

	if (api->enable_capture == NULL) {
		return -ENOSYS;
	}

	return api->enable_capture(dev, channel);
}

/**
 * @brief Disable Timer period/pulse width capture for a single Timer input.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 *
 * @retval 0 If successful.
 * @retval -EINVAL if invalid function parameters were given
 * @retval -ENOSYS if Timer capture is not supported
 * @retval -EIO if IO error occurred while disabling Timer capture
 */
__syscall int timer_ic_disable_capture(const struct device *dev, uint32_t channel);

static inline int z_impl_timer_ic_disable_capture(const struct device *dev,
					     uint32_t channel)
{
	const struct timer_ic_driver_api *api =
		(const struct timer_ic_driver_api *)dev->api;

	if (api->disable_capture == NULL) {
		return -ENOSYS;
	}

	return api->disable_capture(dev, channel);
}

/**
 * @brief Capture a single Timer period/pulse width in clock cycles for a single
 *        Timer input.
 *
 * This API function wraps calls to timer_ic_configure_capture(),
 * timer_ic_enable_capture(), and timer_ic_disable_capture() and passes
 * the capture result to the caller. The function is blocking until either the
 * Timer capture is completed or a timeout occurs.
 *
 * @param[in] dev Timer device instance.
 * @param channel Timer channel.
 * @param flags Timer capture flags.
 * @param[out] cycles Pointer to the memory to store the captured capture count
 *                    (in clock cycles). HW specific.
 * @param timeout Waiting period for the capture to complete.
 *
 * @retval 0 If successful.
 * @retval -EBUSY Timer capture already in progress.
 * @retval -EAGAIN Waiting period timed out.
 * @retval -EIO IO error while capturing.
 * @retval -ERANGE If result is too large.
 */
__syscall int timer_ic_capture_cycles(const struct device *dev, uint32_t channel,
				 timer_ic_flags_t flags, uint32_t *cycles,
				 k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/timer_ic.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_TIMER_IC_H_ */
