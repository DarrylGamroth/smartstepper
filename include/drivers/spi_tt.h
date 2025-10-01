/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for Timer-Triggered API SPI
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_TT_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_TT_H_

/**
 * @brief SPI TT Interface
 * @defgroup spi_interface SPI Interface
 * @since 1.0
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/spi/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef spi_tt_configure
 * @brief Callback API for configuring SPI in timer mode
 * See spi_transceive_signal() for argument descriptions
 */
typedef int (*spi_tt_configure_t)(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf *tx_buf,
				 const struct spi_buf *rx_buf);

/**
 * @typedef spi_tt_start
 * @brief Callback API for starting SPI transfer in timer mode
 * See spi_tt_start() for argument descriptions
 */
typedef int (*spi_tt_start_t)(const struct device *dev);

/**
 * @typedef spi_tt_stop
 * @brief Callback API for stopping SPI transfer in timer mode
 * See spi_tt_stop() for argument descriptions
 */
typedef int (*spi_tt_stop_t)(const struct device *dev);

/**
 * @typedef spi_tt_is_complete
 * @brief Callback API for checking if timer-triggered transfer is complete
 * See spi_tt_is_complete() for argument descriptions
 */
typedef bool (*spi_tt_is_complete_t)(const struct device *dev);

/**
 * @brief Timer-Triggered SPI driver API
 */
__subsystem struct spi_tt_driver_api {
	spi_tt_configure_t config;
	spi_tt_start_t start;
	spi_tt_stop_t stop;
	spi_tt_is_complete_t is_complete;
};

 /**
  * @brief Configure the SPI driver in Timer-Triggered mode.
  * 
  * This function configures the SPI driver to operate in Timer-Triggered mode,
  * allowing for precise timing control over SPI transactions.
  * 
  * @param dev Pointer to the device structure for the driver instance.
  * @param config Pointer to a valid spi_config structure instance.
  *        Pointer-comparison may be used to detect changes from
  *        previous operations.
  * @param tx_buf Pointer to the transmit buffer where data to be sent originates from,
  *        or NULL if none.
  * @param rx_buf Pointer to the receive buffer where data to be read will be written to,
  *        or NULL if none.
  * 
  * @return 0 If successful.
  * @retval -ENOTSUP means some part of the spi config is not supported either
  *	   by the device hardware or the driver software.
  * @retval -EINVAL means that some parameter of the spi_config is invalid.
  * @retval -errno Negative errno code on failure.
  */
__syscall int spi_tt_configure(const struct device *dev, 
			       const struct spi_config *config,
			       const struct spi_buf *tx_buf,
			       const struct spi_buf *rx_buf);

static inline int z_impl_spi_tt_configure(const struct device *dev,
					   const struct spi_config *config,
					   const struct spi_buf *tx_buf,
					   const struct spi_buf *rx_buf)
{
  return DEVICE_API_GET(spi_tt, dev)->config(dev, config, tx_buf, rx_buf);
}

 /**
  * @brief Configure the SPI bus specified in @p spi_dt_spec.
  * 
  * This is equivalent to:
  *
  *     spi_tt_configure(spec->bus, &spec->config, tx_buf, rx_buf);
  * 
  * @param spec SPI specification from devicetree
  * @param tx_buf Buffer where data to be sent originates from,
  *        or NULL if none.
  * @param rx_buf Buffer where data to be read will be written to,
  *        or NULL if none.
  *
  * @return a value from spi_tt_configure().
  */
static inline int spi_tt_configure_dt(const struct spi_dt_spec *spec,
				 const struct spi_buf *tx_buf,
				 const struct spi_buf *rx_buf)
{
	return spi_tt_configure(spec->bus, &spec->config, tx_buf, rx_buf);
}

/**
 * @brief Start the SPI transfer in Timer-Triggered mode.
 * 
 * This function initiates the SPI transfer in Timer-Triggered mode,
 * allowing for precise timing control over the transaction.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * 
 * @return 0 If successful.
 * @retval -ENOTSUP means that the driver does not support starting transfers
 *         in Timer-Triggered mode.
 * @retval -errno Negative errno code on failure.
 */
__syscall int spi_tt_start(const struct device *dev);

static inline int z_impl_spi_tt_start(const struct device *dev)
{
  return DEVICE_API_GET(spi_tt, dev)->start(dev);
}

/**
 * @brief Stop the SPI transfer in Timer-Triggered mode.
 * 
 * This function stops the SPI transfer in Timer-Triggered mode,
 * allowing for precise timing control over the transaction.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * 
 * @return 0 If successful.
 * @retval -ENOTSUP means that the driver does not support stopping transfers
 *         in Timer-Triggered mode.
 * @retval -errno Negative errno code on failure.
 */
__syscall int spi_tt_stop(const struct device *dev);

static inline int z_impl_spi_tt_stop(const struct device *dev)
{
  return DEVICE_API_GET(spi_tt, dev)->stop(dev);
}

/**
 * @brief Check if timer-triggered SPI transfer has completed.
 * 
 * This function checks if the timer-triggered SPI transfer has completed
 * a full cycle. Useful for checking in ISRs (like ADC ISR) if fresh 
 * encoder data is available for FOC control loops.
 * 
 * @param dev Pointer to the device structure for the driver instance.
 * 
 * @return true if transfer cycle is complete and fresh data is available,
 *         false otherwise.
 */
__syscall bool spi_tt_is_complete(const struct device *dev);

static inline bool z_impl_spi_tt_is_complete(const struct device *dev)
{
  return DEVICE_API_GET(spi_tt, dev)->is_complete(dev);
}

/** @} */


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <zephyr/syscalls/spi_tt.h>

/**
 * @}
 */
#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_TT_H_ */
