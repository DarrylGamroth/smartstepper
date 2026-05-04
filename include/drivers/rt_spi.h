/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RT_SPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_RT_SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RT_SPI_MAX_FRAME_BYTES CONFIG_RT_SPI_MAX_FRAME_BYTES

enum rt_spi_result_flags {
	RT_SPI_RESULT_VALID = BIT(0),
	RT_SPI_RESULT_ERROR = BIT(1),
	RT_SPI_RESULT_OVERRUN = BIT(2),
	RT_SPI_RESULT_OVR = BIT(3),
	RT_SPI_RESULT_MODF = BIT(4),
	RT_SPI_RESULT_UDR = BIT(5),
	RT_SPI_RESULT_TIFRE = BIT(6),
	RT_SPI_RESULT_CRCE = BIT(7),
	RT_SPI_RESULT_TIMEOUT = BIT(8),
};

struct rt_spi_transfer {
	const uint8_t *tx;
	uint8_t len;
};

struct rt_spi_result {
	uint32_t timestamp_cycles;
	uint8_t raw[RT_SPI_MAX_FRAME_BYTES];
	uint8_t len;
	uint16_t flags;
};

struct rt_spi_stats {
	uint32_t request_count;
	uint32_t busy_count;
	uint32_t complete_count;
	uint32_t collect_empty_count;
	uint32_t overrun_count;
	uint32_t error_count;
	uint32_t last_error_flags;
	uint32_t max_transaction_cycles;
};

struct rt_spi_config {
	bool cpol;
	bool cpha;
};

typedef int (*rt_spi_request_api)(const struct device *dev,
				  const struct rt_spi_transfer *transfer);
typedef int (*rt_spi_collect_api)(const struct device *dev,
				  struct rt_spi_result *result);
typedef int (*rt_spi_transceive_api)(const struct device *dev,
				     const struct rt_spi_transfer *transfer,
				     struct rt_spi_result *result,
				     uint32_t timeout_us);
typedef bool (*rt_spi_busy_api)(const struct device *dev);
typedef void (*rt_spi_abort_api)(const struct device *dev);
typedef void (*rt_spi_stats_api)(const struct device *dev,
				 struct rt_spi_stats *stats);
typedef void (*rt_spi_reset_stats_api)(const struct device *dev);
typedef int (*rt_spi_configure_api)(const struct device *dev,
				    const struct rt_spi_config *config);
typedef void (*rt_spi_get_config_api)(const struct device *dev,
				      struct rt_spi_config *config);

__subsystem struct rt_spi_driver_api {
	rt_spi_request_api request;
	rt_spi_collect_api collect;
	rt_spi_transceive_api transceive;
	rt_spi_busy_api busy;
	rt_spi_abort_api abort;
	rt_spi_stats_api get_stats;
	rt_spi_reset_stats_api reset_stats;
	rt_spi_configure_api configure;
	rt_spi_get_config_api get_config;
};

static inline int rt_spi_request(const struct device *dev,
				 const struct rt_spi_transfer *transfer)
{
	const struct rt_spi_driver_api *api = dev->api;

	return api->request(dev, transfer);
}

static inline int rt_spi_collect(const struct device *dev,
				 struct rt_spi_result *result)
{
	const struct rt_spi_driver_api *api = dev->api;

	return api->collect(dev, result);
}

static inline int rt_spi_transceive(const struct device *dev,
				    const struct rt_spi_transfer *transfer,
				    struct rt_spi_result *result,
				    uint32_t timeout_us)
{
	const struct rt_spi_driver_api *api = dev->api;

	return api->transceive(dev, transfer, result, timeout_us);
}

static inline bool rt_spi_busy(const struct device *dev)
{
	const struct rt_spi_driver_api *api = dev->api;

	return api->busy(dev);
}

static inline void rt_spi_abort(const struct device *dev)
{
	const struct rt_spi_driver_api *api = dev->api;

	api->abort(dev);
}

static inline void rt_spi_get_stats(const struct device *dev,
				    struct rt_spi_stats *stats)
{
	const struct rt_spi_driver_api *api = dev->api;

	api->get_stats(dev, stats);
}

static inline void rt_spi_reset_stats(const struct device *dev)
{
	const struct rt_spi_driver_api *api = dev->api;

	api->reset_stats(dev);
}

static inline int rt_spi_configure(const struct device *dev,
				   const struct rt_spi_config *config)
{
	const struct rt_spi_driver_api *api = dev->api;

	return api->configure(dev, config);
}

static inline void rt_spi_get_config(const struct device *dev,
				     struct rt_spi_config *config)
{
	const struct rt_spi_driver_api *api = dev->api;

	api->get_config(dev, config);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_RT_SPI_H_ */
