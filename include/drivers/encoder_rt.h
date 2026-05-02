/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ENCODER_RT_H_
#define ZEPHYR_INCLUDE_DRIVERS_ENCODER_RT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

enum encoder_rt_mode {
	ENCODER_RT_MODE_DISABLED = 0,
	ENCODER_RT_MODE_REALTIME,
	ENCODER_RT_MODE_DIAGNOSTIC,
};

enum encoder_rt_sample_flags {
	ENCODER_RT_SAMPLE_VALID = BIT(0),
	ENCODER_RT_SAMPLE_WARNING = BIT(1),
	ENCODER_RT_SAMPLE_ERROR = BIT(2),
	ENCODER_RT_SAMPLE_TRANSPORT_ERROR = BIT(3),
	ENCODER_RT_SAMPLE_FRAME_ERROR = BIT(4),
	ENCODER_RT_SAMPLE_FRAME_PARITY_ERROR = BIT(5),
	ENCODER_RT_SAMPLE_FRAME_STATUS_ERROR = BIT(6),
};

struct encoder_rt_sample {
	uint32_t timestamp_cycles;
	float mechanical_angle_deg;
	float mechanical_angle_rad;
	uint32_t raw_count;
	uint32_t status;
	uint32_t flags;
};

struct encoder_rt_stats {
	uint32_t request_count;
	uint32_t busy_count;
	uint32_t disabled_count;
	uint32_t request_error_count;
	uint32_t collect_count;
	uint32_t pending_count;
	uint32_t empty_count;
	uint32_t collect_error_count;
	uint32_t transport_error_count;
	uint32_t frame_error_count;
	uint32_t frame_parity_error_count;
	uint32_t frame_status_error_count;
	uint32_t warning_count;
};

typedef int (*encoder_rt_set_mode_api)(const struct device *dev, enum encoder_rt_mode mode);
typedef int (*encoder_rt_request_sample_api)(const struct device *dev);
typedef int (*encoder_rt_collect_sample_api)(const struct device *dev,
					     struct encoder_rt_sample *sample);
typedef void (*encoder_rt_get_stats_api)(const struct device *dev,
					 struct encoder_rt_stats *stats);
typedef void (*encoder_rt_reset_stats_api)(const struct device *dev);
typedef uint8_t (*encoder_rt_get_pipeline_delay_api)(const struct device *dev);

__subsystem struct encoder_rt_driver_api {
	encoder_rt_set_mode_api set_mode;
	encoder_rt_request_sample_api request_sample;
	encoder_rt_collect_sample_api collect_sample;
	encoder_rt_get_stats_api get_stats;
	encoder_rt_reset_stats_api reset_stats;
	encoder_rt_get_pipeline_delay_api get_pipeline_delay;
};

static inline int encoder_rt_set_mode(const struct device *dev, enum encoder_rt_mode mode)
{
	return DEVICE_API_GET(encoder_rt, dev)->set_mode(dev, mode);
}

static inline int encoder_rt_request_sample(const struct device *dev)
{
	return DEVICE_API_GET(encoder_rt, dev)->request_sample(dev);
}

static inline int encoder_rt_collect_sample(const struct device *dev,
					    struct encoder_rt_sample *sample)
{
	return DEVICE_API_GET(encoder_rt, dev)->collect_sample(dev, sample);
}

static inline void encoder_rt_get_stats(const struct device *dev,
					struct encoder_rt_stats *stats)
{
	DEVICE_API_GET(encoder_rt, dev)->get_stats(dev, stats);
}

static inline void encoder_rt_reset_stats(const struct device *dev)
{
	DEVICE_API_GET(encoder_rt, dev)->reset_stats(dev);
}

static inline uint8_t encoder_rt_get_pipeline_delay(const struct device *dev)
{
	return DEVICE_API_GET(encoder_rt, dev)->get_pipeline_delay(dev);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ENCODER_RT_H_ */
