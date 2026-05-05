/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor/runtime/persistent_config.h"

#include <stddef.h>

#define MOTOR_PERSISTENT_CRC32_POLY_REFLECTED 0xEDB88320U

static uint32_t motor_persistent_crc32_ieee(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFU;

	for (size_t i = 0U; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0U; bit < 8U; bit++) {
			if ((crc & 1U) != 0U) {
				crc = (crc >> 1) ^ MOTOR_PERSISTENT_CRC32_POLY_REFLECTED;
			} else {
				crc >>= 1;
			}
		}
	}

	return ~crc;
}

uint32_t motor_persistent_config_payload_crc32(
	const struct motor_persistent_config_payload_v1 *payload)
{
	if (payload == NULL) {
		return 0U;
	}

	return motor_persistent_crc32_ieee((const uint8_t *)payload, sizeof(*payload));
}

void motor_persistent_config_v1_prepare(struct motor_persistent_config_v1 *record,
					uint32_t generation,
					uint32_t flags)
{
	if (record == NULL) {
		return;
	}

	record->header.magic = MOTOR_PERSISTENT_CONFIG_MAGIC;
	record->header.schema_version = MOTOR_PERSISTENT_CONFIG_SCHEMA_V1;
	record->header.header_size = sizeof(record->header);
	record->header.payload_size = sizeof(record->payload);
	record->header.record_size = sizeof(*record);
	record->header.generation = generation;
	record->header.flags = flags;
	record->header.payload_crc32 =
		motor_persistent_config_payload_crc32(&record->payload);
}

bool motor_persistent_config_v1_validate(const struct motor_persistent_config_v1 *record)
{
	if (record == NULL) {
		return false;
	}

	if (record->header.magic != MOTOR_PERSISTENT_CONFIG_MAGIC ||
	    record->header.schema_version != MOTOR_PERSISTENT_CONFIG_SCHEMA_V1 ||
	    record->header.header_size != sizeof(record->header) ||
	    record->header.payload_size != sizeof(record->payload) ||
	    record->header.record_size != sizeof(*record)) {
		return false;
	}

	return record->header.payload_crc32 ==
	       motor_persistent_config_payload_crc32(&record->payload);
}
