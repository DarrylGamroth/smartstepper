/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include <zephyr/ztest.h>

#include "motor/runtime/persistent_config.h"

ZTEST(motor_persistent_config, test_prepare_populates_header_and_valid_crc)
{
	struct motor_persistent_config_v1 record = {0};

	record.payload.current_offsets.ia_offset_a = 2.99f;
	record.payload.current_offsets.ib_offset_a = 3.01f;
	record.payload.encoder_mapping.direction_sign = -1;
	record.payload.encoder_mapping.commutation_offset_mech_rad = 0.004f;

	motor_persistent_config_v1_prepare(
		&record,
		42U,
		MOTOR_PERSISTENT_CONFIG_FLAG_CURRENT_OFFSETS_VALID |
			MOTOR_PERSISTENT_CONFIG_FLAG_ENCODER_MAPPING_VALID);

	zassert_equal(record.header.magic, MOTOR_PERSISTENT_CONFIG_MAGIC, NULL);
	zassert_equal(record.header.schema_version, MOTOR_PERSISTENT_CONFIG_SCHEMA_VERSION, NULL);
	zassert_equal(record.header.header_size, sizeof(record.header), NULL);
	zassert_equal(record.header.payload_size, sizeof(record.payload), NULL);
	zassert_equal(record.header.record_size, sizeof(record), NULL);
	zassert_equal(record.header.generation, 42U, NULL);
	zassert_true(motor_persistent_config_v1_validate(&record), NULL);
}

ZTEST(motor_persistent_config, test_validate_rejects_payload_corruption)
{
	struct motor_persistent_config_v1 record = {0};

	record.payload.motor_model.rs_ohm = 5.6f;
	motor_persistent_config_v1_prepare(&record, 1U,
		MOTOR_PERSISTENT_CONFIG_FLAG_MOTOR_MODEL_VALID);
	zassert_true(motor_persistent_config_v1_validate(&record), NULL);

	record.payload.motor_model.rs_ohm = 6.6f;
	zassert_false(motor_persistent_config_v1_validate(&record), NULL);
}

ZTEST(motor_persistent_config, test_validate_rejects_header_mismatch)
{
	struct motor_persistent_config_v1 record = {0};

	motor_persistent_config_v1_prepare(&record, 1U, 0U);
	zassert_true(motor_persistent_config_v1_validate(&record), NULL);

	record.header.schema_version++;
	zassert_false(motor_persistent_config_v1_validate(&record), NULL);
}

ZTEST(motor_persistent_config, test_payload_crc_null_is_zero)
{
	zassert_equal(motor_persistent_config_payload_crc32(NULL), 0U, NULL);
	zassert_false(motor_persistent_config_v1_validate(NULL), NULL);
}

ZTEST_SUITE(motor_persistent_config, NULL, NULL, NULL, NULL, NULL);
