/*
 * Copyright (c) 2026 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <drivers/sensor/brcm_aeat9955.h>
#include <drivers/sensor/magntek_mt6835.h>

static void mt6835_build_frame(struct mt6835_sample *sample, uint32_t position, uint8_t status_bits,
			       bool corrupt_crc)
{
	uint32_t raw24 = ((position & (MT6835_MAX_COUNT - 1U)) << 3) |
			 (status_bits & MT6835_STATUS_MASK);

	memset(sample, 0, sizeof(*sample));
	sample->raw[2] = (uint8_t)(raw24 >> 16);
	sample->raw[3] = (uint8_t)(raw24 >> 8);
	sample->raw[4] = (uint8_t)raw24;

	uint8_t crc = crc8_ccitt(0x00U, &sample->raw[2], 3U);
	sample->raw[5] = corrupt_crc ? (uint8_t)(crc ^ 0x01U) : crc;
}

static void aeat9955_build_frame(struct aeat9955_sample *sample, uint32_t position,
				 bool status_error, bool parity_error)
{
	uint32_t raw24 = (position & (AEAT9955_MAX_COUNT - 1U)) << 4;

	if (status_error) {
		raw24 |= ((uint32_t)AEAT9955_POS_STATUS_ERROR_BIT << 16);
	}

	/* Select parity bit in raw[0] so parity_error outcome is deterministic. */
	raw24 &= ~(1U << 23);
	bool odd_no_parity = (POPCOUNT(raw24) & 1U) != 0U;
	if (odd_no_parity != parity_error) {
		raw24 |= (1U << 23);
	}

	memset(sample, 0, sizeof(*sample));
	sample->raw[0] = (uint8_t)(raw24 >> 16);
	sample->raw[1] = (uint8_t)(raw24 >> 8);
	sample->raw[2] = (uint8_t)raw24;
}

ZTEST(encoder_decode, test_mt6835_valid_frame_reports_no_warning_or_error)
{
	struct mt6835_sample sample;
	float angle_deg = 0.0f;
	uint8_t status = 0U;
	bool warning = true;
	bool error = true;
	bool status_error = true;
	bool parity_error = true;
	bool crc_error = true;
	uint32_t position = 0U;

	mt6835_build_frame(&sample, MT6835_MAX_COUNT / 2U, 0U, false);

	zassert_ok(mt6835_decode_position(sample.raw, &position, &crc_error), NULL);
	zassert_false(crc_error, NULL);
	zassert_equal(position, MT6835_MAX_COUNT / 2U, NULL);

	zassert_ok(mt6835_decode_sample_f32((const uint8_t *)&sample, &angle_deg, &status, &warning,
					    &error, &status_error, &parity_error),
		   NULL);
	zassert_within(angle_deg, 0.0f, 0.001f, NULL);
	zassert_equal(status, 0U, NULL);
	zassert_false(warning, NULL);
	zassert_false(error, NULL);
	zassert_false(status_error, NULL);
	zassert_false(parity_error, NULL);
}

ZTEST(encoder_decode, test_mt6835_status_bits_set_warning_but_not_error)
{
	struct mt6835_sample sample;
	float angle_deg = 0.0f;
	uint8_t status = 0U;
	bool warning = false;
	bool error = false;
	bool status_error = false;
	bool parity_error = false;
	const uint8_t expected_status = MT6835_STATUS_BIT1_WEAK_MAGNETIC;

	mt6835_build_frame(&sample, MT6835_MAX_COUNT / 2U, expected_status, false);

	zassert_ok(mt6835_decode_sample_f32((const uint8_t *)&sample, &angle_deg, &status, &warning,
					    &error, &status_error, &parity_error),
		   NULL);
	zassert_equal(status, expected_status, NULL);
	zassert_true(warning, NULL);
	zassert_false(error, NULL);
	zassert_true(status_error, NULL);
	zassert_false(parity_error, NULL);
}

ZTEST(encoder_decode, test_mt6835_crc_error_sets_error_path)
{
	struct mt6835_sample sample;
	float angle_deg = 0.0f;
	uint8_t status = 0U;
	bool warning = false;
	bool error = false;
	bool status_error = false;
	bool parity_error = false;
	bool crc_error = false;
	uint32_t position = 0U;

	mt6835_build_frame(&sample, MT6835_MAX_COUNT / 2U, 0U, true);

	zassert_equal(mt6835_decode_position(sample.raw, &position, &crc_error), -EIO, NULL);
	zassert_true(crc_error, NULL);

	zassert_equal(mt6835_decode_sample_f32((const uint8_t *)&sample, &angle_deg, &status,
					       &warning, &error, &status_error, &parity_error),
		      -EIO, NULL);
	zassert_true(error, NULL);
	zassert_true(parity_error, NULL);
	zassert_false(status_error, NULL);
}

ZTEST(encoder_decode, test_aeat9955_valid_frame_reports_no_error)
{
	struct aeat9955_sample sample;
	float angle_deg = 0.0f;
	uint8_t status = 0U;
	bool warning = true;
	bool error = true;
	bool status_error = true;
	bool parity_error = true;
	uint32_t position = 0U;
	const uint32_t expected_position = (AEAT9955_MAX_COUNT / 2U) + 123U;

	aeat9955_build_frame(&sample, expected_position, false, false);

	zassert_ok(aeat9955_decode_position(sample.raw, &position, &status_error, &parity_error), NULL);
	zassert_equal(position, expected_position, NULL);
	zassert_false(status_error, NULL);
	zassert_false(parity_error, NULL);

	zassert_ok(aeat9955_decode_sample_f32((const uint8_t *)&sample, &angle_deg, &status, &warning,
					      &error, &status_error, &parity_error),
		   NULL);
	zassert_false(warning, NULL);
	zassert_false(error, NULL);
	zassert_false(status_error, NULL);
	zassert_false(parity_error, NULL);
	zassert_equal(status, sample.raw[0] & AEAT9955_FRAME_STATUS_MASK, NULL);
}

ZTEST(encoder_decode, test_aeat9955_status_error_sets_error_path)
{
	struct aeat9955_sample sample;
	float angle_deg = 0.0f;
	uint8_t status = 0U;
	bool warning = false;
	bool error = false;
	bool status_error = false;
	bool parity_error = false;

	aeat9955_build_frame(&sample, AEAT9955_MAX_COUNT / 2U, true, false);

	zassert_equal(aeat9955_decode_sample_f32((const uint8_t *)&sample, &angle_deg, &status,
						 &warning, &error, &status_error, &parity_error),
		      -EIO, NULL);
	zassert_true(error, NULL);
	zassert_true(status_error, NULL);
	zassert_false(parity_error, NULL);
	zassert_equal(status & AEAT9955_POS_STATUS_ERROR_BIT, AEAT9955_POS_STATUS_ERROR_BIT, NULL);
}

ZTEST(encoder_decode, test_aeat9955_parity_error_sets_error_path)
{
	struct aeat9955_sample sample;
	float angle_deg = 0.0f;
	uint8_t status = 0U;
	bool warning = false;
	bool error = false;
	bool status_error = false;
	bool parity_error = false;

	aeat9955_build_frame(&sample, AEAT9955_MAX_COUNT / 2U, false, true);

	zassert_equal(aeat9955_decode_sample_f32((const uint8_t *)&sample, &angle_deg, &status,
						 &warning, &error, &status_error, &parity_error),
		      -EIO, NULL);
	zassert_true(error, NULL);
	zassert_false(status_error, NULL);
	zassert_true(parity_error, NULL);
}

ZTEST_SUITE(encoder_decode, NULL, NULL, NULL, NULL, NULL);
