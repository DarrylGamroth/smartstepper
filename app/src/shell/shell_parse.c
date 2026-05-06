/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#include "shell_parse.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>

bool shell_parse_finite_float(const char *text, float *value_out)
{
	if (text == NULL || value_out == NULL) {
		return false;
	}

	errno = 0;
	char *endp = NULL;
	float value = strtof(text, &endp);
	if (endp == text || *endp != '\0' || errno == ERANGE || !isfinite(value)) {
		return false;
	}

	*value_out = value;
	return true;
}

bool shell_parse_u32(const char *text, uint32_t *value_out)
{
	if (text == NULL || value_out == NULL) {
		return false;
	}

	errno = 0;
	char *endp = NULL;
	unsigned long value = strtoul(text, &endp, 10);
	if (endp == text || *endp != '\0' || errno == ERANGE || value > UINT32_MAX) {
		return false;
	}

	*value_out = (uint32_t)value;
	return true;
}

bool shell_parse_u16(const char *text, uint16_t *value_out)
{
	uint32_t value = 0U;
	if (!shell_parse_u32(text, &value) || value > UINT16_MAX) {
		return false;
	}

	*value_out = (uint16_t)value;
	return true;
}

bool shell_parse_u8(const char *text, uint8_t *value_out)
{
	uint32_t value = 0U;
	if (!shell_parse_u32(text, &value) || value > UINT8_MAX) {
		return false;
	}

	*value_out = (uint8_t)value;
	return true;
}

bool shell_parse_bool01(const char *text, bool *value_out)
{
	uint32_t value = 0U;
	if (!shell_parse_u32(text, &value) || value > 1U || value_out == NULL) {
		return false;
	}

	*value_out = (value != 0U);
	return true;
}
