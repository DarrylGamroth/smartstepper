/*
 * Copyright (c) 2026 Rubus Technologies Inc
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SHELL_PARSE_H_
#define SHELL_PARSE_H_

#include <stdbool.h>
#include <stdint.h>

bool shell_parse_finite_float(const char *text, float *value_out);
bool shell_parse_u32(const char *text, uint32_t *value_out);
bool shell_parse_u16(const char *text, uint16_t *value_out);
bool shell_parse_u8(const char *text, uint8_t *value_out);
bool shell_parse_bool01(const char *text, bool *value_out);

#endif /* SHELL_PARSE_H_ */
