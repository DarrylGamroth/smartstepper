# Copyright (c) 2023 Rubus Technologies Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_TIMER_IC_STM32 timer_ic_stm32.c)
