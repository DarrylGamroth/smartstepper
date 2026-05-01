# Copyright (c) 2026 Rubus Technologies Inc
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RT_SPI_STM32_H7_FIFO rt_spi_stm32_h7.c)
