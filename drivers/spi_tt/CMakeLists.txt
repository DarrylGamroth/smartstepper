# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SPI_STM32_TT spi_ll_stm32_tt.c)
