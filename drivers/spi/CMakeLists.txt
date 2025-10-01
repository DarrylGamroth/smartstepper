# STM32 SPI RTIO driver build configuration

zephyr_library_sources_ifdef(CONFIG_SPI_RUBUS_STM32 spi_ll_stm32.c)
