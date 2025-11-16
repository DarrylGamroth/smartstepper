# STM32 Injected ADC driver build configuration

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADC_STM32_INJECTED adc_stm32_injected.c)