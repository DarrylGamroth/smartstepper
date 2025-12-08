# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_GATE_DRIVER_TI_DRV8328 ti_drv8328.c)
