# Copyright (c) 2026 Rubus Technologies Inc
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_BRCM_AEAT_9955_FAST brcm_aeat9955_fast.c)
