#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE 
	src/main.c
	src/config.c
	src/angle_observer.c
	src/rs_online.c
	src/shell_commands.c
	src/motor_control_api.c
	src/motor_isr.c
	src/motor_states.c
	src/motor_hardware.c
)
target_include_directories(app PRIVATE include)

# Place interrupt stack in fast memory using code relocation
# The z_interrupt_stacks is in noinit sections from kernel/init.c
if(CONFIG_BOARD_NUCLEO_H753ZI OR CONFIG_BOARD_SMARTSTEPPER_V2)
  # H7 boards: relocate kernel init.c noinit to DTCM (includes z_interrupt_stacks)
  zephyr_code_relocate(FILES ${ZEPHYR_BASE}/kernel/init.c LOCATION DTCM_NOINIT)
elseif(CONFIG_BOARD_SMARTSTEPPER)
  # F4 board: relocate kernel init.c noinit to CCM (includes z_interrupt_stacks)  
  zephyr_code_relocate(FILES ${ZEPHYR_BASE}/kernel/init.c LOCATION CCM_NOINIT)
endif()
