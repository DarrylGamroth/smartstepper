#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../modules/motor_core
		 ${CMAKE_CURRENT_BINARY_DIR}/motor_core)

target_sources(app PRIVATE 
	src/main.c
	src/config.c
	src/shell_commands.c
	src/shell_commands_state.c
	src/shell_motion_profile.c
	src/shell_motion_sequence.c
	src/shell_motion_chopper_calib.c
	src/motor_control_api.c
	src/motor_rls_runtime.c
	src/motor_encoder_pipeline.c
	src/motor_control_loop.c
	src/motor_isr_io.c
	src/motor_states.c
	src/motor_states_calibration.c
	src/motor_states_online.c
	src/motor_hardware.c
)
target_include_directories(app PRIVATE
	include
)

target_link_libraries(app PRIVATE motor_core)

# Place interrupt stack in fast memory using code relocation
# The z_interrupt_stacks is in noinit sections from kernel/init.c
if(CONFIG_BOARD_NUCLEO_H753ZI OR CONFIG_BOARD_SMARTSTEPPER_V2)
  # H7 boards: relocate kernel init.c noinit to DTCM (includes z_interrupt_stacks)
  zephyr_code_relocate(FILES ${ZEPHYR_BASE}/kernel/init.c LOCATION DTCM_NOINIT)
elseif(CONFIG_BOARD_SMARTSTEPPER)
  # F4 board: relocate kernel init.c noinit to CCM (includes z_interrupt_stacks)  
  zephyr_code_relocate(FILES ${ZEPHYR_BASE}/kernel/init.c LOCATION CCM_NOINIT)
endif()
