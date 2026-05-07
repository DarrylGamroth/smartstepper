cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(control_ref_path_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/include
)

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/runtime/command_arbitration.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/protection/interlocks.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/runtime/control_kernel.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/current_loop.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/dq_decoupling.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/pwm_synthesis.c
	src/arm_dsp_stub.c
	src/main.c
)
