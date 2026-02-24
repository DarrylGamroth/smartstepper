cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_foc_voltage_pwm_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/include
)

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/motor_foc_voltage_pwm.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/current_loop.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/decoupling.c
	src/arm_dsp_stub.c
	src/main.c
)
