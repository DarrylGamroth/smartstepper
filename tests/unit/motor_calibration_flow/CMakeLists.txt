cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_calibration_flow_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/include
)

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/calibration/offset.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/calibration/timing.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/calibration/window.c
	src/main.c
)
