cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_core_contracts_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/include
)

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/motion/motion_profile.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/motion/motor_motion_modules.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/observers/motor_position_convert.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/motor_mpr.c
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/control/motor_dob.c
	src/main.c
)
