cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_autonomy_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/include
)

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/motor_autonomy.c
	src/main.c
)
