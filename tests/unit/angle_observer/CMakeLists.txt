cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(angle_observer_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../app/include
)

target_compile_definitions(app PRIVATE
	CONFIG_APP_LOG_LEVEL=3
)

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../app/src/angle_observer.c
	src/main.c
)
