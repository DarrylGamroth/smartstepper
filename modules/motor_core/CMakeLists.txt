# SPDX-License-Identifier: Apache-2.0

set(MOTOR_CORE_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/math
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/filters
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/observers
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/motion
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/control
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/runtime
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/estimation
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/protection
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/calibration
)

add_library(motor_core INTERFACE)
target_include_directories(motor_core INTERFACE ${MOTOR_CORE_INCLUDE_DIRS})

add_subdirectory(src)

target_link_libraries(motor_core INTERFACE
	motor_core_rt
	motor_core_motion
	motor_core_estimation
	motor_core_commission
	zephyr_interface
)
