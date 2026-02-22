# SPDX-License-Identifier: Apache-2.0

add_library(motor_core STATIC
	src/angle_observer.c
	src/motor_dob.c
	src/motor_mpr.c
	src/motor_motion_modules.c
	src/motor_foc_voltage_pwm.c
	src/motor_identification.c
	src/motion_profile.c
	src/motor_autonomy.c
	src/prbs.c
	src/rls_motor_est.c
	src/rs_online.c
	src/thermal_model.c
)

target_include_directories(motor_core
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motor_core PUBLIC zephyr_interface)
