# SPDX-License-Identifier: Apache-2.0

add_library(motor_core STATIC
	src/observers/angle_observer.c
	src/motor_commission_tune.c
	src/motor_dob.c
	src/motor_mpr.c
	src/motion/motor_motion_modules.c
	src/observers/motor_position_convert.c
	src/motor_foc_voltage_pwm.c
	src/motor_commission_id.c
	src/motion/motion_profile.c
	src/motion/motor_autonomy.c
	src/observers/motor_encoder_feedback_core.c
	src/motion/motor_outer_loop_sched.c
	src/motor_current_ref_policy_core.c
	src/math/prbs.c
	src/rls_motor_est.c
	src/rs_online.c
	src/thermal_model.c
)

target_include_directories(motor_core
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/math
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/filters
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/observers
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/motion
)

target_link_libraries(motor_core PUBLIC zephyr_interface)
