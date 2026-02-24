# SPDX-License-Identifier: Apache-2.0

add_library(motor_core STATIC
	src/observers/angle_observer.c
	src/observers/angle_tracking.c
	src/observers/encoder_source.c
	src/runtime/motor_commission_tune.c
	src/runtime/command_arbitration.c
	src/protection/interlocks.c
	src/control/motor_dob.c
	src/control/motor_mpr.c
	src/motion/motor_motion_modules.c
	src/observers/motor_position_convert.c
	src/control/motor_foc_voltage_pwm.c
	src/control/position_regulator.c
	src/control/velocity_regulator.c
	src/estimation/motor_commission_id.c
	src/motion/motion_profile.c
	src/runtime/motor_keepalive_policy.c
	src/observers/motor_encoder_feedback_core.c
	src/motion/motor_outer_loop_sched.c
	src/control/motor_current_ref_policy_core.c
	src/math/prbs.c
	src/estimation/rls_motor_est.c
	src/estimation/rs_online.c
	src/estimation/thermal_model.c
)

target_include_directories(motor_core
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/math
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/filters
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/observers
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/motion
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/control
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/runtime
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/estimation
	${CMAKE_CURRENT_SOURCE_DIR}/include/motor/protection
)

target_link_libraries(motor_core PUBLIC zephyr_interface)
