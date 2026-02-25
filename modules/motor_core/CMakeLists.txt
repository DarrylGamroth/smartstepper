# SPDX-License-Identifier: Apache-2.0

add_library(motor_core STATIC
	src/observers/angle_observer.c
	src/observers/angle_tracking.c
	src/observers/encoder_source.c
	src/runtime/commission_tune.c
	src/runtime/motor_core_step.c
	src/runtime/command_arbitration.c
	src/protection/interlocks.c
	src/control/dob.c
	src/control/mpr.c
	src/control/current_loop.c
	src/control/dq_decoupling.c
	src/control/transforms.c
	src/control/pwm_synthesis.c
	src/motion/motion_planner.c
	src/observers/position_convert.c
	src/control/foc_voltage_pwm.c
	src/control/position_regulator.c
	src/control/velocity_regulator.c
	src/math/matrix_solve.c
	src/estimation/flux_id_estimator.c
	src/estimation/mech_id_estimator.c
	src/motion/motion_profile.c
	src/runtime/keepalive_policy.c
	src/observers/encoder_feedback_core.c
	src/motion/outer_loop_sched.c
	src/control/current_ref_policy_core.c
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
	PRIVATE
	${APPLICATION_SOURCE_DIR}/include
)

target_link_libraries(motor_core PUBLIC zephyr_interface)
