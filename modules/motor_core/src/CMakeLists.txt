# SPDX-License-Identifier: Apache-2.0

function(motor_core_configure_target target_name)
	target_include_directories(${target_name}
		PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/../include
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/math
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/filters
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/observers
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/motion
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/control
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/runtime
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/estimation
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/protection
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/calibration
		${CMAKE_CURRENT_SOURCE_DIR}/../include/motor/compensation
		PRIVATE
		${APPLICATION_SOURCE_DIR}/include
	)

	target_link_libraries(${target_name} PUBLIC zephyr_interface)
endfunction()

add_library(motor_core_motion STATIC
	motion/motion_profile.c
	motion/motion_planner.c
	motion/outer_loop_sched.c
)
motor_core_configure_target(motor_core_motion)

add_library(motor_core_estimation STATIC
	estimation/electrical_id.c
	estimation/rls_motor_est.c
	estimation/rls_runtime.c
	estimation/rs_online.c
	estimation/saliency_id.c
	estimation/thermal_model.c
	math/prbs.c
)
motor_core_configure_target(motor_core_estimation)

add_library(motor_core_commission STATIC
	runtime/commission_tune.c
	calibration/encoder_map_detect.c
	estimation/flux_id_estimator.c
	estimation/mech_id_estimator.c
	math/matrix_solve.c
)
motor_core_configure_target(motor_core_commission)

add_library(motor_core_rt STATIC
	runtime/commission_runtime.c
	runtime/outer_loop_runtime.c
	runtime/current_ref_policy_runtime.c
	runtime/control_policy.c
	runtime/control_kernel.c
	runtime/config_snapshot.c
	runtime/persistent_config.c
	runtime/command_arbitration.c
	runtime/keepalive_policy.c
	protection/interlocks.c
	calibration/align.c
	calibration/offset.c
	calibration/rl_ident.c
	calibration/timing.c
	calibration/window.c
	observers/angle_observer.c
	observers/angle_path.c
	observers/encoder_feedback.c
	observers/encoder_feedback_core.c
	control/dob.c
	control/mpr.c
	control/current_loop.c
	control/dq_decoupling.c
	control/pwm_synthesis.c
	control/foc_voltage_pwm.c
	control/position_regulator.c
	control/velocity_regulator.c
	control/current_ref_policy_core.c
	compensation/detent_map.c
	compensation/electrical_ripple_ff.c
)
motor_core_configure_target(motor_core_rt)

target_link_libraries(motor_core_rt PUBLIC
	motor_core_motion
	motor_core_estimation
	motor_core_commission
)
