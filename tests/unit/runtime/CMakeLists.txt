cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(runtime_unit_tests)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/include
)

# Fast-step boundary: keep ISR process path free of queue/logging calls.
set(MOTOR_CORE_STEP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/src/motor_control_loop.c)
file(READ ${MOTOR_CORE_STEP_SRC} MOTOR_CORE_STEP_TEXT)

foreach(FORBIDDEN_TOKEN "k_msgq_" "k_fifo_" "k_work_" "k_sem_" "LOG_" "printk(")
  string(FIND "${MOTOR_CORE_STEP_TEXT}" "${FORBIDDEN_TOKEN}" TOKEN_POS)
  if (NOT TOKEN_POS EQUAL -1)
    message(FATAL_ERROR "motor_control_loop.c violates fast-step side-effect boundary: found '${FORBIDDEN_TOKEN}'")
  endif()
endforeach()

target_sources(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/motor_core/src/runtime/config_snapshot.c
	src/main.c
)
