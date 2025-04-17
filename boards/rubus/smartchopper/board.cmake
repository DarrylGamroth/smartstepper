# SPDX-License-Identifier: Apache-2.0
board_runner_args(jlink "--device=STM32F407IE" "--speed=10000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
