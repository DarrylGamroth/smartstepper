#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT_DIR="${ROOT_DIR}/build/host_tests"
OUT_BIN="${OUT_DIR}/motor_autonomy_test"

mkdir -p "${OUT_DIR}"

"${CC:-cc}" -std=c17 -Wall -Wextra -Werror \
	-I"${ROOT_DIR}/app/include" \
	"${ROOT_DIR}/app/src/motor_autonomy.c" \
	"${ROOT_DIR}/tests/host/motor_autonomy/test_motor_autonomy.c" \
	-o "${OUT_BIN}"

"${OUT_BIN}"
echo "PASS: motor_autonomy host tests"
