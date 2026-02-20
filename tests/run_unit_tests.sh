#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PODMAN_CONTAINER:-priceless_wiles}"
CONTAINER_PROJECT_PATH="${CHOPPER_CONTAINER_PATH:-/workspace/chopper}"
PLATFORM="${TWISTER_PLATFORM:-native_sim}"
TEST_ROOT="${TWISTER_TEST_ROOT:-tests/unit}"

if [[ $# -gt 0 ]]; then
	CONTAINER_NAME="$1"
	shift
fi

if ! command -v podman >/dev/null 2>&1; then
	echo "ERROR: podman not found in PATH" >&2
	exit 1
fi

if ! podman ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
	echo "ERROR: container '${CONTAINER_NAME}' is not running" >&2
	exit 1
fi

twister_cmd=(
	west twister
	-T "${TEST_ROOT}"
	-p "${PLATFORM}"
	--inline-logs
	-v
)

if [[ $# -gt 0 ]]; then
	twister_cmd+=("$@")
fi

printf -v twister_cmd_str '%q ' "${twister_cmd[@]}"
podman exec "${CONTAINER_NAME}" bash -lc "cd '${CONTAINER_PROJECT_PATH}' && ${twister_cmd_str}"
