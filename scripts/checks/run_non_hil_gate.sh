#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${1:-wonderful_goldberg}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

cd "$ROOT"

echo "[non-hil] container: $CONTAINER"
echo "[non-hil] running unit tests"
./tests/run_unit_tests.sh "$CONTAINER"

echo "[non-hil] running firmware build"
podman exec "$CONTAINER" bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'

echo "[non-hil] PASS"
