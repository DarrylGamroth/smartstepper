#!/usr/bin/env bash
set -euo pipefail

HOST="${CHOPPER_TELNET_HOST:-10.0.0.171}"
LOG_ROOT="hil_logs/regression"
LIVE=0
INCLUDE_VELOCITY=0
INCLUDE_POSITION=0

usage() {
  cat <<'USAGE'
Usage: scripts/hil/run_hil_gate.sh [options]

Options:
  --host <ip>             Telnet shell host. Default: CHOPPER_TELNET_HOST or 10.0.0.171.
  --log-root <dir>        Log root. Default: hil_logs/regression.
  --live                  Run live-motion gates after status.
  --include-velocity      Include velocity_encoder validation. Currently known unstable.
  --include-position      Include position validation as part of encoder-validate.
  -h, --help              Show this help.

Default runs only non-motion HIL status.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="$2"; shift 2 ;;
    --log-root)
      LOG_ROOT="$2"; shift 2 ;;
    --live)
      LIVE=1; shift ;;
    --include-velocity)
      INCLUDE_VELOCITY=1; shift ;;
    --include-position)
      INCLUDE_POSITION=1; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/$STAMP"
mkdir -p "$LOG_DIR"

run_hil() {
  local scenario="$1"; shift
  echo "[hil] $scenario"
  python3 scripts/hil/hil_telnet.py "$scenario" \
    --host "$HOST" \
    --connect-timeout 8 \
    --log-dir "$LOG_DIR" \
    --json-report "$LOG_DIR/${scenario}.json" \
    "$@"
}

echo "[hil] host: $HOST"
echo "[hil] logs: $LOG_DIR"
run_hil status

if [[ "$LIVE" -eq 1 ]]; then
  run_hil boot-commission --yes-live-motion --boot-current 0.15 --boot-hz 0.05 --cycles 1
  run_hil current-validate --yes-live-motion --boot-current 0.15 --boot-hz 0.05 --cycles 1 --current-iq 0.03 --current-hold-ms 160

  if [[ "$INCLUDE_VELOCITY" -eq 1 || "$INCLUDE_POSITION" -eq 1 ]]; then
    extra=()
    if [[ "$INCLUDE_POSITION" -eq 1 ]]; then
      extra+=(--include-position)
    fi
    run_hil encoder-validate --yes-live-motion \
      --boot-current 0.15 --boot-hz 0.05 --cycles 1 \
      --current-iq 0.03 --current-hold-ms 160 \
      --velocity-hz 0.05 --velocity-hold-ms 1000 \
      "${extra[@]}"
  fi
fi

echo "[hil] PASS"
