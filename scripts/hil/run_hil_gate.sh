#!/usr/bin/env bash
set -euo pipefail

HOST="${CHOPPER_TELNET_HOST:-10.0.0.44}"
LOG_ROOT="hil_logs/regression"
LIVE=0
OPEN_LOOP_TRACE=1
CURRENT_VALIDATE=1
INCLUDE_VELOCITY=0
INCLUDE_POSITION=0
KEEP_GOING=0
FAILURES=0

usage() {
  cat <<'USAGE'
Usage: scripts/hil/run_hil_gate.sh [options]

Options:
  --host <ip>             Telnet shell host. Default: CHOPPER_TELNET_HOST or 10.0.0.44.
  --log-root <dir>        Log root. Default: hil_logs/regression.
  --live                  Run live-motion gates after status.
  --skip-open-loop-trace  Skip generated/open-loop encoder trace in live gate.
  --skip-current          Skip current_encoder validation in live gate.
  --include-velocity      Include velocity_encoder validation. Currently known unstable.
  --include-position      Include position validation. Currently experimental.
  --keep-going            Continue after a scenario failure and report aggregate failure.
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
    --skip-open-loop-trace)
      OPEN_LOOP_TRACE=0; shift ;;
    --skip-current)
      CURRENT_VALIDATE=0; shift ;;
    --include-velocity)
      INCLUDE_VELOCITY=1; shift ;;
    --include-position)
      INCLUDE_POSITION=1; shift ;;
    --keep-going)
      KEEP_GOING=1; shift ;;
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
  if python3 scripts/hil/hil_telnet.py "$scenario" \
      --host "$HOST" \
      --connect-timeout 8 \
      --log-dir "$LOG_DIR" \
      --json-report "$LOG_DIR/${scenario}.json" \
      "$@"; then
    return 0
  fi

  FAILURES=$((FAILURES + 1))
  if [[ "$KEEP_GOING" -eq 0 ]]; then
    return 1
  fi
  echo "[hil] scenario failed, continuing because --keep-going is set: $scenario" >&2
  return 0
}

echo "[hil] host: $HOST"
echo "[hil] logs: $LOG_DIR"
run_hil status

if [[ "$LIVE" -eq 1 ]]; then
  run_hil boot-commission --yes-live-motion --boot-current 0.15 --boot-hz 0.10 --cycles 1

  if [[ "$OPEN_LOOP_TRACE" -eq 1 ]]; then
    run_hil encoder-trace-open-loop --yes-live-motion \
      --open-loop-iq 0.12 --open-loop-hz 0.10 \
      --trace-ms 1000 --trace-decimation 1
  fi

  if [[ "$CURRENT_VALIDATE" -eq 1 ]]; then
    run_hil current-validate --yes-live-motion --boot-current 0.15 --boot-hz 0.10 --cycles 1 --current-iq 0.03 --current-hold-ms 160
  fi

  if [[ "$INCLUDE_VELOCITY" -eq 1 ]]; then
    run_hil velocity-validate --yes-live-motion \
      --boot-current 0.15 --boot-hz 0.10 --cycles 1 \
      --velocity-hz 0.50 --velocity-hold-ms 1000
  fi

  if [[ "$INCLUDE_POSITION" -eq 1 ]]; then
    run_hil position-validate --yes-live-motion \
      --boot-current 0.15 --boot-hz 0.10 --cycles 1 \
      --velocity-hz 0.50 --velocity-hold-ms 1000 \
      --position-delta-deg 5 --position-hold-ms 2000
  fi
fi

cat > "$LOG_DIR/summary.json" <<SUMMARY
{
  "host": "$HOST",
  "log_dir": "$LOG_DIR",
  "live": $LIVE,
  "open_loop_trace": $OPEN_LOOP_TRACE,
  "current_validate": $CURRENT_VALIDATE,
  "include_velocity": $INCLUDE_VELOCITY,
  "include_position": $INCLUDE_POSITION,
  "failures": $FAILURES
}
SUMMARY

if [[ "$FAILURES" -ne 0 ]]; then
  echo "[hil] FAIL: $FAILURES scenario(s) failed"
  exit 1
fi

echo "[hil] PASS"
