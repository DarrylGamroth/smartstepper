# Torque Closed-Loop HIL

This procedure validates sensored closed-loop torque mode (`ONLINE_TORQUE`) and
captures fault snapshots for quick diagnosis.

## Preconditions

- Firmware flashed to `smartstepper_v2`.
- Serial shell connected:
  - device: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0`
  - baud: `115200`
- Motor can be safely energized.

## Automated Script (Recommended)

Run from repo root:

```bash
DEV=/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0
LOG=/tmp/torque_mode_retest.log
rm -f "$LOG"
stty -F "$DEV" 115200 raw -echo -echoe -echok -echoctl -echoke

send_read(){
  local cmd="$1"; local dur="${2:-1.0}"; local t
  t=$(mktemp)
  (timeout "$dur" cat "$DEV" | tr -d '\r' > "$t") & local p=$!
  sleep 0.06
  printf "\r\n%s\r\n" "$cmd" > "$DEV"
  wait "$p" || true
  {
    echo "\n### CMD: $cmd"
    cat "$t"
  } >> "$LOG"
  rm -f "$t"
}

run_case(){
  local iq="$1"
  echo "\n===== torque_closed_loop iq=${iq}A =====" >> "$LOG"
  send_read "motor state clear_error" 0.8
  send_read "motor disarm" 0.8
  send_read "motor state idle" 0.8
  send_read "motor safety timeout 0" 0.8
  send_read "motor fault snapshot clear" 0.8
  send_read "motor state offline" 3.8
  send_read "motor arm" 0.8
  send_read "motor state mode torque" 1.0
  send_read "motor state status" 1.0
  send_read "motor current id 0" 0.8
  send_read "motor current iq ${iq}" 1.5
  send_read "motor info live" 1.2
  send_read "motor state status" 1.2
  send_read "motor fault snapshot status" 1.0
  send_read "motor fault snapshot dump 8" 1.4
  send_read "motor current iq 0" 0.8
  send_read "motor disarm" 0.8
  send_read "motor state idle" 0.8
  send_read "motor safety timeout 1000" 0.8
}

run_case 0.03
run_case 0.06
send_read "motor state status" 1.0

echo "Saved log: $LOG"
rg -n "=====|### CMD: motor state mode torque|State:|Error:|Armed:|Iq reference|Iq measured|Speed:|OVERCURRENT|Fault:|Latest:" "$LOG" -S
```

## Pass Criteria

- Mode enters and stays in `ONLINE_TORQUE` while armed.
- `Error: NONE` and no `OVERCURRENT` lines.
- `Iq measured` tracks `Iq reference` within reasonable tolerance.
- Fault snapshot can be dumped successfully.

## Manual Command Sequence

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor fault snapshot clear
motor state offline
motor arm
motor state mode torque
motor current id 0
motor current iq 0.06
motor info live
motor fault snapshot status
motor fault snapshot dump 8
motor current iq 0
motor disarm
motor state idle
motor safety timeout 1000
```

## Notes

- If the motor faults immediately on mode entry, check handoff-related fields in
  `motor info live` and `motor fault snapshot dump` first.
- `Enc flags` warning bit activity can appear without transport failure; correlate
  with `fresh/warn/err/status` in fault snapshot rows.
