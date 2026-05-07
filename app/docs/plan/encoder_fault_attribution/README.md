# Encoder Fault Attribution Plan

Goal: make `ERROR / ENCODER_FAULT` actionable by recording which real-time guard posted the fault.

## Scope

- Add a small encoder-fault reason enum/string helper.
- Latch the reason from the ADC/control ISR report path.
- Add shell visibility in `motor fault snapshot status` and `dump`.
- Add HIL parser visibility for the reason.
- Validate with unit/parser tests and firmware build.

## Non-Goals

- Do not change encoder control policy or thresholds.
- Do not change commissioning behavior beyond diagnostics.
- Do not add heavyweight logging in the ISR.

## Tasks

1. Define reason IDs and string mapping.
2. Set reason at the three encoder-fault post sites in `motor_control_loop_step`.
3. Latch reason in the ISR publish path and fault snapshot sample.
4. Print reason in shell fault snapshot commands.
5. Parse reason in HIL reports.
6. Run parser tests and firmware build.

## Progress

- [x] T1 reason IDs/string mapping
- [x] T2 ISR fault reason assignment
- [x] T3 latch/snapshot storage
- [x] T4 shell visibility
- [x] T5 HIL parser visibility
- [x] T6 validation

## Evidence

- `python3 -m py_compile scripts/hil/hil_telnet.py`
- `python3 -m unittest scripts/hil/test_hil_telnet_parser.py`
  - 22 tests passed.
- Firmware build:
  - `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -p auto -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'`
  - Build passed.
