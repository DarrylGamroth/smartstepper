# Validation Commands

## Build

1. Firmware build (smartstepper_v2):
```bash
podman exec priceless_wiles bash -lc '\
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```

2. Firmware build (smartstepper_v2_mt6835):
```bash
podman exec priceless_wiles bash -lc '\
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mt6835 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```

## Unit Tests

1. Full unit suite:
```bash
./tests/run_unit_tests.sh
```

2. Single suite template:
```bash
./tests/run_unit_tests.sh priceless_wiles -s <suite_name>
```

## HIL (serial shell)

Device: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0` at `115200`.

Baseline smoke sequence:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state prepare
motor arm
motor state mode velocity_generated
motor current id 0
motor current iq 0.15
motor velocity target 5
motor state status
```

## Baseline ISR Metrics (P00)

Capture baseline ISR cycle and stack metrics before refactor work.

1. Start from a clean online run and collect ISR cycle counters:
```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state prepare
motor arm
motor state mode velocity_generated
motor current id 0
motor current iq 0.15
motor velocity target 5
motor info stats
```

2. Capture thread stack watermark (if kernel shell module is enabled):
```text
kernel thread stacks
```

3. If `kernel thread stacks` is unavailable, record this fallback:
  - note `command unavailable` in `app/docs/plan/execution_log.md`
  - continue with ISR-cycle baseline and proceed to the next task

## Per-Task Validation Policy

1. Doc-only tasks: `markdown lint/readability self-check` and no build required.
2. Header/include move tasks: both firmware builds.
3. Runtime/control code tasks: both firmware builds + relevant unit suites.
4. ISR-path behavior tasks: both builds + unit suites + HIL smoke.
