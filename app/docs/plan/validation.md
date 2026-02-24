# Validation Commands

## Build

1. Firmware build (smartstepper_v2):
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```

2. Firmware build (smartstepper_v2_mt6835):
```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2_mt6835 -j4'
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
motor state offline
motor arm
motor state mode velocity_open
motor current id 0
motor current iq 0.15
motor velocity target 5
motor state status
```

## Per-Task Validation Policy

1. Doc-only tasks: `markdown lint/readability self-check` and no build required.
2. Header/include move tasks: both firmware builds.
3. Runtime/control code tasks: both firmware builds + relevant unit suites.
4. ISR-path behavior tasks: both builds + unit suites + HIL smoke.

