# Validation

## Static Validation

```bash
rg -n "motor_resolve_requested_online_mode" app/src app/include
rg -n "requested_online_mode" app/src app/include
rg -n "MOTOR_EVENT_MODE_CHANGE|MOTOR_EVENT_ONLINE|MOTOR_EVENT_CLEAR_ERROR" app/src app/include
```

Expected final direction:

- one requested-online-mode resolver or one shared implementation used by all paths
- no command path that silently stages a mode without transition status visibility
- no duplicate encoder readiness guard behavior

## Unit Tests

Run all unit tests:

```bash
./tests/run_unit_tests.sh
```

Recommended focused test areas to add during this plan:

- transition guard helpers
- mode descriptor to feature-flag derivation
- transition status update logic
- fault recovery state helper logic

## Firmware Build

Use the west build flow, not direct CMake reconfigure commands.

MT6835 profile:

```bash
podman exec wonderful_goldberg bash -lc '
  cd /workspace && west build -p auto \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```

AEAT-9955 profile:

```bash
podman exec wonderful_goldberg bash -lc '
  cd /workspace && west build -p auto \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_aeat9955 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay"'
```

## HIL Smoke

Use telnet when available.

```bash
python3 -u scripts/hil/hil_telnet.py status --host 10.0.0.44 --connect-timeout 8 --command-timeout 3
```

Generated velocity smoke:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state mode velocity_generated
motor state online
motor arm
motor current id 0
motor current iq 0.15
motor velocity target 0.5
motor info live
motor velocity target 0
motor current iq 0
motor disarm
motor state idle
motor safety timeout 1000
```

Encoder current smoke after commissioning/mapping:

```text
motor state clear_error
motor gate reset
motor state idle
motor safety timeout 0
motor state mode current_encoder
motor state online
motor arm
motor current id 0
motor current iq 0.15
motor fault snapshot start 20
motor info live
motor fault snapshot stop
motor fault snapshot status
motor fault snapshot dump 16
motor current iq 0
motor disarm
motor state idle
motor safety timeout 1000
```

Transition validation should explicitly check final state/status, not only command echo text.
