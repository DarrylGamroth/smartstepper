# Chopper Agent Notes

This file captures the known-good build and test commands for this workspace.

## Environment

- Host repo path: `/home/dgamroth/workspaces/zephyr-workspace/chopper`
- Podman container name: `priceless_wiles`
- Container workspace mount: `/workspace/chopper`
- Zephyr workspace root in container: `/workspace`

If the container is not running:

```bash
podman start priceless_wiles
```

## Firmware Build (smartstepper_v2)

Incremental build (fast path):

```bash
podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'
```

Clean reconfigure + build (MT6835 profile overlay enabled):

```bash
podman exec priceless_wiles bash -lc '\
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="app/boards/smartstepper_v2.overlay;app/configs/motor_mt6835_2a.overlay"'
```

Overlay note:

- The build requires a motor profile overlay that defines `/user_parameters` and `/motor_parameters`.
- Default profile is `app/configs/motor_mt6835_2a.overlay`.
- For AEAT-9955 hardware, switch to `app/configs/motor_aeat9955_067a.overlay`.

## Unit Tests

Run all unit tests (recommended wrapper):

```bash
./tests/run_unit_tests.sh
```

Run all unit tests directly in container:

```bash
podman exec priceless_wiles bash -lc 'cd /workspace/chopper && west twister -T tests/unit -p native_sim --inline-logs -v'
```

Run one unit test suite (example):

```bash
./tests/run_unit_tests.sh priceless_wiles -s chopper.motion_profile.unit
```

## Serial Shell

- Device: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0`
- Baud rate: `115200`
- Use this serial shell to run HIL commands against the actual motor hardware
  (state transitions, current/velocity/position modes, commissioning, and safety checks).

## Debug Probe

- J-Link is available on the network at `10.0.0.70`.

## Hardware Access Permissions

- You are allowed to flash the real target hardware from this workspace using the Zephyr `west` J-Link runner.
- Preferred command:

```bash
podman exec priceless_wiles bash -lc 'west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

- You are allowed to control and test the real motor through the serial shell interface listed above.

## Encoder Direction Mapping

- Default devicetree mapping for this hardware is `encoder-direction-sign = <(-1)>`.
- Files:
  - `app/boards/smartstepper_v2.overlay`
  - `app/boards/smartstepper.overlay`
- Verification method (velocity-open direction check):
  1. Run `motor state offline`, then `motor arm`.
  2. Enter `motor state mode velocity_open`.
  3. Set `motor current iq 0.15` and `motor velocity target 5`.
  4. Capture encoder data (`motor encoder capture start 1`, wait, `motor encoder capture stop`).
  5. Compare with generated reference (`motor encoder capture compare 96 gen`).
  6. Correct mapping is when encoder mechanical direction matches generated mechanical direction over the capture window.

## Notes

- `tests/run_unit_tests.sh` defaults:
  - container: `priceless_wiles`
  - test root: `tests/unit`
  - platform: `native_sim`
- The wrapper passes extra args through to `west twister`, so use twister flags after the optional container argument.

## Plan Pack Usage (LLM)

Use the plan pack at `app/docs/plan/` as the execution contract for the motor-core refactor.

Start each new LLM session by reading:

1. `app/docs/plan/README.md`
2. `app/docs/plan/00_scope.md`
3. `app/docs/plan/01_architecture_target.md`
4. `app/docs/plan/tasks/index.md`
5. `app/docs/plan/validation.md`

Execution rules:

1. Execute tasks in the order listed in `app/docs/plan/tasks/index.md`.
2. For each task card `app/docs/plan/tasks/Txxxx.yaml`, obey `touch_files`, `do_not_touch`, and `constraints`.
3. Run the task `validation` commands before marking done.
4. Append progress/evidence to `app/docs/plan/execution_log.md`.
5. Include task ID(s) in commit messages.
6. Avoid extra scope beyond the current task unless required to satisfy validation.

Reference seed commit for this plan pack: `f22b4b4`.
