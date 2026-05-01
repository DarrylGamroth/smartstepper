# Chopper Agent Notes

This file captures the known-good build and test commands for this workspace.

## Environment

- Host repo path: `/home/dgamroth/workspaces/zephyr-workspace/chopper`
- Podman container name: `wonderful_goldberg`
- Container workspace mount: `/workspace/chopper`
- Zephyr workspace root in container: `/workspace`

If the container is not running:

```bash
podman start wonderful_goldberg
```

## Firmware Build (smartstepper_v2)

Use `west build` for firmware validation. Prefer explicit west arguments so the
board, snippets, overlays, and config files are visible in the command. This
matches the intent of the VS Code tasks in `../.vscode/tasks.json`.

Fast incremental rebuild of the currently configured west build:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

Use the incremental rebuild for code-only changes when the build directory is
already configured for the desired motor overlay. Reconfigure when changing
overlays, snippets, Kconfig fragments, board, or generated devicetree inputs.

Reconfigure + build (AEAT-9955 serial shell, matches
`West build Serial Shell AEAT-9955 (app)`):

```bash
podman exec wonderful_goldberg bash -lc '\
  west build \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay" \
    -DEXTRA_CONF_FILE="debug.conf;logging.conf"'
```

Reconfigure + build (MT6835 serial shell):

```bash
podman exec wonderful_goldberg bash -lc '\
  west build \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay" \
    -DEXTRA_CONF_FILE="debug.conf;logging.conf"'
```

Pristine reconfigure is only needed if CMake cache/config state is stale:

```bash
podman exec wonderful_goldberg bash -lc '\
  west build -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2 \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay" \
    -DEXTRA_CONF_FILE="debug.conf;logging.conf"'
```

Overlay note:

- The build requires a motor profile overlay that defines `/user_parameters` and `/motor_parameters`.
- Default profile is `configs/motor_mt6835_2a.overlay`.
- For AEAT-9955 hardware, switch to `configs/motor_aeat9955_067a.overlay`.

## Unit Tests

Run all unit tests (recommended wrapper):

```bash
./tests/run_unit_tests.sh
```

Run all unit tests directly in container:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace/chopper && west twister -T tests/unit -p native_sim --inline-logs -v'
```

Run one unit test suite (example):

```bash
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motion_profile.unit
```

## Serial Shell

- Device: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0`
- Baud rate: `115200`
- Use this serial shell to run HIL commands against the actual motor hardware
  (state transitions, current/velocity/position modes, commissioning, and safety checks).

### Reliable Serial Workflow (Important)

- Use a single persistent serial session (recommended: `tio`) for command/response cycles.
- Do **not** use repeated short-lived `cat`/`timeout` open-close cycles for each command; this can leave reads empty/intermittent even when writes still work.
- Recommended interactive command:

```bash
tio -b 115200 /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0
```

- For automation in this environment, keep one PTY session open to `tio` and send commands through that persistent session.
- Debug/HIL builds include larger shell buffers in `app/debug.conf`:
  `CONFIG_SHELL_CMD_BUFF_SIZE=512`,
  `CONFIG_SHELL_HISTORY_BUFFER=1024`, and
  `CONFIG_SHELL_BACKEND_SERIAL_RX_RING_BUFFER_SIZE=8192`.
- Even with the larger RX ring, automation should pace commands and wait for
  output/prompt boundaries. Avoid pasting long multi-command bursts into the
  serial shell while logs are active.

## Debug Probe

- J-Link is available on the network at `10.0.0.70`.

## Hardware Access Permissions

- You are allowed to flash the real target hardware from this workspace using the Zephyr `west` J-Link runner.
- Preferred command:

```bash
podman exec wonderful_goldberg bash -lc 'west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
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

## AEAT-9955 Telemetry-Only Use

- The AEAT-9955 may be usable for coarse before/after motion telemetry even
  when it is not good enough for closed-loop commutation.
- For generated-angle modes such as `velocity_open` and `profile_open`, use the
  Zephyr sensor shell to sample it before and after a move:

```text
sensor get aeat9955@0
```

- Do not treat this as proof that the AEAT-9955 is safe as the FOC angle
  source; it only verifies gross rotor movement and sensor availability.

## Notes

- `tests/run_unit_tests.sh` defaults:
  - container: `wonderful_goldberg`
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
