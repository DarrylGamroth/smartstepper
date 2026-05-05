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

Reconfigure + build (AEAT-9955 HIL shell, recommended for real motor motion
tests):

```bash
podman exec wonderful_goldberg bash -lc '\
  west build \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_hil_shell \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay" \
    -DEXTRA_CONF_FILE="hil_shell.conf;logging.conf"'
```

Use the HIL shell build for hardware motion checks. Avoid `debug.conf` for
motion validation unless specifically debugging faults; it enables debug
optimization and debug log volume that can perturb timing and serial behavior.

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
- Prefer the telnet shell for command automation when Ethernet/DHCP is
  available. Use UART for boot logs, recovery, and fallback.

## Telnet Shell

- The firmware enables the Zephyr telnet shell when the network stack is built.
- Prefer telnet over UART for HIL command automation because it avoids UART RX
  ring overruns when logs are active or commands are sent quickly.
- Get the DHCP address from boot logs or `net iface`. Current lab target
  observed during HIL testing:

```text
10.0.0.171
```

- Recommended interactive command:

```bash
telnet 10.0.0.171
```

- For automation, keep a single persistent telnet session open and pace commands
  by waiting for response/prompt boundaries.
- Repeatable Python HIL workflows are available in `scripts/hil/hil_telnet.py`.
  Live-motion scenarios require `--yes-live-motion`.

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171
python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion --host 10.0.0.171
```

- If the DHCP address is unknown or telnet is unavailable, fall back to the
  persistent UART workflow below.

### Reliable Serial Workflow (Important)

- Use a single persistent serial session (recommended: `tio`) for command/response cycles.
- Do **not** use repeated short-lived `cat`/`timeout` open-close cycles for each command; this can leave reads empty/intermittent even when writes still work.
- Recommended interactive command:

```bash
tio -b 115200 /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0
```

- For automation in this environment, keep one PTY session open to `tio` and send commands through that persistent session.
- Debug/HIL builds include larger shell buffers in `app/debug.conf` or
  `app/hil_shell.conf`:
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

- AEAT-9955 and MT6835 motor profile overlays use
  `encoder-direction-sign = <1>` for closed-loop commutation.
- Files:
  - `app/configs/motor_aeat9955_067a.overlay`
  - `app/configs/motor_mt6835_2a.overlay`
- Verification method (closed-loop commutation check):
  1. Build and flash the matching motor profile overlay.
  2. Run `motor safety timeout 0`, then `motor state calibrate`.
  3. Enter `motor state mode current_encoder`.
  4. Start encoder telemetry capture.
  5. Arm and apply a conservative positive q-axis current, for example
     `motor current iq 0.10`.
  6. Stop current, disarm, stop capture, and verify rotor movement with
     `motor encoder capture summary`.
- HIL note: on the AEAT-9955 setup, runtime `-1` regulated current but
  snapped roughly one electrical quadrant to a holding point. Runtime `+1`
  produced real closed-loop torque response; high current (`0.50 A`) caused
  overcurrent, so use conservative currents while debugging.

## Encoder Mapping Commissioning

- Runtime boot ALIGN is intentionally single-vector and trajectory-ramped.
- Dual-polarity ALIGN is no longer part of the normal boot calibration path.
- Use the generated-sweep commissioning command to validate/stage encoder sign
  and offset:

```text
motor commission encoder run <current_a> <mech_hz> <cycles>
motor commission encoder status
motor commission encoder apply
motor commission encoder clear
```

- The command drives `velocity_generated`, samples generated electrical phase
  and raw encoder angle, and reports direction, offset, residuals, warnings,
  and errors.
- Warning status is reported separately; a small bounded number of rejected
  parity/transport/sample errors is tolerated when the fit quality remains high.
- If the result is invalid with low measured motion, increase the generated
  sweep current/speed or debug open-loop motion first. The command intentionally
  leaves runtime parameters unchanged unless `apply` is run after a valid result.
- AEAT-9955 HIL evidence shows parity-clean implausible angle jumps can occur.
  The encoder acquisition path has a `glitch` counter in `motor encoder acquisition`
  and rejects jumps above the ISR plausibility threshold before the observer uses
  them.

## AEAT-9955 Telemetry-Only Use

- The AEAT-9955 may be usable for coarse before/after motion telemetry even
  when it is not good enough for closed-loop commutation.
- For generated-angle modes such as `velocity_generated` and `position_generated`, use the
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
