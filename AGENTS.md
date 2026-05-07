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

Sysbuild + MCUboot build (MT6835 profile):

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build --sysbuild -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mcuboot \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'
```

Sysbuild + MCUboot build (AEAT-9955 profile):

```bash
podman exec wonderful_goldberg bash -lc '\
  cd /workspace && \
  west build --sysbuild -p always \
    -b smartstepper_v2/stm32h743xx \
    /workspace/chopper/app \
    -d /workspace/build/chopper/smartstepper_v2_mcuboot_aeat \
    -S serial-shell -S serial-console -- \
    -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_aeat9955_067a.overlay"'
```

What sysbuild means here:

- `west build --sysbuild` is Zephyr's multi-image build flow.
- It configures and builds a standalone MCUboot image plus the application image
  in one build directory.
- MCUboot is not linked into the app. It is a separate bootloader image that
  runs first after reset, validates the signed app, then jumps to it.
- The application is built MCUboot-aware: linked for slot0 and signed for
  MCUboot validation.
- Current outputs:
  - bootloader: `/workspace/build/chopper/smartstepper_v2_mcuboot/mcuboot/zephyr/zephyr.bin`
  - signed app: `/workspace/build/chopper/smartstepper_v2_mcuboot/app/zephyr/zephyr.signed.bin`
  - signed app hex: `/workspace/build/chopper/smartstepper_v2_mcuboot/app/zephyr/zephyr.signed.hex`
- Use normal app-only builds for fast motor/control development.
- Use sysbuild for bootloader, partition, signing, rollback, settings-retention,
  and field-update validation.

MCUboot notes:

- The smartstepper_v2 base DTS allocates internal flash as 128 KiB MCUboot,
  896 KiB slot0, 896 KiB slot1, and 128 KiB `settings_storage`.
- The sysbuild requires the `zcbor` Zephyr module for MCUmgr/CBOR support.
  Checksum-verified MCUmgr image upload also requires Zephyr's
  `tf-psa-crypto` module through `CONFIG_IMG_ENABLE_IMAGE_CHECK=y`. Both are
  included in `west.yml`; if a workspace is missing either module, run
  `west update`.
  If the container global Git config rewrites HTTPS GitHub URLs to SSH and
  breaks unauthenticated fetches, run the update with `GIT_CONFIG_GLOBAL=/dev/null`.
- Development sysbuild currently uses Zephyr/MCUboot's default ECDSA-P256 key.
  Replace this with a production key before field deployment.
- Development flashing still uses the J-Link runner.
- To flash the bootloader-managed image during development, flash the sysbuild
  directory:

```bash
podman exec wonderful_goldberg bash -lc '\
  west flash -d /workspace/build/chopper/smartstepper_v2_mcuboot \
    --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

- Field updates use MCUboot + MCUmgr SMP over UDP/IPv4 Ethernet on port `1337`.
  The normal update payload is `app/zephyr/zephyr.signed.bin`.
- The firmware enables Zephyr's built-in `mcuboot` shell command through
  `CONFIG_MCUBOOT_SHELL`; there is no project-specific `motor update` command.
- The firmware enables MCUmgr SMP over UDP/IPv4 on port `1337` for Ethernet
  field updates. Use the host-installed `mcumgrctl` tool
  (`/home/dgamroth/.cargo/bin/mcumgrctl`) rather than looking for `mcumgr` in
  the container.
- Field-update operator documentation lives in
  `app/docs/field_update_user_guide.md`.
- HIL field-update verification was completed on 2026-05-07 against the MT6835
  target at `10.0.0.44`: MCUmgr upload/test/reset/confirm over UDP worked and
  the confirmed test image stayed active after a second reset.

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

## Planning and Scope Control

Keep implementation plans small enough to complete, validate, and review in one
focused work cycle. Prefer one plan per owner concept:

- one architectural cleanup,
- one control feature,
- one commissioning workflow,
- one shell/API cleanup,
- one HIL validation target.

Avoid broad plans that combine algorithm design, runtime refactor, shell naming,
commissioning, persistence, and HIL validation at the same time. If a request
spans multiple concepts, create a program index and split the work into narrow
execution plans.

## Git Workflow

- Commit changes as work progresses so each functional step has a rollback
  point.
- Commit before starting a new major plan/task and again after validation passes.
- Keep commit messages specific to the completed task or evidence produced.
- Do not let large refactors, HIL experiments, and documentation updates pile up
  uncommitted unless explicitly asked to pause before committing.

Each plan should use this structure:

```md
# Title

## Goal
One sentence.

## Non-Goals
Explicitly exclude related work.

## Current Problem
What is broken or confusing now.

## Design
Small target architecture.

## Implementation Phases
Phase 1:
Phase 2:
Phase 3:

## Acceptance Criteria
Measurable pass/fail.

## HIL Evidence
Commands, expected result, log path.

## Risks
What could invalidate the plan.

## Done State
What code/docs/tests prove completion.
```

Plan execution rules:

- Read the relevant plan before editing code.
- Do not expand scope just because related code is nearby.
- Run the validation listed in the plan before marking it complete.
- Record evidence in the plan or a linked HIL log.
- If implementation reveals a larger architectural issue, create a follow-up
  plan instead of silently expanding the current one.

Current focused control-system plan set:

- `app/docs/control_system_improvement_index.md`
- `app/docs/encoder_sample_quality_plan.md`
- `app/docs/angle_observer_contract_plan.md`
- `app/docs/detent_learning_v2_plan.md`
- `app/docs/mpr_bandwidth_interface_plan.md`
- `app/docs/dob_enable_policy_plan.md`
- `app/docs/hil_motion_regression_plan.md`
- `app/docs/motion_control_status_shell_plan.md`

Recommended execution order:

1. Encoder sample quality.
2. Angle observer contract.
3. HIL motion regression baseline.
4. MPR bandwidth interface.
5. Detent learning v2.
6. DOB enable policy.
7. Shell/status cleanup.

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

- AEAT-9955 board device: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTE3B04Y-if00-port0`
- MT6835 board device: `/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTHC021S-if00-port0`
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
  addresses observed during HIL testing:

```text
AEAT-9955 board: 10.0.0.171
MT6835 board:    10.0.0.44
```

- Recommended interactive command:

```bash
telnet 10.0.0.171
telnet 10.0.0.44
```

- For automation, keep a single persistent telnet session open and pace commands
  by waiting for response/prompt boundaries.
- Repeatable Python HIL workflows are available in `scripts/hil/hil_telnet.py`.
  Live-motion scenarios require `--yes-live-motion`.

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171
python3 scripts/hil/hil_telnet.py recovery-status --host 10.0.0.171
python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion --host 10.0.0.171
```

- If the DHCP address is unknown or telnet is unavailable, fall back to the
  persistent UART workflow below.
- For state-machine regressions, always check both `motor state transition` and
  `motor state recovery`. The HIL `status` and `recovery-status` scenarios do
  this automatically.

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
  `encoder-direction-sign = <(-1)>` for closed-loop commutation.
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
- The application supports the fast `encoder_rt` path only for encoder control.
  Motor profile overlays should alias `encoder1` to the fast encoder child; do
  not add a separate `rtspi0` alias.
- AEAT-9955 HIL evidence shows parity-clean implausible angle jumps can occur.
  The encoder acquisition path has a `glitch` counter in `motor encoder acquisition`
  and rejects jumps above the ISR plausibility threshold before the observer uses
  them.

## AEAT-9955 Telemetry-Only Use

- The AEAT-9955 may be usable for coarse before/after motion telemetry even
  when it is not good enough for closed-loop commutation.
- For generated-angle modes such as `velocity_generated` and `position_generated`, use
  the project encoder trace commands to sample it before and after a move:

```text
motor encoder trace clear
motor encoder trace start 100
motor position target 30
motor encoder trace stop
motor encoder trace summary
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

## System Review Recommendations Plan

Use `app/docs/reviews/system_review_recommendations_plan.md` as the execution
contract for the recommendations from
`app/docs/reviews/system_review_2026-05-05.md`.

Progress and validation evidence must be recorded in:

```text
app/docs/reviews/system_review_recommendations_progress.md
```

Execution rules:

1. Work phases in order unless an explicit blocker requires a different order.
2. Do not mark a phase complete until the code/docs/scripts are committed and
   the required unit-test, build, and HIL evidence is recorded.
3. Include validation commands, summarized results, HIL log paths, and open
   risks in the progress log.
4. Use telnet HIL automation when Ethernet shell is available.

## Regression Gate

- Non-HIL gate:

```bash
scripts/checks/run_non_hil_gate.sh wonderful_goldberg
```

- HIL status gate after flash:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.171
```

- Live-motion gate when validating runtime motor behavior:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.171 --live
```

- Velocity/position encoder gates are currently known unstable and should be run
  explicitly when tuning those loops:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.171 --live --include-velocity
```
