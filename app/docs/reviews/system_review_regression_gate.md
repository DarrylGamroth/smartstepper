# System Review Regression Gate

Date: 2026-05-05

This gate captures the repeatable checks for this project after the system review
implementation work. It separates required non-motion checks from live-motion
HIL checks and known unstable control-loop checks.

## Required Non-HIL Gate

Run before committing control/library changes:

```bash
scripts/checks/run_non_hil_gate.sh wonderful_goldberg
```

This runs:

- all native_sim unit tests through `tests/run_unit_tests.sh`.
- the configured `smartstepper_v2` firmware build through `west build` in the
  podman container.

Pass criteria:

- 100% unit-test pass.
- firmware build exits 0.

## Required Non-Motion HIL Gate

Run after flashing any firmware that touches app control, drivers, ISR paths, or
shell/HIL behavior:

```bash
podman exec wonderful_goldberg bash -lc \
  'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'

scripts/hil/run_hil_gate.sh --host 10.0.0.171
```

This runs `hil_telnet.py status` and stores logs under
`hil_logs/regression/<timestamp>/`.

Pass criteria:

- no fatal text.
- motor error is `NONE`.
- fault snapshot latch is clear.
- encoder acquisition counters are within thresholds.

## Required Live-Motion Bring-Up Gate

Run before declaring encoder/current-control behavior stable:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.171 --live
```

This runs:

- `boot-commission` with generated-sweep mapping.
- `current-validate` with `Iq=0.03 A`.

Pass criteria:

- boot commissioning completes and applies encoder mapping.
- current_encoder validation moves in opposite directions for `+Iq` and `-Iq`.
- no motor fault or fault snapshot latch.
- encoder transport/status counters stay within thresholds for the scenario.

## Known Unstable Checks

Velocity/position encoder loops are intentionally separated because they are not
stable enough yet to be release blockers for non-motion refactors.

Run velocity anyway when working on controller tuning:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.171 --live --include-velocity
```

Run position as part of full encoder validation once velocity is stable:

```bash
scripts/hil/run_hil_gate.sh --host 10.0.0.171 --live --include-velocity --include-position
```

Current known state:

- `current_encoder` validation passes during the P8/P9 evidence runs.
- `velocity_encoder` validation fails scripted thresholds with both conservative
  default gains and trial gains. Treat this as a control tuning/open-loop risk,
  not as a non-motion refactor blocker.
- `position_encoder` should not be treated as stable until velocity validation
  passes repeatably.

## Evidence Retention

Keep these artifacts for each phase or PR-level validation:

- unit-test summary from `tests/run_unit_tests.sh`.
- firmware build summary including memory usage.
- HIL JSON reports from `scripts/hil/hil_telnet.py` or `run_hil_gate.sh`.
- HIL raw logs under `hil_logs/<phase>/` or `hil_logs/regression/<timestamp>/`.

Do not commit `hil_logs/` unless explicitly requested; reference paths in the
progress log instead.

## Failure Handling

If a required gate fails:

1. Stop and capture the log path.
2. Return the motor to IDLE/disarmed state if live motion was involved.
3. Record whether the failure is a regression or a known open risk.
4. Do not mark the phase complete until the required gate passes or the progress
   log explicitly documents why the failure is outside that phase.

## Agent Commands

Common sequence for implementation phases:

```bash
scripts/checks/run_non_hil_gate.sh wonderful_goldberg
podman exec wonderful_goldberg bash -lc \
  'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
scripts/hil/run_hil_gate.sh --host 10.0.0.171
```

Add `--live` only when the phase changes runtime motor behavior and hardware is
available.
