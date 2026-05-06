# HIL Motion Regression Plan

Date: 2026-05-05

## Goal

Create repeatable HIL evidence for commissioning and motion modes before tuning
advanced features further.

## Non-Goals

- Do not change control algorithms in this plan.
- Do not tune MPR, DOB, detent, or PI gains unless the test itself requires a
  documented parameter.
- Do not add persistence.

## Current Problem

Manual shell testing has found real issues, but it is hard to compare behavior
across commits without repeatable scripted runs and machine-readable output.

## Design

Use `scripts/hil/hil_telnet.py` as the primary HIL runner. Each scenario should:

- require `--yes-live-motion` for motor movement,
- start from a safe idle state,
- run one validation layer,
- stop motion and return to idle,
- write a JSON report,
- fail nonzero on fault or inconclusive result.

## Implementation Phases

Phase 1: Baseline scenarios.

- status,
- boot commissioning,
- current encoder validation,
- velocity encoder validation,
- position encoder validation,
- open-loop encoder trace.

Phase 2: Advanced feature scenarios.

- detent capture/validate,
- MPR bandwidth sweep,
- DOB enable validation,
- feature-combination matrix.

Phase 3: Report quality.

- Standardize metrics:
  - encoder errors,
  - motor error,
  - velocity error,
  - `Iq_ref` peak/RMS,
  - pass/fail verdict,
  - log path.

## Acceptance Criteria

- Each scenario can be run from a fresh boot.
- Each motion scenario ends in safe idle.
- JSON report exists for every HIL run.
- Generated/open-loop regression is kept separate from encoder-control tests.

## HIL Evidence

Baseline command set:

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.44 \
  --json-report hil_logs/status.json

python3 scripts/hil/hil_telnet.py encoder-validate --host 10.0.0.44 \
  --yes-live-motion \
  --json-report hil_logs/encoder_validate.json
```

Advanced command set:

```bash
python3 scripts/hil/hil_telnet.py mpr-dob-detent --host 10.0.0.44 \
  --yes-live-motion \
  --json-report hil_logs/mpr_dob_detent.json
```

## Risks

- HIL tests can be invalidated by hardware wiring, power faults, or motor
  changes.
- Telnet DHCP addresses can change.
- Log volume can perturb shell timing if debug logging is enabled.

## Done State

- HIL scenarios are documented.
- Baseline MT6835 HIL run passes.
- Known AEAT-9955 limitations are documented separately.

## Implementation Evidence

Status: HIL workflow implemented; current hardware/control baseline is not yet
fully passing.

Implemented:

- `scripts/hil/hil_telnet.py` now validates `encoder-trace-open-loop` motion
  instead of allowing a no-motion trace to pass.
- Open-loop trace validation parses stored samples, raw/control delta, clean
  sample count, warning/error/io counts, and delta-drop count.
- Status, boot commissioning, and generated trace scenarios now clear stale
  error/fault-snapshot state before running so each scenario is judged
  independently.
- `boot-commission` now returns the controller to safe idle at the end.
- `scripts/hil/run_hil_gate.sh` now defaults to MT6835 telnet host
  `10.0.0.44`, writes per-scenario JSON reports, writes `summary.json`, supports
  `--keep-going`, and keeps generated/open-loop trace before encoder-control
  validation.
- Added `--skip-current` so generated/open-loop regression can be proven
  independently while current-encoder issues are under investigation.
- Updated `scripts/hil/README.md` with the current gate commands and
  generated-vs-encoder-control separation.

Validation:

```bash
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
```

Result: PASS, 11/11 tests.

```bash
bash -n scripts/hil/run_hil_gate.sh
```

Result: PASS.

```bash
podman exec wonderful_goldberg bash -lc \
  'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

Result: PASS, no firmware rebuild required for script-only changes.

Status-only HIL gate:

```bash
scripts/hil/run_hil_gate.sh \
  --host 10.0.0.44 \
  --log-root hil_logs/control_plan/hil_motion_regression_status
```

Result: PASS.

Report directory:

- `hil_logs/control_plan/hil_motion_regression_status/20260506_002028`

Live HIL findings:

```bash
scripts/hil/run_hil_gate.sh \
  --host 10.0.0.44 \
  --log-root hil_logs/control_plan/hil_motion_regression \
  --live \
  --keep-going
```

Result: FAIL, but produced useful independent evidence:

- `status`: PASS
- `boot-commission`: PASS after scenario-safe-idle fix
- `current-validate`: FAIL with insufficient clean samples / no current motion,
  while motor error and acquisition counters remained clean
- `encoder-trace-open-loop`: FAIL in one keep-going run because preceding
  current-validation failure left acquisition unavailable; this drove the gate
  ordering change to run generated trace before current validation

Report directory:

- `hil_logs/control_plan/hil_motion_regression/20260506_002154`

Safe generated/live gate:

```bash
scripts/hil/run_hil_gate.sh \
  --host 10.0.0.44 \
  --log-root hil_logs/control_plan/hil_motion_regression_safe \
  --live \
  --skip-current
```

Result: FAIL on a later run because boot commissioning reported
`Control ISR is not advancing in velocity_generated mode (err -116)` and current
offsets were zero. This is treated as a real HIL/system issue, not a HIL runner
problem.

Report directory:

- `hil_logs/control_plan/hil_motion_regression_safe/20260506_002828`

Caveats and follow-up:

- Current encoder validation is not a stable baseline yet. The gate now exposes
  this instead of hiding it.
- A separate system/control investigation is needed for the intermittent
  "control ISR is not advancing" failure seen during boot commissioning.
- Generated/open-loop trace validation is now strong enough to fail when no
  samples or no motion are captured.
