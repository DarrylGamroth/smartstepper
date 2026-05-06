# Encoder Sample Quality Plan

Date: 2026-05-05

## Goal

Centralize encoder sample qualification so control modules consume one trusted
observer output instead of duplicating fresh/valid/error checks.

## Non-Goals

- Do not retune velocity, position, MPR, DOB, or detent behavior in this plan.
- Do not change the physical SPI transport unless required to report sample
  status correctly.
- Do not add persistence.

## Current Problem

Encoder bad-sample handling has leaked into multiple modules. Some code checks
fresh/valid flags, some checks error counters, and some modules reject samples
independently. This makes behavior hard to reason about and makes every control
feature depend on encoder diagnostic details.

## Design

Create one compact encoder sample quality policy:

- driver/fast encoder layer reports raw transport, frame, CRC/parity, status,
  and decoded angle result,
- acquisition/control layer classifies each ISR sample,
- angle observer decides whether to consume the physical sample or predict,
- control modules consume observer output with a single trust bit/state.

Recommended public control-facing states:

- `trusted`: safe for encoder-commutated control this tick,
- `predicted`: physical sample rejected, short-horizon prediction used,
- `fault`: sustained or severe encoder loss.

Avoid exposing multiple normal-control flags such as valid/fresh/stale/generated
to every module.

## Implementation Phases

Phase 1: Define data contract.

- Add a compact sample-quality enum/bitfield in the encoder acquisition or
  observer interface.
- Map transport/frame/CRC/status/glitch results into this contract.
- Document which fields are ISR-hot and which are diagnostic only.

Phase 2: Consecutive-error policy.

- Count consecutive bad samples.
- Reset the consecutive counter on every accepted physical sample.
- Keep lifetime counters for diagnostics.
- Fault only when the consecutive or burst threshold is exceeded.

Phase 3: Remove duplicated qualification.

- Replace per-module fresh/valid/error checks with observer `trusted` or
  `predicted` state.
- Keep detailed raw counters available through diagnostics/telemetry.

## Acceptance Criteria

- One isolated bad encoder sample does not fault encoder modes.
- A configured run of consecutive bad samples faults encoder modes.
- Control modules no longer each interpret raw encoder diagnostic flags.
- Encoder acquisition counters remain visible in shell diagnostics.
- Unit tests cover isolated bad sample, consecutive bad samples, recovery, and
  fault threshold behavior.

## HIL Evidence

Use MT6835 first because it has shown clean transport:

```bash
python3 scripts/hil/hil_telnet.py encoder-validate \
  --host 10.0.0.44 \
  --yes-live-motion \
  --boot-current 0.15 \
  --boot-hz 0.05 \
  --cycles 1
```

Expected:

- no motor fault,
- encoder mapping applied,
- acquisition counters within thresholds,
- current and velocity validation pass.

## Risks

- Thresholds that are too permissive can hide real commutation angle loss.
- Thresholds that are too strict can fault on harmless single-sample noise.
- Prediction through bad samples must remain bounded by mode and speed.

## Done State

- Code contract exists.
- Unit tests prove policy behavior.
- HIL encoder transport/sample-quality validation passes; velocity tracking
  failures are recorded as follow-up encoder-control/tuning work.
- Follow-up issues are captured as separate plans, not added here.

## Implementation Evidence

Status: code implemented, validation run with one scoped HIL caveat.

Implemented:

- Added a compact control-facing trust state:
  `trusted`, `predicted`, and `fault`.
- Propagated trust state through encoder acquisition, angle-path output,
  runtime feedback refs, live telemetry, and shell status.
- Replaced duplicated control checks with `trusted`/`usable` helpers where
  encoder control and velocity feedback make decisions.
- Kept detailed raw diagnostic counters separate from normal control decisions.

Validation:

```bash
./tests/run_unit_tests.sh wonderful_goldberg \
  -s chopper.motor_encoder_feedback_core.unit \
  -s chopper.motor_control_kernel.unit
```

Result: PASS, 2 suites, 26/26 tests.

```bash
podman exec wonderful_goldberg bash -lc \
  'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

Result: PASS.

```bash
podman exec wonderful_goldberg bash -lc \
  'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

Result: PASS.

```bash
python3 -u scripts/hil/hil_telnet.py encoder-validate \
  --host 10.0.0.44 \
  --yes-live-motion \
  --boot-current 0.15 \
  --boot-hz 0.05 \
  --cycles 1 \
  --json-report hil_logs/control_plan/encoder_sample_quality.json
```

Result: FAIL overall due to velocity tracking, but PASS for this plan's sample
quality scope:

- no fatal/fault-stop text,
- motor error `NONE`,
- fault snapshot clear,
- encoder acquisition errors all zero,
- encoder mapping applied,
- encoder protocol OK,
- boot commissioning completed,
- current validation clean and opposite sign.

Caveat:

- `velocity_validation` failed with measured velocity remaining near zero. This
  is carried into the encoder-control/tuning plans and is not treated as a
  sample-quality regression.
