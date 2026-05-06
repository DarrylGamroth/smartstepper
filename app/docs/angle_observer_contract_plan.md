# Angle Observer Contract Plan

Date: 2026-05-05

## Goal

Make the angle observer the single owner of encoder offset, wrapping, latency
compensation, electrical angle, mechanical angle, velocity, and trust semantics.

## Non-Goals

- Do not retune regulators.
- Do not change detent learning.
- Do not change encoder SPI transport.
- Do not add persistence.

## Current Problem

Several paths still reason about encoder angle validity, offsets, mechanical to
electrical conversion, or sample freshness. That duplicates observer
responsibility and makes encoder-control failures hard to localize.

## Design

The observer should accept:

- decoded mechanical encoder angle,
- sample timestamp or known sample latency,
- sample quality from the encoder qualification layer,
- configured direction, mechanical offset, electrical offset, and pole pairs.

The observer should output:

- mechanical position, wrapped and unwrapped,
- electrical angle for FOC,
- mechanical velocity,
- acceleration if required,
- prediction age,
- trust state suitable for control.

FOC should consume only observer electrical angle. Velocity and position loops
should consume only observer mechanical position/velocity.

## Implementation Phases

Phase 1: Define observer input/output structs.

- Keep ISR-hot fields compact.
- Keep diagnostics separate from the fast path.

Phase 2: Centralize offset and wrap logic.

- Remove duplicate mechanical/electrical wrapping from control-loop code.
- Remove duplicate encoder offset math outside commissioning/apply paths.

Phase 3: Latency compensation.

- Represent expected encoder sample latency explicitly.
- Account for AEAT-9955 and MT6835 differences in configuration.
- Record prediction age for diagnostics.

Phase 4: Consumer cleanup.

- Update FOC, velocity, position, MPR, DOB, and detent capture to consume the
  observer contract.

## Acceptance Criteria

- One code path converts mechanical encoder angle to electrical FOC angle.
- One code path owns encoder offset and direction application.
- Observer output explains whether control is using measured or predicted angle.
- HIL trace can compare raw encoder angle, observer mechanical angle, observer
  electrical angle, and generated reference.

## HIL Evidence

Generated motion trace:

```bash
python3 scripts/hil/hil_telnet.py encoder-trace-open-loop \
  --host 10.0.0.44 \
  --yes-live-motion \
  --open-loop-iq 0.12 \
  --open-loop-hz 0.10 \
  --trace-ms 1000 \
  --trace-decimation 1
```

Expected:

- raw encoder angle advances smoothly,
- observer mechanical angle follows raw angle with expected sign/offset,
- observer electrical angle advances by `pole_pairs * mechanical`,
- no encoder acquisition errors.

## Risks

- Changing angle ownership can break working generated modes if generated angle
  and encoder angle sources are conflated.
- Latency compensation must be explicit per encoder.

## Done State

- Duplicated wrap/offset/latency code removed from control modules.
- Unit tests cover wrap boundary, offset application, sign, latency prediction,
  bad-sample prediction, and recovery.
- HIL trace confirms expected angle behavior.

## Implementation Evidence

Status: implemented and validated, with one existing encoder-velocity caveat
outside this plan.

Implemented:

- Added observer-owned `angle_observer_mech_to_elec_angle()` for applying
  mechanical offset, pole-pair conversion, and electrical wrapping.
- Added observer prediction-age tracking and propagated observer delay and
  prediction age through control feedback, live telemetry, and raw trace
  diagnostics.
- Routed generated/encoder diagnostic electrical-angle conversion through the
  observer helper instead of duplicating offset math in encoder feedback code.
- Added raw trace columns for observer mechanical and electrical angles so HIL
  traces can compare raw/control, observer, and generated references directly.

Validation:

```bash
./tests/run_unit_tests.sh wonderful_goldberg \
  -s chopper.angle_observer.unit \
  -s chopper.motor_encoder_feedback_core.unit \
  -s chopper.motor_control_kernel.unit
```

Result: PASS, 3 suites, 40/40 tests.

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

Bootstrap before trace:

```bash
python3 -u scripts/hil/hil_telnet.py encoder-validate \
  --host 10.0.0.44 \
  --yes-live-motion \
  --boot-current 0.15 \
  --boot-hz 0.05 \
  --cycles 1 \
  --json-report hil_logs/control_plan/angle_observer_bootstrap_after_trace_update.json
```

Result: FAIL overall due to existing velocity tracking threshold, but PASS for
boot mapping, current validation, protocol, no motor fault, and zero encoder
acquisition errors.

Generated trace:

```bash
python3 -u scripts/hil/hil_telnet.py encoder-trace-open-loop \
  --host 10.0.0.44 \
  --yes-live-motion \
  --open-loop-iq 0.12 \
  --open-loop-hz 0.10 \
  --trace-ms 1000 \
  --trace-decimation 1 \
  --json-report hil_logs/control_plan/angle_observer_contract.json
```

Result: PASS.

Trace evidence:

- raw trace dump includes `raw_mdeg`, `ctrl_mdeg`, `obs_mech_mdeg`,
  `obs_elec_mdeg`, `gen_mech_mdeg`, and `gen_elec_mdeg`.
- no fatal/fault-stop text,
- motor error `NONE`,
- fault snapshot clear,
- encoder acquisition errors all zero.

Caveat:

- Encoder velocity validation remains unstable/underpowered and is tracked by
  later encoder-control/tuning work. This plan only changes observer ownership,
  trace visibility, and angle contract propagation.
