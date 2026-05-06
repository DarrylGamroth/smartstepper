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

