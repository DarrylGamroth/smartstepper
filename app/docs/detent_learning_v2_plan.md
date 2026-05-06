# Detent Learning V2 Plan

Date: 2026-05-05

## Goal

Make detent feedforward learning repeatable and useful by learning only
position-periodic current/torque and validating the result before enabling it.

## Non-Goals

- Do not make detent feedforward default-on.
- Do not tune MPR or DOB in this plan.
- Do not add persistence.
- Do not redesign encoder qualification; consume the centralized observer
  trust contract when available.

## Current Problem

The current detent capture can produce structurally complete maps, but HIL
showed a map that passed structural gates while making `0.05 Hz` ripple slightly
worse. The capture is also sensitive to low-speed velocity regulation quality.

## Design

Use a controlled forward/reverse learning run:

- run at moderate speed, not ultra-slow speed,
- integrate over more cycles,
- accumulate forward and reverse bins separately,
- subtract model terms when available,
- average direction pairs to suppress friction/load terms,
- validate off/on behavior before recommending apply.

Recommended starting capture:

```text
motor commission detent run 0.10 10 1 0.12
```

If stable, also try:

```text
motor commission detent run 0.20 10 1 0.12
```

## Implementation Phases

Phase 1: Capture workflow refinement.

- Make default recommended speed/cycles explicit in command help/docs.
- Add warnings when capture speed is likely too low for reliable learning.
- Keep DOB and detent disabled during capture.

Phase 2: Integration quality.

- Use centralized observer trust state.
- Report accepted/rejected samples as accepted/total ratio.
- Report forward/reverse coverage and agreement.

Phase 3: Validation before apply.

- Keep `motor commission detent validate <mech_hz> <duration_ms>`.
- Add a clear recommendation:
  - `RECOMMEND_APPLY`,
  - `DO_NOT_APPLY`,
  - `INCONCLUSIVE`.

Phase 4: Optional TI-style online learner.

- Consider adding a runtime learner similar to TI `vib_comp`:
  `table[index] = alpha * table[index] + beta * iq_residual`.
- Keep it disabled unless explicitly commanded.
- Use centralized observer trust instead of per-module quality logic.

## Acceptance Criteria

- A detent map is not recommended unless validation improves or matches
  baseline.
- Capture at `0.10 Hz` with `10` cycles gives better coverage/agreement than
  `0.05 Hz` with `3` cycles.
- Encoder errors remain within thresholds.
- Detent feedforward can be enabled/disabled independently of PI, MPR, and DOB.

## HIL Evidence

```text
motor commission run slow apply
motor safety timeout 0
motor arm
motor commission detent clear
motor commission detent run 0.10 10 1 0.12
motor commission detent status
motor commission detent validate 0.10 3000
```

Expected:

- high bin coverage,
- reasonable forward/reverse agreement,
- validation reports `RECOMMEND_APPLY` or a clear reason not to apply.

## Risks

- Running too slowly learns friction/stiction instead of detent torque.
- Running too fast can learn inertia/phase lag.
- Velocity loop instability can dominate the learned table.

## Done State

- Command guidance reflects tested speed/cycle recommendations.
- HIL logs compare at least `0.05 Hz x 3` and `0.10 Hz x 10`.
- Apply recommendation is based on validation, not structural coverage alone.

## Implementation Evidence

Status: implemented with live HIL comparison deferred until the commissioning
baseline is stable enough for repeatable velocity-encoder operation.

Code changes:

- `motor commission detent run` now prints the recommended starting command
  when invoked incorrectly:
  - `motor commission detent run 0.10 10 1 0.12`
- Capture warns when speed is below `0.08 Hz` or cycles are below `10`, because
  those runs are more likely to learn friction/stiction instead of position-
  periodic detent torque.
- Status reports accepted/total ratio, forward/reverse/both-direction coverage,
  and the latest validation recommendation.
- Validation now reports one of:
  - `RECOMMEND_APPLY`
  - `DO_NOT_APPLY`
  - `INCONCLUSIVE`
- Enabled apply is blocked unless validation recommends apply. Operators can
  still apply the staged table disabled with `motor commission detent apply 0`
  for inspection/debugging.

Validation:

```text
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_detent_map.unit
```

Result:

```text
7 of 7 executed test cases passed
```

Firmware build:

```text
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

Result:

```text
zephyr/zephyr.elf linked successfully
```

HIL note:

- The required `0.05 Hz x 3` versus `0.10 Hz x 10` comparison remains open
  until velocity-encoder commissioning is stable enough to produce repeatable
  validation passes.
