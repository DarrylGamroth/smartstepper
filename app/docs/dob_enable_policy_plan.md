# DOB Enable Policy Plan

Date: 2026-05-05

## Goal

Make disturbance-observer enablement deterministic, reversible, and safe after
the base velocity loop is already stable.

## Non-Goals

- Do not tune MPR in this plan.
- Do not change detent learning.
- Do not change encoder transport.
- Do not add persistence.

## Current Problem

DOB can be useful, but it can also mask model error, detent ripple, encoder
artifacts, or velocity-loop instability. It should not enable unless the system
is in a known-good state.

## Design

DOB readiness should require:

- commissioning complete,
- encoder mapping applied,
- observer output trusted,
- valid `Kt`, `J`, and current limits,
- no current motor fault,
- no recent encoder diagnostic burst,
- stable velocity-control mode.

DOB should reset on:

- enable/disable,
- mode entry/exit,
- zero target,
- target sign change,
- large target step,
- observer trust loss,
- transition to idle/error.

## Implementation Phases

Phase 1: Define readiness contract.

- Use centralized observer trust state when available.
- Keep a clear reason string for shell status.

Phase 2: Define reset hooks.

- Reset DOB state at deterministic state/mode boundaries.
- Reset on target discontinuities.

Phase 3: Low-speed behavior.

- Add low-speed deadband or clamp.
- Keep `iq_ff` inside configured bound.

Phase 4: HIL validation.

- Validate PI only, PI+DOB, MPR only, and MPR+DOB.

## Acceptance Criteria

- DOB cannot enable when readiness fails.
- Status explains why readiness fails.
- DOB enable/disable does not create a torque transient.
- DOB never prevents fallback to PI-only encoder velocity.

## HIL Evidence

```text
motor commission run slow apply
motor outer mode pi
motor velocity dob defaults safe
motor velocity dob status
motor velocity dob enable 1
motor commission validate velocity 0.05 1000 active
motor velocity dob enable 0
motor outer mode mpr
motor velocity dob enable 1
motor commission validate velocity 0.05 1000 active
```

Expected:

- no fault,
- DOB readiness `YES`,
- `iq_ff` bounded,
- no increase in overshoot or ripple compared with no-DOB baseline.

## Risks

- DOB can compensate wrong model values and make later tuning misleading.
- Enabling DOB at very low speed can fight friction/detent behavior.
- Reset behavior must not occur every tick.

## Done State

- Readiness and reset policy documented in code and shell status.
- Unit tests cover enable gating where practical.
- HIL evidence covers PI+DOB and MPR+DOB.

