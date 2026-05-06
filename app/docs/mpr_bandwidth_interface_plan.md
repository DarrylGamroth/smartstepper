# MPR Bandwidth Interface Plan

Date: 2026-05-05

## Goal

Provide a SpinTAC-style bandwidth interface for MPR tuning while retaining raw
MPR parameters for engineering diagnostics.

## Non-Goals

- Do not remove raw MPR commands.
- Do not retune current PI controllers.
- Do not change DOB or detent behavior.
- Do not add persistence.

## Current Problem

The current user-facing MPR interface exposes implementation parameters and
presets:

- `q_speed`,
- `r_delta_iq`,
- horizon,
- `max_delta_iq`.

This is useful for debugging, but it is not a good operator interface. A motion
controller should generally expose bandwidth and bounded aggressiveness.

## Design

Add these operator commands:

```text
motor velocity mpr bandwidth <hz>
motor position mpr bandwidth <hz>
```

The commands compute bounded MPR parameters from:

- commissioned inertia `J`,
- torque constant `Kt`,
- active current limit,
- loop sample time,
- configured horizon,
- conservative safety bounds.

Keep these engineering commands:

```text
motor velocity mpr set ...
motor position mpr set ...
motor velocity mpr preset ...
```

Presets remain bring-up shortcuts; bandwidth is the preferred tuning interface.

## Implementation Phases

Phase 1: Define bandwidth-to-MPR mapping.

- Use commissioned model values when valid.
- Fall back to conservative defaults when model confidence is low.
- Clamp bandwidth to safe min/max.
- Clamp `dIq` and horizon to HIL-proven bounds.

Phase 2: Add shell commands.

- Implement `motor velocity mpr bandwidth <hz>`.
- Implement `motor position mpr bandwidth <hz>`.
- Update status to print requested bandwidth and derived raw parameters.

Phase 3: Unit tests.

- Test low, nominal, and high bandwidth mapping.
- Test invalid model fallback.
- Test clamps.

Phase 4: HIL sweep.

- Compare PI and MPR at fixed bandwidths and velocities.
- Preserve PI as fallback.

## Acceptance Criteria

- Operator can tune velocity and position MPR with bandwidth commands.
- Raw MPR parameters remain visible in status.
- Invalid bandwidth values are rejected or clamped with clear shell output.
- HIL proves at least one bandwidth setting tracks safely in both directions.

## HIL Evidence

```text
motor commission run slow apply
motor outer mode mpr
motor velocity mpr bandwidth 0.5
motor commission validate velocity 0.05 1000 active
motor velocity mpr bandwidth 1.0
motor commission validate velocity 0.05 1000 active
```

Expected:

- no fault,
- no encoder errors,
- bounded `Iq_ref`,
- MPR tracking comparable to or better than conservative preset behavior.

## Risks

- A single bandwidth number can hide important constraints if clamps are not
  visible.
- Model-derived tuning can be unsafe when commissioned `J` or `Kt` is wrong.
- Position MPR may require different bandwidth limits than velocity MPR.

## Done State

- Commands exist exactly as:
  - `motor velocity mpr bandwidth <hz>`
  - `motor position mpr bandwidth <hz>`
- Unit tests cover mapping and clamps.
- HIL evidence documents safe bandwidth values.

