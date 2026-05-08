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
```

Velocity MPR presets were removed from the public shell interface. Use
bandwidth for operator-facing tuning and raw `set` only for engineering
diagnosis.

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

## Implementation Evidence

Status: implemented with HIL execution deferred until the current HIL baseline
is stable again.

Code changes:

- Added `motor_mpr_velocity_config_from_bandwidth()` and
  `motor_mpr_position_config_from_bandwidth()` in `motor_core`.
- Moved bandwidth clamp constants out of `app/src/shell_commands.c` and into
  `modules/motor_core/include/motor/control/mpr.h`.
- Updated shell commands to call the `motor_core` mapping helpers:
  - `motor velocity mpr bandwidth <hz>`
  - `motor position mpr bandwidth <hz>`
- Velocity MPR bandwidth now uses commissioned `J`/`Kt` when valid and falls
  back to conservative limits when the active model is not valid.
- MPR status prints an estimated bandwidth derived from the active raw tuning.

Validation:

```text
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_mpr.unit
```

Result:

```text
18 of 18 executed test cases passed
```

Firmware build:

```text
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

Result:

```text
zephyr/zephyr.elf linked successfully
```

HIL update, 2026-05-08:

- The MT6835 PI baseline is now validated at `10 Hz`, `zeta=1.0`,
  `Iq=0.225 A`.
- The first MPR bandwidth mapping produced identical effective parameters over
  the useful range because the real motor model clamped `q_speed` to
  `MOTOR_MPR_VELOCITY_BW_Q_MIN`.
- The mapper was updated so requested bandwidth produces distinct `q_speed`,
  `r_delta_iq`, and `dIq` values on the MT6835 model.
- Focused unit validation passed:

```text
./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_mpr.unit
20 of 20 executed test cases passed
```

- Firmware build passed:

```text
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2_mt6835_id'
```

- HIL did not pass the PI reference. Representative logs:
  - `hil_logs/20260508_125647_velocity-sweep.log`: MPR `5 Hz` with lower
    `r_delta_iq` still failed `+/-0.5` and `+/-1 Hz`.
  - `hil_logs/20260508_130505_velocity-sweep.log`: MPR `10 Hz` still failed
    the same points.
  - `hil_logs/20260508_131013_velocity-sweep.log`: extending the horizon to
    `32` ISR ticks made behavior worse, so the default horizon was restored to
    `8`.
  - `hil_logs/20260508_131405_custom.log`: raw aggressive tuning
    `q=0.500`, `r=0.010`, `horizon=8`, `dIq=0.010 A/sample` still averaged
    only about `0.325 Hz` at a `0.5 Hz` command.

Current conclusion:

- The bandwidth interface is now more meaningful than before, but MPR is not
  validated for control use.
- The remaining issue is likely algorithm-level: missing disturbance/feedforward
  authority, static-friction/detent handling, or an outer-loop contract mismatch.
- Keep PI as the validated baseline and do not persist or recommend MPR until a
  new MPR design pass passes HIL.
