# Detent Feedforward Plan

## Goal

Add a position-periodic feedforward term for hybrid stepper detent/cogging
torque. This is modeled after the useful part of TI `VIB_COMP`, but adapted to
this project architecture:

- the angle observer owns wrapping, offset, direction, and latency compensation,
- the compensation module only maps mechanical angle to an `Iq` feedforward term,
- learning/commissioning is separated from the ISR lookup path,
- runtime use is explicit, bounded, and disabled by default.

## Why This Is Cleaner Than TI `VIB_COMP`

TI `VIB_COMP` combines several concerns in one block:

- reconstructs mechanical angle from electrical angle,
- updates/learns the table from measured `Iq`,
- applies phase advance,
- optionally outputs compensation.

For this project that is the wrong ownership boundary. The angle observer already
provides the correct mechanical angle. The detent module should not duplicate
angle reconstruction or offset handling. It should be a small deterministic
lookup block:

```text
mechanical angle -> table index/interpolation -> bounded Iq feedforward
```

That makes the ISR cost predictable and keeps commissioning policy out of the
control step.

## Phase 1: Core Module

Create `motor/compensation/detent_map.[ch]`.

Runtime API:

- `motor_detent_map_init()`
- `motor_detent_map_reset()`
- `motor_detent_map_step_fast()`
- `motor_detent_map_clear()`
- `motor_detent_map_set_bin()`
- `motor_detent_map_learn_sample()`

Runtime behavior:

- disabled unless explicitly enabled,
- wraps mechanical angle to `[0, 2pi)`,
- supports linear interpolation,
- supports integer-bin phase advance,
- clamps output to `iq_ff_limit_a`,
- stores the last index and last feedforward for telemetry.

## Phase 2: Unit Tests

Add native unit tests for:

- disabled output,
- exact-bin lookup,
- interpolation,
- wraparound lookup,
- phase advance,
- output clamp,
- learning update.

## Phase 3: Runtime Wiring

Wire the module into `motor_outer_loop_runtime`.

Insertion point:

```text
velocity PI/MPR output
  + optional DOB
  + detent_map(theta_mech)
  -> Iq clamp
```

The module remains disabled by default until a commissioning command creates a
validated table.

## Phase 4: Commissioning Command

Later command:

```text
motor commission detent run <current_a> <mech_hz> <cycles>
motor commission detent status
motor commission detent apply
motor commission detent clear
```

Capture strategy:

- run slow generated motion in both directions,
- record mechanical angle and controller effort,
- subtract model feedforward terms (`J*alpha`, `B*omega`, `Tc*sign(omega)`),
- average forward/reverse bins to reduce viscous/friction bias,
- reject bins with encoder errors/glitches,
- smooth table,
- stage result before applying.

## Phase 5: Validation

Validation compares low-speed motion before/after compensation:

- lower velocity ripple,
- lower velocity PI/MPR effort ripple,
- no increase in encoder errors,
- no current spikes,
- no regressions in generated/open-loop modes.

