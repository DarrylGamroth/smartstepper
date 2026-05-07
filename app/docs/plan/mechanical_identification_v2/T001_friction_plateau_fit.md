# T001 - Steady-Speed Friction Plateau Fit

## Problem

The current 4-parameter mechanical fit tries to estimate viscous friction,
Coulomb friction, torque bias, and inertia in one solve. This couples `B` and
`J` through noisy acceleration estimates and can produce non-physical results.

## Objective

Estimate `B`, `Tc`, and `T0` using only near-steady-speed samples where
acceleration is small:

```text
Kt * Iq ~= B * omega + Tc * sign(omega) + T0
```

## Implementation

- Add a motor-core estimator for friction plateau fitting.
- Input samples:
  - measured mechanical speed `omega`,
  - measured current `Iq`,
  - active `Kt`,
  - optional sample quality flags.
- Accept only samples where:
  - encoder sample is valid,
  - PWM/current is not saturated,
  - `|omega|` is above a configurable deadband,
  - `|alpha|` is below a configurable steady-state threshold,
  - velocity tracking error is within commissioning limits.
- Fit `[B, Tc, T0]` with least squares.
- Run positive and negative speed plateaus so `Tc` and `T0` are observable.

## Acceptance Criteria

- `B >= 0`.
- `Tc >= 0`.
- Fit has enough positive and negative speed samples.
- Residual RMS is below a configurable threshold.
- Result exposes confidence and rejection counters.

## Tests

- Unit test ideal positive/negative plateau data.
- Unit test rejection of negative `B`.
- Unit test bias separation with asymmetric torque offsets.
- Unit test insufficient positive/negative coverage.

## HIL

- Add or extend a command to run a speed plateau sequence.
- Record per-plateau average `omega`, `Iq`, residual, and accepted/rejected samples.
- Do not apply the result automatically until T004.
