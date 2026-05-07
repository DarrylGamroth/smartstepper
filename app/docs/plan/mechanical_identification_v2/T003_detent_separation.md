# T003 - Detent Torque Separation

## Problem

Hybrid stepper detent torque is angle-periodic. Treating it as Coulomb friction
forces a periodic disturbance into `Tc`, `B`, or `J`, corrupting the mechanical
fit.

## Objective

Keep detent torque separate from generic friction and inertia identification.

## Implementation

- Define a commissioning-time detent model interface in motor-core.
- Use the existing detent feedforward map work where possible.
- During friction/inertia identification:
  - either disable detent compensation and mark the fit as raw,
  - or subtract a previously measured detent map and mark the fit as corrected.
- Record whether `J/B/Tc` were estimated with detent correction enabled.

## Acceptance Criteria

- Mechanical model metadata states detent correction source: none, staged, or active.
- Friction and inertia estimators can consume a detent torque estimate per sample.
- Detent map errors do not silently improve confidence; residuals remain visible.

## Tests

- Unit test periodic detent contamination increases residual when not subtracted.
- Unit test detent-subtracted data improves residual without changing true `J`.
- Unit test missing detent map leaves estimator behavior unchanged.

## HIL

- Run friction/inertia identification with detent correction disabled first.
- Run again with detent map enabled after detent commissioning is stable.
- Compare residual RMS and repeatability, not just pass/fail.
