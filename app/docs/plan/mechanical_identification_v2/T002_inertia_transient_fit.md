# T002 - Transient Inertia Fit

## Problem

Inertia should be identified from acceleration-rich data, not from a combined
fit dominated by steady friction/detent behavior.

## Objective

Estimate `J` after subtracting the friction model from T001:

```text
J * alpha ~= Kt * Iq - B * omega - Tc * sign(omega) - T0
```

## Implementation

- Add a motor-core transient inertia estimator.
- Use acceleration windows from commanded velocity ramps or bounded generated
  acceleration profiles.
- Accept only samples where:
  - `|alpha|` is above a configurable minimum,
  - encoder data is clean,
  - current/PWM is not saturated,
  - commanded and measured velocity signs are coherent,
  - sample is outside the low-speed detent/deadband region.
- Fit `J` as a one-parameter least-squares problem after friction subtraction.
- Capture positive and negative acceleration in both rotation directions.

## Acceptance Criteria

- `J > 0`.
- `J` is plausible relative to configured fallback inertia.
- Positive and negative acceleration estimates agree within a configured ratio.
- Residual RMS and confidence are reported.
- If confidence is low, result is staged only and not applied.

## Tests

- Unit test ideal transient data recovers known `J`.
- Unit test friction-subtracted data with nonzero `B/Tc/T0`.
- Unit test rejection of low-acceleration samples.
- Unit test rejection of sign-incoherent captures.

## HIL

- Run a bounded acceleration profile with current limit below commissioning max.
- Verify full-cycle motion is visible and sample coverage includes both signs.
- Compare result against configured NEMA17 fallback inertia as a sanity check.
