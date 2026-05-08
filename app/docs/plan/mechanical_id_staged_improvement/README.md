# Mechanical Identification Staged Improvement Plan

Date: 2026-05-07

## Problem

The MT6835 commissioning flow now passes the electrical and encoder-mapping
stages, but the mechanical identification stage rejects otherwise plausible
captures at the confidence gate. The latest HIL run produced clean encoder
transport counters and mechanical fit confidence around `0.48..0.49`, just
below the `0.50` acceptance threshold.

The immediate goal is not to relax quality gates. The first fix should improve
the excitation and diagnostics so the fit has enough information and failures
identify the weak component.

## Stage 1 - Low-Risk Capture Improvement

Status: in progress

- Increase the default mechanical capture window without exceeding the bounded
  commissioning sample buffer.
- Use a richer symmetric speed-dither pattern with stronger acceleration and
  deceleration content.
- Keep current limits, velocity limits, and acceptance thresholds unchanged.
- Print component-level mechanical fit diagnostics:
  - friction sample count and residual,
  - inertia sample count and residual,
  - inertia plausibility ratio,
  - confidence threshold and margin,
  - capture accept/reject counters.

Validation:

- Optimized MT6835 west build passes.
- Full unit tests pass or a focused build-only validation is recorded if no unit
  logic changes are made.
- HIL commissioning shows whether mechanical confidence improves without
  changing thresholds.

## Stage 2 - Configurable Quality Policy

Status: pending

- Move the mechanical confidence threshold and plausibility warning limits into
  the commissioning config path rather than hard-coding them in runtime code.
- Keep production defaults strict, but allow controlled HIL experiments via
  overlay or shell without recompiling motor_core.
- Ensure output clearly differentiates `hard invalid`, `plausibility warning`,
  and `confidence rejected`.

## Stage 3 - Fit Robustness

Status: pending

- Review the friction and inertia estimators for sensitivity to low-speed
  hybrid-stepper detent torque.
- Prefer physically constrained results:
  - inertia must be positive,
  - viscous friction must be non-negative,
  - Coulomb friction must be non-negative.
- Investigate whether detent-feedforward-compensated samples should be used for
  the production mechanical fit after detent map commissioning is reliable.

## Stage 4 - HIL Repeatability Gate

Status: pending

- Run at least three full MT6835 commissioning attempts from a clean boot.
- Record:
  - electrical `Rs`, `Ld`, `Lq`,
  - encoder mapping confidence and counters,
  - mechanical run confidence values,
  - aggregate `J`, `B`, `Tc`,
  - final state and fault status.
- Only promote mechanical-ID settings to baseline when repeatability is better
  than accepting a single marginal run.

## Non-Goals

- Do not persist mechanical parameters yet.
- Do not loosen thresholds as the first response to a marginal failure.
- Do not reintroduce legacy `RS_EST` into the normal commissioning flow.

