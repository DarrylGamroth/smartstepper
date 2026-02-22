# SpinTAC Gap Plan

This captures the agreed roadmap to reach SpinTAC-equivalent behavior and then exceed it.

## Equivalence Gaps (1-6)

1. Add a true `position_convert` module in `motor_core`.
2. Build a one-command identify+tune flow (`motor commission auto`) that runs capture, validates fit quality, computes PI/MPR/DOB defaults, and stages apply.
3. Add persistence for commissioned/tuned values (schema versioning + rollback-safe defaults).
4. Formalize mode transitions and fallback policy (MPR↔PI, encoder quality degradation, DOB fallback).
5. Tighten motion semantics parity (replan while running, bounded stop, blend guarantees, abort/hold guarantees).
6. Add HIL verification (timing/fault injection), beyond unit tests.

## Beyond SpinTAC

1. Adaptive scheduling of controller and DOB gains versus speed/load/temperature.
2. External trigger phase-lock mode for chopper/camera synchronization.
3. Confidence-weighted estimator fusion (encoder health + model residuals).
4. Commissioning uncertainty estimation with confidence-gated apply.

## Execution Order

1. `position_convert`
2. transition/fallback policy
3. motion semantics parity
4. commission auto
5. persistence
6. HIL verification and release gating

## Acceptance Gate

- Firmware build passes.
- Full unit test pass.
- No regression in current shell workflow.
- Operator command workflow documented for each new feature.
