# Production Electrical Identification Plan

This plan adds a production-oriented electrical identification path alongside the existing Rs/R-over-L estimator.

## Goal

Measure electrical parameters accurately enough for production current-loop tuning:

- `Rs`
- `Lavg`
- `Ld`
- `Lq`
- `Lq-Ld`
- recommended current PI gains
- validation metrics

## Existing Path

The current Rs/R-over-L estimator remains available as a fallback/bootstrap path. It is useful when the production pulse/HFI path is unavailable or rejected, but it should not be the long-term source of production tuning values unless validated.

## Reference

VESC reference implementation:

- `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`: `mcpwm_foc_measure_resistance()`
- `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`: `mcpwm_foc_measure_inductance()`
- `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`: `mcpwm_foc_measure_res_ind()`
- `/home/dgamroth/workspaces/motor/bldc/motor/foc_math.c`: saliency use via `foc_motor_ld_lq_diff`

## Design Rules

- Do not remove the existing Rs/R-over-L implementation.
- Add the new production electrical measurement as an explicit command/path.
- Use vendor/devicetree nominal values for safe startup only.
- Do not automatically promote measured values until validation passes.
- Keep all high-rate sampling/accumulation ISR-safe and bounded.
- Keep shell commands orchestration-only; measurement math belongs in `motor_core`.
- Treat `Ld/Lq` as optional but valuable for hybrid steppers.
- Persist nothing in this plan; persistence comes later.

## Task Order

1. `T001_requirements.md` - Define measurement requirements, safety limits, and outputs.
2. `T002_core_api.md` - Add motor-core production electrical ID API and result structs.
3. `T003_rs_measure.md` - Add production Rs locked-rotor measurement alongside existing Rs estimator.
4. `T004_inductance_pulse.md` - Add pulse/HFI-inspired inductance measurement for `Lavg` and `Lq-Ld`.
5. `T005_ld_lq_derivation.md` - Derive, validate, and report `Ld/Lq` with repeatability metrics.
6. `T006_current_pi_recommend.md` - Generate recommended current PI gains from measured electrical parameters.
7. `T007_step_validation.md` - Validate current-loop step response and revert/reject on failure.
8. `T008_shell_hil.md` - Add shell commands and HIL scripts for repeatable production electrical ID.
9. `T009_integration_policy.md` - Define promotion/fallback rules for existing vs production electrical ID.

## Validation

- Unit tests for estimator math, plausibility gates, and PI recommendation.
- Firmware build for MT6835 and AEAT overlays.
- HIL command script that reports pass/fail without persistence.
- Comparison report against existing Rs/R-over-L path.
