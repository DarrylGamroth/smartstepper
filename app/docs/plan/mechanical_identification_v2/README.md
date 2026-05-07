# Mechanical Identification V2 Plan Package

Date: 2026-05-07
Target hardware: `smartstepper_v2` with `configs/motor_mt6835_2a.overlay`
Primary motor type: 50 pole-pair hybrid stepper

## Objective

Replace the current one-shot 4-parameter mechanical fit with a staged,
physically constrained identification workflow that is robust enough to tune
PI/MPR/DOB without blindly applying low-confidence `J/B/Tc` estimates.

The current model is useful as a diagnostic:

```text
Kt * Iq = J * alpha + B * omega + Tc * sign(omega) + T0
```

But fitting `J`, `B`, `Tc`, and `T0` simultaneously from one closed-loop
velocity capture is poorly conditioned on this hardware. Hybrid steppers add
periodic detent torque, and acceleration is currently derived from noisy
encoder velocity. This package splits the problem into smaller, testable parts.

## Scope

- Add staged mechanical identification alongside the existing 4-parameter fit.
- Keep the existing fit as a diagnostic/fallback until V2 is proven on HIL.
- Estimate friction from steady-speed plateaus.
- Estimate inertia from transient acceleration segments after friction removal.
- Add explicit physical constraints and confidence gates.
- Treat detent torque as a separate angle-dependent disturbance, not as generic
  Coulomb friction.
- Improve acceleration estimation for commissioning data.
- Do not persist identified values yet.

## Non-Goals

- Replacing current electrical ID.
- Changing encoder transport.
- Changing normal ISR control behavior outside commissioning capture.
- Adding NVM/settings persistence.
- Tuning DOB/detent/MPR as part of this package, except to expose improved
  mechanical-model confidence to those later steps.

## Key Design Rules

- `B` must never be negative in an accepted result.
- `J` must be positive and plausible for the configured motor/load.
- `Tc` is Coulomb/running-friction magnitude, not holding torque.
- Holding torque should be inferred separately from `Kt * Imax`.
- Low confidence must prevent automatic application of measured `J/B/Tc`.
- Devicetree motor values remain the safe fallback.
- Every HIL run must record encoder error counters and sample rejection counts.

## Task Order

1. `T001_friction_plateau_fit.md` - Steady-speed friction estimator.
2. `T002_inertia_transient_fit.md` - Transient inertia estimator.
3. `T003_detent_separation.md` - Detent torque separation and map interface.
4. `T004_constraints_confidence_apply.md` - Physical constraints and apply gate.
5. `T005_acceleration_estimation.md` - Offline/windowed acceleration estimator.
6. `T006_shell_hil_workflow.md` - Shell commands, HIL scripts, and evidence.

## Validation Summary

Each task must include:

- Unit tests for new motor-core estimators.
- Firmware build with MT6835 overlay.
- HIL run when behavior touches live commissioning.
- Evidence in `execution_log.md` before marking the task complete.

## References

Current implementation:

- `modules/motor_core/src/estimation/mech_id_estimator.c`
- `modules/motor_core/src/runtime/commission_runtime.c`
- `app/src/shell/shell_commission_auto.c`

VESC reference:

- VESC has robust production electrical ID but does not appear to perform a
  direct firmware-side `J/B/Tc` mechanical fit like this package proposes.
- Relevant electrical-ID references remain useful for staged commissioning and
  confidence gating:
  - `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`
  - `mcpwm_foc_measure_resistance()`
  - `mcpwm_foc_measure_inductance()`
  - `mcpwm_foc_measure_res_ind()`
