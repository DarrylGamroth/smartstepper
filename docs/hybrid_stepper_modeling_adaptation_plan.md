# Hybrid Stepper Modeling Adaptation Plan

Date: 2026-02-22  
Scope: Adapt existing `motor_core` + app control integration for hybrid stepper motor physics and commissioning fidelity.

## Objective

Decide what must change (and what should stay) when running the current FOC/MPR/DOB stack on a hybrid stepper motor, then implement those changes with low risk and measurable validation gates.

## Status

- Phase 0: complete (control units/frame/sign contract documented)
  - Reference: `docs/motor_control_units_contract.md`
- Phase 1: complete (canonical active torque gain parameter wired through runtime/app/tuning paths)

## Current Baseline

The current control stack is generally valid for hybrid stepper use:

- Inner current-loop FOC (`motor_foc_voltage_pwm`)
- Position/velocity outer loops (PI or MPR)
- Velocity DOB
- Motion profile + sequence modules
- Commissioning + tuning flows

Primary risk is not architecture, but model parameterization and scaling assumptions.

## Key Gaps For Hybrid Stepper

1. Torque conversion uses a PMSM-style fixed form (`1.5 * pole_pairs * psi_f`) in multiple app paths.
2. Electrical angle mapping/sign sensitivity is high; small convention mistakes strongly degrade torque control.
3. Hybrid stepper torque ripple/detent effects are not represented in the current mechanical model.
4. Decoupling/feedforward assumptions may be too optimistic under saliency/saturation.
5. Tuning and commissioning defaults do not explicitly account for hybrid stepper resonances and model uncertainty.

## Phase Plan

## Phase 0: Contracts and Units Freeze

Goal: lock conventions before algorithm changes.

Tasks:

- Freeze/control-document conventions:
  - current scaling (A), angle (rad), speed (rad/s), torque (Nm)
  - mechanical/electrical angle mapping and sign convention
  - Clarke/Park normalization assumptions used by torque conversion
- Add a short “control units contract” section to motor docs.

Acceptance:

- Single documented convention used by app and `motor_core` public APIs.

## Phase 1: Torque Gain Refactor (Highest Impact)

Goal: remove hardcoded torque scaling dependency and make torque conversion explicit/calibratable.

Tasks:

- Add active runtime parameter: `torque_gain_nm_per_a_active` (or equivalent).
- Replace direct `1.5 * p * psi_f` usage in:
  - `app/src/motor_control_loop.c`
  - `app/src/motor_commission.c`
  - `app/src/shell_commands.c`
- Keep `psi_f` path as optional derived estimate, not the only torque path.
- Surface shell visibility/config for torque gain.
- Update commissioning/tuning outputs to consume explicit torque gain.

Acceptance:

- All torque-domain calculations use one canonical torque gain source.
- No direct hardcoded torque conversion remains in runtime control paths.

## Phase 2: Electrical Angle Mapping Verification

Goal: ensure encoder-to-electrical angle mapping is correct and robust.

Tasks:

- Add commissioning checks for:
  - encoder direction vs commanded q-axis torque sign
  - electrical angle offset correctness
  - pole-pair mapping consistency
- Add shell status output for mapping/validation results and confidence.

Acceptance:

- Automated pass/fail check confirms direction, offset, and pole mapping.

## Phase 3: Hybrid-Specific Mechanical Model Extension

Goal: improve model fit quality for outer-loop and commissioning.

Tasks:

- Extend mechanical model with optional ripple term(s):
  - detent/cogging periodic term map (table or low-order harmonics)
- Keep base model fallback:
  - inertia + viscous + Coulomb + offset
- Add runtime enable/disable and clear fallback behavior.

Acceptance:

- Commissioning fit residual decreases on hybrid stepper when ripple enabled.
- Control remains stable with ripple model disabled.

## Phase 4: Decoupling/Feedforward Hardening For Stepper Use

Goal: prevent over-aggressive decoupling causing current spikes/overcurrent.

Tasks:

- Add speed/load dependent clamps or scheduling for feedforward terms.
- Add explicit disable gates for uncertain model conditions (low confidence ID, stale sensor quality, etc.).
- Add shell-visible diagnostics for feedforward clamp activity.

Acceptance:

- No regression in current-loop stability during ALIGN/OFFLINE->ONLINE transitions.
- Overcurrent incidents attributable to decoupling are eliminated in validation scenarios.

## Phase 5: Commissioning/Tuning Updates

Goal: tune defaults and identification flow for hybrid stepper realities.

Tasks:

- Add direct torque-gain calibration path (in addition to flux-derived path).
- Cross-check torque gain from multiple methods and reject low-confidence fits.
- Reduce default outer-loop aggressiveness for resonance-prone operating regions.
- Stage/apply only when confidence gates pass.

Acceptance:

- `motor commission auto` outputs stable defaults on hybrid stepper without manual gain rescue.

## Phase 6: Verification and Release Gates

Goal: prevent regression and prove hybrid-stepper readiness.

Tasks:

- Unit tests:
  - torque gain canonical path
  - mapping validation logic
  - ripple model on/off behavior
- HIL tests:
  - startup and mode transitions
  - torque mode disturbance rejection
  - high-speed profile sequencing with encoder quality events

Acceptance:

- Unit + HIL gates pass with documented thresholds.

## Recommended Execution Order

1. Phase 0 (contracts)
2. Phase 1 (torque gain refactor)
3. Phase 2 (mapping verification)
4. Phase 4 (decoupling hardening)
5. Phase 5 (commissioning/tuning updates)
6. Phase 3 (ripple model extension)
7. Phase 6 (full verification gates)

Rationale: stabilize correctness and safety-critical scaling first, then add model complexity.

## Immediate Next Step

Implement Phase 1 first: introduce canonical torque gain parameter and remove hardcoded torque conversion usage from control/tuning/runtime paths.
