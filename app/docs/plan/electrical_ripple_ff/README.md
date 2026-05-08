# Electrical Ripple Feedforward Plan

Date: 2026-05-08

## Goal

Replace the mechanical-revolution detent-map learning experiment with a hybrid-stepper-appropriate electrical-angle ripple feedforward path.

The compensation target is construction-driven torque ripple that repeats with electrical angle. Mechanical-load disturbances remain the responsibility of DOB, future TI-style `vib_comp`, or later adaptive/repetitive compensation.

## Design

Runtime command composition:

```text
Iq_ref = regulator_iq
       + electrical_ripple_ff(theta_e)
       + optional mechanical_detent_ff(theta_m)
       + optional DOB_ff
```

The initial implementation uses a compact table indexed by electrical angle:

- default table: 64 bins per electrical revolution,
- forward/reverse capture accumulation,
- mechanical model subtraction when active `J/B/Tc/Kt` are available,
- zero-mean normalization before staging,
- validation before enabled apply,
- disabled by default.

## Non-Goals

- Do not persist the table yet.
- Do not integrate into `motor commission run` yet.
- Do not implement harmonic fitting or LMS in this phase.
- Do not remove the existing mechanical detent map yet.

## Phases

### Phase 1 - Core Module

- Add `motor/compensation/electrical_ripple_ff` to motor_core.
- Provide table init/reset/clear/set/lookup/step/mean/remove-mean helpers.
- Unit-test wrap, interpolation, phase advance, clamping, disabled output, and mean removal.

### Phase 2 - Runtime Hook

- Add runtime config/state/table to `motor_parameters`.
- Add live shell-visible `electrical_ripple_iq_ff_a`.
- Feed electrical angle from the observer/feedback path into outer-loop runtime.
- Apply electrical ripple FF only when enabled and feedback is trusted.

### Phase 3 - Commissioning Commands

Add shell commands:

```text
motor commission ripple run <velocity_hz> <duration_ms> [decimation] [iq_limit_a]
motor commission ripple status
motor commission ripple validate <velocity_hz> <duration_ms>
motor commission ripple apply [enable] [gain] [limit_a]
motor commission ripple dump [start_bin] [count]
motor commission ripple clear
```

`velocity_hz` is the commanded mechanical velocity used to move through the electrical
angle table. The table itself is indexed only by electrical angle. During capture, the
measured velocity is used to qualify samples so acceleration, stall, saturation, or poor
tracking does not contaminate the staged feedforward table.

### Phase 4 - Validation

- Native unit tests pass.
- Firmware build passes.
- HIL capture stages a full electrical-angle table.
- Enabled apply is recommended only if validation improves or does not worsen ripple.

## Later Work

- Fit harmonic coefficients from the electrical table.
- Add a TI-style repetitive `vib_comp` module for mechanical/trajectory periodic residuals.
- Add optional adaptive/LMS update only after the fixed table proves useful and safe.
