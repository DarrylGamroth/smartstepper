# Motor Control Units Contract

Date: 2026-02-22  
Scope: `app/src` control loop + `modules/motor_core` interfaces.

## Purpose

Freeze the control-math units, frame mapping, and sign conventions used by the runtime so future hybrid-stepper updates can change algorithms without silently changing physical meaning.

## Canonical Runtime Units

- Current: amperes (`A`)
- Voltage: volts (`V`)
- Angle: radians (`rad`) in control math
- Speed: radians per second (`rad/s`)
- Acceleration: radians per second squared (`rad/s^2`)
- Flux linkage: webers (`Wb`)
- Resistance: ohms (`Ohm`)
- Inductance: henries (`H`)
- Torque: newton-meters (`Nm`)
- Inertia: kilogram-meter squared (`kg*m^2`)
- PWM command: per-unit duty (`[0, 1]`)

Notes:

- Shell commands may expose Hz/deg for usability; runtime storage and control math are rad-based.
- `arm_sin_cos_f32` takes degrees, so code performs explicit rad->deg conversion at the call site.

## Frame and Angle Mapping Contract

## Mechanical/Electrical mapping

- Electrical angle:
  - `theta_e = wrap_2pi((theta_m + theta_offset_m) * pole_pairs)`
- Electrical speed:
  - `omega_e = omega_m * pole_pairs`
- Alignment offset convention:
  - `theta_offset_m = -theta_m_align` (so aligned d-axis is electrical zero after ALIGN_SAMPLE)

Source paths:

- `modules/motor_core/src/angle_observer.c`
- `app/src/motor_states_calibration.c`

## Park/inverse-Park usage

- Runtime treats sensed two-phase currents as stationary orthogonal inputs to Park:
  - `arm_park_f32(Ia, Ib, &Id, &Iq, sin(theta_e), cos(theta_e))`
- Inverse Park produces phase voltage commands:
  - `arm_inv_park_f32(Vd, Vq, &Va, &Vb, sin(theta_e_pred), cos(theta_e_pred))`

Source paths:

- `app/src/motor_control_loop.c`
- `modules/motor_core/src/motor_foc_voltage_pwm.c`

## Current Sign and Polarity Contract

- Sense polarity is configured via:
  - `CURRENT_SENSE_POLARITY_0`
  - `CURRENT_SENSE_POLARITY_1`
- These polarity constants define the sign of measured `Ia/Ib` before Park transform.
- Any board-specific inversion must be handled in polarity config, not ad-hoc in control math.

Source paths:

- `app/include/config.h`
- `app/src/motor_control_loop.c`

## Torque-Domain Contract (Current Implementation)

- Current runtime torque conversion path is:
  - `Kt = 1.5 * pole_pairs * flux_linkage_wb_active`
- This feeds velocity MPR/DOB limits and commissioning/tuning outputs.
- This is a temporary contract and will be replaced by explicit torque-gain parameterization in Phase 1.

Source paths:

- `app/src/motor_control_loop.c`
- `app/src/motor_commission.c`
- `app/src/shell_commands.c`
- `modules/motor_core/src/motor_commission_tune.c`

## Flux and Parameter Normalization Contract

- Devicetree flux input is interpreted as electrical V/Hz and converted to Wb by:
  - `flux_wb = flux_vph_elec / (2*pi)`
- Decoupling/feedforward uses `Ld`, `Lq`, and `flux_linkage_wb` in SI units (`H`, `Wb`).

Source paths:

- `app/include/config.h`
- `modules/motor_core/src/motor_foc_voltage_pwm.c`

## Required Invariants

- All public `motor_core` APIs remain SI/rad-based.
- Angle/speed conversions happen only at hardware/API boundaries (encoder input, CMSIS trig calls, shell I/O).
- Torque-domain controllers (MPR/DOB/tuning) must consume one canonical `Kt` source.

## Phase 0 Outcome

This document is the Phase 0 baseline for hybrid-stepper adaptation. Any change to these conventions must update this file and the affected module docs in the same change.
