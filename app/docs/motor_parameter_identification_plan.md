# Motor Parameter Identification Plan

This plan defines the commissioning flow for identifying the motor parameters needed by the velocity/position regulators, DOB, MPR, and decoupling/feedforward logic.

Target hardware is a hybrid stepper driven with FOC current control. The encoder path may be used for measurement, but the commissioning flow must tolerate imperfect low-speed motion, detent torque, and a minimum current threshold before the rotor moves smoothly.

## Reference Direction

VESC is the better practical reference for this project than directly following TI MotorControl SDK.

TI MotorControl SDK provides a mature estimator-oriented architecture with Rs online, Ls, and flux estimator APIs, but the identification flow is tightly coupled to TI's estimator stack and application state machines.

VESC provides explicit commissioning-style routines for resistance, inductance, flux linkage, encoder detection, and open-loop excitation. That maps better to this project because we already have explicit shell-driven commissioning, trace capture, current-loop operation, and staged application of identified values.

Do not copy VESC code. Use the same operating pattern: bounded current excitation, conservative open-loop/encoder-assisted motion, sample rejection, fit-quality checks, and staged apply.

## Parameters

Required motor parameters:

- `Rs`: phase resistance used by voltage model, flux identification, and decoupling.
- `Ld`, `Lq`: dq inductance estimates used by voltage model and decoupling.
- `psi_f`: flux linkage in Wb.
- `Kt`: torque constant in Nm/A, derived from `psi_f` for FOC as `Kt = 1.5 * pole_pairs * psi_f`.
- `J`: rotor/load inertia in kg m^2.
- `B`: viscous friction in Nm/(rad/s).
- `Tc`: Coulomb friction magnitude in Nm.
- `Iq_move_min`: minimum q-axis current that produces smooth measurable motion.

## Existing Building Blocks

Existing code already provides most of the estimator foundation:

- `modules/motor_core/src/estimation/flux_id_estimator.c`
- `modules/motor_core/src/estimation/mech_id_estimator.c`
- `modules/motor_core/src/runtime/commission_runtime.c`
- `modules/motor_core/src/runtime/commission_tune.c`
- Shell entry points under `motor commission ...`

The existing auto-tune path computes PI, MPR, and DOB defaults from identified `psi_f`, `J`, `B`, and `Kt`. Keep this staged by default. Applying identified values must remain explicit.

## Phase 0: Preconditions

Before trusting parameter identification:

- Current offsets calibrated.
- Current loop stable in generated/open-loop mode.
- `current_encoder` mode produces correct electrical angle and does not fault at conservative current.
- Encoder transport errors are reported and counted.
- Decoupling disabled unless `psi_f`, `Rs`, `Ld`, and `Lq` are already trusted.
- DOB and MPR disabled during identification unless explicitly under test.
- Command timeout disabled or autonomously petted during commissioning.

## Phase 1: Motion Threshold Identification

Goal: determine the minimum useful q-axis current for this motor/load before running flux or mechanical identification.

Run in `current_encoder` mode after alignment.

Procedure:

1. Sweep positive `Iq` from a conservative low value upward.
2. Repeat for negative `Iq`.
3. Record encoder position, velocity, current reference, measured current, and fault/error flags.
4. Detect the first current level where motion is continuous enough for identification.
5. Store/report:
   - `Iq_move_min_pos`
   - `Iq_move_min_neg`
   - recommended `Iq_id_min = max(abs(pos), abs(neg))`
   - direction consistency
   - encoder warning/error counts

Acceptance:

- Encoder position changes monotonically for each direction.
- Direction agrees with control sign.
- Motion is not just a single detent jump.
- No overcurrent or encoder transport error burst.

Reason:

Hybrid steppers have detent torque and friction. Samples below the true movement threshold corrupt both flux and mechanical fits.

## Phase 2: Flux Linkage / Kt Identification

Goal: estimate `psi_f`, then compute `Kt`.

Use a VESC-style steady-speed sweep, adapted to the current project:

- Prefer `velocity_encoder` with conservative PI if stable.
- If velocity regulation is not stable enough, use generated velocity/open-loop current and use the encoder only for measurement.
- Keep `Id = 0`.
- Use `Iq >= Iq_id_min`.
- Sweep several speeds above the detent-dominated region.
- Reject saturated PWM/current samples.
- Reject samples with encoder transport errors.
- Reject low electrical speed samples.

Estimator model currently used:

```text
y = Vq - Rs * Iq - Lq * dIq/dt - omega_e * Ld * Id
y ~= bias + psi_f * omega_e
```

For steady samples with `Id ~= 0` and small `dIq/dt`, this reduces to:

```text
psi_f ~= (Vq - Rs * Iq) / omega_e
```

Outputs:

- `psi_f_wb`
- `Kt_nm_per_a = 1.5 * pole_pairs * psi_f_wb`
- residual RMS voltage
- R2
- sample count
- rejected sample counters

Acceptance:

- Enough samples across a useful speed span.
- Positive `psi_f`.
- R2 above threshold.
- Residual RMS below threshold.
- Estimated `Kt` is physically plausible.

## Phase 3: Mechanical Parameter Identification

Goal: estimate `J`, `B`, and `Tc`.

Use `current_encoder` mode with controlled q-axis current excitation. This avoids velocity-loop dynamics contaminating the fit.

Model:

```text
Kt * Iq = J * alpha + B * omega + Tc * sign(omega) + T0
```

Procedure:

1. Move the rotor into a speed range above detent/stiction-dominated behavior.
2. Apply bounded PRBS or square-wave `Iq` perturbations around a safe bias.
3. Capture `Iq`, encoder velocity, and encoder acceleration.
4. Fit the linear model with normal equations.
5. Reject samples near zero speed unless explicitly measuring Coulomb friction.

Outputs:

- `J`
- `B`
- `Tc`
- offset torque `T0`
- residual RMS torque
- R2
- sample count

Acceptance:

- Positive inertia.
- Non-negative viscous friction.
- Residual RMS below threshold.
- R2 above threshold, or explicit warning when detent/friction makes R2 weak.
- Enough acceleration excitation in both directions.

Required cleanup before relying on this fit:

- Keep residual RMS denominator as `n - 4` for the 4-parameter model.
- Clamp `sst` non-negative before computing R2.
- Document that negative R2 means worse than predicting mean torque.
- Keep using the accumulated unmodified regressor vector for SSE.

## Phase 4: Auto-Tune From Identified Parameters

Use identified values to compute conservative initial gains.

Velocity PI:

```text
Kp = (2 * zeta * omega_bw * J - B) / Kt
Ki = (omega_bw^2 * J) / Kt
```

Position PI:

```text
position_bw = velocity_bw * ratio
Kp = 2 * zeta * omega_position
Ki = omega_position^2
```

Rules:

- Stage values first.
- Do not auto-enable DOB by default.
- Do not auto-enable MPR by default.
- Apply only after explicit operator command.
- Record tune source and fit quality with the staged values.

## Phase 5: Shell Workflow

Recommended operator flow:

```text
motor state calibrate
motor safety timeout 0
motor arm
motor state mode current_encoder
motor commission motion threshold ...
motor commission flux run ...
motor commission mech run ...
motor commission auto status
motor commission auto apply
```

The existing `motor commission auto run [apply]` should become the high-level wrapper, but it should internally run the threshold step before choosing flux/mech excitation currents.

## Phase 6: HIL Validation

For each commissioning run, capture and report:

- encoder error/warning counts
- rejected sample counters
- current saturation counters
- PWM saturation counters
- measured speed span
- measured acceleration span
- fit R2
- residual RMS
- accepted sample count
- final staged values

Validation after applying gains:

- Conservative `velocity_encoder` step response.
- Conservative `position_encoder` move.
- No DOB/MPR initially.
- No decoupling until `psi_f`, `Rs`, `Ld`, and `Lq` are trusted.

## Implementation Order

1. Add/verify motion-threshold commissioning.
2. Harden mechanical estimator finalization and tests.
3. Improve commissioning sample rejection and reporting.
4. Wire threshold current into auto flux/mech defaults.
5. Keep auto-tune staged and PI-only by default.
6. Run HIL identification on the AEAT hardware.
7. Only then consider enabling DOB, MPR, and decoupling from identified values.

## Non-Goals

- Do not persist values yet.
- Do not require sensorless estimation.
- Do not copy TI or VESC implementation code.
- Do not depend on MPR or DOB during identification.
- Do not trust low-current low-speed samples on a hybrid stepper.

## Progress

- 2026-05-04: Added `motor commission motion threshold <start_a> <stop_a> <step_a> <hold_ms> [min_motion_deg]`.
- 2026-05-04: Motion threshold results are stored in `motor_commission_results` and shown in `motor commission status`.
- 2026-05-04: `motor commission auto run` now runs the motion threshold sweep first and uses the recommended moving current to choose flux/mechanical excitation currents.
- 2026-05-04: Confirmed the mechanical estimator review fixes are already present: residual RMS uses `n - 4`, `sst` is clamped non-negative, and the SSE path documents use of the unmodified accumulated regressor.
