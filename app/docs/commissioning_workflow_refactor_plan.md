# Commissioning Workflow Refactor Plan

Date: 2026-05-05

## Problem

Commissioning currently works as a set of loosely related commands and state-machine paths:

- `motor state commission`
- `motor commission boot`
- `motor commission encoder run/apply`
- `motor commission validate current|velocity|position`
- `motor commission auto run`
- `motor state calibrate`

These paths share runtime state, encoder acquisition, current references, generated-angle state, timeout state, and staged commissioning results. The result is fragile sequencing: a failed intermediate step can leave encoder acquisition enabled, overwrite previously measured offsets, or require undocumented manual state transitions.

The observed MT6835 run also exposed a concrete regression:

- `RS_EST` measured the expected resistance after wiring was corrected: about `2.26 ohm`.
- `ROVERL_MEAS` measured an impossible inductance: about `3.23 uH`.
- The `ROVERL_MEAS` state configured a rotating current-vector excitation, but the generic generated-angle path later overwrote the angle-generator velocity in the ISR.

## Target Operator Workflow

Every controller restart should use one standard commissioning command until persistence is implemented:

```text
motor commission run
```

This command should own the complete sequence:

1. Enter a known safe state.
2. Disable command timeout for the duration of commissioning.
3. Reset/clear stale commissioning, trace, acquisition, and generated-motion state.
4. Measure current offsets.
5. Measure `Rs`.
6. Measure `R/L` or `L`.
7. Run generated-sweep encoder mapping.
8. Apply encoder mapping.
9. Validate direct encoder-current motion.
10. Estimate flux linkage.
11. Estimate mechanical parameters.
12. Stage controller defaults.
13. Print a single pass/fail report with the exact failed stage.
14. Return to `IDLE`, disarmed, with outputs disabled.
15. Restore command timeout.

## Invariants

- Calibration/commissioning excitation owns the angle generator while active.
- Motion profile and online generated-motion code must not overwrite calibration excitation.
- Every commissioning exit path must stop current, stop generated velocity, disable trace/acquisition side effects, and return to a safe state.
- Each stage must have explicit pass/fail bounds.
- Results should be staged, not silently applied, unless the command explicitly requests apply.
- Boot calibration should only measure current offsets until persistence exists.

## Phase 1: Fix ROVERL Angle Ownership

`ROVERL_MEAS` sets the angle-generator velocity to:

```text
omega_mech = 2*pi*ROVERL_EST_FREQ_HZ / pole_pairs
```

The ISR must run this generator without replacing its velocity from the online motion reference.

Acceptance:

- `ROVERL_MEAS` no longer lets generated-motion policy overwrite the planned excitation velocity.
- Unit tests still pass.
- HIL `motor state commission` reports `Rs` near expected resistance and `L` in a plausible range for the motor.

## Phase 2: Standard Commission Command

Add or repurpose a single high-level command:

```text
motor commission run [slow|confirm] [apply]
```

The command should run the full standard workflow and encapsulate all required state transitions.

Acceptance:

- User does not need to manually run `state commission`, `commission boot`, `arm`, `mode current_encoder`, or `auto run slow` in a specific order.
- Any failure leaves the motor safe and prints the failed stage.

## Phase 3: Stage Validation Gates

Add explicit numeric bounds:

- `Rs` min/max from motor profile.
- `L` min/max from motor profile.
- Encoder mapping minimum correlation, motion, and pole-pair consistency.
- Current smoke-test minimum motion threshold.
- Flux fit minimum R2 and maximum residual.
- Mechanical fit sign consistency, minimum R2, maximum RMS, and repeatability.

Acceptance:

- Bad results are rejected with a specific reason.
- Plausible results are printed and staged.

## Phase 4: Robust Cleanup and Recovery

Create shared cleanup helpers for commissioning commands:

- Stop Id/Iq.
- Stop velocity target.
- Disable detent feedforward during commissioning.
- Stop encoder trace/capture.
- Abort active commissioning context.
- Return to `IDLE` and disarm.
- Restore timeout.

Acceptance:

- Failed boot validation no longer leaves encoder acquisition enabled/busy.
- A second commissioning run can start without reflashing or power-cycling.

## Phase 5: Persistence Integration

When persistence is available, split the workflow:

- `motor commission run`: measure and stage.
- `motor commission apply`: apply staged values.
- `motor commission save`: persist staged/applied values.
- Boot loads saved current offsets, encoder mapping, motor parameters, and controller defaults.

Acceptance:

- Restart does not require full commissioning unless requested or stored data is invalid.

## Implementation Status

Implemented in this change:

- Phase 1: `ROVERL_MEAS`, `RS_EST`, and ALIGN/calibration states now own the
  angle generator while active. The online generated-motion path no longer
  overwrites calibration excitation velocity.
- Phase 2: added `motor commission run [slow|confirm] [apply]` as the standard
  restart workflow wrapper.
- Phase 4 partial: the standard wrapper disables timeout while running, returns
  to `IDLE` disarmed on success/failure, stops current/velocity commands, and
  restores the previous timeout.

Hardware evidence on the MT6835 2 A profile after the Phase 1 fix:

```text
motor commission run slow
  Rs=2.2669 ohm L=0.002789 H R/L=812.8 rad/s
  Encoder mapping result: valid=YES dir=-1 corr=-0.9357 off_mech=1.017 deg
  +Iq validation: Iq=0.150 A net=619.848 deg samples=49 warn=0 err=0
  Motion threshold: pos=0.150 A neg=0.150 A rec=0.150 A
  Flux result: psi_f=0.00458883 Wb R2=0.9897 rms=0.0935V N=230
  Mechanical runs: 3/3 individually valid
  Final mechanical validation: rejected, rms=0.015903 Nm
```

The remaining failure is now a mechanical validation quality gate, not an
electrical `R/L` measurement failure or an encoder-mode transition failure.
