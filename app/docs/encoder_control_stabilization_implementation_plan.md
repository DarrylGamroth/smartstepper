# Encoder-Control Stabilization Implementation Plan

## Goal

Make encoder-based operation deterministic and boring:

1. Run boot commissioning after every reset until persistence exists.
2. Prove encoder commutation with `current_encoder`.
3. Bring up `velocity_encoder` using conservative PI first.
4. Bring up `position_encoder` only after velocity is stable.
5. Keep MPR/DOB/detent feedforward out of the first-pass validation path.
6. Make encoder CRC/transport errors visible without letting one bad frame destabilize FOC.

This plan intentionally prioritizes a working PI-based closed-loop system before
returning to MPR tuning.

## Current Diagnosis

The encoder is not proven faulty. The observed behavior points to three separate
issues:

- Encoder acquisition can produce occasional CRC/parity/status/glitch diagnostics.
- A single bad encoder sample currently degrades feedback quality enough that
  velocity control can hold/reset instead of smoothly riding through the dropout.
- Encoder modes still depend on a multi-step commissioning/tuning sequence that
  is not fully enforced as one deterministic workflow.

Generated/open-loop motion works, so the current loop, PWM path, and motor drive
are not the primary blockers.

## Observer Behavior on Bad Encoder Samples

Yes, the observer should advance when a bad encoder sample is received.

Required behavior:

- Never feed CRC/parity/transport/status-error angle data into the observer
  correction term.
- On a single bad sample, propagate the observer using its current speed estimate:

  ```text
  theta_hat[k+1] = theta_hat[k] + Ts * omega_hat[k]
  omega_hat[k+1] = omega_hat[k]
  ```

- Mark the control feedback as `VALID` but not `FRESH` for a bounded number of
  consecutive propagated samples.
- Keep `ERROR` reserved for hard feedback failure after the dropout threshold is
  exceeded.
- Let the FOC/current loop use the propagated electrical angle during the bounded
  dropout window.
- Do not update outer velocity/position integrators from propagated samples.
  Outer loops should skip their update when the feedback is not fresh.

This keeps commutation continuous through isolated bad frames while preventing
the velocity/position regulators from integrating on stale data.

Current code already has pieces of this:

- `angle_observer_update()` advances the state when given the current observer
  angle.
- `motor_angle_path_step()` selects a propagated source when the encoder sample
  is not usable.
- `outer_loop_runtime` already checks `MOTOR_FEEDBACK_QUALITY_FRESH` before
  updating the velocity loop.

The missing piece is quality semantics: propagated observer output should be
control-valid for a short dropout window, while the raw bad frame remains counted
as a diagnostic error.

## Phase 1: Force Conservative Bring-Up Defaults

### Work

- Ensure all encoder-mode commissioning/test scripts explicitly run:

  ```text
  motor outer mode pi
  motor velocity dob enable 0
  motor commission detent clear
  ```

- Keep generated/open-loop modes unaffected.
- Ensure `motor commission boot ...` leaves the target in a known safe state:
  generated velocity mode or idle, zero current command, PI outer loop selected.

### Acceptance

- After reset, the documented workflow starts from a known state.
- `motor outer status` reports `PI`.
- DOB and detent feedforward are disabled unless explicitly enabled.

## Phase 2: Harden Encoder Bad-Sample Ride-Through

### Work

- Add an explicit observer propagation API:

  ```c
  void angle_observer_predict(struct angle_observer_state *obs);
  ```

  This should advance `angle_est_rad`, `mech_angle_rad`, `elec_angle_rad`, and
  predicted outputs using the existing speed estimate, without measurement
  correction.

- Update `motor_angle_path_step()`:
  - fresh good encoder sample: update observer from encoder.
  - generated source: update observer from generated angle.
  - bad/missing encoder sample in encoder-control mode: call
    `angle_observer_predict()`.

- Add a bounded propagation counter in the encoder feedback state.
- For propagated samples within the limit:
  - set `MOTOR_FEEDBACK_QUALITY_VALID`.
  - clear `MOTOR_FEEDBACK_QUALITY_FRESH`.
  - do not set `MOTOR_FEEDBACK_QUALITY_ERROR`.
- After the limit:
  - clear `VALID`.
  - set `ERROR`.
  - trigger the existing hold/fault path.

- Keep raw encoder diagnostic counters separate:
  - CRC/parity/transport/status counters still increment immediately.
  - The control-quality `ERROR` bit only means feedback is no longer safe for
    control.

### Acceptance

- Unit tests cover:
  - one bad sample between good samples.
  - several bad samples below threshold.
  - threshold exceeded.
  - recovery on next good sample.
- HIL trace shows isolated CRC errors do not cause current spikes or controller
  resets.

## Phase 3: Make Boot Commissioning the Gate

### Work

- Keep persistence out of scope for now.
- Require a successful boot commissioning sequence before encoder modes:

  ```text
  motor commission boot <current_a> <mech_hz> <cycles>
  ```

- Make the command perform, validate, and report:
  - current offset calibration.
  - generated-sweep encoder mapping.
  - direction sign.
  - mechanical/electrical offset.
  - observer offset application.
  - acquisition counters.
  - readiness gate result.

- Add one short `+Iq` validation pulse after mapping, if safe:
  - verify measured encoder direction matches the expected torque direction.
  - do not require free spin; small angle displacement or consistent torque
    tendency is acceptable for low current.

### Acceptance

- `motor encoder control_status` reports ready only after mapping is applied.
- Failed mapping prevents encoder modes.
- Generated/open-loop mode still works even if encoder mapping is absent.

## Phase 4: Validate `current_encoder`

### Work

- Add a repeatable shell workflow or command wrapper for current-encoder smoke:

  ```text
  motor state mode current_encoder
  motor arm
  motor current id 0
  motor current iq 0.03
  motor current iq 0.06
  motor current iq 0
  ```

- Capture:
  - current tracking.
  - observer angle.
  - encoder diagnostic counters.
  - fault snapshot if enabled.

### Acceptance

- No overcurrent, gate fault, hard fault, or violent buzzing.
- `Id/Iq` measured values track the command.
- Positive and negative `Iq` produce consistent opposite torque tendencies.
- Isolated encoder CRC/status events are reported but do not destabilize FOC.

## Phase 5: Validate `velocity_encoder` With PI

### Work

- Use PI, not MPR:

  ```text
  motor outer mode pi
  ```

- Disable optional feedforward:

  ```text
  motor velocity dob enable 0
  motor commission detent clear
  ```

- Add conservative command sequence:

  ```text
  motor velocity pi set <kp> <ki> <iq_limit>
  motor state mode velocity_encoder
  motor arm
  motor velocity target 0.05
  motor velocity target 0
  motor velocity target -0.05
  motor velocity target 0
  ```

- Add anti-windup/zero-target review:
  - zero target should reset/bleed velocity integrator when measured speed is
    near zero.
  - target sign changes should not carry stale Iq integrator.

### Acceptance

- Low-speed positive and negative targets move in the expected directions.
- No runaway, large overshoot, overcurrent, or gate fault.
- Velocity settles near zero after zero target.
- If low-speed motion is detent-limited, increasing current limit within safe
  bounds produces movement without instability.

## Phase 6: Validate `position_encoder` After Velocity Works

### Work

- Use bounded profile semantics only.
- Position target must not create an aggressive direct velocity jump.
- Keep position bandwidth lower than velocity bandwidth.
- Add a small-move workflow:

  ```text
  motor state mode position_encoder
  motor arm
  motor position target <current_position + small_delta_deg>
  motor position target <current_position>
  ```

- Ensure position commands:
  - reset stale position/velocity integrators on entry.
  - use profile limits.
  - clamp velocity target and acceleration.

### Acceptance

- Hold mode is stable.
- Small moves complete without overshoot/runaway.
- Position profile produces bounded velocity/current commands.
- Encoder diagnostic counters remain low and visible.

## Phase 7: Reintroduce MPR Only After PI Passes

### Work

- Treat MPR as an alternate outer-loop regulator, not the bring-up default.
- Tune using the same validated commissioning values.
- Start with velocity MPR only.
- Add position MPR after velocity MPR is stable.

### Acceptance

- `motor outer mode mpr` can be enabled after PI validation.
- `velocity_encoder` MPR response is at least as stable as PI at low speed.
- `position_encoder` MPR does not command unsafe velocity/current transients.

## HIL Evidence Required

Create one HIL log per validation pass:

- boot commissioning result.
- encoder acquisition/protocol status.
- current encoder test.
- velocity encoder PI test.
- position encoder PI test.
- optional MPR test after PI passes.

Each log should include:

```text
motor state status
motor outer status
motor encoder control_status
motor encoder acquisition
motor info live
motor fault snapshot status
```

## Out of Scope

- Persistence to EEPROM/settings.
- Detent feedforward tuning.
- DOB tuning.
- MPR as the default regulator.
- Hardware redesign or encoder replacement.

