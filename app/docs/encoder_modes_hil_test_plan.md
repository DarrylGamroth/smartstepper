# Encoder Modes HIL Test Plan

## Goal

Verify that the encoder-commutated control modes work after the runtime boot
commissioning workflow:

1. `current_encoder`
2. `velocity_encoder`
3. `position_encoder`

The test must separate three failure classes:

1. Encoder acquisition/mapping failure.
2. FOC commutation/current-loop failure.
3. Outer-loop tuning or motion-profile failure.

## Prerequisites

- Target flashed with the current firmware build.
- Telnet shell preferred when network is available.
- Serial shell fallback is acceptable, but avoid repeated short-lived open/close
  sessions.
- Motor starts from a safe mechanical condition.
- Power supply current limit is set conservatively.
- Boot commissioning has not been persisted yet, so it must be run after every
  reset or power cycle.

## Global Safety Rules

- Start every test from `IDLE` or a known online generated mode.
- Use low currents first.
- Keep `Iq` at `0` before changing modes unless the test explicitly says
  otherwise.
- Disable or avoid optional feedforward features unless the test is specifically
  validating them.
- If PSU current rises unexpectedly, immediately run the recovery sequence or
  reset the target.

Recovery sequence:

```text
motor current iq 0
motor current id 0
motor velocity target 0
motor disarm
motor state idle
motor state clear_error
motor gate reset
motor fault snapshot status
```

## Required Baseline After Every Boot

Run:

```text
motor commission boot 0.15 0.10 1
motor encoder control_status
motor state status
```

Pass criteria:

- `motor commission boot` returns success.
- Encoder mapping is valid.
- Mapping direction is plausible for the hardware profile.
- `motor encoder control_status` reports `Ready: YES`.
- Acquisition error counters do not increment during the sweep.
- State is `ONLINE_VELOCITY_GENERATED`.
- `Enc mapped: YES`.

Record:

- current offsets
- direction sign
- mechanical offset
- sample count
- rejected/warn/error counts
- acquisition counters

## Telemetry Setup

Use raw trace for high-rate encoder/commutation debugging:

```text
motor encoder trace clear
motor encoder trace start 200
...
motor encoder trace stop
motor encoder trace summary
motor encoder trace dump 0 32
```

Use fault snapshot only around suspicious tests:

```text
motor fault snapshot clear
motor fault snapshot start 1
...
motor fault snapshot stop
motor fault snapshot status
motor fault snapshot dump 32
```

Pass criteria:

- No transport/parity/CRC/glitch/status errors during clean operation.
- Raw angle changes smoothly when the rotor moves.
- Electrical angle advances continuously with the expected pole-pair scaling.
- Trace dumps are kept small enough to avoid shell overload.

## Phase 0: Generated Mode Sanity Check

Purpose: prove current loop, generated angle, and telemetry are still healthy
before testing encoder commutation.

Commands:

```text
motor state mode velocity_generated
motor current id 0
motor current iq 0.10
motor encoder trace clear
motor encoder trace start 200
motor velocity target 0.5
motor velocity target 0
motor encoder trace stop
motor encoder trace summary
motor current iq 0
```

Pass criteria:

- Motor rotates smoothly in the expected direction.
- No overcurrent or gate fault.
- Encoder raw trace reports smooth mechanical motion.
- Trace error counters remain zero.

Failure interpretation:

- If this fails, do not continue to encoder modes. The issue is not specific to
  encoder commutation.

## Phase 1: `current_encoder` Mode

Purpose: validate encoder angle into FOC without involving velocity or position
regulators.

Entry commands:

```text
motor current iq 0
motor current id 0
motor state mode current_encoder
motor encoder control_status
motor fault snapshot clear
motor fault snapshot start 1
```

Step 1: low positive current.

```text
motor current iq 0.03
motor info live
motor encoder trace clear
motor encoder trace start 200
motor current iq 0.06
motor current iq 0.00
motor encoder trace stop
motor encoder trace summary
motor fault snapshot stop
motor fault snapshot status
```

Step 2: signed current check.

```text
motor fault snapshot clear
motor fault snapshot start 1
motor current iq 0.04
motor current iq 0.00
motor current iq -0.04
motor current iq 0.00
motor fault snapshot stop
motor fault snapshot dump 32
```

Pass criteria:

- Mode enters `ONLINE_CURRENT_ENCODER`.
- `motor encoder control_status` remains `Ready: YES`.
- No overcurrent, hard fault, or gate fault.
- Positive and negative `Iq` produce consistent opposite torque tendencies.
- Motor may not spin freely at very low current due to detent/friction, but it
  must not buzz violently or pull excessive current.
- Fault snapshot shows sane `Id/Iq` and bounded voltage/PWM commands.

Failure interpretation:

- Immediate high current or harsh buzzing points to electrical-angle mapping,
  sign, offset, or FOC angle usage.
- Encoder errors in trace point to acquisition/transport.
- Smooth holding torque but no rotation at low current may simply be below the
  movement threshold.

Exit:

```text
motor current iq 0
motor current id 0
```

## Phase 2: `velocity_encoder` Mode

Purpose: validate encoder feedback plus the velocity regulator. This phase
assumes `current_encoder` already passed.

Conservative setup:

```text
motor velocity gains defaults safe
motor velocity decimation 10
motor velocity dob enable 0
motor current iq 0
motor state mode velocity_encoder
motor encoder control_status
motor velocity status
```

Low-speed step sequence:

```text
motor fault snapshot clear
motor fault snapshot start 1
motor encoder trace clear
motor encoder trace start 200
motor velocity target 0.10
motor velocity status
motor velocity target 0.25
motor velocity status
motor velocity target 0.00
motor velocity status
motor encoder trace stop
motor encoder trace summary
motor fault snapshot stop
motor fault snapshot status
```

Direction sequence:

```text
motor fault snapshot clear
motor fault snapshot start 1
motor velocity target 0.15
motor velocity target 0.00
motor velocity target -0.15
motor velocity target 0.00
motor fault snapshot stop
motor fault snapshot dump 32
```

Pass criteria:

- Mode enters `ONLINE_VELOCITY_ENCODER`.
- Target velocity sign matches measured velocity sign.
- Regulator current stays within configured `iq_limit`.
- No sustained runaway, direction reversal, overcurrent, or rough oscillation.
- Encoder trace confirms measured velocity follows the target direction.
- Commanding `0 Hz` returns to near-zero velocity and near-zero `Iq` after
  settling.

Failure interpretation:

- Runs backward: velocity feedback sign or encoder direction is wrong.
- Jumps/oscillates: gains too aggressive, decimation too low, or angle is noisy.
- Current saturates without acceleration: commutation offset/sign problem or
  insufficient current limit.
- Works in `current_encoder` but not here: outer-loop tuning/decimation problem.

Exit:

```text
motor velocity target 0
motor current iq 0
```

## Phase 3: `position_encoder` Mode

Purpose: validate position regulation and profile/position command behavior.
This phase assumes `current_encoder` and low-speed `velocity_encoder` passed.

Conservative setup:

```text
motor position gains defaults safe
motor position decimation 20
motor velocity gains defaults safe
motor velocity decimation 10
motor velocity dob enable 0
motor state mode position_encoder
motor encoder control_status
motor position status
```

Hold test:

```text
motor fault snapshot clear
motor fault snapshot start 1
motor encoder trace clear
motor encoder trace start 200
motor position target 0
motor position status
motor encoder trace stop
motor encoder trace summary
motor fault snapshot stop
motor fault snapshot status
```

Small move sequence:

```text
motor fault snapshot clear
motor fault snapshot start 1
motor encoder trace clear
motor encoder trace start 200
motor position target 5
motor position status
motor position target 0
motor position status
motor position target -5
motor position status
motor position target 0
motor position status
motor encoder trace stop
motor encoder trace summary
motor fault snapshot stop
motor fault snapshot dump 32
```

Larger bounded move sequence:

```text
motor position target 15
motor position status
motor position target 0
motor position status
```

Pass criteria:

- Mode enters `ONLINE_POSITION_ENCODER`.
- Holding a target does not produce runaway current.
- Small positive and negative moves produce corresponding encoder motion.
- Position settles without sustained oscillation.
- Velocity loop current demand remains bounded.
- Commanding the original target returns near the starting position.

Failure interpretation:

- Holds but does not move: position loop gain too low or velocity target limit
  too conservative.
- Oscillates: position/velocity gains too high, decimation too low, or detent
  compensation needed later.
- Moves wrong direction: sign error in position feedback or target convention.
- Works in velocity but not position: position regulator/profile path issue.

Exit:

```text
motor position target 0
motor velocity target 0
motor current iq 0
```

## Phase 4: Transition Tests

Purpose: validate deterministic transitions after each individual mode works.

Commands:

```text
motor state mode current_encoder
motor current iq 0.03
motor current iq 0
motor state mode velocity_encoder
motor velocity target 0.10
motor velocity target 0
motor state mode position_encoder
motor position target 5
motor position target 0
motor state idle
motor state status
```

Pass criteria:

- Each transition completes without hard fault or overcurrent.
- Current and velocity targets return to zero when commanded.
- State machine reports the expected state after each command.

## Phase 5: Fault/Recovery Checks

Purpose: verify failures are diagnosable and recoverable.

Checks:

- Disable encoder acquisition only if a safe command exists, then confirm
  encoder modes refuse entry or fault cleanly.
- Force command timeout in an encoder mode and verify disarm/timeout behavior.
- Trigger recovery sequence and verify generated boot commissioning can be run
  again without power cycling.

Pass criteria:

- Faults are reported through state/status commands.
- The board does not silently hang.
- Gate reset and `clear_error` recover expected faults.

## Result Table

| Test | Status | Evidence | Notes |
| --- | --- | --- | --- |
| Boot commissioning | pass | Historical `motor commission boot 0.15 0.10 1` valid; direction `-1`; corrected commutation offset `0.202 deg`; `control_status Ready: YES`. | Current implementation now uses an `Id` d-axis generated sweep so the detected offset is applied directly; re-run HIL before treating the old Iq-sweep numbers as current evidence. |
| Phase 0 generated sanity | pass | `velocity_generated`, `Iq=0.10 A`, `target=0.5 Hz`; raw trace clean (`warn=0 err=0 io=0`). | Trace overran because it was left running beyond the motion interval; clean window was still valid. |
| Phase 1 current_encoder entry | pass | Entered `ONLINE_CURRENT_ENCODER`; `Iq=0.03/0.06 A` tracked current with no faults and clean trace. | Mode entry disarms control, so test sequence must arm after mode entry. |
| Phase 1 current_encoder signed current | pass | Historical `Iq=+0.04/-0.04 A` tracked with no state fault; `Iq=0.15 A` moved after commutation-offset correction. | The old evidence used an Iq-excited sweep plus phase correction. Current boot commissioning validates `+Iq` motion after applying the direct Id-sweep offset. |
| Phase 2 velocity_encoder low-speed steps | partial pass | Entered `ONLINE_VELOCITY_ENCODER`; target `0.25 Hz` produced motion after correction; encoder trace clean at stop. | Safe gains integrate too slowly; nominal gains overshot to roughly `0.5-0.7 Hz`. Needs velocity tuning and anti-windup/zero-target behavior review. |
| Phase 2 velocity_encoder direction | partial pass | Positive target produced positive measured velocity after correction. | Reverse target not run after correction because velocity tuning still needs cleanup. |
| Phase 3 position_encoder hold | pass | Entered `ONLINE_POSITION_ENCODER`; hold target near current position stayed stable with small position error. | Uses safe position/velocity gains and decimation. |
| Phase 3 position_encoder small moves | fail | Commanding `237 deg` from about `232 deg` caused a large transient/overshoot and high velocity command. | Position loop is not ready for moves; likely needs target semantics, profile limiting, and gain/limit review. |
| Phase 4 transitions | partial pass | IDLE to current/velocity/position encoder entries completed after acquisition counters were cleared. | High-speed current test latched AEAT CRC/status errors; `motor encoder acquisition_reset` was required before re-entering encoder modes. |
| Phase 5 fault/recovery | partial pass | `motor disarm` and `motor state idle` recovered from test exits; no hard faults observed. | Did not intentionally inject faults or require `motor gate reset` in this run. |

## Execution Log

### 2026-05-04 HIL Run

Build and flash:

```text
west build --build-dir /workspace/build/chopper/smartstepper_v2
west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip
```

Initial boot commissioning passed with the old apply behavior:

- offsets: `Ia=2.9956`, `Ib=2.9988`
- mapping: valid, direction `-1`, generated-reference offset `-1.587 deg`
- control readiness: `Ready: YES`

`current_encoder` before correction:

- `Iq=0.03/0.06 A`: current tracked, no faults, no motion.
- `Iq=0.15 A`: current tracked at about `0.154 A`, no faults, essentially no
  motion.
- Encoder trace remained clean.
- Conclusion: encoder transport was not the blocker; the applied offset was not
  the FOC rotor d-axis offset.

Manual trim experiment:

- Applied `motor encoder trim 90`.
- `current_encoder`, `Iq=0.15 A` produced immediate motion, roughly `6 Hz`
  mechanical in the raw trace.
- Conclusion: the old Iq-excited generated-sweep mapping needed `+90 electrical
  degrees` correction for positive `Iq` excitation. The current implementation
  avoids that convention problem by using `Id` d-axis excitation during mapping.

After implementing automatic correction:

- boot command result: valid, direction `-1`, generated-reference offset
  `-1.598 deg mechanical`, corrected commutation offset `0.202 deg mechanical`.
- `current_encoder`, `Iq=0.15 A`: produced motion, confirming corrected
  commutation offset.
- `velocity_encoder`: entered and moved with positive target; nominal gains
  overshot, safe gains were sluggish.
- `position_encoder`: hold passed; small absolute move failed with large
  transient/overshoot.

Open issues from this run:

1. Tune `velocity_encoder` gains/limits now that the commutation offset is
   correct.
2. Review velocity PI I-term behavior when target returns to zero.
3. Review `position_encoder` command semantics and profile limiting before
   allowing direct small moves.
4. Investigate AEAT CRC/status errors during faster `current_encoder` spin; they
   were not present in the slow commissioning sweep or stationary encoder modes.

## Recommended Execution Order

1. Run boot commissioning.
2. Run generated sanity.
3. Run `current_encoder` with `Iq <= 0.06 A`.
4. If smooth, increase `current_encoder` to the minimum needed to overcome
   detent/friction, but do not exceed the known safe test current.
5. Run `velocity_encoder` with safe gains and decimation.
6. Tune velocity only after direction and stability are proven.
7. Run `position_encoder` with safe gains and small targets.
8. Only then test larger moves, transitions, and fault recovery.

## Stop Conditions

Stop testing and investigate before proceeding if any of these occur:

- PSU current is unexpectedly high.
- Motor runs away or reverses unexpectedly.
- Encoder acquisition counters increment.
- Fault snapshot shows saturated voltage/current immediately after mode entry.
- Shell/network stops responding.
- State machine enters `ERROR` without a clear fault reason.
