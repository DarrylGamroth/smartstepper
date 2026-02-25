# Encoder/Angle Path Simplification Plan

Date: 2026-02-25

## Refactor Policy

This work is a simplification/refactor effort and does **not** require backward compatibility with the current encoder/angle path structure, intermediate APIs, or legacy telemetry field shapes.

Allowed:
- breaking internal module interfaces to reduce complexity
- removing redundant state/flags/modules
- replacing existing shell/debug command layouts when needed to match the simplified architecture

Required:
- preserve functional behavior targets (safe control operation, diagnosable faults)
- provide clear migration notes for renamed/removed interfaces and shell commands

## Why This Plan

Current encoder handling is still doing too much in one place (`app/src/motor_encoder_feedback.c`), mixing:
- transport/sample normalization
- source arbitration
- observer handoff logic
- quality/stale tracking
- capture/debug generation

This is harder to reason about than needed for real-time control.

## External Patterns Reviewed

Reviewed:
- `../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/observers/encoder/source/encoder.c`
- `../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/solutions/servo_drive_with_can/common/source/qep_sensor.c`
- `../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/utilities/angle_gen/source/angle_gen.c`
- `../../motor/spinner/drivers/feedback/halls_stm32.c`
- `../../motor/spinner/include/spinner/drivers/feedback.h`

Key patterns:
1. Sensor module is narrow: angle (+ speed) update and a small state machine.
2. Control loop consumes simple outputs (angle, speed, validity), not transport details.
3. Alignment/index state is explicit and orthogonal to transport warnings.
4. Debug/capture is separated from control signal path.

## Target Design (Simplified)

### 1) One control-facing sample contract

Define a minimal control sample (all radians in control path):
- `angle_mech_rad`
- `speed_mech_rad_s` (from observer)
- `quality` bits: `VALID`, `FRESH`, `ERROR`
- `source`: `ENCODER` or `GENERATED`

No control dependency on:
- warning/io_fault/status internals
- capture deltas
- additional intermediate source enums

### 2) Driver owns frame validity semantics

Driver/decoder should already classify frame issues (CRC/parity/status transport faults).
Control path only sees condensed error state, not frame parsing detail.

### 3) Collapse source + tracking glue

Replace `encoder_source + angle_tracking + app glue` with one small step function:
- input: latest raw encoder sample (optional), generated angle, mode flags
- output: chosen angle input + source + quality
- then call observer update exactly once

No “measurement locked” state in separate module. If handoff reset is required, make it explicit in state transition code (mode entry), not hidden in per-sample logic.

### 4) Keep observer focused

`angle_observer` responsibilities:
- wrap/unwarp
- offset application
- optional delay compensation
- speed estimate

No transport policy and no capture policy inside observer.

### 5) Move capture/debug out of control updater

Control updater should return only control-relevant outputs.
Capture comparison (`generated vs observer`, error columns, source snapshots) should run in a separate optional function behind feature flag.

### 6) Telemetry-owned encoder raw trace buffer

Add a dedicated telemetry circular buffer for raw encoder transport samples:
- timestamp / sample index
- raw angle (as reported by driver/decoder before observer smoothing)
- raw status/error bits (CRC/parity/status/io_fault/warn/error summary)
- selected source and quality snapshot at that tick

Requirements:
1. ISR-safe, non-blocking write path (single-producer ring).
2. Shell-accessible dump/readout command from telemetry subsystem.
3. Optional compile-time enable + runtime enable/disable.
4. Overflow counter and dropped-sample counter exposed in shell.

This preserves deep debug visibility without polluting the control path.

## Phased Execution

## Phase 0: Freeze contracts and behavior targets

1. Add a short architecture note for encoder path ownership:
   - driver/decoder: frame validity + angle extraction
   - control: arbitration + observer + quality
2. Define exact semantics for:
   - `VALID`
   - `FRESH`
   - `ERROR`
3. Keep existing runtime behavior as baseline.

Acceptance:
1. Agreed semantics documented.
2. No code changes required.

## Phase 1: Introduce minimal control sample type

1. Add `motor_encoder_control_sample` (or equivalent) in `motor_core`.
2. Update ISR control path to consume this type only.
3. Keep existing telemetry fields populated temporarily via adapter.

Acceptance:
1. Control loop no longer reads transport-specific flags directly.
2. Build and unit tests unchanged in behavior.

## Phase 2: Collapse arbitration + tracking into one module

1. Replace:
   - `motor_encoder_source_*`
   - `motor_angle_tracking_update`
   with one function (example: `motor_angle_path_step`).
2. Observer handoff reset (if needed) moves to mode-entry/state transition code.
3. Remove `measurement_locked`-style implicit gating.

Acceptance:
1. Fewer modules and fewer booleans in ISR hot path.
2. Angle/speed outputs remain stable in velocity_open and torque mode bring-up tests.

## Phase 3: Separate capture/debug path from control path

1. Split current `motor_encoder_feedback_update` into:
   - control update (always)
   - debug capture update (optional)
2. Gate capture work with compile-time/runtime flags.
3. Route raw encoder trace capture through telemetry ring buffer (not control structs).

Acceptance:
1. Control function returns only control fields.
2. Disabling capture removes extra compute/branches from fast path.
3. Shell dump can retrieve raw samples with transport flags for decoder/SPI debugging.

## Phase 4: Trim state and telemetry to essentials

1. Remove stale temporary counters/fields that are not used by control policy.
2. Keep only:
   - health counters needed for fault policy
   - minimal shell introspection fields
3. Update shell output to reflect final simplified model.

Acceptance:
1. Smaller hot state footprint.
2. Shell remains sufficient for bring-up and fault triage.

## Phase 5: Verification

1. Unit:
   - source selection behavior
   - observer wrap and delay compensation
   - quality flag transitions (`VALID/FRESH/ERROR`)
2. HIL:
   - velocity_open run with encoder logging
   - torque mode startup from offline->align->online
   - induced bad frame path (CRC/parity/status) confirms `ERROR` handling

Acceptance:
1. No regressions in known good open-loop behavior.
2. Closed-loop startup path remains deterministic and diagnosable.

## Immediate First Cut (Recommended)

Implement Phases 1-2 first, because they remove most complexity while keeping behavior stable:
1. Create minimal control sample.
2. Replace `encoder_source + angle_tracking` with single `angle_path_step`.
3. Keep capture logic untouched temporarily to reduce migration risk.
