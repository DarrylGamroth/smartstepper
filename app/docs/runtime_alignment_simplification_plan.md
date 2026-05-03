# Runtime Alignment Simplification Plan

Date: 2026-05-03

## Objective

Simplify boot/runtime rotor alignment and move richer encoder validation into an explicit
commissioning command.

The runtime ALIGN path should be deterministic, short, and easy to reason about. It should only
establish the encoder-to-electrical-zero offset needed to start encoder-based FOC. It should not try
to validate encoder direction, pole-pair ratio, or dual-polarity behavior during every calibration
boot sequence.

## Reference Patterns Reviewed

### Arduino-FOC

- Uses a simple static alignment vector for normal sensor zeroing.
- Uses generated/open-loop electrical phase motion to validate sensor direction separately.
- Does not use a dual-polarity static alignment check in the normal runtime path.

### VESC BLDC Firmware

- Provides `foc_encoder_detect` as an explicit commissioning routine.
- Uses open-loop FOC current and generated electrical phase sweeps to detect:
  - encoder inversion
  - encoder/electrical ratio
  - encoder offset
- Uses `rotor_lock_openloop` as a simple static generated-phase rotor lock.
- Keeps runtime encoder use simple: encoder angle is read, transformed by sign/ratio/offset, and
  consumed by FOC.

## Design Decision

Remove dual-polarity alignment from boot/runtime ALIGN.

Runtime ALIGN becomes:

1. Ramp `Id` to the configured alignment current through `traj_Id`.
2. Hold a generated electrical frame at the alignment angle.
3. Collect a small fresh encoder sample window after settle.
4. Compute circular mean of mechanical encoder angle.
5. Set mechanical alignment offset from that mean.
6. Ramp `Id` back to zero through `traj_Id`.
7. Transition to the requested post-calibration state.

Richer validation becomes a manually invoked commissioning/diagnostic flow:

1. Move generated electrical phase through one or more electrical cycles.
2. Record generated electrical phase and encoder mechanical/electrical phase.
3. Estimate direction/sign and offset.
4. Optionally validate known pole-pair ratio.
5. Report residuals, error counts, warning counts, and pass/fail status.
6. Stage results for explicit apply.

## Non-Goals

1. Do not persist results yet.
2. Do not reintroduce sensor-shell compatibility requirements.
3. Do not make AEAT magnet HI/LO warning status a hard fault.
4. Do not estimate hybrid-stepper pole-pair ratio during normal runtime; the configured value is
   authoritative.
5. Do not add blocking or kernel calls to zero-latency ISR paths.

## Phase 1: Simplify Runtime ALIGN State Machine

### Scope

Remove `ALIGN_NEG_INJECT` and `ALIGN_NEG_SAMPLE` from the normal boot calibration sequence.

### Changes

1. Replace the ALIGN path with:
   - `ALIGN`
   - `ALIGN_POS_INJECT`
   - `ALIGN_POS_SAMPLE`
   - finalize
2. Keep generated-angle commutation enabled during sample collection.
3. Keep encoder read enabled only for the sample window.
4. Ensure current injection and current removal are trajectory-ramped through `traj_Id`.
5. Make `ALIGN_POS_SAMPLE` apply offset directly after a valid sample window.
6. If the encoder sample window fails, fall back only when explicitly safe:
   - If encoder mode was requested, fail calibration with a clear error.
   - If generated/open-loop mode was requested, allow fallback to generated operation.

### Files

- `app/include/motor_states.h`
- `app/include/motor_states_calibration.h`
- `app/include/config.h`
- `app/src/motor_states.c`
- `app/src/motor_states_calibration.c`
- `app/src/motor_control_loop.c`
- `app/src/shell_commands_state.c`

### Acceptance

1. Runtime state list no longer exposes negative ALIGN states.
2. Calibration no longer applies negative `Id` during normal ALIGN.
3. `motor state status` and logs show a simple one-vector ALIGN.
4. HIL: `motor state offline` completes alignment without dual-polarity warnings.

## Phase 2: Clean `motor_core` Alignment API

### Scope

Convert `motor_core` alignment support from "dual-polarity-first" to "single-vector runtime,
sweep-detect commissioning".

### Changes

1. Rename `motor_align_fallback_offset_from_mech()` to a positive, primary API name:
   - `motor_align_offset_from_mech_sample()`
2. Keep circular mean accumulator helpers for runtime sample windows.
3. Remove `motor_align_compute_dual_polarity()` and `motor_align_resolve_offset()` from the public
   runtime API.
4. If dual-polarity math is retained temporarily, move it behind a diagnostic-only/private path.
5. Keep `motor_align_plan_id_traj()` as the only supported ALIGN injection planner.

### Files

- `modules/motor_core/include/motor/calibration/align.h`
- `modules/motor_core/src/calibration/align.c`
- `tests/unit/motor_align/src/main.c`

### Acceptance

1. Public `align.h` describes single-vector runtime alignment.
2. Unit tests cover:
   - empty sample rejection
   - circular mean across wrap
   - single-vector offset calculation
   - trajectory-ramped injection planning
3. Unit tests no longer encode dual-polarity as required runtime behavior.

## Phase 3: Add VESC-Style Encoder Mapping Commissioning Module

### Scope

Add a commissioning algorithm that uses generated electrical phase sweeps to validate encoder
mapping.

### Proposed `motor_core` API

```c
struct motor_encoder_map_detect_config {
	float32_t pole_pairs;
	float32_t min_mech_motion_rad;
	float32_t max_offset_residual_rad;
	float32_t max_direction_residual_rad;
	bool estimate_ratio;
};

struct motor_encoder_map_detect_sample {
	float32_t generated_elec_rad;
	float32_t encoder_mech_rad;
	uint32_t flags;
};

struct motor_encoder_map_detect_result {
	bool valid;
	bool direction_valid;
	bool offset_valid;
	bool ratio_valid;
	int8_t direction_sign;
	float32_t offset_mech_rad;
	float32_t offset_elec_rad;
	float32_t ratio;
	float32_t direction_corr;
	float32_t offset_residual_rad;
	float32_t mech_motion_rad;
	uint32_t sample_count;
	uint32_t rejected_samples;
	uint32_t encoder_error_count;
	uint32_t encoder_warning_count;
};
```

### Algorithm

1. During sweep, sample generated electrical phase and encoder mechanical angle.
2. Unwrap encoder mechanical angle.
3. Determine direction from correlation between generated electrical motion and encoder motion.
4. Convert encoder mechanical angle to electrical angle using configured pole pairs and candidate
   direction.
5. Estimate offset by circular averaging:
   - `offset_elec = mean(generated_elec - direction * pole_pairs * encoder_mech)`
6. Compute residual error over the sweep.
7. Reject result if motion, residual, transport errors, or sample count are outside thresholds.
8. For known hybrid stepper configuration, validate ratio against configured pole pairs instead of
   changing it automatically.

### Files

- `modules/motor_core/include/motor/calibration/encoder_map_detect.h`
- `modules/motor_core/src/calibration/encoder_map_detect.c`
- `modules/motor_core/src/CMakeLists.txt`
- `tests/unit/motor_encoder_map_detect/`

### Acceptance

1. Algorithm is pure library code with no Zephyr kernel dependency.
2. Tests cover:
   - positive direction
   - negative direction
   - offset near wrap
   - insufficient motion rejection
   - noisy samples
   - encoder error sample rejection
   - configured pole-pair validation

## Phase 4: Add App Commissioning Command

### Scope

Expose the encoder mapping detection as a manual shell commissioning flow.

### Command Shape

Recommended hierarchy:

```text
motor commission encoder run <current_a> <mech_hz> <cycles>
motor commission encoder status
motor commission encoder apply
motor commission encoder clear
```

Alternative if keeping all mapping checks grouped:

```text
motor commission mapping encoder run <current_a> <mech_hz> <cycles>
```

### Runtime Behavior

1. Require control armed.
2. Require no active fault.
3. Use generated-angle mode for actuation.
4. Use `traj_Id` to ramp alignment/sweep current.
5. Sweep generated electrical phase slowly enough for the rotor to follow.
6. Capture samples through the existing ISR-safe telemetry/capture path.
7. Process/finalize in thread context, not in the ISR.
8. Stage detected direction/offset for explicit `apply`.

### Files

- `app/src/shell_motion_commission.c`
- `app/include/shell_commands_commission.h`
- `app/src/motor_states_commissioning.c` or existing commissioning runtime adapter
- `app/include/config.h`
- `modules/motor_core/src/runtime/commission_runtime.c` only if shared commissioning context is reused

### Acceptance

1. Command can run without using encoder feedback for commutation.
2. Command reports direction, offset, residual, sample count, errors, and warnings.
3. `apply` updates active runtime offset/direction only when result passes.
4. Failed validation leaves runtime parameters unchanged.

## Phase 5: HIL Validation

### Runtime ALIGN Smoke

Use AEAT hardware:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor state offline
motor state status
motor encoder pipeline
motor encoder fast
```

Expected:

1. No negative ALIGN state appears.
2. No dual-polarity warning appears.
3. Alignment completes or fails with a direct encoder/sample reason.
4. Parity/transport errors remain zero for a passing run.

### Generated Motion Regression

```text
motor arm
motor state mode velocity_generated
motor current id 0
motor current iq 0.15
motor velocity target 5
motor encoder capture clear
motor encoder capture start 1
```

Expected:

1. Generated/open-loop motion still works.
2. Encoder capture still records motion.
3. No overcurrent regression from ALIGN cleanup.

### Encoder Mapping Detect

```text
motor commission encoder run 0.10 0.25 2
motor commission encoder status
```

Expected:

1. Direction sign agrees with generated-motion capture.
2. Offset residual is bounded.
3. Warning status is reported separately from parity/transport errors.
4. Result is staged, not automatically applied.

## Phase 6: Documentation Cleanup

### Changes

1. Update `app/docs/motor_align_motor_core_migration_plan.md` with a historical note that
   dual-polarity runtime ALIGN was superseded.
2. Update HIL workflow docs with the new runtime ALIGN and encoder mapping commands.
3. Record the Arduino-FOC/VESC design comparison in the plan or commissioning docs.

### Files

- `app/docs/motor_align_motor_core_migration_plan.md`
- `app/docs/isr_measurement_workflow.md`
- `AGENTS.md` if shell workflows change

## Risks

1. Single-vector alignment can still produce a poor offset if rotor does not move to the commanded
   field.
   - Mitigation: fail encoder-based modes on missing/invalid sample quality; use commissioning
     sweep for validation.
2. Encoder warnings from AEAT magnet status may obscure real failures.
   - Mitigation: report warning counts separately; only parity/transport errors are hard errors.
3. Generated sweep may slip if current is too low.
   - Mitigation: report insufficient motion/residual and require a higher commissioning current.
4. Removing negative ALIGN states may break shell/status expectations.
   - Mitigation: no backward compatibility required; update status output and docs.

## Definition of Done

1. Runtime ALIGN is single-vector and trajectory-ramped.
2. Runtime ALIGN no longer uses negative `Id` or dual-polarity validation.
3. `motor_core` public alignment API no longer presents dual-polarity as the normal runtime path.
4. Encoder mapping validation exists as an explicit generated-sweep commissioning command.
5. Unit tests cover alignment and encoder-map detection algorithms.
6. Firmware builds with AEAT overlay.
7. HIL confirms generated/open-loop motion still works and runtime ALIGN no longer emits
   dual-polarity mismatch logs.

## Implementation Notes

Implemented on 2026-05-03.

Runtime ALIGN:

- Boot calibration now runs `ALIGN -> ALIGN_POS_INJECT -> ALIGN_POS_SAMPLE -> ONLINE`.
- `ALIGN_POS_INJECT` plans the alignment current with `motor_align_plan_id_traj()` and ramps
  through `traj_Id`.
- `ALIGN_POS_SAMPLE` computes a circular mean from the encoder sample window and applies
  `motor_align_offset_from_mech_sample()`.
- The negative-current ALIGN states were removed from the runtime state list and shell state
  names.

Encoder mapping commissioning:

- Added `motor_encoder_map_detect_compute()` as pure `motor_core` library code.
- Added shell commands:

```text
motor commission encoder run <current_a> <mech_hz> <cycles>
motor commission encoder status
motor commission encoder apply
motor commission encoder clear
```

- The command uses generated-angle velocity actuation and temporarily enables raw encoder trace
  telemetry so encoder samples are collected without making the encoder part of the commutation
  policy.
- Warning-only status is counted separately. Missing samples, I/O faults, parity/transport/frame
  errors, insufficient motion, and excessive residual reject the staged result.

Validation evidence:

- `chopper.motor_align.unit` passed on `native_sim`.
- `chopper.motor_encoder_map_detect.unit` passed on `native_sim`.
- Firmware build passed for `smartstepper_v2_hil_shell`.
- HIL runtime ALIGN completed with the simplified single-vector path and no dual-polarity warning.
- HIL generated-sweep command collected raw encoder samples. A low-speed 0.25 Hz sweep had
  insufficient mechanical motion; a 5 Hz sweep collected motion but failed residual/correlation
  quality gates, which is expected behavior for a commissioning check when the encoder/motion
  relationship is not yet reliable.
- Follow-up HIL on 2026-05-03 showed the AEAT encoder stream contains parity-clean but implausible
  angle jumps. The encoder pipeline now counts and rejects those as `glitch` frame errors before
  the observer consumes them. Example evidence from one 0.8 s sweep after adding the gate:
  `parity=157`, `glitch=831`, and the clean raw trace span was only `-23.040 deg`, so encoder
  signal/data integrity remains the blocker for encoder-based commutation.
