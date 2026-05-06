# `motor_control_loop_step` Refactor Plan

Date: 2026-02-24  
Primary target: `app/src/motor_control_loop.c` (`motor_control_loop_step`)

## Goals

1. Reduce complexity and branching in `motor_control_loop_step`.
2. Make encoder/angle handling deterministic and easier to reason about.
3. Move reusable, hardware-agnostic logic into `modules/motor_core`.
4. Preserve ISR timing and behavior (no dynamic allocation, no blocking).

## Current Pain Points

1. One function mixes too many responsibilities: source selection, observer/position conversion, calibration sampling, capture logging, outer loops, current control, safety fallback, and telemetry.
2. Encoder path uses multiple overlapping booleans (`enabled`, `available`, `fresh`, `warning`, `error`, `io_fault`) and mixed semantics between control and capture.
3. Degrees/radians and wrapped/unwrapped domains are repeatedly converted in different branches.
4. Control decisions and diagnostics/capture concerns are interleaved, making changes risky.

## Target Structure

Split `motor_control_loop_step` into a staged pipeline with explicit data contracts:

1. `motor_control_preflight()`
2. `motor_control_update_angle_feedback()`
3. `motor_control_update_position_velocity()`
4. `motor_control_run_outer_loops()`
5. `motor_control_resolve_current_refs()`
6. `motor_control_run_foc_and_pwm()`
7. `motor_control_publish_telemetry()`

Each stage consumes/produces a small context struct. No stage performs device I/O.

## Encoder Simplification Plan (Primary)

Define one normalized encoder feedback object per ISR:

```c
struct motor_encoder_feedback {
	bool control_enabled;
	bool sample_present;
	bool fresh;
	bool warn;
	bool err;
	bool io_fault;
	uint8_t status;
	uint8_t source;          /* GENERATED | ENCODER | PROPAGATED */
	float32_t sensor_mech_rad;   /* raw sensor mapped to control sign */
	float32_t observer_input_rad;
	float32_t mech_rad;          /* observer output */
	float32_t elec_rad;          /* observer output */
	float32_t mech_speed_rad_s;
	uint8_t quality_flags;       /* feedback quality (valid/fresh/error) */
};
```

Rules:

1. Convert units once at boundary (prefer radians in control path).
2. One source-of-truth for offset/wrap is the observer path.
3. Separate control quality from capture diagnostics:
   - Control uses normalized feedback fields only.
   - Capture uses a copy of normalized feedback plus optional generated-reference fields.
4. Consolidate encoder fault/warn counter updates into one helper.

## Library Extraction Candidates

Move to `modules/motor_core` after behavior is stabilized:

1. `motor_encoder_feedback.[ch]`
   - Normalizes encoder sample flags + source arbitration.
   - Applies observer input selection and delay policy.
   - Provides consistent fault/warn/error classification.
2. `motor_outer_loop_cascade.[ch]`
   - Position PI/MPR + velocity PI/MPR + decimation scheduler.
   - Produces `{Id_ref, Iq_ref, velocity_target, velocity_ref}`.
3. `motor_current_ref_policy.[ch]`
   - Arbitration for commanded-current mode, interlock neutralization, quality fallbacks.
4. `motor_control_telemetry.[ch]`
   - Snapshot packing for params/fault/capture outputs.

Keep app-side:

1. State machine transitions and mode policy wiring.
2. Error posting (`motor_api_post_error`) and app-specific fault latching.
3. Board/config-specific constants and shell-facing behavior.

## Execution Phases

### Phase 0: Guardrails

1. Add focused tests around current behavior:
   - Encoder source transitions (generated -> encoder -> propagated).
   - Feedback quality gating for torque/velocity closed loops.
   - Current-ref arbitration under disarm/timeout.
2. Capture baseline ISR timing counters before refactor.

Acceptance:

1. Existing HIL flows still pass.
2. No increase in fault regressions during ALIGN/PREPARE_ONLINE/ONLINE transitions.

### Phase 1: In-file Structural Split

1. Introduce local `motor_control_step_ctx` with typed fields.
2. Extract static helpers in same file for each stage.
3. No behavior changes except explicit bug fixes already agreed.

Acceptance:

1. Bit-for-bit equivalent outputs in unit tests for representative fixtures.
2. ISR cycle budget within measurement noise.

### Phase 2: Encoder Path Extraction (App-local first)

1. Create `app/src/motor_encoder_feedback.c` + header.
2. Move all encoder source selection, observer handoff, position-convert update, and encoder counter logic there.
3. Keep capture logging as a separate helper consuming normalized feedback.

Acceptance:

1. `motor_control_loop_step` no longer contains raw encoder-branch tree.
2. Encoder debug shell outputs unchanged in meaning.

### Phase 3: Outer-loop/Arbitration Extraction

1. Move position/velocity controller scheduling and current-ref arbitration into dedicated helpers/modules.
2. Keep FOC call site minimal and explicit.

Acceptance:

1. Closed-loop modes match baseline behavior.
2. No new overcurrent events in existing bring-up sequences.

### Phase 4: Promote Reusable Modules to `motor_core`

1. Move generic encoder feedback and outer-loop code into `modules/motor_core`.
2. Add unit tests under `tests/unit` for:
   - Encoder feedback classification and source arbitration.
   - Decimation and cascade scheduling.
   - Current-ref arbitration edge cases.

Acceptance:

1. App layer only provides policy/config and consumes module outputs.
2. All new module tests pass on `native_sim`.

### Phase 5: Cleanup and Documentation

1. Remove duplicated legacy fields and dead branches.
2. Document data-flow and signal domains (mech/elec, wrapped/unwrapped, deg/rad).

Acceptance:

1. `motor_control_loop_step` reduced to high-level orchestration.
2. Encoder handling code path is single-entry, single-exit, and test-covered.

## Performance Constraints

1. No heap allocation or blocking calls in control path.
2. Avoid extra trigonometric calls beyond current behavior.
3. Prefer precomputed constants and stack structs.
4. Keep branch count and memory traffic bounded; verify with ISR cycle stats after each phase.

## Suggested First Slice

1. Execute Phase 1 + Phase 2 only.
2. Start by extracting `motor_control_update_angle_feedback()` and `motor_control_update_position_velocity()`.
3. Validate with:
   - `motor info live`
   - `motor encoder capture compare`
   - torque/velocity_generated/velocity_encoder smoke tests on hardware.
