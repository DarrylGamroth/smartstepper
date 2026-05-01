# TI-Style ISR Latency Reduction Plan

## Goal

Make the motor-control ISR path closer to TI MotorControl SDK style: deterministic, flat, low-stack, and composed from small modules with explicit init/config and cheap `run/step` calls.

This work should happen before returning to the broader Motion vs Actuator Decoupling plan. The motion/actuator abstraction is useful, but the current ISR implementation is still too struct-heavy and stack-heavy to be the foundation for additional backend work.

## Current Evidence

Disassembly from the current `smartstepper_v2` build shows:

| Symbol | Code size | Stack frame observed | Notes |
| --- | ---: | ---: | --- |
| `motor_control_loop_step` | `0x254` / 596 B | `452 B` local + saved regs/FPU | Top-level frame remains live across child calls. |
| `motor_control_step_reference_stage` | `0x290` / 656 B | `148 B` local + saved regs | Builds outer-loop and current-policy input/output structs. |
| `motor_control_step_foc_stage` | `0x354` / 852 B | `156 B` local + saved regs | Builds FOC input/output structs and performs decoupling gating. |
| `motor_foc_voltage_pwm_step_fast` | `0x158` / 344 B | `156 B` local + saved regs/FPU | Still decomposes into local decoupling/current/PWM structs. |
| `motor_outer_loop_runtime_step` | `0x50a` / 1290 B | `80 B` local + saved regs/FPU | Largest hot-path control module; combines position, velocity, MPR/PI fallback, and DOB. |
| `motor_control_step_read_encoder` | `0x182` / 386 B | `96 B` local + saved regs | Copies encoder context to stack. |

Approximate worst nested stack for the common closed-loop FOC path is `motor_control_loop_step` plus reference stage plus `motor_outer_loop_runtime_step`, then FOC stage plus `motor_foc_voltage_pwm_step_fast`, not counting deeper helpers. This is acceptable on the current Cortex-M7 stack budget if measured timing passes, but it is not the style we want for a reusable real-time motor library.

## Design Rules

1. Keep ISR code deterministic and bounded.
2. Prefer persistent runtime structs over large per-ISR stack structs.
3. Prefer small `static inline` or small C functions for tiny math/control modules.
4. Keep full validation in init/config/update paths.
5. Keep ISR runtime checks limited to catastrophic runtime conditions: null only at public boundary, invalid bus voltage, obvious non-finite if `CONFIG_MOTOR_ISR_SANITY_CHECKS` is enabled, fault thresholds, and stale/invalid feedback gates.
6. Do not allocate dynamically.
7. Do not call shell/logging/message-queue APIs from ISR.
8. Keep telemetry opt-in, bounded, and decimated.
9. Do not preserve backwards compatibility for internal app/runtime APIs if simplification requires changing them.
10. Validate each phase with unit tests, firmware build, and stack/code-size evidence.

## Target ISR Shape

The ISR should eventually look like a small app-owned orchestration shell around persistent runtime state:

```c
collect_adc_and_encoder(&rt->sample);
feedback_step_fast(&rt->feedback, &rt->sample);
measure_step_fast(&rt->meas, adc_values, &rt->feedback);
motion_step_fast(&rt->motion, &rt->feedback);
servo_step_fast(&rt->effort, &rt->motion, &rt->feedback);
backend_step_fast(&rt->pwm, &rt->effort, &rt->feedback);
telemetry_step_fast(&rt->telemetry, optional_flags);
```

The exact function names can differ, but the structure should avoid repeated construction of large local input/output structs.

## Runtime Context Target

Create a dedicated ISR context owned by app runtime storage, separate from the broad `struct motor_parameters`:

```c
struct motor_rt_control_ctx {
    struct motor_rt_mode_snapshot mode;
    struct motor_rt_sample sample;
    struct motor_rt_feedback feedback;
    struct motor_rt_measurements meas;
    struct motor_motion_ref motion;
    struct motor_current_ref current;
    struct motor_angle_ref angle;
    struct motor_commutation_ref commutation;
    struct motor_pwm_ref pwm;
    struct motor_commission_observation commission_obs;
    struct motor_rls_runtime_state rls_runtime;
    struct motor_control_step_report report_scratch;
};
```

Rules:

1. The context may live inside `motor_parameters` temporarily, but the ISR stages should consume `motor_rt_control_ctx *` instead of broad `motor_parameters *` where practical.
2. Persistent scratch must be overwritten field-by-field, not blindly zeroed every ISR unless the struct is small.
3. State/config pointers remain owned by `motor_parameters` or runtime adapter structs; the ISR context should not duplicate long-lived controller state.

## Phase 0: Baseline And Guardrails

Purpose: capture current evidence before making latency changes.

Tasks:

1. Add a repeatable command note for collecting symbol sizes and relevant disassembly snippets.
2. Record current code-size/stack evidence in this document or `app/docs/plan/execution_log.md`.
3. If practical, enable or document stack-usage file generation for the app build.
4. Run current unit tests and firmware build before changing ISR structure.

Acceptance:

1. Current symbol sizes for `motor_control_loop_step`, reference stage, FOC stage, FOC fast step, and outer-loop runtime are recorded.
2. Current firmware build passes.
3. Full unit tests pass.

## Phase 1: Persistent ISR Scratch Context

Purpose: remove the largest top-level stack objects from `motor_control_loop_step`.

Tasks:

1. Add a persistent app-owned ISR scratch/context struct.
2. Move the following top-level locals into persistent context:
   - `motor_control_measurements`
   - `motor_motion_ref`
   - `motor_feedback_ref`
   - `motor_angle_ref`
   - `motor_current_ref`
   - `motor_commutation_ref`
   - `motor_encoder_stage_result`
   - `motor_rls_runtime_state`
   - `motor_commission_observation`
3. Update stages to take a context pointer rather than separate local pointers where this simplifies call signatures.
4. Keep behavior unchanged.

Acceptance:

1. `motor_control_loop_step` stack frame is reduced materially from the current `452 B` local frame.
2. Firmware build passes.
3. Full unit tests pass.
4. No new ISR shell/log/queue dependencies are introduced.

## Phase 2: Flatten FOC Backend Fast Path

Purpose: stop constructing nested FOC/decoupling/current/PWM structs in the ISR path.

Tasks:

1. Keep `motor_foc_voltage_pwm_step()` as the defensive/test-friendly API.
2. Replace ISR use of `motor_foc_voltage_pwm_step_fast()` with a flatter FOC-current backend step that reads from `motor_rt_control_ctx` and writes directly to `motor_commutation_ref`/PWM output.
3. Convert decoupling input/output and PWM synthesis input/output into direct scalar or compact ref calls on the ISR path.
4. Keep the existing composable functions for unit tests and non-ISR usage if useful.
5. Ensure decoupling is still gated before applying feedforward.

Acceptance:

1. `motor_control_step_foc_stage` plus FOC fast path stack usage is reduced.
2. FOC unit tests still cover the composable API.
3. A new runtime/backend test covers ISR fast-path equivalence for a representative FOC case.
4. Firmware build passes.

## Phase 3: Split Outer Loop Runtime Into Small Modules

Purpose: reduce `motor_outer_loop_runtime_step()` size and make position/velocity/DOB execution explicit.

Tasks:

1. Split current `motor_outer_loop_runtime_step()` into small steps:
   - position profile/position regulator step,
   - velocity trajectory step,
   - velocity regulator/MPR step,
   - DOB feedforward step.
2. Keep MPR/DOB init/config outside the per-update fast calls.
3. Avoid local input/output struct construction for each substep where scalar/ref APIs are cleaner.
4. Preserve decimation behavior.

Acceptance:

1. `motor_outer_loop_runtime_step` is either removed or becomes a thin orchestrator.
2. Code-size hot spot is split into bounded modules with smaller stack frames.
3. Existing outer-loop behavior tests pass.
4. Full unit tests pass.

## Phase 4: Remove Encoder Context Copying

Purpose: reduce encoder stage stack and copying.

Tasks:

1. Stop copying `motor_encoder_feedback_ctx` to stack each ISR unless there is a specific race being avoided.
2. Make encoder direction/config updates publish through an explicit small snapshot or persistent context.
3. Keep raw encoder telemetry as optional/deferred trace data.

Acceptance:

1. `motor_control_step_read_encoder` no longer copies the full encoder context to stack.
2. Encoder unit tests pass.
3. Velocity-open encoder capture workflow remains functional.

## Phase 5: Compile-Time ISR Feature Gating

Purpose: avoid carrying unused algorithm cost in builds that do not need it.

Tasks:

1. Add or audit Kconfig gates for optional ISR-path features:
   - MPR,
   - DOB,
   - RLS runtime update,
   - commissioning capture/update,
   - fault snapshot telemetry,
   - raw encoder trace.
2. Ensure disabled features compile out rather than branch in the 20 kHz path where practical.
3. Keep shell visibility honest when a feature is compiled out.

Acceptance:

1. A default production config has only required ISR features enabled.
2. Debug config can enable telemetry/commissioning features.
3. Build matrix or at least targeted builds validate enabled/disabled paths.

## Phase 6: Measurement And Regression Gates

Purpose: make ISR latency improvements measurable and prevent backsliding.

Tasks:

1. Add a documented disassembly/symbol-size check command.
2. Add a documented HIL command sequence to record ISR max/average cycles in `velocity_open`, `torque`, and `velocity_closed` if hardware is stable.
3. Record before/after numbers in `app/docs/plan/execution_log.md`.
4. Consider a lightweight source-level test that scans for forbidden ISR dependencies: shell, `k_msgq_put`, logging, allocation.

Acceptance:

1. ISR stack/code-size evidence is recorded after each major phase.
2. HIL ISR max-cycle evidence is recorded before returning to Motion vs Actuator Decoupling.
3. No real-time boundary regressions are found.

## Phase 7: Resume Motion vs Actuator Decoupling

Only resume actuator backend work after the ISR path is lean enough to be a stable base.

Expected state before resuming:

1. `motor_control_loop_step` uses persistent ISR scratch/state.
2. FOC current backend is implemented as one backend path, not a large nested struct pipeline.
3. Outer-loop regulators are split into smaller fast modules.
4. Optional telemetry/estimation features are gated or clearly bounded.
5. Measured ISR cycle data is available.

## Suggested Execution Order

1. Phase 0: record baseline.
2. Phase 1: persistent ISR scratch context.
3. Phase 2: flatten FOC backend fast path.
4. Phase 3: split outer-loop runtime.
5. Phase 4: simplify encoder context handling.
6. Phase 5: compile-time ISR feature gating.
7. Phase 6: measure and document results.
8. Return to Motion vs Actuator Decoupling.

## Non-Goals

1. Do not redesign the state machine in this plan.
2. Do not add new actuator backend kinds in this plan.
3. Do not change public shell command semantics unless required for feature gating visibility.
4. Do not remove existing tests; update them to cover the new smaller modules.
5. Do not optimize by hiding errors; catastrophic guardrails remain required.
