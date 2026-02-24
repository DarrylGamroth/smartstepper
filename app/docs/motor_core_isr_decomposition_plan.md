# `motor_core` ISR Decomposition Plan

Date: 2026-02-24  
Scope: `modules/motor_core`, `app/src/motor_control_loop.c`, `app/src/motor_isr_io.c`

## Objective

Decompose control logic into composable, ISR-safe modules with explicit hot-path contracts, while reducing stack and context bloat introduced by large step/context structs.

## Constraints

1. No dynamic allocation in ISR path.
2. No blocking or kernel APIs in direct ISR context.
3. Preserve current control-loop behavior and safety interlocks.
4. Keep encoder async pipeline (`RTIO`) semantics intact.
5. Maintain shell/state-machine compatibility during migration.

## External Reference Review

### C2000Ware patterns to adopt

1. Decimated loop scheduling embedded in controller object and run path (`CTRL_run`) with explicit counter/tick gating in one place (`../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/control/ctrl/include/ctrl.h:2036`).
2. Module-object + handle style with `init/run/reset` contracts and no hidden allocations (`../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/control/ctrl/source/ctrl.c:104`, `../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/utilities/traj/include/traj.h:73`).
3. High-rate execution timing instrumentation as first-class utility (`../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/utilities/cpu_time/include/cpu_time.h:118`).
4. Angle/speed helpers with single-purpose run functions and bounded state (`../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/utilities/angle_gen/include/angle_gen.h:74`, `../../motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries/observers/speedcalc/include/speedcalc.h:71`).

### MotorWare / SpinTAC patterns to adopt

1. Position stack split into explicit modules: convert, move, control (`../../motor/motorware/modules/spintac/src/32b/spintac_pos_conv.h:55`, `../../motor/motorware/modules/spintac/src/32b/spintac_pos_move.h:95`, `../../motor/motorware/modules/spintac/src/32b/spintac_vel_ctl.h:72`).
2. ISR integration via short orchestration helpers (`ST_runPosConv`, `ST_runPosCtl`) instead of one monolithic function (`../../motor/motorware/solutions/instaspin_motion/src/proj_lab13f.c:1306`).
3. Decimation and motion/profile logic controlled outside current loop, but fed as compact references into FOC ISR (`../../motor/motorware/solutions/instaspin_motion/src/proj_lab13f.c:573`).

## Current State Review (Chopper)

### Strengths

1. `motor_core` already contains many reusable algorithmic modules (`modules/motor_core/CMakeLists.txt:3`).
2. `adc_callback` already separates encoder request (`encoder1_callback`) from encoder drain/control (`adc_callback`) (`app/src/motor_isr_io.c:57`, `app/src/motor_isr_io.c:152`).
3. Phase 4 extracted reusable arbitration/scheduling helpers into `motor_core`.

### Issues to address

1. `motor_parameters` is monolithic and mixes hot ISR state, commissioning buffers, capture rings, and shell-facing telemetry (`app/include/config.h:104`).
2. `motor_control_loop_step` still has large local state and cross-cutting concerns (`app/src/motor_control_loop.c:273`).
3. `motor_control_step_ctx` includes heavy nested feedback (`struct motor_encoder_feedback`) that also carries capture/debug fields not always needed for control (`app/src/motor_control_loop.c:216`, `app/include/motor_encoder_feedback.h:21`).
4. ISR dataflow in `adc_callback` still mixes encoder drain policy, profile tick generation, control invocation, PWM commit, and timing stats in one function (`app/src/motor_isr_io.c:57`).
5. App wrappers pass multiple medium-size input/output structs per call in the hot path (`app/src/motor_control_loop.c:571`, `app/src/motor_control_loop.c:596`).

## Target Architecture

### Runtime partition

1. `motor_rt_fast_state` (hot): minimal fields needed every ISR tick.
2. `motor_rt_slow_state` (slow): commissioning, calibration captures, shell diagnostics, fault snapshots.
3. `motor_rt_cfg_snapshot` (read-only per tick): coherent state + feature + loop-period snapshot.

### ISR pipeline contract

`adc_callback` should become a thin orchestrator:

1. `motor_isr_begin()`
2. `motor_encoder_io_collect()`
3. `motor_trigger_source_step()`
4. `motor_core_step_fast()`
5. `motor_pwm_commit()`
6. `motor_isr_end()`

Each stage has a compact I/O struct and strict ownership.

### Module layering

1. `modules/motor_core/rt/*`: pure control math and state update.
2. `app/src/motor_isr_io.c`: hardware I/O glue only.
3. `app/src/motor_states*.c` and shell: mode/policy/config management only.

## Multi-Phase Plan

## Phase 0: Baseline and Guardrails

1. Add explicit baseline metrics for ISR cycle min/max/avg and stack watermark.
2. Add compile-time `sizeof` checks for hot structs.
3. Freeze behavior with regression tests for encoder quality transitions, torque/velocity closed-loop gating, and fault posting order.

Deliverables:
1. `app/tests` + `tests/unit` coverage for current behavior.
2. Baseline metrics document in `app/docs`.

Acceptance:
1. No functional change.
2. Baseline captured for both hardware profiles.

## Phase 1: `adc_callback` Structural Split

1. Split `adc_callback` into static helpers in `app/src/motor_isr_io.c`:
   - `adc_collect_encoder_sample()`
   - `adc_step_profile_trigger()`
   - `adc_run_control_step()`
   - `adc_apply_pwm()`
   - `adc_update_timing_stats()`
2. Keep exact behavior, just isolate concerns and ownership.

Deliverables:
1. Refactored `app/src/motor_isr_io.c` with no logic change.
2. Updated function-level comments and call graph doc.

Acceptance:
1. Bit-equivalent behavior in existing HIL scripts.
2. No ISR cycle regression beyond measurement noise.

## Phase 2: Hot/Cold State Separation

1. Introduce `struct motor_rt_fast_state` in `motor_core` for ISR-only mutable data.
2. Move large diagnostics/commissioning buffers out of hot struct into `motor_runtime_diag` owned by app layer.
3. Replace direct `params` writes in hot path with targeted writes to fast state and a compact telemetry mirror.

Deliverables:
1. New headers under `modules/motor_core/include/` for fast-state contracts.
2. `config.h` reduced hot-path footprint (same external behavior).

Acceptance:
1. `motor_parameters` field count materially reduced in hot region.
2. No stack growth in ISR.

## Phase 3: Context Shrink and Contract Cleanup

1. Split `motor_encoder_feedback` into:
   - `motor_encoder_feedback_control` (hot)
   - `motor_encoder_feedback_capture` (optional debug)
2. Refactor `motor_control_step_ctx` into:
   - immutable tick snapshot
   - compact step scratch
3. Remove duplicate local variables in `motor_control_loop_step`; use stage structs with clear ownership.

Deliverables:
1. New control-only feedback API in `motor_core`.
2. Capture/debug path moved behind explicit opt-in call.

Acceptance:
1. Reduced stack usage in `motor_control_loop_step`.
2. All phase-4 unit tests still pass.

## Phase 4: Compose Fast Control Pipeline in `motor_core`

1. Add `motor_core_step_fast(...)` API that consumes compact inputs and emits compact outputs.
2. Internally sequence:
   - sensor normalization
   - observer/position convert
   - outer-loop scheduling/arbitration
   - FOC voltage/PWM synthesis
3. Keep app-owned side effects (error posting, state transitions, event queueing) outside core.

Deliverables:
1. New pipeline API in `modules/motor_core`.
2. Adapter layer in app preserving existing shell/state behavior.

Acceptance:
1. `app/src/motor_control_loop.c` becomes orchestration + policy only.
2. Full unit suite + HIL smoke pass.

## Phase 5: Coherent Snapshot and Concurrency Hardening

1. Replace separate `state_for_isr` + `feature_flags` reads with coherent snapshot publish/consume.
2. Use a lock-free snapshot protocol (versioned double-buffer or seqlock style) between state thread and ISR.
3. Move profile trigger policy read into same snapshot.

Deliverables:
1. `motor_rt_cfg_snapshot` publish/consume API.
2. Unit tests for mixed-epoch prevention.

Acceptance:
1. No mixed state/feature epoch observed in stress tests.
2. Existing command/state transitions remain deterministic.

## Phase 6: Performance and Packaging

1. Split `motor_core` build into sub-libraries:
   - `motor_core_rt` (hard real-time)
   - `motor_core_motion`
   - `motor_core_estimation`
   - `motor_core_commission`
2. Keep only ISR-required objects linked into real-time path.
3. Add optional section-placement macros for hot functions.

Deliverables:
1. Updated `modules/motor_core/CMakeLists.txt` decomposition.
2. Link-map comparison report.

Acceptance:
1. Reduced text/data footprint for realtime target.
2. No regression in control-loop jitter.

## Proposed `adc_callback` End-State (Conceptual)

```c
void adc_callback(...) {
    motor_isr_begin(...);

    enc = adc_collect_encoder_sample(...);
    trig = adc_step_profile_trigger(...);

    motor_core_step_fast(&rt_cfg, &rt_fast, &enc, values, &step_out);

    adc_apply_pwm(&step_out.pwm);
    adc_publish_telemetry(&rt_fast, &step_out);

    motor_isr_end(...);
}
```

## Verification Plan Per Phase

1. Unit tests (`native_sim`) for each extracted contract.
2. Build validation for both targets:
   - `/workspace/build/chopper/smartstepper_v2`
   - `/workspace/build/chopper/smartstepper_v2_mt6835`
3. HIL regression:
   - open-loop velocity
   - torque closed-loop engage/disengage
   - velocity closed-loop engage
   - offline->align transitions
   - encoder transport/frame-error handling
4. ISR budget tracking before/after each phase.

## Immediate Next Step

Start with Phase 1, because it is low risk and creates a clean seam for Phases 2-4.
