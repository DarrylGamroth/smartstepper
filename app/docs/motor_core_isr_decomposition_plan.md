# `motor_core` ISR Decomposition Plan

Date: 2026-02-24  
Scope: `modules/motor_core`, `app/src/motor_control_loop.c`, `app/src/motor_isr_io.c`

## Objective

Decompose control logic into composable, ISR-safe modules with explicit hot-path contracts, while reducing stack and context bloat introduced by large step/context structs.

## Design Direction (TI-Style Modularization)

Adopt a strict module decomposition similar to C2000Ware/MotorWare: each algorithmic feature is an independent module with:

1. `config` (immutable or rarely changed),
2. `state` (runtime mutable),
3. `input`/`output` structs,
4. `init/reset/step` APIs,
5. no hidden global state and no dynamic allocation.

This moves us from “large control function with rich cross-module context” to “small composable modules with narrow contracts.”

## Proposed Library Organization (TI-Style)

Organize `motor_core` by function families (like TI libraries), not by call-site wrappers:

1. `modules/motor_core/include/motor/math/*`
2. `modules/motor_core/include/motor/filters/*`
3. `modules/motor_core/include/motor/observers/*`
4. `modules/motor_core/include/motor/motion/*`
5. `modules/motor_core/include/motor/control/*`
6. `modules/motor_core/include/motor/estimation/*`
7. `modules/motor_core/include/motor/runtime/*`

Source layout mirrors include layout:

1. `modules/motor_core/src/math/*`
2. `modules/motor_core/src/filters/*`
3. `modules/motor_core/src/observers/*`
4. `modules/motor_core/src/motion/*`
5. `modules/motor_core/src/control/*`
6. `modules/motor_core/src/estimation/*`
7. `modules/motor_core/src/runtime/*`

API conventions:

1. Algorithm modules (`filters`, `observers`, `motion`, `control`, `estimation`, `runtime` where stateful):
   - `*_config` struct
   - `*_state` struct
   - `*_input` struct
   - `*_output` struct
   - `*_init()`, `*_reset()`, `*_step()` functions
2. Utility modules (`math`, unit conversion, small stateless helpers):
   - header-only `static inline` or pure functions
   - no mandatory state/config wrappers

## Constraints

1. No dynamic allocation in ISR path.
2. No blocking or kernel APIs in direct ISR context.
3. Preserve current control-loop behavior and safety interlocks.
4. Keep encoder async pipeline (`RTIO`) semantics intact.
5. No requirement to preserve legacy file/include layout during migration.
6. Keep `Process` stage deterministic: fixed work per tick except compile-time gated diagnostics.

## Telemetry Gating Model

1. Compile-time gates:
   - `CONFIG_MOTOR_TELEM_LIVE`: compact live telemetry mirror in ISR.
   - `CONFIG_MOTOR_TELEM_ISR_DIAG`: optional ISR diagnostic captures/rings.
   - `CONFIG_MOTOR_TELEM_DEFERRED`: deferred telemetry pipeline using SPSC queue.
2. Runtime gates:
   - enable/disable capture families and decimation values from shell/config.
3. Default production profile:
   - live telemetry ON
   - ISR diagnostics OFF
   - deferred telemetry ON (for low-rate diagnostics)

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
4. ISR dataflow in `adc_callback` still mixes encoder drain gating, profile tick generation, control invocation, PWM commit, and timing stats in one function (`app/src/motor_isr_io.c:57`).
5. App wrappers pass multiple medium-size input/output structs per call in the hot path (`app/src/motor_control_loop.c:571`, `app/src/motor_control_loop.c:596`).

## Target Module Taxonomy

### Terminology Clarification

To avoid ambiguity:

1. Use `position regulator` instead of `outer-loop position`.
2. Use `velocity regulator` instead of `outer-loop velocity`.
3. Use `command arbitration` instead of generic `policy`.
4. Use `interlocks` for armed/disarmed and safety gating logic.
5. Reserve `policy` only for high-level state-machine behavior outside ISR math modules.

### A. Math/Foundation Modules

1. `motor_math_clamp` (shared clamp/saturation helpers)
2. `motor_math_wrap` (angle wrapping/unwrapping helpers)
3. `motor_math_units` (deg/rad, mech/elec conversions, sign application)

### B. Signal Conditioning Modules

1. `motor_adc_scale` (Q31 -> physical units)
2. `motor_current_offset` (offset removal/filtering)
3. `motor_bus_monitor` (vbus validity/limits)

### C. Sensor/Observer Modules

1. `motor_encoder_source` (source arbitration + frame/transport classification)
2. `motor_angle_track` (observer update + delay/handoff logic)
3. `motor_position_convert` (unwrap, velocity, accel, quality flags)
4. `motor_speed_filter` (optional notch/LPF)

### D. Reference Generation Modules

1. `motor_ref_align` (ALIGN/ROVERL/RS current references)
2. `motor_ref_motion` (traj + quintic + sequence progression)
3. `motor_ref_position_regulator` (position error -> velocity command)
4. `motor_ref_velocity_regulator` (velocity error -> torque/current command)
5. `motor_ref_command_arbitration` (command source selection and precedence)
6. `motor_ref_interlocks` (armed/disarmed and torque-enable gating in `Process`)

### E. Current Loop / FOC Modules

1. `motor_foc_transform` (Park/iPark/SVPWM prep and electrical frame handling)
2. `motor_current_ctrl` (Id/Iq PI and anti-windup/limit handling)
3. `motor_dq_decoupling` (cross-coupling/feedforward enable + validation)
4. `motor_pwm_synth` (duty computation / output packing)

### F. Protection/Fault Modules

1. `motor_fault_limits` (overcurrent/overvoltage/vbus validity checks)
2. `motor_fault_snapshot` (fault ring update)
3. `motor_error_post` (deferred error posting path outside direct ISR, fed by fault flags)

### G. Runtime Orchestration Modules

1. `motor_rt_cfg_snapshot` (coherent state+feature snapshot)
2. `motor_rt_fast_state` (hot runtime state only)
3. `motor_rt_diag_state` (slow/diagnostic/capture state)
4. `motor_step_pipeline` (ordered invocation of modules A-F)

## Existing Building Blocks (Keep and Rehome)

The following blocks are already strong and should be retained while relocating under the new taxonomy:

1. `angle_gen` -> `motor/motion/angle_gen`
2. `angle_observer` -> `motor/observers/angle_observer`
3. `motor_position_convert` -> `motor/observers/position_convert`
4. `motor_mpr` -> `motor/control/mpr`
5. `motor_dob` -> `motor/control/dob`
6. `motion_profile` + `motor_motion_modules` -> `motor/motion/profile`
7. `rs_online` + `rls_motor_est` + `thermal_model` + `motor_commission_id` -> `motor/estimation/*`
7. `motor_foc_voltage_pwm` -> split across `motor/control/current_ctrl`, `motor/control/foc_transform`, `motor/control/pwm_synth`

## Target Architecture

### Runtime partition

1. `motor_rt_fast_state` (hot): minimal fields needed every ISR tick.
2. `motor_rt_slow_state` (slow): commissioning, calibration captures, shell diagnostics, fault snapshots.
3. `motor_rt_cfg_snapshot` (read-only per tick): coherent state + feature + loop-period snapshot.

### ISR pipeline contract

`adc_callback` should become a thin orchestrator with exactly four stages:

1. `Collect`
2. `Process`
3. `Apply`
4. `Telemetry`

Stage names above are conceptual. Helper/function naming is implementation-defined.

### Stage contract details

1. `Collect`: read ADC, drain encoder pipeline, normalize input status, fetch coherent runtime snapshot, and advance trigger bookkeeping.
2. `Process`: run observer chain, reference chain, interlock gating, protection checks (flag-only), and current-loop chain, then produce compact actuator command and status outputs.
3. `Apply`: commit PWM output from command object and execute protection actions (fault latch/disable path) from `Process` fault flags.
4. `Telemetry`:
   - ISR-minimal: always-on compact live telemetry mirror and core timing counters.
   - Optional ISR diagnostics: compile-time + runtime gated captures only.
   - Deferred diagnostics: enqueue records to a consumer thread via SPSC queue.

Each stage owns its own input/output contract and must not depend on ad-hoc globals.

### Module layering

1. `modules/motor_core/rt/*`: pure control math and state update.
2. `app/src/motor_isr_io.c`: hardware I/O glue only.
3. `app/src/motor_states*.c` and shell: mode management, command/config publication, and diagnostics control.
4. Safety/interlock evaluation for torque-producing commands executes in `Process` for deterministic same-cycle gating.

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

## Phase 1: ISR Stage Shell (Collect/Process/Apply/Telemetry)

1. Split `adc_callback` into static helpers in `app/src/motor_isr_io.c`:
   - one helper (or inlined block) per stage boundary:
   - Collect
   - Process
   - Apply
   - Telemetry
2. Keep exact behavior, just isolate concerns and ownership.

Deliverables:
1. Refactored `app/src/motor_isr_io.c` with no logic change.
2. Updated stage contracts in docs and comments.

Acceptance:
1. Bit-equivalent behavior in existing HIL scripts.
2. No ISR cycle regression beyond measurement noise.

## Phase 2: Module Rehome Cutover (TI-style folders)

1. Create new folder taxonomy under `modules/motor_core/include/motor/*` and `src/*`.
2. Move headers/sources without behavior changes; update all call sites to new include paths directly.
3. Add per-family CMake grouping (`math`, `filters`, `observers`, `motion`, `control`, `protection`, `runtime`, `telemetry`).
4. Add include migration table (`old include` -> `new include`) in docs.

Deliverables:
1. New directory structure with stable build.
2. Migration table with owner and completion status.
3. All app/tests includes switched to new paths.

Acceptance:
1. Zero behavior changes.
2. All builds/tests green.
3. Zero references to legacy include paths in repo.

## Phase 3: Runtime State Decomposition

1. Introduce `struct motor_rt_fast_state` in `motor_core` for ISR-only mutable data.
2. Move large diagnostics/commissioning buffers out of hot struct into `motor_runtime_diag` owned by app layer.
3. Replace direct `params` writes in hot path with targeted writes to fast state and a compact telemetry mirror.

Deliverables:
1. New headers under `modules/motor_core/include/` for fast-state contracts.
2. `config.h` reduced hot-path footprint (same external behavior).

Acceptance:
1. `motor_parameters` field count materially reduced in hot region.
2. No stack growth in ISR.

## Phase 4: Sensor/Observer Module Split (C)

1. Split `motor_encoder_feedback` into independent modules:
   - `motor_encoder_source`
   - `motor_angle_track`
   - `motor_position_convert` (already present, narrow interface)
2. Move capture/debug fields out of control feedback object.
3. Refactor `motor_control_step_ctx` into:
   - immutable tick snapshot
   - compact step scratch.

Deliverables:
1. New control-only feedback API in `motor_core`.
2. Capture/debug path moved behind explicit opt-in call.

Acceptance:
1. Reduced stack usage in `motor_control_loop_step`.
2. All phase-4 unit tests still pass.

## Phase 5: Reference-Path Module Split (D)

1. Split reference generation into explicit modules:
   - `motor_ref_align`
   - `motor_ref_motion`
   - `motor_ref_position_regulator`
   - `motor_ref_velocity_regulator`
   - `motor_ref_command_arbitration`
   - `motor_ref_interlocks`
2. Replace multi-purpose reference adapters with narrow module APIs.
3. Add tests for each reference module independently.

Deliverables:
1. New reference modules in `modules/motor_core`.
2. State/shell integration paths updated to new module APIs.

Acceptance:
1. Reference-path logic is independent of ADC/PWM I/O.
2. Per-module unit tests cover edge cases and transitions.

## Phase 6: Current-Loop / FOC Module Split (E)

1. Isolate current control path into:
   - `motor_current_ctrl`
   - `motor_dq_decoupling`
   - `motor_foc_transform`
   - `motor_pwm_synth`
2. Keep module contracts fixed-size and scalar-heavy to reduce stack pressure.
3. Ensure each module can be invoked independently in ISR.

Deliverables:
1. New current-loop module APIs under `motor_core`.
2. Reference integration path uses module outputs only.

Acceptance:
1. Closed-loop torque/velocity behavior unchanged.
2. No ISR-time regression; reduced local stack in control step.

## Phase 7: Compose `motor_core_step_fast(...)` Pipeline (G)

1. Add one orchestrator API in `motor_core` that sequences modules A-F.
2. Keep app-owned side effects (state transitions and non-realtime event handling) outside core.
3. Use compact stage-local structs instead of one large cross-stage context.

Deliverables:
1. `motor_core_step_fast(...)` and associated contracts.
2. App control loop reduced to orchestration and mode/config boundary handling.
3. `motor_core_step_fast(...)` contract explicitly marks side-effect boundaries:
   - allowed: algorithm state updates
   - disallowed: queue ops, logging, kernel calls, blocking I/O

Acceptance:
1. `app/src/motor_control_loop.c` becomes thin orchestration.
2. Full unit suite + HIL smoke pass.

## Phase 8: Coherent Snapshot and Concurrency Hardening

1. Replace separate `state_for_isr` + `feature_flags` reads with coherent snapshot publish/consume.
2. Use a lock-free snapshot protocol (versioned double-buffer or seqlock style) between state thread and ISR.
3. Move profile trigger source/settings read into same snapshot.

Deliverables:
1. `motor_rt_cfg_snapshot` publish/consume API.
2. Unit tests for mixed-epoch prevention.

Acceptance:
1. No mixed state/feature epoch observed in stress tests.
2. Existing command/state transitions remain deterministic.

## Phase 9: Packaging and Link-Time Partitioning

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

## Phase 10: Optional RAM Placement and Micro-Optimizations

1. Add opt-in section-placement macros for hottest modules/functions.
2. Add per-stage cycle counters (`stage_min/max/avg`) similar to TI `cpu_time`.
3. Tune inlining and struct layout for deterministic stack/latency.

Deliverables:
1. Kconfig-controlled hot-section placement.
2. Stage-level timing visibility in shell telemetry.

Acceptance:
1. Verified deterministic jitter budget on hardware.
2. No functional regression.

## Proposed `adc_callback` End-State (Conceptual)

```c
void adc_callback(...) {
    collect = collect_stage(...);
    process = process_stage(&collect, ...);
    apply_stage(&process.actuator_cmd, ...);
    telemetry_stage(&collect, &process, ...);
}
```

## `motor_core_step_fast(...)` End-State (Conceptual)

```c
int motor_core_step_fast(const struct motor_rt_cfg_snapshot *cfg,
                         struct motor_rt_fast_state *fast,
                         const struct motor_collect_frame *in,
                         struct motor_process_frame *out);
```

`motor_core_step_fast(...)` only implements the `Process` stage.

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

Start with Phase 1 (ISR stage shell), then Phase 2 (module rehome skeleton) before any algorithmic movement.
