# Motor ALIGN Migration Plan (`motor_core`)

Date: 2026-03-04  
Scope: Move ALIGN algorithms from app state-machine implementation into `modules/motor_core`, while preserving real-time behavior and simplifying app-side calibration states.

## Objective

1. Relocate alignment math/logic into reusable `motor_core` modules.
2. Keep app state machine focused on sequencing and policy only.
3. Require trajectory-driven d-axis injection during ALIGN (no direct step change).
4. Use `arm_sin_cos_f32` for dual-polarity circular mean accumulation.

## Non-Negotiable Constraints

1. No dynamic allocation.
2. No blocking in ISR paths.
3. Keep control-loop timing deterministic.
4. Dual-polarity ALIGN trig path must use `arm_sin_cos_f32` (not `sinf`/`cosf` pairs for accumulation).
5. ALIGN current injection must be generated through `traj` (`traj_Id`) ramping.
6. Backward compatibility is not required for this refactor; existing APIs may be changed to improve module boundaries and ISR correctness.

## Current Baseline (Before Refactor)

1. ALIGN state sequencing and algorithm details live in `app/src/motor_states_calibration.c`.
2. ALIGN sample accumulation currently occurs from ISR path in `modules/motor_core/src/runtime/motor_core_step.c`.
3. Dual-polarity algorithm computes circular means and offset validation in app layer.
4. `traj_Id` is already used for ALIGN injection target ramping in app code.

## Target Architecture

Create a dedicated alignment module family in `motor_core`:

1. `modules/motor_core/include/motor/calibration/align.h`
2. `modules/motor_core/src/calibration/align.c`
3. Optional helper split if needed:
   - `modules/motor_core/include/motor/calibration/align_dual_polarity.h`
   - `modules/motor_core/src/calibration/align_dual_polarity.c`

App layer keeps only:

1. State transitions/timers (`ALIGN_*` states).
2. Feature flag enable/disable for ISR behavior.
3. Final state transition decision (IDLE/ONLINE mode selection).
4. Error posting policy.

## Proposed `motor_core` ALIGN API

```c
struct motor_align_config {
    float32_t pole_pairs;
    float32_t opposed_elec_tol_rad;
    uint16_t min_fresh_samples;
    uint8_t max_sample_retries;
};

struct motor_align_sample_accum {
    float32_t sum_sin;
    float32_t sum_cos;
    uint16_t count;
};

struct motor_align_dual_result {
    bool valid;
    bool fallback_recommended;
    float32_t pos_mech_rad;
    float32_t neg_mech_rad;
    float32_t final_offset_rad;
    float32_t measured_delta_mech_rad;
    float32_t expected_delta_mech_rad;
};

void motor_align_accum_reset(struct motor_align_sample_accum *acc);
void motor_align_accum_push(struct motor_align_sample_accum *acc, float32_t mech_angle_rad);
bool motor_align_circular_mean(const struct motor_align_sample_accum *acc, float32_t *mean_rad);

bool motor_align_compute_dual_polarity(const struct motor_align_config *cfg,
                                       const struct motor_align_sample_accum *pos,
                                       const struct motor_align_sample_accum *neg,
                                       struct motor_align_dual_result *out);
```

Notes:

1. `motor_align_accum_push()` must use `arm_sin_cos_f32` for sin/cos accumulation.
2. Wrapping helpers should continue to use shared wrap utilities from `motor/math`.

## Execution Phases

## Phase 0: Guardrails and API Skeleton

1. Add `align.h` public interface and private implementation skeleton in `motor_core`.
2. Add unit test target scaffold under `tests/unit` for alignment module.
3. Keep app behavior unchanged.

Acceptance:

1. Build passes.
2. New tests compile and run (even if minimal in this phase).

## Phase 1: Move Dual-Polarity Math into `motor_core`

1. Move circular mean calculation and dual-polarity offset solve from `app/src/motor_states_calibration.c` into new `motor_core` alignment module.
2. Replace all accumulation trig in moved logic with `arm_sin_cos_f32`.
3. Keep fallback policy in app, but consume `motor_core` result object.

Acceptance:

1. App ALIGN states call `motor_core` APIs for mean/dual solve.
2. No algorithmic regressions in existing ALIGN behavior.

## Phase 2: Integrate ALIGN Accumulator Path Through `motor_core`

1. Replace direct app accumulator field mutations with module helper calls.
2. In ISR sample-accumulation path, call `motor_align_accum_push()`.
3. Keep sample gating (fresh/no warning/no error) unchanged.

Acceptance:

1. Accumulator writes no longer duplicate sin/cos logic in app/runtime code.
2. ALIGN sample counts/means remain consistent with baseline behavior.

## Phase 3: Enforce Trajectory-Based Injection Contract

1. Ensure ALIGN inject entry paths only set `traj_Id` target/ramp (no direct `Idq_ref` jumps).
2. Keep current helper (`motor_align_set_injection_target`) or move equivalent policy into `motor_core` as an injection planner helper.
3. Document this as invariant in code comments and test assertions.

Acceptance:

1. ALIGN +Id/-Id injections are always ramped via `traj_Id`.
2. No hard step current transitions introduced by refactor.

## Phase 4: App State Simplification

1. Reduce `app/src/motor_states_calibration.c` ALIGN sections to state orchestration and policy.
2. Remove duplicated local ALIGN math helpers replaced by `motor_core` APIs.
3. Keep logging and transition semantics clear and deterministic.

Acceptance:

1. ALIGN code in app is sequencing-oriented and significantly smaller.
2. Module boundaries are clear: app = state orchestration, core = algorithm/math.

## Validation Plan

## Build

1. `podman exec priceless_wiles bash -lc 'cmake --build /workspace/build/chopper/smartstepper_v2 -j4'`

## Unit tests

1. `./tests/run_unit_tests.sh`
2. Add alignment-specific unit tests covering:
   - circular mean validity/degenerate vectors
   - dual-polarity expected separation validation
   - offset circular averaging robustness near wrap boundaries

## HIL smoke checks

1. `motor state offline`
2. Verify ALIGN completes and transitions without hard fault.
3. `motor state mode velocity_open`, `motor current iq 0.15`, `motor velocity target 5`
4. Confirm no regression in movement startup after ALIGN.

## Files Expected to Change

1. `modules/motor_core/include/motor/calibration/align.h` (new)
2. `modules/motor_core/src/calibration/align.c` (new)
3. `modules/motor_core/CMakeLists.txt`
4. `modules/motor_core/src/runtime/motor_core_step.c`
5. `app/src/motor_states_calibration.c`
6. `app/include/motor_states_calibration.h` (only if API touch needed)
7. `tests/unit/*` (new alignment tests)

## Risks and Mitigations

1. Risk: behavior drift in fallback conditions.
   - Mitigation: keep fallback decision in app until phase 4 and compare logs against baseline.
2. Risk: ISR overhead increase from abstraction.
   - Mitigation: keep helpers `static inline` where hot; use simple POD structs.
3. Risk: hidden direct Id steps reintroduced.
   - Mitigation: explicit invariant checks in review/tests that ALIGN injection uses `traj` only.

## Definition of Done

1. ALIGN algorithmic math lives in `motor_core`.
2. Dual-polarity accumulation path uses `arm_sin_cos_f32`.
3. ALIGN injection is trajectory-ramped via `traj_Id`.
4. App ALIGN code is reduced to state sequencing/policy.
5. Build + unit tests pass; HIL ALIGN and velocity_open smoke pass.
