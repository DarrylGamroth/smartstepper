# `motor_control_loop_step` Phase 4 Execution

Date: 2026-02-24

## Phase 4 Goal

Promote reusable encoder feedback/source arbitration, outer-loop scheduling, and current-reference arbitration helpers into `modules/motor_core`, then add dedicated `native_sim` unit coverage.

## Implemented

1. Added reusable core modules in `modules/motor_core`:
   - `modules/motor_core/include/motor_encoder_feedback_core.h`
   - `modules/motor_core/src/motor_encoder_feedback_core.c`
   - `modules/motor_core/include/motor_outer_loop_sched.h`
   - `modules/motor_core/src/motor_outer_loop_sched.c`
   - `modules/motor_core/include/motor_current_ref_policy_core.h`
   - `modules/motor_core/src/motor_current_ref_policy_core.c`
2. Wired app-side wrappers to consume core modules:
   - `app/src/motor_encoder_feedback.c`
   - `app/src/motor_control_outer_loops.c`
   - `app/src/motor_current_ref_policy.c`
3. Updated motor_core library build inputs:
   - `modules/motor_core/CMakeLists.txt`
4. Added phase-4 unit suites:
   - `tests/unit/motor_encoder_feedback_core/*`
   - `tests/unit/motor_outer_loop_sched/*`
   - `tests/unit/motor_current_ref_policy_core/*`

## Behavior Notes

1. App layer retains policy/config wiring and side effects (state/mode implications, PI reset calls, trajectory/observer interactions).
2. Reusable core logic now has explicit module-level APIs for:
   - encoder source arbitration + counter/threshold classification,
   - decimation scheduler tick/update behavior,
   - current-reference policy arbitration decisions.
3. `motor_control_loop_step` remains orchestrator-centric, with reduced duplicated branch logic.

## Validation

1. Firmware build passed:
   - `/workspace/build/chopper/smartstepper_v2`
   - `/workspace/build/chopper/smartstepper_v2_mt6835`
2. New unit suites passed:
   - `chopper.motor_encoder_feedback_core.unit` (8/8)
   - `chopper.motor_outer_loop_sched.unit` (6/6)
   - `chopper.motor_current_ref_policy_core.unit` (8/8)

## Next Candidate (Phase 5)

1. Remove remaining duplicated legacy branch fragments in `motor_control_loop_step`.
2. Tighten docs around mech/elec and wrapped/unwrapped signal ownership.
3. Add one integration-level unit fixture that covers full stage handoff behavior across encoder quality transitions.
