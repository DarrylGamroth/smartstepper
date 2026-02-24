# `motor_control_loop_step` Phase 3 Execution

Date: 2026-02-24

## Phase 3 Goal

Extract outer-loop cascade logic and current-reference arbitration out of `motor_control_loop_step` into dedicated modules while preserving behavior and ISR constraints.

## Implemented

1. Added outer-loop module:
   - `app/include/motor_control_outer_loops.h`
   - `app/src/motor_control_outer_loops.c`
2. Added current-reference policy module:
   - `app/include/motor_current_ref_policy.h`
   - `app/src/motor_current_ref_policy.c`
3. Added shared quality helper:
   - `app/include/motor_control_quality.h`
4. Wired modules into control loop:
   - `app/src/motor_control_loop.c` now calls:
     - `motor_control_outer_loops_step(...)`
     - `motor_current_ref_apply_policy(...)`
5. Updated build sources:
   - `app/CMakeLists.txt`

## Behavior Notes

1. Position/velocity cascade behavior (PI/MPR, decimation, trajectory handoff, DOB feedforward) now lives in `motor_control_outer_loops.c`.
2. Commanded-current override and arm/disarm neutralization now live in `motor_current_ref_policy.c`.
3. Decoupling decision remains in `motor_control_loop.c` near FOC callsite (as intended for Phase 3).

## Validation

1. Build passed:
   - `/workspace/build/chopper/smartstepper_v2`
   - `/workspace/build/chopper/smartstepper_v2_mt6835`

## Next Candidate (Phase 4)

Promote the generic parts of these modules into `modules/motor_core` with unit tests (`native_sim`) and keep app-specific policy wiring in `app`.
