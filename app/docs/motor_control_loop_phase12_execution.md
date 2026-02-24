# `motor_control_loop_step` Phase 1+2 Execution Plan

Date: 2026-02-24

## Scope

Implement Phase 1 and Phase 2 from `app/docs/motor_control_loop_refactor_plan.md`:

1. Phase 1: in-file structural split with typed context and staged helpers.
2. Phase 2: extract encoder/observer/position-convert path into app-local module.

## Planned Tasks

1. Add local control-step context in `motor_control_loop.c`.
2. Route existing top-level state/feature/dt/decimation initialization through that context.
3. Create `motor_encoder_feedback.[ch]` for:
   - encoder sample normalization/counter updates
   - source arbitration (generated/encoder/propagated)
   - observer handoff/update
   - position-convert update and quality propagation
4. Replace inline encoder branch tree in `motor_control_loop_step` with the new helper.
5. Keep align/capture behavior wired through normalized feedback outputs.
6. Build both board targets and verify no compile regressions.

## Status

1. Completed: Added `struct motor_control_step_ctx` + `motor_control_step_ctx_init()` in `app/src/motor_control_loop.c`.
2. Completed: Added `app/include/motor_encoder_feedback.h`.
3. Completed: Added `app/src/motor_encoder_feedback.c`.
4. Completed: Integrated helper into `motor_control_loop_step` and removed duplicated encoder/observer/position-convert block.
5. Completed: Updated `app/CMakeLists.txt` to compile new source.
6. Completed: Built successfully:
   - `/workspace/build/chopper/smartstepper_v2`
   - `/workspace/build/chopper/smartstepper_v2_mt6835`

## Notes

1. This slice is intended as behavior-preserving refactor. Known, previously accepted bug fixes remain in place (e.g., braking uses current-cycle `Iq_ref_A`).
2. Full module promotion to `modules/motor_core` is intentionally deferred to later phases.
