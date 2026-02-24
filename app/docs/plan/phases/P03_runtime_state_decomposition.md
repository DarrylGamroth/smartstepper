# Phase P03 - Runtime State Decomposition

## Objective

Split hot ISR runtime state from slow diagnostic/commissioning state.

## Prerequisites

1. P02 complete.

## Touch Files

1. runtime headers under `modules/motor_core/include/motor/runtime/*`
2. `app/include/config.h`
3. `app/src/motor_control_loop.c`
4. dependent tests

## Do Not Touch

1. Driver protocol implementations.

## Tasks

1. Introduce `motor_rt_fast_state` and `motor_rt_diag_state`.
2. Move non-hot fields out of fast path.
3. Update call sites to use fast-state writes for hot loop.

## Acceptance

1. Reduced hot-struct size tracked in log.
2. ISR stack does not increase.
3. Build + unit tests pass.

