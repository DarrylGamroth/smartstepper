# Phase P05 - Reference Path Split

## Objective

Decompose reference generation into explicit modules: align, motion, regulators, arbitration, interlocks.

## Prerequisites

1. P04 complete.

## Touch Files

1. `modules/motor_core/include/motor/control/*`
2. `modules/motor_core/include/motor/motion/*`
3. corresponding source files
4. `app/src/motor_control_outer_loops.c`
5. `app/src/motor_current_ref_policy.c`
6. `app/src/motor_control_loop.c`

## Do Not Touch

1. Low-level PWM hardware I/O glue.

## Tasks

1. Extract position regulator and velocity regulator modules.
2. Extract command arbitration and interlocks modules.
3. Keep regulation behavior and limits unchanged.

## Acceptance

1. Reference-path unit tests added/updated and passing.
2. Closed-loop behavior parity in HIL smoke.
3. Both firmware builds pass.

