# Phase P01 - ISR Stage Shell

## Objective

Restructure `adc_callback` into explicit stage boundaries: Collect, Process, Apply, Telemetry, with no behavior change.

## Prerequisites

1. P00 complete.

## Touch Files

1. `app/src/motor_isr_io.c`
2. `app/include/motor_isr.h` (if signatures/comments are updated)
3. related docs in `app/docs/plan/`

## Do Not Touch

1. Algorithm internals in `modules/motor_core/src/*`.

## Tasks

1. Introduce explicit stage-level code blocks or static helpers.
2. Keep exact encoder drain/request semantics.
3. Keep exact PWM apply and timing update semantics.

## Acceptance

1. No behavior delta in HIL smoke.
2. ISR cycle budget unchanged within noise.
3. Both firmware targets build.

