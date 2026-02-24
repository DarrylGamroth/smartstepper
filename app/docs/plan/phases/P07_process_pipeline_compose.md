# Phase P07 - Process Pipeline Compose

## Objective

Compose `motor_core_step_fast(...)` as the Process-stage entry point over extracted modules.

## Prerequisites

1. P06 complete.

## Touch Files

1. runtime pipeline headers/sources
2. `app/src/motor_control_loop.c`
3. tests around pipeline integration

## Do Not Touch

1. Direct ISR callbacks other than integration wiring.

## Tasks

1. Implement `motor_core_step_fast(...)` over stage-local contracts.
2. Enforce side-effect boundaries (no queue/log/kernel/blocking).
3. Make app control step thin orchestration.

## Acceptance

1. Unit tests and builds pass.
2. HIL smoke pass.
3. Side-effect boundary documented and tested.

