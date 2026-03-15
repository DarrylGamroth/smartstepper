# Phase P07 - Process Pipeline Compose

## Objective

Compose the app-owned process stage in `app/src/motor_control_loop.c` over extracted `motor_core` modules.

## Prerequisites

1. P06 complete.

## Touch Files

1. `app/src/motor_control_loop.c`
2. runtime integration headers/sources
3. tests around pipeline integration

## Do Not Touch

1. Direct ISR callbacks other than integration wiring.

## Tasks

1. Implement the process stage in `app/src/motor_control_loop.c` over stage-local contracts.
2. Enforce side-effect boundaries (no queue/log/kernel/blocking).
3. Make `motor_core` provide reusable submodules rather than a monolithic fast-step API.

## Acceptance

1. Unit tests and builds pass.
2. HIL smoke pass.
3. Side-effect boundary documented and tested.
