# Phase P02 - Module Rehome Cutover

## Objective

Move module files to TI-style directory taxonomy with direct include-path cutover.

## Prerequisites

1. P01 complete.

## Touch Files

1. `modules/motor_core/include/motor/**`
2. `modules/motor_core/src/**`
3. call sites across `app/` and `tests/`
4. `modules/motor_core/CMakeLists.txt`
5. migration docs

## Do Not Touch

1. Algorithm behavior and constants.

## Tasks

1. Rehome headers/sources by family (`math`, `filters`, `observers`, `motion`, `control`, `protection`, `runtime`, `telemetry`).
2. Update all includes directly to new paths.
3. Record migration table and completion status.

## Acceptance

1. Zero legacy include references.
2. Both firmware targets build.
3. Full unit suite passes.

