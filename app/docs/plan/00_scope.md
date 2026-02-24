# Scope

## Goals

1. Reorganize `motor_core` into TI-style module families.
2. Enforce ISR staging: Collect, Process, Apply, Telemetry.
3. Reduce hot-path context and stack pressure.
4. Make module contracts explicit and testable.
5. Keep hard real-time behavior deterministic.

## Non-Goals

1. Feature additions unrelated to control-path decomposition.
2. Temporary compatibility wrappers for old include/layout structure.
3. UI/shell redesign beyond command/path updates required by refactor.

## Hard Constraints

1. No dynamic allocation in ISR path.
2. No blocking or kernel API use in direct ISR callbacks.
3. Preserve control behavior and safety outcomes.
4. Keep encoder RTIO semantics intact.
5. Direct cutover migration: no legacy include compatibility layer.
6. Process stage work is bounded and deterministic per tick.

## Glossary

1. `Collect`: Acquire inputs and coherent runtime snapshot.
2. `Process`: Run observers, regulators, interlocks, protection checks, and current-loop math.
3. `Apply`: Commit actuator outputs and protection actions.
4. `Telemetry`: Update live snapshot plus optional diagnostics.
5. `Position regulator`: Position error to velocity command.
6. `Velocity regulator`: Velocity error to torque/current command.
7. `Command arbitration`: Select active command source and precedence.
8. `Interlocks`: Armed/disarmed and safety gating for torque-producing outputs.

## Out of Scope Boundaries

1. Devicetree schema redesign.
2. Sensor driver protocol changes.
3. New control algorithms beyond decomposition and existing behavior-preserving extraction.

