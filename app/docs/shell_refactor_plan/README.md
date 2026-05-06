# Shell Refactor Plan Pack

Date: 2026-05-06

## Goal

Make the Zephyr motor shell implementation match the modular command tree while preserving command behavior.

## Non-Goals

- Do not redesign motor control behavior.
- Do not change commissioning algorithms.
- Do not remove existing commands in this pass unless replaced with compatible nested forms.
- Do not touch drivers or realtime ISR code.

## Execution Order

1. `SH001`: document current command/file ownership.
2. `SH010`: isolate root command registration from command implementations.
3. `SH020`: split state/safety/gate/info/observer/encoder/fault implementations by domain.
4. `SH030`: normalize encoder acquisition/control nested commands while keeping old aliases for one transition period.
5. `SH040`: validate build/tests and record evidence.

## Acceptance Gate

- MT6835 west build passes.
- Unit tests pass.
- HIL parser tests pass.
- `app/src/shell_commands.c` only owns global shell context and command registration.
- Large diagnostic implementations are no longer bundled in `shell_commands_state.c`.
