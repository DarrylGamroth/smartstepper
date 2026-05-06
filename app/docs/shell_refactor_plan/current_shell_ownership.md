# Current Shell Ownership

## Command Tree

The command tree is mostly coherent and organized by domain:

- `motor state`
- `motor current`
- `motor velocity`
- `motor position`
- `motor profile`
- `motor encoder`
- `motor commission`
- `motor safety`
- `motor gate`
- `motor fault`
- `motor info`
- `motor observer`
- `motor outer`
- `motor control`
- `motor params`

## File Ownership Before Refactor

- `app/src/shell_commands.c`: mixed root registration, params, current, velocity, position, outer, control, RLS, shared shell globals, and helper functions.
- `app/src/shell_commands_state.c`: mixed state, arm/disarm, safety, gate, info, observer, encoder status, encoder protocol, encoder acquisition, encoder trace/capture, and fault snapshot.
- Existing focused files already align reasonably well: motion profile, motion sequence, chopper calibration, and commissioning submodules.

## Target Ownership

- `shell_commands.c`: root command tree and `g_motor_params` accessors only.
- `shell_control.c`: current, velocity, position, outer, control, params, and optional RLS command implementations.
- `shell_state.c`: lifecycle state commands.
- `shell_safety.c`: arm/disarm and command watchdog commands.
- `shell_gate.c`: gate-driver diagnostics/recovery.
- `shell_info.c`: config/measured/live/stat telemetry.
- `shell_observer.c`: observer status.
- `shell_encoder*.c`: encoder status, protocol, acquisition, trace, and capture.
- `shell_fault.c`: fault snapshot and recovery diagnostics.
