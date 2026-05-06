# Execution Log

Append validation evidence as tasks complete.

## 2026-05-06 - SH001 - Current shell ownership map
Status: done
Validation:
- current_shell_ownership.md created: PASS
Notes:
- Command tree is mostly clean; source files do not match command domains.

## 2026-05-06 - SH010 - Root registration split
Status: done
Validation:
- west build MT6835: PASS
Notes:
- `shell_commands.c` now owns shell globals and command registration only.
- Current/velocity/position/outer/control/RLS/params implementations moved to `shell_control.c`.

## SH020 - Domain implementation split

- Split `app/src/shell_commands_state.c` into domain implementation files:
  - `shell_state.c`
  - `shell_safety.c`
  - `shell_gate.c`
  - `shell_info.c`
  - `shell_observer.c`
  - `shell_encoder.c`
  - `shell_encoder_protocol.c`
  - `shell_encoder_trace.c`
  - `shell_encoder_capture.c`
  - `shell_fault.c`
- Added private shared helper header `app/src/shell_state_common.h` for shell-only formatting/parsing helpers used by multiple domains.
- Removed `shell_commands_state.c` from the build and left it as a tombstone documenting the split.
- Validation: MT6835 west build passed using `west build -p auto -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"`.
