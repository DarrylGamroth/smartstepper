# Execution Log

Append one entry per task execution.

| Date | Task ID | Status | Commit | Notes |
| --- | --- | --- | --- | --- |
| YYYY-MM-DD | T0000 | pending/in_progress/done/blocked | <hash or n/a> | short note |
| 2026-02-24 | T0001 | done | n/a | Added baseline ISR cycle + stack watermark procedure and baseline entry template. |
| 2026-02-24 | T0002 | done | n/a | Added P00 guardrail checklist for sizeof checks, regressions, and baseline evidence. |
| 2026-02-24 | T0101 | done | n/a | Split `adc_callback` into Collect/Process/Apply/Telemetry stages; both firmware targets build cleanly. |
| 2026-02-24 | T0102 | done | n/a | Parity validation complete: unit tests 21/21 passing (`./tests/run_unit_tests.sh`) and `smartstepper_v2` build passed. |
| 2026-02-24 | T0201 | done | n/a | Rehomed math/filter families under `include/motor/{math,filters}` and moved `prbs.c` to `src/math`; `smartstepper_v2` build passed. |
| 2026-02-24 | T0202 | done | n/a | Rehomed observer/motion modules, migrated app+unit include paths to `motor/{observers,motion,math,filters}`, `smartstepper_v2` build and full unit tests passed. |
| 2026-02-24 | T0203 | done | n/a | Rehomed control/protection/runtime/telemetry modules and includes to `motor/...` family paths; both firmware builds and full unit tests passed. |
| 2026-02-24 | T0204 | done | n/a | Filled P02 migration table. Legacy include scan in code files (`*.c/*.h`) returned zero matches; unfiltered scan only matched task text in `app/docs/plan/tasks/T0204.yaml`. |

## Blocker Template

```text
Task: Txxxx
Blocked on: <condition>
Observed: <error/log>
Next action: <specific unblock step>
```

## Baseline Entry Template (P00)

Use this template for baseline captures before refactor phases:

```text
Task: T0001
Date: YYYY-MM-DD
Firmware: smartstepper_v2 | smartstepper_v2_mt6835
Mode: velocity_open
Commanded: iq=<A>, velocity=<rad/s>
ISR count: <u32>
ISR max cycles: <u32>
ISR avg cycles: <u32>
Stack watermark command: kernel thread stacks | unavailable
Stack notes:
  - thread=<name> unused=<bytes> used=<bytes>
Notes:
  - encoder faults=<u32>, warn=<u32>, error=<u32>
```
