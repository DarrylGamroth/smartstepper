# Execution Log

Append one entry per task execution.

| Date | Task ID | Status | Commit | Notes |
| --- | --- | --- | --- | --- |
| YYYY-MM-DD | T0000 | pending/in_progress/done/blocked | <hash or n/a> | short note |
| 2026-02-24 | T0001 | done | n/a | Added baseline ISR cycle + stack watermark procedure and baseline entry template. |

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
