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
| 2026-02-24 | T0301 | done | n/a | Added `motor_rt_fast_state` and `motor_rt_diag_state` runtime headers with update cadence comments; wired scaffold fields into `motor_parameters`; both firmware builds passed. |
| 2026-02-24 | T0302 | done | n/a | Moved non-hot runtime mirrors to `rt_diag`, kept ISR-per-sample mirrors in `rt_fast`, added telemetry scaffold file, and synced mirror paths in `motor_control_loop.c`; build + unit tests passed. |
| 2026-02-24 | T0303 | done | n/a | Added compile-time runtime footprint guards in `config.h` (`rt_fast <= 64B`, `rt_diag <= 96B`) and verified `smartstepper_v2` build; before: no explicit guardrails, after: bounded hot/diag budgets. |
| 2026-02-24 | T0401 | done | n/a | Split observer pipeline into `encoder_source` and `angle_tracking` modules and rewired `motor_encoder_feedback.c` to use them while preserving delay/handoff behavior; observer unit test and firmware build passed. |
| 2026-02-24 | T0402 | done | n/a | Added explicit `motor_control_feedback` and `motor_capture_feedback` payloads; control logic now consumes compact feedback while capture ring writes consume extended capture payload; `smartstepper_v2` build passed. |
| 2026-02-24 | T0403 | done | n/a | Added observer split regression coverage for source gating, delay/handoff reset behavior, and invalid-frame burst/recovery transitions; full unit suite passed (21/21) and observer/encoder parity check evidence recorded for split path behavior. |
| 2026-02-24 | T0501 | done | n/a | Extracted PI-style position/velocity regulators into `motor/control/{position_regulator,velocity_regulator}` and rewired `motor_control_outer_loops.c` to use module APIs; added `chopper.pi_controller.unit`; firmware build and full unit suite passed (22/22). |
| 2026-02-24 | T0502 | done | n/a | Extracted reference arbitration into `motor/runtime/command_arbitration` and timeout/disarm gating into `motor/protection/interlocks`; rewired `motor_current_ref_policy.c` and `motor_control_loop.c` to use module interfaces with unchanged priority order; `smartstepper_v2` build and full unit suite passed (22/22). |
| 2026-02-24 | T0503 | done | n/a | Added `chopper.control_ref_path.unit` covering arbitration priority, disarm interlock precedence, timeout boundary behavior, and keepalive bypass; full unit suite passed (23/23). Closed-loop parity smoke result: `smartstepper_v2_mt6835` build passed after reference-path split. |
| 2026-02-24 | T0601 | done | n/a | Split decoupling and current PI into dedicated `motor/control/{decoupling,current_loop}` modules; rewired `motor_foc_voltage_pwm.c` to compose them and updated `motor_control_loop.c` to use `motor_decoupling_is_enabled(...)` for deterministic decoupling gating; `smartstepper_v2` build and full unit suite passed (23/23). |

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
