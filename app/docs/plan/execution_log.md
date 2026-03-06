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
| 2026-02-24 | T0601 | done | n/a | Split decoupling and current PI into dedicated `motor/control/{dq_decoupling,current_loop}` modules; rewired `motor_foc_voltage_pwm.c` to compose them and updated `motor_control_loop.c` to use `motor_dq_decoupling_is_enabled(...)` for deterministic decoupling gating; `smartstepper_v2` build and full unit suite passed (23/23). |
| 2026-02-24 | T0602 | done | n/a | Isolated transform and PWM synthesis contracts into `motor/control/{transforms,pwm_synthesis}`; `motor_control_loop.c` now uses `motor_transforms_park(...)`, `motor_foc_voltage_pwm.c` composes current-loop/decoupling/transform/pwm modules, and ISR apply stage remains finalized modulation only; `smartstepper_v2` build passed. |
| 2026-02-24 | T0603 | done | n/a | Expanded `tests/unit/control` with FOC regression for current-loop saturation headroom, decoupling enable/disable determinism, transform finite round-trip, and PWM clamp behavior; full Twister run passed (`23/23`, `183/183`) via `west twister ... --outdir /tmp/twister-full`. ISR-time parity note: no ISR cycle-budget regression introduced in P06 split (apply stage still consumes finalized modulation only). |
| 2026-02-25 | T0701 | done | n/a | Composed process-stage entrypoint as `motor_core_step_fast(...)` in `modules/motor_core/src/runtime/motor_core_step.c`; `app/src/motor_control_loop.c` now acts as a thin orchestration wrapper into runtime core-step API. |
| 2026-02-25 | T0702 | done | n/a | Added runtime boundary checks in `tests/unit/runtime` to fail on queue/logging side-effect tokens in fast-step source, documented side-effect policy in architecture target, and validated unit + firmware build parity after pipeline composition. |
| 2026-02-25 | T0801 | done | n/a | Added coherent config snapshot handoff (`motor/runtime/config_snapshot`) with epoch tagging and lock-free double buffer; state thread now publishes complete ISR config snapshots each SMF cycle and fast process step consumes one snapshot per cycle for state/feature/decimation inputs. |
| 2026-02-25 | T0802 | done | n/a | Added P08 runtime mixed epoch snapshot regression tests (`tests/unit/runtime`) with adversarial alternating publish/read scenarios and monotonic-epoch checks; validated transition determinism by full unit run (`24/24`, `186/186`) and firmware build pass. |
| 2026-02-25 | T0901 | done | 150fcb1 | Partitioned `motor_core` at link level into `motor_core_rt`, `motor_core_motion`, `motor_core_estimation`, and `motor_core_commission` via `modules/motor_core/src/CMakeLists.txt`; validation build passed for `smartstepper_v2_mt6835`. |
| 2026-02-25 | T0902 | done | n/a | Captured P09 size/link map/jitter evidence: link map shows new `libmotor_core_*` archives; `smartstepper_v2_mt6835` moved from `FLASH=231704,RAM=109984` to `FLASH=232128,RAM=110112` (`+424B/+128B`), while `smartstepper_v2` remained `FLASH=236560,RAM=111008`; no dedicated HIL jitter run in P09 (evidence-only). |
| 2026-03-06 | n/a | done | n/a | Refactored `motor_core_step_fast()` into explicit helper stages (commission init, PWM init, keepalive+timeout gate, encoder stage, finalization) with no behavior change intent; `smartstepper_v2` build passed and unit suites `tests/unit/runtime` + `tests/unit/motor_commission_estimators` passed on `native_sim`. |
| 2026-03-06 | n/a | done | n/a | Grouped profile sequence runtime fields into `struct motor_profile_sequence_ctx` (`params->profile_seq.*`) and migrated app/runtime call sites from flat `params->profile_sequence_*` fields; build + targeted unit suites passed. |
| 2026-03-06 | n/a | done | n/a | Grouped chopper calibration runtime fields into `struct motor_chopper_cal_ctx` (`params->chopper_cal.*`) and migrated app/runtime call sites from flat `params->chopper_cal_*` fields; `smartstepper_v2` build and `tests/unit/runtime` passed. |
| 2026-03-06 | n/a | done | n/a | Grouped calibration/ALIGN runtime fields into `struct motor_calibration_ctx` (`params->calibration.*`) and migrated state/shell/runtime call sites; `smartstepper_v2` build and `tests/unit/runtime` passed. |
| 2026-03-06 | n/a | done | n/a | Grouped telemetry capture rings into context structs: `encoder_capture`, `encoder_raw_trace`, and `fault_snapshot`; migrated ISR/shell/telemetry call sites from flat fields; `smartstepper_v2` build and unit suites `tests/unit/runtime` + `tests/unit/motor_align` passed. |
| 2026-03-06 | n/a | done | n/a | Grouped estimator runtime into `rls` and `thermal` contexts (`params->rls.*`, `params->thermal.*`) and migrated state/runtime/shell usages from flat fields; `smartstepper_v2` build and unit suites `tests/unit/runtime` + `tests/unit/motor_rl_ident` passed. |

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
