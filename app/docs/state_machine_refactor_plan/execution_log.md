# Execution Log

Append entries in this format:

```text
## YYYY-MM-DD - <task id> - <summary>
Status: done|blocked
Commit: <sha or pending>
Validation:
- <command>: PASS|FAIL|SKIPPED, evidence
Notes:
- <important observations>
```

## 2026-05-06 - SMF0001/SMF0002 - Baseline docs
Status: done
Commit: 87f7c8d
Validation:
- test -f app/docs/state_machine_refactor_plan/current_transition_matrix.md: PASS
- rg transition fields in transition_status_contract.md: PASS
Notes:
- Captured observed staged-mode ambiguity and required transition status fields before code changes.

## 2026-05-06 - SMF0103/SMF0202/SMF0203 - Workflow mode ownership and transition parser
Status: done
Commit: 23bd53b
Validation:
- west build MT6835: PASS
- python3 -m py_compile scripts/hil/hil_telnet.py: PASS
- python3 -m unittest scripts/hil/test_hil_telnet_parser.py: PASS
Notes:
- Commissioning helpers now use motor_commission_request_online_mode() to own requested_online_mode explicitly.
- HIL status scenario now queries motor state transition and fails on rejected/fault/timeout result.

## 2026-05-06 - SMF0401/SMF0402 - Centralize online mode reset policy
Status: done
Commit: 875d4d1
Validation:
- west build MT6835: PASS
- python3 -m py_compile scripts/hil/hil_telnet.py: PASS
- python3 -m unittest scripts/hil/test_hil_telnet_parser.py: PASS
Notes:
- Online substates now use motor_operating_mode descriptors for ISR feature masks.
- Entry/exit reset behavior for direct current, generated velocity/position, and encoder velocity/position modes is centralized in motor_operating_mode_apply_*_policy().

## 2026-05-06 - SMF0501/SMF0502/SMF0503 - Explicit fault recovery status
Status: done
Commit: pending
Validation:
- west build MT6835: PASS
- python3 -m py_compile scripts/hil/hil_telnet.py: PASS
- python3 -m unittest scripts/hil/test_hil_telnet_parser.py: PASS
Notes:
- ERROR entry records whether gate reset and/or encoder recovery is required.
- `motor state clear_error` now rejects recovery-incomplete faults instead of implicitly pulsing gate-driver recovery.
- Added shell visibility via `motor state recovery`, `motor fault recovery`, and explicit `motor encoder recover`.
- HIL parser now checks recovery readiness and has unit coverage for incomplete gate recovery.
