# 01 HIL Baseline

## Goal

Make the HIL harness prove the prerequisites before controller tuning starts.

## Implementation

- Ensure `custom` HIL scenarios are treated as live-motion scenarios.
- Always run the standard stop/status postlude for custom commands.
- Use the current command tree:
  - `motor encoder acquisition reset`
  - `motor encoder acquisition status`
- Ensure verdict evaluation parses encoder acquisition counters for all live scenarios.

## Validation

Run after firmware is already flashed:

```bash
python3 scripts/hil/hil_telnet.py custom --host 10.0.0.44 --yes-live-motion \
  --command 'motor state status' \
  --command 'motor encoder acquisition status' \
  --command 'motor fault snapshot status'
```

Acceptance:

- verdict is `PASS`, or `FAIL` only for a real system condition,
- no `INCONCLUSIVE encoder_acquisition_errors`,
- stop/status postlude appears in the log.
