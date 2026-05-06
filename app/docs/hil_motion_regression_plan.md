# HIL Motion Regression Plan

Date: 2026-05-05

## Goal

Create repeatable HIL evidence for commissioning and motion modes before tuning
advanced features further.

## Non-Goals

- Do not change control algorithms in this plan.
- Do not tune MPR, DOB, detent, or PI gains unless the test itself requires a
  documented parameter.
- Do not add persistence.

## Current Problem

Manual shell testing has found real issues, but it is hard to compare behavior
across commits without repeatable scripted runs and machine-readable output.

## Design

Use `scripts/hil/hil_telnet.py` as the primary HIL runner. Each scenario should:

- require `--yes-live-motion` for motor movement,
- start from a safe idle state,
- run one validation layer,
- stop motion and return to idle,
- write a JSON report,
- fail nonzero on fault or inconclusive result.

## Implementation Phases

Phase 1: Baseline scenarios.

- status,
- boot commissioning,
- current encoder validation,
- velocity encoder validation,
- position encoder validation,
- open-loop encoder trace.

Phase 2: Advanced feature scenarios.

- detent capture/validate,
- MPR bandwidth sweep,
- DOB enable validation,
- feature-combination matrix.

Phase 3: Report quality.

- Standardize metrics:
  - encoder errors,
  - motor error,
  - velocity error,
  - `Iq_ref` peak/RMS,
  - pass/fail verdict,
  - log path.

## Acceptance Criteria

- Each scenario can be run from a fresh boot.
- Each motion scenario ends in safe idle.
- JSON report exists for every HIL run.
- Generated/open-loop regression is kept separate from encoder-control tests.

## HIL Evidence

Baseline command set:

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.44 \
  --json-report hil_logs/status.json

python3 scripts/hil/hil_telnet.py encoder-validate --host 10.0.0.44 \
  --yes-live-motion \
  --json-report hil_logs/encoder_validate.json
```

Advanced command set:

```bash
python3 scripts/hil/hil_telnet.py mpr-dob-detent --host 10.0.0.44 \
  --yes-live-motion \
  --json-report hil_logs/mpr_dob_detent.json
```

## Risks

- HIL tests can be invalidated by hardware wiring, power faults, or motor
  changes.
- Telnet DHCP addresses can change.
- Log volume can perturb shell timing if debug logging is enabled.

## Done State

- HIL scenarios are documented.
- Baseline MT6835 HIL run passes.
- Known AEAT-9955 limitations are documented separately.

