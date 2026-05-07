# T006 - Shell And HIL Workflow

## Problem

The current one-command commissioning flow hides too much detail when mechanical
identification is weak. We need repeatable commands and HIL evidence for each
stage before applying values.

## Objective

Expose staged mechanical ID commands and repeatable HIL scripts for friction,
inertia, detent-corrected identification, and apply gating.

## Implementation

- Add shell commands under `motor commission mech` or equivalent orthogonal tree:
  - `plateau run/status/apply-staged`,
  - `inertia run/status/apply-staged`,
  - `validate`,
  - `active`,
  - `clear`.
- Add script support in `scripts/hil/hil_telnet.py` or a small companion script.
- Ensure every run prints:
  - active electrical model source,
  - active/staged mechanical model source,
  - encoder error counters,
  - sample rejection counts,
  - accepted directional coverage,
  - confidence and plausibility result.

## Acceptance Criteria

- HIL can run each stage independently.
- Full commissioning can still run as one command after staged pieces pass.
- Logs contain enough data to compare against previous runs without manual notes.
- Live-motion commands require existing `--yes-live-motion` acknowledgement.

## Tests

- Python parser tests for new HIL output.
- Build test with MT6835 overlay.
- HIL smoke for plateau-only and inertia-only stages.

## HIL

Recommended final gate:

```bash
python3 scripts/hil/hil_telnet.py mechanical-id-v2 \
  --host 10.0.0.44 \
  --yes-live-motion
```

If telnet is unavailable, use the documented persistent serial workflow from
`AGENTS.md` and record the log path in `execution_log.md`.
