# Motion Control Status Shell Plan

Date: 2026-05-05

## Goal

Make shell status output clearly explain what the controller is doing and why,
without exposing unnecessary internal flags in normal operator commands.

## Non-Goals

- Do not change control algorithms.
- Do not rename every command in one pass.
- Do not remove engineering/debug commands without replacement.
- Do not add persistence.

## Current Problem

The shell can expose many raw details, but normal status does not always answer
the practical questions:

- What lifecycle state is active?
- What motion mode is requested?
- What feedback source is used?
- What angle source is used?
- Which regulator is active?
- Which feedforward modules are enabled?
- Is feedback trusted?
- What fault risk is currently blocking operation?

## Design

Separate normal status from diagnostics:

- normal status: concise operator state,
- control status: regulator/feedforward/limits,
- observer status: trusted/predicted/fault and angles,
- encoder diagnostics: raw counters and transport details,
- feature diagnostics: MPR/DOB/detent raw parameters.

## Implementation Phases

Phase 1: Define command tree.

Recommended normal commands:

```text
motor state status
motor control status
motor observer status
motor encoder status
motor velocity mpr status
motor velocity dob status
motor commission detent status
```

Phase 2: Split operator and diagnostic output.

- Keep normal status short.
- Move raw counters to diagnostic subcommands.

Phase 3: Align wording.

- Avoid overloaded terms like pipeline, generated feedback, or sidework.
- Use consistent terms:
  - lifecycle state,
  - operating mode,
  - feedback source,
  - angle source,
  - regulator,
  - feedforward,
  - observer trust.

Phase 4: Update HIL parsers.

- Keep script parsing stable after command changes.

## Acceptance Criteria

- A user can determine active regulator and feedforward state from one command.
- Encoder transport counters are still accessible.
- Observer trust state is visible.
- HIL scripts still pass.

## HIL Evidence

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.44 \
  --json-report hil_logs/status_after_shell_cleanup.json
```

Expected:

- report parses state and encoder counters,
- no motor fault,
- status output is concise enough for normal use.

## Risks

- Renaming commands can break scripts.
- Too much status output can reintroduce shell/telnet timing issues.

## Done State

- Command tree documented.
- HIL parser updated.
- Operator and diagnostic status paths are distinct.

