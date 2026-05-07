# T005 - Boot Semantics And HIL Timing

## Problem

`motor commission boot` currently leaves the controller online/generated, and custom HIL command timeouts are too short for long commissioning commands.

## Implementation

- Make boot commissioning return to idle/disarmed by default after applying encoder mapping.
- Add an optional `online` argument if a live generated-mode handoff is wanted.
- Increase or auto-select custom HIL command timeout for long commissioning commands.

## Done When

- Boot command completion state is deterministic.
- HIL custom scenarios do not overlap subsequent commands with an in-progress commissioning command.
