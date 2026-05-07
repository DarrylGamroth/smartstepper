# T001 - Shared Commissioning Preconditions

## Problem

Commissioning commands can start a calibration or motion stage while stale current targets, current trajectory state, or online mode state are still active.

## Implementation

- Add a shared helper that requests zero current, disarms, idles, clears velocity target, forces zero slew state, and waits a short settle interval.
- Use it before `motor commission boot` starts offset calibration.
- Use it at the start of the standard commissioning workflow.

## Done When

- Boot commissioning starts from a deterministic idle/zero-current state.
- Standard commissioning uses the same precondition path.
