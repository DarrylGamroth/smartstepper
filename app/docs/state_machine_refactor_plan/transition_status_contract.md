# Transition Status Contract

Every externally requested state or mode transition should update a status record.

## Fields

- `request_sequence`: monotonically increasing request counter.
- `request_event`: event type that initiated the transition.
- `requested_state`: requested lifecycle state or online mode.
- `source_state`: state when the request was accepted/processed.
- `final_state`: state reached after processing, if known.
- `fallback_state`: fallback target when the requested state could not be entered.
- `result`: idle, requested, accepted, completed, rejected, fallback, fault, or timeout.
- `error_code`: motor error code associated with rejection/fault.
- `reason`: short operator-readable reason.
- `timestamp_ms`: uptime when the status was updated.
- `loop_count`: control loop count when the status was updated.

## Consumers

- `motor state status`: concise current state plus latest transition result.
- `motor state transition`: detailed latest transition result.
- HIL scripts: verify actual final state and transition result after mode/state commands.
- Commissioning scripts: verify setup and mode entry before applying moving commands.

## Rules

1. Request posting is not completion.
2. Rejected guard checks must update the reason.
3. Fallbacks must name both requested and fallback states.
4. Workflows must not leave stale requested modes without an explicit status update.
