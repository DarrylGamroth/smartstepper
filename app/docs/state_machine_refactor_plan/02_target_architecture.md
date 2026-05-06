# Target Architecture

## Conceptual Layers

1. Shell/API layer
   - Parses user commands.
   - Posts transition requests.
   - Reports transition result/status.
   - Does not directly enable hardware.

2. State-machine layer
   - Owns lifecycle transitions.
   - Runs setup workflows.
   - Applies operating mode transitions.
   - Owns fault latching and recovery sequencing.

3. Transition policy layer
   - Pure or mostly pure guard helpers.
   - Determines whether a mode can be entered.
   - Produces human-readable rejection reasons.
   - Should live in `motor_core` when not hardware-specific.

4. Hardware recovery layer
   - Gate-driver reset/re-enable.
   - Encoder protocol/acquisition recovery.
   - PWM safe output programming.
   - Remains in app/driver code.

5. ISR control layer
   - Consumes a coherent runtime snapshot.
   - Does not infer high-level state transitions.
   - Uses feature flags and mode policy only after the state machine publishes them.

## Proposed Lifecycle Hierarchy

```text
INIT
  HW_INIT
  CONTROL_INIT

SAFE
  IDLE

SETUP_POWERED
  CURRENT_OFFSET
  ELECTRICAL_ID
    R_OVER_L
    RS
    FLUX
  ENCODER_MAP
  MECHANICAL_ID
  TUNE_APPLY

RUN
  CURRENT_GENERATED
  CURRENT_ENCODER
  VELOCITY_GENERATED
  VELOCITY_ENCODER
  POSITION_GENERATED
  POSITION_ENCODER

FAULT
  LATCHED
  RECOVER_GATE
  RECOVER_ENCODER
  SAFE_IDLE
```

This is a target structure, not a required one-shot rewrite. The task cards move
toward this hierarchy incrementally.

## Runtime Mode Contract

Each operating mode should have a single descriptor:

- mode enum
- required setup flags
- required hardware features
- required ISR features
- angle source
- feedback source
- current source
- actuator/backend kind
- entry reset policy
- exit reset policy
- allowed source lifecycle states

The state machine should derive ISR feature flags from this descriptor rather
than each entry/exit handler manually toggling partial bits.

## Transition Result Contract

Every externally requested transition should update a shared transition status:

- request sequence number
- requested target
- accepted/rejected
- completed state/mode
- fallback state/mode, if any
- error code
- rejection reason
- timestamp/loop count

Shell/HIL should inspect this status when it needs proof that a transition completed.

## Fault Recovery Contract

Fault recovery should be explicit:

1. Latch cause and capture diagnostics.
2. Disable ISR features and force safe PWM.
3. Disable gate-driver channels.
4. Reset gate-driver fault latch if requested.
5. Reset encoder protocol/acquisition counters if requested.
6. Clear command targets and stale requested mode.
7. Enter `IDLE` only when recovery guards pass.

## Shell Command Direction

The command tree should make staging vs running obvious:

```text
motor mode set <current_encoder|velocity_generated|velocity_encoder|position_generated|position_encoder>
motor run <mode>
motor stop
motor transition status
motor recover status
motor recover gate
motor recover encoder
motor recover clear
```

Existing commands can be updated or replaced. Backwards compatibility is not a
hard requirement when old names obscure behavior.
