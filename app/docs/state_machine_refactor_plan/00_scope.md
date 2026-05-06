# Scope

## Goals

1. Make state transitions deterministic and inspectable.
2. Split lifecycle state from operating mode and setup workflow semantics.
3. Eliminate duplicate transition guard logic.
4. Make shell/API command names and results match what actually happens.
5. Provide explicit fault notification and recovery flow.
6. Preserve ISR safety: no kernel queue calls from zero-latency ISR paths.
7. Preserve generated/open-loop operation while fixing encoder-mode entry and validation.
8. Make HIL workflows repeatable for commissioning, current encoder, velocity encoder, and position encoder.

## Non-Goals

1. Do not add persistence in this plan.
2. Do not tune velocity/position controllers in this plan except where needed for transition validation.
3. Do not redesign FOC math, encoder drivers, MPR, DOB, or detent algorithms.
4. Do not add scripting or new external control interfaces.
5. Do not change hardware overlays except when a task explicitly requires a test-only overlay adjustment.

## Constraints

1. The ADC control ISR is zero-latency and must not use kernel APIs.
2. The state-machine thread may use kernel queues, timers, and shell-visible diagnostics.
3. `motor_core` should own reusable transition/policy helpers when they are hardware-independent.
4. Application code should own hardware recovery, gate-driver control, and Zephyr shell/API glue.
5. HIL tests should prefer telnet when available and serial only as fallback.
6. Current unpersisted commissioning data means boot workflows must explicitly run required setup before encoder modes.

## Terms

- Lifecycle state: broad system phase such as init, idle, setup, run, fault.
- Operating mode: commanded runtime control mode such as current_encoder or velocity_generated.
- Setup workflow: explicit measurement/commissioning sequence such as current offset, electrical ID, encoder mapping, or mechanical ID.
- Transition request: a command/event asking for a state or mode change.
- Transition result: observable outcome: accepted, completed, rejected, fallback, timeout, or fault.
- Guard: deterministic validation that must pass before entering a state or mode.
