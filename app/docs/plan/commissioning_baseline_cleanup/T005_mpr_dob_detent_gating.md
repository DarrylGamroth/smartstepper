# T005 - Gate MPR, DOB, and Detent Feedforward

## Problem

Advanced features are useful but can obscure baseline problems. They should not
be required for commissioning and should only enable after baseline validation.

## Work

- Keep PI as the default outer-loop regulator.
- Make bandwidth commands the preferred MPR interface:
  - `motor velocity mpr bandwidth <hz>`
  - `motor position mpr bandwidth <hz>`
- Keep raw MPR `set` commands only as engineering/expert diagnostics, or hide
  them behind debug/engineering Kconfig if shell clarity suffers.
- Keep DOB disabled by default and gated on baseline readiness plus stable
  velocity control.
- Keep detent feedforward disabled by default and require validation before
  enabled apply.
- For chopper use, add note that detent feedforward may be less valuable for
  22.5 degree indexed moves than for continuous low-speed motion; validate
  profile endpoint settling and current ripple before enabling.

## Constraints

- Advanced feature enable must be reversible and reset state cleanly.
- Enabling DOB/detent must not be possible from a non-baseline-ready state.

## Validation

- Shell status explains why MPR/DOB/detent are unavailable when baseline is not
  ready.
- PI baseline validation still passes with all advanced features disabled.
