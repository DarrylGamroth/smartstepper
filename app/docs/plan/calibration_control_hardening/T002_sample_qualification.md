# T002 - Commissioning Sample Qualification

## Problem

Flux and mechanical commissioning reject bad encoder/fault/saturation samples, but a clean sample can still be invalid if actual velocity is far from the requested velocity.

## Implementation

- Extend commissioning observation with requested/measured velocity tracking error fields.
- Add runtime configuration thresholds for velocity tracking during flux/mechanical capture.
- Reject samples where velocity tracking is outside the configured tolerance.
- Report the reject counter in auto-commission failure output.

## Done When

- Poor velocity tracking cannot silently produce a valid flux/mechanical model.
