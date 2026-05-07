# T001 - Split Baseline, Identify, and Advanced Commissioning

## Problem

`motor commission run` currently mixes required baseline bring-up with advisory
mechanical/advanced identification. This makes a good electrical/encoder system
look failed when mechanical ID confidence is marginal.

## Work

- Add or refactor shell flows so baseline commissioning can complete without
  mechanical ID.
- Recommended command shape:
  - `motor commission baseline run [apply]`
  - `motor commission identify run [flux|mech|accel|all]`
  - `motor commission advanced ...` for detent/MPR/DOB validation, or retain
    existing subtrees with clear status labels.
- Keep existing `motor commission run` only if it delegates to these levels and
  prints exactly which level failed.
- Add status fields:
  - `Baseline ready: YES/NO`
  - `Electrical ready: YES/NO`
  - `Encoder mapped: YES/NO`
  - `Flux ready: YES/NO/FALLBACK`
  - `Mechanical ID: advisory valid/invalid`
  - `Advanced features: disabled/unvalidated/validated`

## Constraints

- Do not require mechanical ID for baseline ready.
- Do not make MPR/DOB/detent part of baseline.
- Keep current safety behavior: IDLE/disarmed cleanup, timeout restore, current
  slew reset.

## Validation

- `motor commission baseline run apply` completes on MT6835.
- `motor commission status` distinguishes baseline and advisory failures.
- Existing generated velocity/position modes still work.
