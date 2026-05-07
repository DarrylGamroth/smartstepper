# T004 - Physical Constraints And Apply Gate

## Problem

A fit can have acceptable validation residual while still producing implausible
or low-confidence model parameters. Applying those values directly can degrade
PI/MPR/DOB tuning.

## Objective

Make physical constraints and confidence gating authoritative before updating
active `J/B/Tc`.

## Implementation

- Add a mechanical model acceptance function in motor-core or app runtime.
- Required constraints:
  - `J > 0`,
  - `B >= 0`,
  - `Tc >= 0`,
  - enough directional coverage,
  - repeatability across runs,
  - residual below threshold,
  - confidence above `commission-auto-mech-min-confidence-mpu`.
- Add plausibility checks relative to devicetree fallback:
  - warning range, e.g. `0.25x..4x`,
  - reject range, e.g. `0.10x..10x`, unless explicitly overridden.
- If rejected:
  - keep devicetree or last known-good active model,
  - mark model source as fallback,
  - keep staged result visible for diagnostics.

## Acceptance Criteria

- Low-confidence mechanical result cannot set `mech_model_source=MEASURED`.
- Shell output clearly distinguishes staged vs active mechanical model.
- Auto-tune uses measured mechanical values only when accepted.
- Failed mechanical confidence does not invalidate successful electrical ID.

## Tests

- Unit test low confidence prevents apply.
- Unit test plausible high-confidence result applies.
- Unit test implausible `J` rejects even with low residual.
- Unit test fallback active model remains unchanged after rejection.

## HIL

- Re-run full commissioning on MT6835.
- Expected current behavior before improvement: low confidence should stage but not apply.
- Verify `motor info measured` reports fallback or measured source correctly.
