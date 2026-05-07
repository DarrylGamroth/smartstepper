# T002 - Remove Legacy RS_EST From Default Flow

## Problem

There are multiple Rs paths. The legacy `RS_EST` state is no longer the best
production path; bidirectional production Rs is cleaner and has better recent
HIL evidence.

## Work

- Remove `RS_EST` from the default baseline commissioning state-machine path.
- Keep R/L bootstrap as fallback if still needed for early current PI defaults.
- Remove or hide shell/docs that suggest legacy `RS_EST` is the preferred route.
- Remove stale devicetree properties used only by legacy `RS_EST`, unless still
  needed by a fallback command.

## Constraints

- Do not break ROVERL fallback yet.
- Do not remove production electrical Rs.
- Preserve ability to compare fallback R/L with production electrical status.

## Validation

- Build passes.
- Baseline commissioning uses production Rs, not legacy `RS_EST`.
- `motor commission electrical status` still reports fallback/current values for
  comparison.
