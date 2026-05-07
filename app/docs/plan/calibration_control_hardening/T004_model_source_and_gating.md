# T004 - Model Source And Gating

## Problem

MPR/DOB/detent may use active model values without making it obvious whether they came from measured commissioning data or fallback overlay defaults.

## Implementation

- Add lightweight model-source flags to runtime state.
- Mark flux/Kt and mechanical parameters as measured only when applied from valid commissioning results.
- Print source in commissioning status.
- Require measured model values for DOB readiness.

## Done When

- Shell status makes measured vs fallback model use visible.
- DOB cannot be enabled from fallback-only model data.
