# T003 - Promote Flux Identification and BEMF Velocity Limit

## Problem

Flux identification has recent good HIL evidence and is directly useful for
BEMF-aware velocity limiting. It should be part of baseline when it passes
quality gates, but should not block baseline if it fails.

## Work

- Run flux ID after electrical ID and encoder mapping, or as part of baseline if
  safe/currently bounded.
- Promote valid flux into active runtime model and torque constant.
- Use active flux/Kt in velocity limit calculation and shell reporting.
- If flux is invalid, use devicetree fallback and mark velocity limit as
  fallback-confidence.
- Add status output showing:
  - flux source: measured/fallback,
  - BEMF-limited max speed,
  - configured profile max speed,
  - active command speed limit.

## Constraints

- Flux ID must use bounded velocity/current and clean encoder samples.
- Flux failure must not prevent baseline readiness if current loop and encoder
  mapping are valid.

## Validation

- Baseline HIL shows flux measured and active on MT6835.
- Velocity target above BEMF-safe limit is clamped/rejected with clear message.
