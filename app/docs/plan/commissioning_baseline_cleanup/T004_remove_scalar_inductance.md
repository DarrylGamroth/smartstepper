# T004 - Remove Pulse/Integrated Scalar Inductance User Path

## Problem

Pulse/integrated scalar inductance has disagreed with R/L and demodulated D/Q
results. It is not part of the production commissioning baseline and should no
longer be exposed as a normal operator path.

## Work

- Remove user-facing shell commands for scalar pulse/integrated inductance now.
- Remove HIL scenario references that treat scalar pulse L as production.
- Keep demodulated D/Q Ld/Lq as production electrical ID.
- Before deleting core implementation code, run a dependency check against
  demodulated D/Q, saliency, tests, and HIL scripts.
- If internal helper code is still used by demod/saliency/tests, keep it private
  and rename/comment it as an internal excitation/capture helper rather than a
  scalar inductance estimator.
- If dependency checking shows the scalar estimator is standalone, delete the
  core estimator and tests with the shell path.

## Constraints

- Do not remove `measure demod`, `demod_sweep`, or production `electrical run`.
- Do not break current PI recommendation from Ld/Lq.

## Validation

- `motor commission electrical plan` no longer lists scalar pulse L as a normal
  path.
- No HIL script or operator workflow uses scalar pulse L as a production value.
- Any remaining scalar/pulse helper code is private, dependency-justified, and
  not reachable from the shell as an estimator.
- Electrical unit tests and MT6835 build pass.
