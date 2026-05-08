# T004 - Legacy DC RS_EST Retirement

## Goal

Remove the legacy DC `RS_EST` path from normal commissioning after RoverL-first
and production bidirectional Rs are operational.

## Work

1. Identify all consumers of:
   - `rs-est-current-ma`,
   - `rs-est-rampup-ms`,
   - `rs-est-duration-ms`,
   - `RS_EST_CURRENT_A`,
   - `RS_EST_RAMPUP_S`,
   - `RS_EST_DURATION_S`,
   - `MOTOR_STATE_RS_EST`.
2. Remove `RS_EST` from the default commissioning state flow.
3. Decide whether to keep the state as hidden diagnostic/fallback or delete it.
4. If kept temporarily:
   - mark shell/docs as diagnostic only,
   - move `rs-est-*` defaults to an experimental/fallback overlay.
5. If deleted:
   - remove binding properties,
   - remove config macros,
   - remove state-machine entries,
   - remove shell references,
   - remove tests/docs that call it.
6. Remove fallback logic where production electrical properties fallback to
   `rs-est-*`; fall back to RoverL or max-current-derived defaults instead.

## Constraints

- Do not remove RoverL.
- Do not remove the production bidirectional Rs path.
- Do not break safe boot with erased settings.

## Validation

- Build fails if stale `rs-est-*` properties remain required by bindings after
  retirement.
- `rg "RS_EST|rs-est"` only returns archived docs or explicitly retained
  diagnostic code.
- Baseline commissioning still measures Rs through production bidirectional Rs.
