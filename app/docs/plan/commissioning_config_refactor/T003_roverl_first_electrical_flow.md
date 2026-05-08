# T003 - RoverL-First Electrical Flow

## Goal

Make the baseline electrical commissioning sequence explicit and implement it as
the default safe path.

## Target Flow

1. Use devicetree fallback values to configure a conservative current PI.
2. Run current-offset calibration.
3. Run RoverL at safe current using `roverl-est-*` defaults.
4. Compute provisional `Rs` and average `L` from RoverL.
5. Update the current PI controllers with provisional RoverL values.
6. Run production bidirectional Rs measurement.
7. Run production demodulated inductance measurement for `Ld` and `Lq`.
8. Update current PI controllers with production `Rs/Ld/Lq` values.
9. Mark electrical baseline ready only after production values pass confidence
   gates.

## Work

1. Audit current commissioning shell/script flow and state-machine transitions.
2. Ensure RoverL result application updates active current PI gains before
   bidirectional Rs/demod L are run.
3. Ensure production electrical ID overwrites the provisional RoverL model only
   if quality gates pass.
4. Add status output showing model source:
   - `fallback_dt`,
   - `roverl_provisional`,
   - `production_electrical`,
   - `settings_loaded` once persistence is enabled.
5. Ensure failed production electrical ID leaves the RoverL provisional model
   active but reports reduced confidence.

## Constraints

- Do not require encoder feedback for RoverL or production electrical ID.
- Do not allow production electrical ID to exceed motor safe current limits.
- Do not save values to Settings in this task.

## Validation

- Unit tests cover current PI update from provisional and production model
  sources.
- HIL electrical commissioning shows PI gains update after RoverL and again
  after bidirectional Rs + demod L.
- Failed production demod does not discard the provisional RoverL PI model.
