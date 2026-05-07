# T006 - Current PI Validation And Ld/Lq Estimation

## Problem

The current R/L and Rs measurements are adequate for a conservative current PI baseline, but they are not yet validated as a high-confidence electrical model. The code currently applies a scalar inductance to both d and q axes after electrical ID.

## VESC Reference

VESC uses two complementary measurements:

- Resistance: lock the motor with current, ramp current slowly, wait for the motor/current to settle, average current and voltage, then compute `R = V/I`.
- Inductance: stop normal control, run HFI/short-pulse excitation, average the inverse-inductance response, estimate average inductance and `Lq-Ld`, then apply a conservative scale factor before using the value.

Relevant VESC files:

- `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`: `mcpwm_foc_measure_resistance()`
- `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`: `mcpwm_foc_measure_inductance()`
- `/home/dgamroth/workspaces/motor/bldc/motor/mcpwm_foc.c`: `mcpwm_foc_measure_res_ind()`
- `/home/dgamroth/workspaces/motor/bldc/motor/foc_math.c`: `foc_precalc_values()` and saliency-adjusted inductance use

## Recommendations

1. Keep the existing Rs/R-over-L path for initial current PI tuning.
2. Add acceptance bounds before applying current PI gains:
   - `Rs > 0`, finite, within plausible devicetree-relative bounds.
   - `L > 0`, finite, within plausible devicetree-relative bounds.
   - No voltage saturation during capture.
   - Sufficient accumulated signal/current.
3. Add current-loop validation after applying gains:
   - Small positive/negative Id steps.
   - Small positive/negative Iq steps if encoder mapping is available and current mode is safe.
   - Measure rise time, overshoot, settling, final error, saturation, and overcurrent/fault status.
4. Revert to conservative devicetree gains if validation fails.
5. Add optional `Ld/Lq` estimation as a second phase, not a boot dependency.
6. Store scalar `Lavg` and `Ld/Lq` confidence separately so current PI can remain conservative even if saliency estimation is rejected.

## Ld/Lq Strategy For This Project

Start with a simple, deterministic method suitable for the existing current-loop infrastructure:

1. Use generated-angle locked rotor excitation.
2. Measure d-axis response with Id perturbation and Iq=0.
3. Measure q-axis response with Iq perturbation and Id hold only after encoder mapping/current validation are clean.
4. Estimate `Ld` and `Lq` from voltage/current derivative windows or a pulse-response fit.
5. Reject if the d/q estimates are not repeatable or if the response is dominated by resistance/deadtime.

A VESC-style HFI pulse estimator can be a later improvement if the simpler method is not robust enough.

## Done When

- Current PI gains are only applied automatically after bounded Rs/L plausibility checks.
- A current-step validation command reports pass/fail metrics.
- Failed validation reverts to conservative gains.
- The plan for optional `Ld/Lq` estimation is implemented behind an explicit commissioning command.
