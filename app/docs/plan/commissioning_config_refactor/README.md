# Commissioning Configuration Refactor Plan

Date: 2026-05-07

## Goal

Make commissioning configuration understandable, composable, and safe:

- keep devicetree overlays as hardware/profile bootstrap defaults,
- split motor/current-class safety values from generic commissioning recipes,
- keep RoverL as the first electrical bootstrap estimator,
- replace legacy DC `RS_EST` as the normal resistance path,
- use production bidirectional Rs and demodulated D/Q inductance to update the
  active current PI model,
- prepare longer-term Settings/ZMS persistence without storing bad values
  automatically.

## Key Decision

RoverL remains part of the baseline flow. The intended electrical sequence is:

1. Start from safe devicetree fallback model and current limits.
2. Run current-offset calibration.
3. Run RoverL to estimate initial `R` and average `L` safely.
4. Update current PI controllers from RoverL `R/L` results.
5. Run production bidirectional Rs measurement.
6. Run production demodulated inductance measurement for `Ld` and `Lq`.
7. Update current PI controllers from production `Rs/Ld/Lq` results.
8. Continue to encoder mapping, flux ID, and outer-loop tuning.

Legacy DC `RS_EST` is not the preferred path. It may remain temporarily as a
fallback/diagnostic state while code is cleaned up, but its devicetree properties
should be removed once the production bidirectional Rs path is fully wired into
baseline commissioning.

## Parameter Ownership Summary

| Source | Owns | Examples |
| --- | --- | --- |
| Board overlay | PCB topology and measurement hardware | ADC channels, gains, shunts, PWM/timer wiring, UART, Ethernet, SPI pinmux |
| Encoder overlay | Selected encoder transport/protocol | `encoder1` alias, RT SPI child, SPI mode, CS, encoder direction if tied to assembly |
| Motor safe-ID overlay | Motor identity, hard limits, conservative fallback model | pole pairs, max current, max speed, fallback Rs/Ld/Lq/flux/J |
| Shared commissioning overlay | Generic commissioning recipe defaults | sample counts, settle times, demod cycles, confidence thresholds |
| Runtime RAM | Active measured/tuned values | active Rs/Ld/Lq, active PI gains, encoder offset, flux, limits staged by commissioning |
| Settings/ZMS | Per-unit accepted commissioned values | validated baseline electrical model, encoder map/offset, user limits, controller bandwidths |

## Naming Policy

- Use `commission-*` only for commissioning procedure knobs.
- Do not prefix general control defaults such as PWM frequency, loop frequency,
  current-loop bandwidth, velocity limits, or current command ramp.
- Keep `roverl-est-*` while RoverL remains a baseline bootstrap step.
- Mark `rs-est-*` as legacy/fallback, then remove when DC `RS_EST` is removed
  from baseline code and no required fallback references remain.
- Move experimental feature defaults, such as RLS and thermal model parameters,
  out of normal motor safe-ID overlays unless the corresponding feature is part
  of baseline commissioning.

## Long-Term Target

The long-term model should allow a unit to boot from conservative devicetree
fallbacks, run commissioning, preview the measured values, and explicitly save
validated values to Settings/ZMS. Boot autoload should remain gated until HIL
shows rollback-safe behavior.

## Tasks

Execute in order:

1. `T001_inventory_and_taxonomy.md`
2. `T002_shared_commissioning_overlay.md`
3. `T003_roverl_first_electrical_flow.md`
4. `T004_legacy_rs_est_retirement.md`
5. `T005_binding_and_macro_cleanup.md`
6. `T006_settings_zms_long_term.md`
7. `T007_validation_and_hil.md`

## Acceptance Gate

- Safe-ID overlays contain motor identity, hard limits, and conservative fallback
  model values only.
- Shared commissioning overlay contains generic recipe defaults.
- RoverL-first electrical flow is documented and implemented before retiring
  legacy DC `RS_EST`.
- Current PI gains are updated first from RoverL and then from production
  bidirectional Rs + demodulated `Ld/Lq`.
- Firmware builds with both MT6835 and AEAT-9955 composed overlay stacks.
- HIL evidence proves baseline commissioning still works after cleanup.
