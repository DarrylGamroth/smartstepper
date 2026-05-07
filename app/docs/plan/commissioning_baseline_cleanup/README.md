# Commissioning Baseline Cleanup Plan

Date: 2026-05-07  
Primary hardware: `smartstepper_v2` + `configs/motor_mt6835_2a.overlay`  
Source inventory: `app/docs/control_commissioning_inventory_2026-05-07.md`

## Goal

Narrow the default commissioning/control path to the pieces with current HIL
evidence, remove duplicated legacy paths, and prepare a safe persistence model
without allowing bad commissioned values to disturb bring-up.

## Decisions Captured

- Remove legacy `RS_EST` from the normal code path.
- Promote flux identification to baseline if the fit passes quality gates; use
  it for BEMF-aware velocity limiting.
- Remove pulse/integrated scalar inductance measurement from the user-facing
  commissioning flow.
- Keep production electrical ID as bidirectional Rs plus demodulated D/Q
  inductance.
- Keep `encoder_rt`/`rt_spi` as the control encoder path.
- Keep `angle_observer` as the owner of wrapping, offset, latency, electrical
  angle, and velocity.
- Keep PI as the baseline outer loop.
- Keep MPR/DOB/detent optional and explicitly gated.
- Clean devicetree so it contains hardware topology and generic safe defaults,
  not a growing list of tuned runtime values.
- Add persistence with explicit preview/load/save/clear semantics, no automatic
  boot apply until HIL gates pass.

## Non-Goals

- Do not remove the old R/L bootstrap estimator yet; it remains a fallback.
- Do not make MPR, DOB, detent feedforward, or mechanical ID required for
  baseline commissioning.
- Do not enable persistence autoload in this plan.
- Do not store detent tables in the first persistence implementation.
- Do not preserve shell/API compatibility for removed experimental commands if
  simplification benefits the system.

## Target Commissioning Levels

### Baseline

Required before normal encoder-control work:

1. Fresh current offsets.
2. Production electrical ID: bidirectional Rs + demodulated Ld/Lq.
3. Apply current PI from production electrical ID.
4. Generated Id-axis encoder mapping.
5. Flux identification if quality passes; otherwise keep fallback flux and mark
   velocity-limit confidence reduced.
6. Velocity PI bandwidth/defaults staged from measured/fallback model.

### Identify

Advisory diagnostics:

- mechanical ID v2,
- acceleration capability / profile limit characterization,
- saliency diagnostics,
- R/L fallback comparison,
- RLS/online Rs experiments.

### Advanced

Optional performance features:

- MPR bandwidth tuning,
- DOB enablement,
- detent feedforward capture/validation.

## Task Order

1. `T001_baseline_workflow.md` - split commissioning levels and define shell flow.
2. `T002_remove_legacy_rs_est.md` - remove legacy `RS_EST` from default runtime.
3. `T003_promote_flux_velocity_limit.md` - promote flux to baseline and use for BEMF velocity limiting.
4. `T004_remove_scalar_inductance.md` - remove pulse/integrated scalar L user path.
5. `T005_mpr_dob_detent_gating.md` - clarify optional advanced control gates.
6. `T006_devicetree_config_cleanup.md` - clean DT vs runtime/persistent config split.
7. `T007_persistence_settings_backend.md` - add guarded settings persistence plan/API.
8. `T008_validation_hil.md` - prove baseline and non-regression.

## Acceptance Gate

- Unit tests pass for affected motor_core modules.
- West build passes for MT6835 overlay.
- HIL baseline commissioning passes without mechanical ID being required.
- Encoder counters remain clean on MT6835 during baseline.
- `motor state status`/commissioning status clearly shows baseline-ready versus
  advisory/advanced status.
- Persistence, if implemented, defaults to disabled autoload and cannot apply
  while armed/online.

## Progress

See `execution_log.md`.
