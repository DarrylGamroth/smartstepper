# Control and Commissioning Inventory

Date: 2026-05-07  
Hardware baseline: `smartstepper_v2` with `configs/motor_mt6835_2a.overlay`  
Purpose: separate known-good commissioning/control paths from experimental or duplicated paths before adding more MPR, DOB, detent, or mechanical-ID work.

## Summary

The project now has enough overlapping mechanisms that the default workflow should be narrowed. The current known-good path is:

1. Fresh current-offset calibration.
2. Production electrical ID using bidirectional Rs plus demodulated D/Q inductance.
3. Apply current PI from the production electrical result.
4. Generated Id-axis encoder mapping.
5. Use generated-angle modes and PI-based encoder velocity as the baseline control paths.

Mechanical ID, detent feedforward, MPR, DOB, saliency sweep, and RLS/online estimators should remain available for diagnostics or development, but they should not block commissioning until they have repeatable HIL evidence. The legacy RS_EST and scalar pulse inductance paths have been removed from the supported code path.

## Known Good

| Area | Status | Evidence / Notes | Default Commissioning Role |
| --- | --- | --- | --- |
| Current-offset calibration | Known good after stale-complete invalidation | Recent HIL forced fresh offsets and state-machine logs matched shell output. | Required first step. |
| Production Rs measurement | Known good on MT6835 | Recent HIL: `Rs ~= 2.273 ohm`, confidence about `0.91`. Uses bidirectional current. | Required. |
| Demodulated Ld/Lq measurement | Known good on MT6835 | Recent HIL: `Ld ~= 4.55-4.63 mH`, `Lq ~= 1.85-1.91 mH`; current PI validation passes. | Required for current PI. |
| Current PI from Rs/Ld/Lq | Known good | Recent electrical apply produced stable current validation and no faults. | Required. |
| Generated Id-axis encoder mapping | Known good on MT6835 | Recent HIL: mapping valid, sign `-1`, motion about one mechanical rev, no encoder errors. | Required before encoder-control modes. |
| MT6835 fast encoder transport | Known good | Recent counters remain zero for transport/frame/parity/crc/status/glitch. | Default encoder path. |
| Generated-angle velocity/position modes | Known good enough for commissioning and diagnostics | Used for electrical ID and encoder mapping. | Keep as safe diagnostic/mapping path. |
| Velocity PI minimal baseline | Known-good minimal evidence | `velocity-sweep` minimal PI at `+/-0.5 Hz` passed when reusing active measured model/mapping. | Baseline for encoder velocity control. |
| Current slew for Id/Iq | Known good conceptually and required | Centralized Id/Iq slew avoids current slams. | Required for all current commands. |

## Working But Not Yet Production-Gated

| Area | Status | Why It Is Not Default Yet | Next Gate |
| --- | --- | --- | --- |
| Flux identification | Promising | Recent flux fit is good (`R2 ~= 0.985`), but flux use in torque limits/tuning still needs explicit validation. | Allow staged/apply only after current/encoder baseline; do not block commissioning. |
| Mechanical ID v2 | Not reliable enough | Gets plausible subfits, but confidence is near threshold and repeat acceptance fails. Current excitation is not rich enough and detent/friction contaminate J. | Make advisory; replace gate with profile capability / acceleration-limit commissioning. |
| Velocity MPR | Prototype working at conservative settings | Minimal `+/-0.5 Hz` MPR passed, but tracking was looser than PI and depends on model quality. | Tune from bandwidth after PI baseline and measured acceleration capability. |
| Position MPR | Implemented, insufficient HIL evidence | Depends on velocity loop stability and model quality. | Validate after velocity PI/MPR baseline. |
| DOB | Implemented and gated | Readiness/reset policy exists, but HIL PI+DOB/MPR+DOB validation is deferred. DOB can hide bad model or detent issues. | Enable only after PI/MPR baseline is stable. |
| Detent feedforward | Implemented, not proven | Structural map validation exists, but HIL off/on ripple improvement remains open. | Build map after stable velocity PI; apply only if validation recommends apply. |
| Saliency sweep Ld/Lq | Diagnostic | Demodulated D/Q currently gives better/cleaner values. Saliency sweep still useful to study hybrid-stepper saliency. | Keep diagnostic, not default. |
| AEAT-9955 fast encoder | Partially improved | SPI4-8 mode/CRC path exists, but AEAT hardware has known noise/history. | Treat as secondary target until MT6835 baseline is stable. |

## Experimental / Diagnostic / Fallback Only

| Area | Current Role | Reason |
| --- | --- | --- |
| State-machine `ROVERL_MEAS` | Bootstrap/fallback only | Fast and useful historically, but production demodulated Ld/Lq is currently the preferred current-PI source. |
| Legacy `RS_EST` | Removed | Production bidirectional Rs replaces legacy Rs for normal commissioning. |
| Pulse/integrated scalar inductance | Removed from user-facing path | Has shown disagreement with R/L and demodulated methods. Demodulated D/Q inductance is the production path. |
| RLS parameter estimator / PRBS | Experimental | Not currently part of the working commissioning flow; possible future online Rs/temperature tracking, not inductance production ID. |
| Online Rs / thermal model | Experimental/observer | Potential future temperature compensation; not required for commissioning baseline. |
| Sensor-subsystem encoder drivers | Debug/legacy only | This project should use `encoder_rt` over `rt_spi` for control. Sensor shell compatibility is not a control requirement. |
| `drivers/spi_old/spi_ll_stm32.c` | Dormant | Historical RTIO/SPI experiment; do not use for current encoder-control path. |
| AEAT RTIO path/history plans | Archived/historical | Keep for reference only; do not use as evidence for MT6835 current status. |

## Duplicated Ways To Achieve The Same Thing

### Electrical Identification

Current overlapping paths:

1. State-machine `ROVERL_MEAS` for R/L.
2. Production bidirectional Rs measurement.
3. Production `motor commission electrical measure rs`.
4. Production `motor commission electrical measure demod`.
5. Demodulated D/Q inductance measurement.
6. `saliency_sweep` diagnostic path.
7. RLS/PRBS estimator.

Decision:

- Default: production bidirectional Rs + demodulated D/Q Ld/Lq.
- Fallback: ROVERL/default values only if production electrical ID fails or has not been run.
- Diagnostic only: saliency sweep, RLS/PRBS.
- Removed user-facing scalar pulse/integrated inductance.
- Removed legacy `RS_EST`; `motor commission run` should use the standard production electrical + encoder-mapping flow.

### Encoder Handling

Current overlapping paths:

1. `encoder_rt` fast drivers over `rt_spi`.
2. Sensor-subsystem drivers and shell/debug paths.
3. Historical RTIO/SPI paths.
4. Raw trace/capture telemetry.

Decision:

- Default control path: `encoder_rt` fast driver only.
- Telemetry: raw trace belongs to telemetry/debug and should not add alternate control semantics.
- Sensor drivers: keep only as optional debug drivers if they do not complicate control builds.
- Archived RTIO path: do not revive unless `rt_spi` fails on a target.

### Angle / Feedback Handling

Current pieces:

1. `angle_observer` owns wrap/offset/latency/electrical angle.
2. `encoder_feedback*`, `angle_path`, `feedback_quality`, and runtime feedback wrappers.
3. Generated-angle `angle_gen` for generated modes.

Decision:

- `angle_observer` remains the single owner for encoder-derived mechanical/electrical angle and velocity.
- Generated modes use `angle_gen`; encoder can be sampled for telemetry but not required.
- Avoid reintroducing large `fresh/valid/generated/stale` state machines. Use a small trust contract plus diagnostic counters.

### Outer-Loop Control

Current overlapping paths:

1. Velocity PI.
2. Position PI.
3. Velocity MPR.
4. Position MPR.
5. DOB feedforward.
6. Detent feedforward.

Decision:

- Default baseline: PI only.
- MPR: optional advanced regulator after PI baseline is stable.
- DOB: optional residual disturbance compensator after PI/MPR baseline is stable.
- Detent feedforward: optional feedforward after validation proves improvement.
- Commissioning should not require MPR/DOB/detent to pass.

## Recommended Default Commissioning Contract

Until settings autoload is enabled, every boot should run a short working
sequence:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor commission run confirm apply
motor velocity pi bandwidth 20 0.7 0.225
motor outer mode pi
```

Then validate the baseline:

```text
motor commission electrical status
motor commission encoder status
motor velocity pi status
motor encoder acquisition status
motor state status
```

Do not require this sequence to run mechanical ID, detent learning, MPR, or DOB.

After this sequence passes and the staged values are applied, save only the
validated groups:

```text
motor settings save model electrical
motor settings save model encoder
```

Save controller settings separately only after velocity/position PI or MPR
tuning has been validated:

```text
motor settings save controllers
```

## Default Control Bring-Up Order

1. Electrical/current-loop baseline.
2. Encoder mapping.
3. Generated-angle smoke test.
4. Encoder velocity PI at conservative bandwidth.
5. Encoder position PI after velocity PI is stable.
6. Profile capability / acceleration-limit characterization.
7. MPR bandwidth tuning.
8. Detent feedforward validation.
9. DOB enable validation.

## What Needs Work Before It Becomes Default

### Mechanical / Acceleration Capability

Problem:

- Current mechanical ID v2 can produce plausible `J/B/Tc` subfits but rejects repeatability/confidence.
- For the chopper wheel, the production value we really need is safe achievable acceleration, not necessarily perfect physical `J`.

Action:

- Add a profile capability commissioning path that measures repeatable acceleration under the actual velocity/current loop.
- Produce `alpha_safe_rad_s2`, current limit used, following-error bound, and confidence.
- Keep `J/B/Tc` as diagnostics until they become repeatable.

### MPR

Problem:

- Multiple low-level parameters are exposed, but the desired operator interface is bandwidth.

Action:

- Keep raw `set` command for engineering only.
- Preferred command remains `motor velocity mpr bandwidth <hz>` and `motor position mpr bandwidth <hz>`.
- Gate MPR tuning on a known-good PI baseline and valid acceleration/current limits.

### DOB

Problem:

- DOB can compensate model errors and make bad tuning look acceptable.

Action:

- Keep disabled by default.
- Enable only after PI/MPR baseline validation.
- Require readiness status to pass and reset on mode/target/trust discontinuities.

### Detent Feedforward

Problem:

- A structurally complete map can still hurt ripple if learned from friction/stiction/controller transients.

Action:

- Learn after velocity PI is stable.
- Capture forward/reverse at moderate speed over multiple cycles.
- Apply only if validation reports improvement or no regression.

## Documentation Cleanup Needed

The following docs are historical or partially superseded and should not be used as current execution contracts without review:

- `app/docs/archive/aeat_9955_history/*`
- `app/docs/archive/superseded_plans/*`
- older AEAT encoder HIL reports when discussing MT6835 behavior
- `app/docs/mpr_dob_detent_improvement_plan.md` current baseline values; some entries reference older successful commissioning that no longer matches the latest HIL state
- `app/docs/plan/control_tuning/execution_log.md` should be updated after this inventory to distinguish MT6835 known-good minimal PI/MPR tests from blocked full standard commissioning

## Immediate Recommendation

Before adding or tuning advanced control features:

1. Done: `motor commission run` no longer treats mechanical ID as required for
   the baseline commissioned state.
2. Split standard commissioning into explicit stages:
   - `baseline`: current offsets + production electrical ID + encoder map + PI defaults.
   - `identify`: flux/mechanical/acceleration capability diagnostics.
   - `advanced`: detent/MPR/DOB validation.
3. Add a shell-visible `baseline ready` status that reports exactly what is ready and what is still advisory.
4. Use `baseline ready` as the prerequisite for velocity/position PI testing.
5. Use measured acceleration capability, not mechanical-ID confidence, as the gate for fast chopper profiles.

## Execution Plan

The concrete cleanup plan package is:

```text
app/docs/plan/commissioning_baseline_cleanup/README.md
```

Execute its task cards in order before resuming MPR/DOB/detent tuning.
