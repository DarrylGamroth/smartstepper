# MPR, DOB, and Detent Feedforward Improvement Plan

Date: 2026-05-05  
Hardware focus: `configs/motor_mt6835_2a.overlay`  
Persistence: intentionally out of scope until runtime performance is proven.

## Goal

Make the optional advanced motion features useful and repeatable on the hybrid
stepper motor:

1. improve detent feedforward map quality,
2. make detent commissioning robust,
3. tune MPR from a safe baseline,
4. gate and validate DOB enablement,
5. keep MPR, DOB, and detent feedforward roles separated.

The target behavior is not just "does not fault". The system should demonstrate
lower low-speed ripple, bounded current, predictable direction changes, no
encoder diagnostic regressions, and clean behavior when each feature is enabled
or disabled independently.

## Current Baseline

Latest HIL on MT6835 hardware showed:

- Standard commissioning completes and applies.
- Encoder transport is clean during control:
  - `transport=0`
  - `frame=0`
  - `parity=0`
  - `crc=0`
  - `status=0`
  - `glitch=0`
- Commissioned model values are plausible enough for initial control:
  - `psi_f ~= 0.00455 Wb`
  - `Kt ~= 0.341 Nm/A`
  - `J ~= 1.0e-4 kgm2`
  - `B ~= 2.7e-3 Nm/(rad/s)`
  - `Tc ~= 0.030 Nm`
- Velocity PI is already a known-good baseline.
- Conservative velocity MPR is stable but not tightly tuned.
- DOB runs bounded and does not destabilize the loop.
- Detent feedforward improves low-speed MPR behavior, but the current map is
  sparse (`201/256` bins in the last run).

## Non-Goals

- Do not add non-volatile persistence in this plan.
- Do not replace the existing PI baseline.
- Do not enable DOB or detent feedforward by default until validation passes.
- Do not make MPR the only supported outer-loop regulator.
- Do not hide raw MPR/DOB parameters; keep them available for engineering
  diagnosis even if bandwidth commands become the preferred user interface.

## Safety Constraints

All HIL validation must start from a known safe state:

```text
motor state idle
motor safety timeout 0
motor commission run slow apply
motor outer mode pi
motor velocity dob enable 0
motor commission detent clear
```

Test commands must end with:

```text
motor state idle
motor safety timeout 1000
```

Acceptance criteria for every phase:

- no `OVERCURRENT`, gate-driver fault, hard fault, or spontaneous reset,
- encoder acquisition errors remain zero or within explicitly accepted limits,
- `Id/Iq` references stay within configured current limits,
- generated/open-loop modes are not regressed,
- velocity/position modes can return to `IDLE` cleanly.

## Phase 1: Improve Detent Map Quality

### Problem

The current detent map can be captured and applied, but the latest capture only
populated `201/256` bins. Sparse bins make interpolation uneven and can inject
incorrect feedforward in unobserved regions.

### Implementation Work

1. Add stricter detent capture quality metrics:
   - populated bin count,
   - minimum per-bin sample count,
   - maximum per-bin sample count,
   - rejected sample count,
   - forward/reverse coverage if available,
   - velocity-in-band ratio.
2. Add configurable acceptance thresholds for staged maps:
   - minimum populated bins, default target `250/256`,
   - minimum samples per populated bin,
   - maximum rejected ratio,
   - maximum absolute `Iq` feedforward,
   - maximum adjacent-bin step after smoothing.
3. Add a hole-filling pass before staging:
   - interpolate short missing regions,
   - reject long missing regions,
   - mark filled bins separately in diagnostic output.
4. Add optional smoothing:
   - low-cost circular moving average or first-harmonic-preserving smoothing,
   - configurable smoothing window,
   - never smooth across a long unobserved gap without marking it.
5. Add shell output that explicitly reports whether the staged map is:
   - raw-valid,
   - fill-valid,
   - apply-valid.

### Recommended HIL Workflow

Use a moderate multi-cycle capture first. Avoid ultra-slow captures on hybrid
steppers unless you are explicitly studying stiction; too-slow motion can learn
friction/stick-slip instead of position-periodic detent torque.

```text
motor commission run confirm apply
motor safety timeout 0
motor arm
motor commission detent clear
motor commission detent run 0.10 10 1 0.12
motor commission detent status
```

If motion is smooth and you need more averaging, increase speed and keep the
cycle count high:

```text
motor commission detent run 0.20 10 1 0.12
```

### Acceptance

- `>= 250/256` bins populated or filled from short gaps.
- Encoder errors remain zero during capture on MT6835 hardware.
- Applied map has bounded output and no sharp adjacent-bin discontinuities.
- `motor commission detent status` clearly explains any rejected map.

## Phase 2: Make Detent Commissioning Robust

### Problem

A detent map should represent position-periodic torque only. If capture includes
velocity-loop transient current, viscous friction, Coulomb friction, or DOB
output, the map learns non-periodic controller behavior instead of detent torque.

### Implementation Work

1. Capture forward and reverse passes separately.
2. Store per-direction bin sums and counts.
3. Subtract model feedforward before accumulation:
   - `J * alpha`,
   - `B * omega`,
   - `Tc * sign(omega)`,
   - optionally known velocity-loop feedforward terms.
4. Combine forward/reverse bins to cancel odd friction terms:
   - same-position periodic component is retained,
   - direction-dependent terms are rejected or reported separately.
5. Only accumulate samples when:
   - encoder feedback is valid and fresh,
   - velocity is inside a configured band around target,
   - acceleration magnitude is below a capture threshold unless explicitly
     compensating `J * alpha`,
   - velocity loop is not saturated,
   - DOB is disabled,
   - detent feedforward is disabled.
6. Add a staged-map confidence score:
   - coverage score,
   - forward/reverse agreement score,
   - current ripple score,
   - rejected-sample penalty.
7. Add a validation command that runs low-speed motion before/after applying
   the staged map and reports ripple metrics.

### Suggested Commands

Current command remains:

```text
motor commission detent run <mech_hz> <cycles> [decimation] [iq_limit_a]
motor commission detent apply [enable] [gain] [limit_a]
```

Add or extend with:

```text
motor commission detent validate <mech_hz> <duration_ms>
motor commission detent dump [start_bin] [count]
```

### Acceptance

- Capturing with DOB/feedforward disabled is enforced by the command.
- Forward/reverse map agreement is visible in status.
- Applying the map reduces low-speed velocity ripple or RMS current effort.
- If the map does not improve behavior, the command reports that and does not
  recommend applying it.

## Phase 3: Tune MPR From a Safe Baseline

### Problem

The initial model-derived MPR values were too aggressive. Conservative MPR now
runs without faults, but it does not yet track as tightly as the PI baseline.
MPR tuning needs a repeatable sweep and objective metrics.

### Implementation Work

1. Keep conservative commissioned defaults as the safe starting point:
   - horizon `8`,
   - small `dIq`,
   - large `r_delta_iq`,
   - MPR internal disturbance integrator disabled.
2. Add a scripted or shell-driven MPR sweep:
   - `q_speed`,
   - `r_delta_iq`,
   - `max_delta_iq_a`,
   - horizon fixed at `8` initially.
3. Prefer user-facing bandwidth commands, but make them map into bounded MPR
   ranges proven by HIL.
4. Compare MPR against the known-good PI baseline at:
   - `0.10 Hz`,
   - `0.25 Hz`,
   - `0.50 Hz`,
   - `1.00 Hz` if safe.
5. Collect metrics for each step:
   - mean velocity error,
   - RMS velocity error,
   - peak overshoot,
   - settling time,
   - RMS `Iq_ref`,
   - peak `Iq_ref`,
   - encoder error counters,
   - fault status.
6. Add an MPR tuning result summary command or HIL script report.
7. Update commissioning auto-tune only after the sweep identifies a stable
   range across both directions.

### Candidate Sweep Range

Use the bandwidth interface for operator-facing tuning:

```text
motor velocity mpr bandwidth 1.0
motor velocity mpr bandwidth 2.0
motor velocity mpr bandwidth 3.0
```

Raw `motor velocity mpr set ...` remains available for engineering diagnosis,
but should not be the default HIL/operator interface.

### Acceptance

- MPR matches or improves PI tracking at low speed without increasing current
  spikes or fault risk.
- Positive and negative velocity targets behave symmetrically enough for
  practical operation.
- Zero target returns to near-zero velocity without integrator-like drift.
- Commissioned MPR defaults remain conservative if no better universal values
  are proven.

## Phase 4: Gate and Validate DOB Enablement

### Problem

DOB is useful only after the base velocity loop is stable. If enabled too early,
it can compensate model error, detent ripple, or measurement artifacts in ways
that obscure root-cause tuning.

### Implementation Work

1. Keep DOB disabled after commissioning by default.
2. Add DOB readiness checks:
   - encoder feedback valid/fresh,
   - velocity loop stable,
   - active `Kt`, `J`, and current limits valid,
   - no recent encoder diagnostic bursts,
   - no recent current/gate faults.
3. Reset DOB state on:
   - mode entry,
   - target sign change,
   - target step above threshold,
   - encoder freshness loss,
   - transition to `IDLE`,
   - disabling DOB.
4. Add low-speed clamps:
   - tighter `iq_ff_limit_a` below a configured speed,
   - optional deadband around zero velocity.
5. Improve status output:
   - DOB enabled/disabled reason,
   - disturbance estimate,
   - feedforward current,
   - residual velocity,
   - saturation flag.
6. Add validation workflow:
   - PI only,
   - PI + DOB,
   - MPR only,
   - MPR + DOB,
   - MPR + DOB + detent.

### Acceptance

- DOB can be enabled/disabled at runtime without transients.
- DOB feedforward stays inside the configured bound.
- DOB does not increase overshoot or steady-state ripple in the HIL sweep.
- If DOB readiness fails, status explains why it is disabled.

## Phase 5: Keep Roles Separated and Test Combinations

### Problem

MPR, DOB, and detent feedforward can all affect low-speed motion. If their roles
are not separated, tuning becomes ambiguous and one feature can mask a problem in
another.

### Role Definition

- **PI baseline**: known-good fallback regulator.
- **MPR**: primary velocity/position regulator candidate.
- **DOB**: broadband residual disturbance/load/model-error compensation.
- **Detent feedforward**: position-periodic compensation only.

### Implementation Work

1. Add explicit feature-state status:
   - outer regulator mode,
   - DOB enabled/readiness state,
   - detent enabled/map confidence,
   - active current limits.
2. Ensure each feature has an independent enable/disable path.
3. Ensure each feature can be reset independently.
4. Add HIL combination tests:
   - PI only,
   - PI + detent,
   - PI + DOB,
   - MPR only,
   - MPR + detent,
   - MPR + DOB,
   - MPR + DOB + detent.
5. Use the same velocity targets and acceptance metrics for every combination.
6. Keep generated/open-loop velocity as a regression check, not as a substitute
   for encoder-control validation.
7. Document recommended operational profiles:
   - safe bring-up,
   - normal encoder velocity,
   - low-speed chopper profile,
   - engineering debug.

### Acceptance

- Feature combinations are deterministic and reversible.
- The best-performing combination is selected from evidence, not by assumption.
- Status output makes it obvious which features are active and why.
- Optional features never prevent falling back to PI-only encoder control.

## HIL Evidence Template

For each tested configuration, record:

```text
Date/time:
Firmware commit:
Motor profile overlay:
Commission command:
Feature set:
Velocity target sequence:
Position target sequence, if any:
Encoder errors:
Fault status:
Mean velocity error:
RMS velocity error:
Peak velocity error:
Peak Iq_ref:
RMS Iq_ref:
Observed sound/smoothness:
Verdict:
```

## Proposed Execution Order

1. Phase 1: detent map quality and diagnostics.
2. Phase 2: robust forward/reverse detent commissioning.
3. Phase 3: MPR sweep and safe bandwidth mapping.
4. Phase 4: DOB readiness gates and validation.
5. Phase 5: combination test matrix and recommended profiles.

## Done Criteria

This plan is complete when:

- detent capture reliably produces a high-confidence table on MT6835 hardware,
- MPR has a documented stable tuning range and safe commissioned defaults,
- DOB enablement is gated and validated,
- MPR+DOB+detent is objectively compared against PI-only control,
- all results are reproducible with documented shell/HIL workflows,
- no persistence is required to reproduce behavior after a fresh boot and
  commissioning run.

## Implementation Progress

Updated: 2026-05-06

### Completed

- Phase 1 detent map quality:
  - added raw/fill/apply validity states,
  - added raw/filled bin accounting,
  - added min/max bin counts,
  - added rejected-sample breakdown,
  - added adjacent-bin step check,
  - added short-hole interpolation and circular smoothing,
  - added `motor commission detent dump [start_bin] [count]`.
- Phase 2 robust detent commissioning:
  - forward and reverse bin accumulators are captured independently,
  - residual current subtracts `J*alpha`, `B*omega`, and `Tc*sign(omega)`,
  - capture requires valid/fresh encoder feedback,
  - capture rejects velocity, acceleration, and saturation outliers,
  - forward/reverse agreement is reported,
  - `motor commission detent validate <mech_hz> <duration_ms>` compares
    detent off/on velocity ripple and current effort.
- Phase 3 MPR safe tuning:
  - added `motor velocity mpr bandwidth <hz>`,
  - bandwidth mapping keeps horizon and `dIq` rates bounded,
  - raw `motor velocity mpr set ...` remains available for engineering
    diagnosis.
- Phase 4 DOB gates:
  - DOB defaults stage tuning without enabling,
  - `motor velocity dob enable 1` checks readiness,
  - status reports readiness reason,
  - DOB is rejected while detent feedforward is enabled, because both are
    disturbance compensation mechanisms and the combined path has not produced
    repeatable improvement yet,
  - DOB state resets on disable, zero target, sign change, and large target
    steps.
- Phase 5 combination testing:
  - `motor outer status` reports PI/MPR, DOB readiness, and detent FF state,
  - `motor commission validate velocity ... active` validates the active
    PI/MPR/DOB/detent feature combination instead of forcing PI-only,
  - `scripts/hil/hil_telnet.py mpr-dob-detent` runs standard commissioning,
    detent capture, detent validation, and selected feature-combination
    velocity checks.
  - HIL velocity validation now fails if the internal velocity reference does
    not follow the requested target. This catches a dead/stuck command path that
    can otherwise appear to pass at very low speeds.

### 2026-05-06 Runtime Fixes

- Fixed degraded-feedback velocity hold behavior:
  - before: transient untrusted encoder feedback cleared the velocity
    trajectory target to zero,
  - after: the requested target is preserved and the ramped reference is frozen
    at measured speed until feedback becomes trusted again.
- Tightened detent validation:
  - detent-on validation must not worsen RMS velocity error by more than the
    allowed match ratio,
  - detent-on validation must also not worsen peak velocity error by more than
    the allowed match ratio,
  - status and recommendation output now report both RMS and peak off/on
    errors.

### Validation Evidence

- Parser checks:

```text
python3 -m py_compile scripts/hil/hil_telnet.py
python3 -m unittest scripts/hil/test_hil_telnet_parser.py
Result: PASS, 13/13 tests.
```

- Unit tests:

```text
./tests/run_unit_tests.sh wonderful_goldberg \
  -s chopper.motor_detent_map.unit \
  -s chopper.motor_mpr.unit \
  -s chopper.motor_dob.unit \
  -s chopper.motor_commission_tune.unit
Result: PASS, 4/4 suites, 38/38 test cases.
```

- Firmware build:

```text
podman exec wonderful_goldberg bash -lc \
  'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
Result: PASS.
FLASH: 457192 B / 2 MB
RAM:   199984 B / 512 KB
DTCM:  5888 B / 128 KB
SRAM3: 16 KB / 32 KB
```

- HIL reduced PI/MPR scenario on MT6835 target:

```text
python3 -u scripts/hil/hil_telnet.py mpr-dob-detent \
  --host 10.0.0.44 \
  --yes-live-motion \
  --feature-combo pi \
  --feature-combo mpr \
  --mpr-bandwidth-hz 1.0 \
  --commission-profile confirm \
  --detent-hz 0.10 \
  --detent-cycles 10 \
  --detent-iq-limit 0.12 \
  --detent-validate-hz 0.10 \
  --detent-validate-ms 3000 \
  --velocity-hz 0.50 \
  --velocity-hold-ms 1000 \
  --json-report hil_logs/mpr_dob_detent/reduced_pi_mpr_after_fixes.json
Result: PASS.
```

Observed HIL details:

- standard commissioning completed,
- encoder acquisition errors stayed at zero,
- detent capture staged a map with `256/256` populated/filled bins,
- detent map quality gate passed,
- PI and MPR velocity validations passed with no faults.

Important caveat from HIL:

- The captured detent map passed structural quality gates, but the off/on
  validation at `0.05 Hz` reported `WORSE` for this run. Treat this map as an
  engineering artifact, not a default behavior, until repeated captures show
  consistent ripple improvement.
- A later full matrix run exposed a false-pass condition: MPR validations could
  show `ref=0.000 Hz` for nonzero targets and still pass the old HIL parser
  because target/measured error remained below the loose low-speed threshold.
  The HIL parser now checks `|target-ref|`.
- A subsequent clean HIL rerun was blocked because the telnet shell stopped
  responding after target reset/flash. No live pass is claimed for the
  2026-05-06 runtime fixes until the shell/target state is recovered and the
  narrowed PI/MPR/MPR+DOB validation is rerun.
- 2026-05-06 resume attempt:
  - flashed current firmware successfully through the J-Link runner,
  - safe status check passed and confirmed the current command tree was present,
  - the narrowed validation was started but commissioning immediately latched
    `HARDWARE_BREAK`,
  - `motor info live` reported `Vbus: 0.0 V`, so the power stage was not in a
    state suitable for live motion,
  - `motor gate reset` and `motor state clear_error` recovered the state machine
    to `IDLE`, but `Vbus` remained `0.0 V`,
  - command timeout was restored to `1000 ms`,
  - no MPR/DOB/detent conclusion is drawn from this attempt.

### Current Completion State

- MPR bandwidth interface and unit-tested core implementation are complete.
  Public velocity MPR presets have been removed from the shell; use bandwidth
  for operator-facing tuning and raw `set` only for engineering diagnosis.
- DOB readiness/status and conservative enable flow are complete.
- Detent map capture/validation infrastructure is implemented but not accepted
  as default behavior. The map must show repeatable RMS and peak improvement
  before enabled apply should be used.
- `MPR+DOB+detent` is intentionally blocked by readiness policy until detent
  feedforward is validated independently and a combined compensation strategy is
  retuned.

### Next Live Validation Command

After confirming the power stage reports a valid `Vbus` and the target shell is
responsive, rerun the narrowed validation before any full detent matrix:

```bash
python3 -u scripts/hil/hil_telnet.py mpr-dob-detent \
  --host 10.0.0.44 \
  --yes-live-motion \
  --feature-combo pi \
  --feature-combo mpr \
  --feature-combo mpr_dob \
  --mpr-bandwidth-hz 1.0 \
  --velocity-hz 0.50 \
  --velocity-hold-ms 1000 \
  --detent-hz 0.10 \
  --detent-cycles 10 \
  --detent-iq-limit 0.12 \
  --detent-validate-hz 0.10 \
  --detent-validate-ms 3000 \
  --json-report hil_logs/mpr_dob_detent/narrow_pi_mpr_mprdob_after_hold_fix.json
```

### 2026-05-06 ADC Sampling Regression Fix

- Root cause: `motor_control_loop_step()` skipped the ADC measurement stage when
  `MOTOR_FEATURE_PWM_OUTPUT` was not enabled. This made `motor info live` report
  stale/zero `Vbus` in `IDLE` and obscured the real power-stage state.
- Fix: ADC current/Vbus measurement now runs every ISR tick before the PWM-output
  gate. Live Vbus telemetry is updated unconditionally from the injected ADC
  sample.
- Fault policy: invalid/overvoltage Vbus faults are still only raised in active
  calibration/online modes where the power stage requires valid bus voltage.
- Validation:
  - MT6835 west build passed.
  - Firmware flashed successfully.
  - `motor info live` in `IDLE` reports `Vbus: 23.6 V`.
  - Controller recovered to `IDLE`, `Error: NONE`, command timeout restored to
    `1000 ms` after the interrupted live commissioning attempt.
