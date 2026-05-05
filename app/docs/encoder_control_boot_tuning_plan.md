# Encoder Control Boot Tuning Plan

## Goal

Make encoder-based current, velocity, and position modes safe enough to continue
HIL tuning without relying on persisted commissioning data.

Every boot still requires:

1. Current offset calibration.
2. Generated-sweep encoder mapping.
3. Apply runtime encoder mapping.
4. Validate that positive `Iq` produces measurable encoder motion.

## Offset Convention

Generated-sweep encoder mapping uses d-axis excitation:

- `Id = current_a`
- `Iq = 0`
- generated mechanical angle is swept at `mech_hz`

With this convention the detected generated-reference offset is the mechanical
FOC commutation offset directly. There is no `+/-90 electrical degree`
correction. The earlier correction was only required because the sweep used
`Iq` excitation, which intentionally applies the stator vector on the q-axis.

## Implementation Phases

### Phase 1: Explicit Offset Convention

- Update comments and shell status text to say the mapping sweep is an `Id`
  d-axis sweep.
- Apply the staged encoder mapping directly from the detected offset.
- Update boot commissioning documentation to describe the d-axis convention.

### Phase 2: Boot +Iq Validation

- Extend `motor commission boot` with a final validation stage.
- Switch to `current_encoder`, apply a short bounded `+Iq` pulse, and verify
  raw encoder trace reports measurable motion.
- Always return current commands to zero before exiting the validation path.

### Phase 3: Velocity Encoder Defaults

- Make safe/nominal velocity PI defaults less likely to command excessive
  current.
- Use the commissioned motion-threshold result, when available, to choose a
  practical `Iq` limit.
- Keep the defaults conservative until velocity HIL tuning is complete.

### Phase 4: Anti-Windup and Zero-Target Reset

- Add conditional-integration anti-windup to position and velocity PI modules.
- Reset the velocity PI integrator when the velocity target and reference are
  zero and measured speed is near zero.
- Reset the position PI integrator when holding the target with small position
  error.

### Phase 5: Bounded Position Target

- Rework `motor position target <deg>` to plan a bounded quintic move from the
  measured position.
- Use configured profile velocity and acceleration limits to choose a safe
  duration.
- Reject targets only if a bounded segment cannot be planned.

## Validation

- Unit test PI anti-windup behavior.
- Build firmware with the standard west build path.
- Flash target and run:

```text
motor commission boot 0.15 0.10 1
motor encoder control_status
motor state mode current_encoder
motor arm
motor current iq 0.06
motor current iq 0
motor state mode velocity_encoder
motor velocity gains defaults safe
motor velocity target 0.25
motor velocity target 0
motor state mode position_encoder
motor position target <nearby absolute deg>
```

## Progress

- [x] Phase 1 complete.
- [x] Phase 2 complete.
- [x] Phase 3 complete.
- [x] Phase 4 complete.
- [x] Phase 5 complete.

## Evidence

- `python3 zephyr/scripts/twister -T chopper/tests/unit -p native_sim --outdir /tmp/twister-unit-pi --inline-logs -v -s chopper.pi_controller.unit`: passed, 8/8 cases.
- `west build --build-dir /workspace/build/chopper/smartstepper_v2`: passed.
- HIL `motor commission boot 0.15 0.10 1` with Id-axis sweep:
  - mapping valid, direction `-1`, commutation offset about `0.20 deg mechanical`.
  - `+Iq=0.060 A` validation produced positive control-coordinate motion.
  - one run observed `warn=1 err=1` during the short validation pulse, so the validation path now uses the same small error budget as mapping and reports the counts.
- HIL `velocity_encoder` with the first retuned safe defaults showed the original profile-max-speed-based gains were too small, then the higher gains saturated and oscillated. Safe defaults were reduced to use a `0.060 A` current limit and a low-speed gain point. Further closed-loop velocity tuning is still required on hardware.
- `motor velocity gains defaults <safe|nominal>` and `motor position gains defaults <safe|nominal>` now prefer commissioned `psi_f/J/B/Kt` model gains when valid identification data or applied auto-tune data exists. The commands print `source=model` when those values are used and `source=empirical` when falling back to bring-up heuristics.
- HIL command smoke test after flash, before identification data existed:
  - `motor velocity gains defaults safe`: `source=empirical`, `Kp=0.00955`, `Ki=0.00955`, `Iq limit=0.060 A`.
  - `motor position gains defaults safe`: `source=empirical`.
- HIL `motor commission auto run slow` initially rejected every mechanical fit
  even though capture quality was good. Added per-attempt diagnostics; the
  captures had no rejected samples and high `R2`, but the fitted `J/B` signs
  were negative. Mechanical ID now evaluates both torque polarities and selects
  the physically valid result.
- Final HIL slow auto commissioning after the torque-sign fix:
  - `psi_f=0.00141178 Wb`, flux `R2=0.9281`, `N=312`.
  - `J=0.00002685 kgm2`, `B=0.00017403 Nm/(rad/s)`,
    `Tc=0.00266001 Nm`, mechanical `R2=0.9218`, validation `PASS`.
  - selected mechanical fit torque sign `-1`; mapping direction diagnostic
    still reports a negative acceleration/current correlation for the
    velocity-loop path.
  - staged auto-tune values:
    `velocity Kp=0.03022`, `Ki=1.00093`, `Iq limit=0.101 A`;
    `position Kp=25.13274`, `Ki=157.91368`.
  - after valid identification data exists, `motor velocity gains defaults safe`
    and `motor position gains defaults safe` report `source=model`.
