# Advanced Motion Features for Hybrid Stepper FOC

This document defines five high-value motion-control features to add next:

1. Torque ripple compensation map
2. Anti-resonance control
3. Jerk-limited online trajectory replanning
5. Trigger phase-lock mode
7. Commissioning auto-tune pipeline with confidence gating

The design targets this codebase structure:

- `modules/motor_core/src/control`
- `modules/motor_core/src/motion`
- `modules/motor_core/src/observers`
- `modules/motor_core/src/estimation`
- `modules/motor_core/src/runtime`
- `modules/motor_core/src/telemetry`

And this runtime partitioning philosophy:

- Fast loop: deterministic ISR-safe control math
- Deferred path: event processing, fit/solve, and logging

## Common Design Rules

- Keep ISR math bounded and branch-light.
- Add explicit `init()/reset()/step()` APIs for every new module.
- Keep all units SI internally (rad, rad/s, A, Nm, s).
- Use Kconfig for feature enable/disable and compile-time sizing.
- Use devicetree for hardware and installation parameters.
- Use shell only as command/control surface; heavy processing stays outside ISR.

## 1) Torque Ripple Compensation Map

### Goal

Reduce periodic speed ripple, audible roughness, and current oscillation caused by:

- detent torque/cogging
- electrical nonlinearity
- mechanical periodic disturbance

### Proposed Method

Add a periodic feedforward term indexed by electrical angle:

- `iq_ff_ripple = LUT(theta_elec)`
- optional 2D extension: `LUT(theta_elec, speed_bin)`

Apply in the velocity/torque command path before final `iq` clamp.

### Module Layout

- New: `modules/motor_core/include/motor/control/ripple_ff.h`
- New: `modules/motor_core/src/control/ripple_ff.c`

Suggested API:

- `motor_ripple_ff_init(cfg, state)`
- `motor_ripple_ff_reset(state)`
- `motor_ripple_ff_eval(cfg, state, theta_elec_rad, omega_mech_rad_s, &iq_ff_a)`

### Data Model

- fixed-size LUT (e.g., 128 or 256 points per electrical cycle)
- optional speed bins (e.g., 4-8 bins)
- linear interpolation between adjacent bins

### Commissioning Flow

1. run constant-speed sweep in `velocity_open` or stable `velocity_closed`
2. record periodic speed error vs electrical angle
3. fit feedforward LUT minimizing periodic error
4. validate reduction in RMS ripple before apply

### Safety/Constraints

- clamp `iq_ff_ripple` to configured limit
- disable if encoder quality degrades
- include "ripple_ff active" flag in telemetry

### Validation

- Unit: LUT interpolation, wrap behavior, clamps
- HIL: compare speed ripple RMS before/after at fixed speeds

## 2) Anti-Resonance Control

### Goal

Stabilize hybrid-stepper mid-band resonance and load-coupled vibration while preserving bandwidth.

### Proposed Method

Two-layer approach:

1. Narrow notch filter on velocity feedback at identified resonance frequency.
2. Active damping term proportional to estimated oscillatory component.

Start with notch-only path, then add active damping if needed.

### Module Layout

- New: `modules/motor_core/include/motor/control/anti_resonance.h`
- New: `modules/motor_core/src/control/anti_resonance.c`

Suggested API:

- `motor_anti_res_init(cfg, state)`
- `motor_anti_res_reset(state)`
- `motor_anti_res_step(cfg, state, omega_meas, iq_cmd_in, &omega_used, &iq_cmd_out)`

### Parameterization

- notch center frequency `f0`
- notch depth/quality `Q`
- optional damping gain `k_damp`

### Identification

- commissioning chirp/PRBS excitation
- detect dominant resonance peak from response
- auto-propose `f0` and `Q`, operator can override

### Safety/Constraints

- constrain damping injection current
- bypass module on invalid speed/angle quality
- expose resonance suppression status in telemetry

### Validation

- Unit: filter stability and coefficient bounds
- HIL: sweep response with/without anti-resonance, confirm reduced peak magnitude

## 3) Jerk-Limited Online Trajectory Replanning

### Goal

Allow live target updates with guaranteed continuity and bounded jerk:

- no command discontinuities
- smooth replan while already moving
- deterministic abort/hold/replan semantics

### Proposed Method

Use online S-curve (jerk-limited) segment generation with continuity constraints:

- maintain continuity in `position`, `velocity`, and `acceleration`
- cap `jerk`, `accel`, and `velocity`
- support replan from current state at each trigger/update

### Module Layout

- Extend `modules/motor_core/src/motion/motion_profile.c`
- or add: `modules/motor_core/src/motion/scurve_profile.c`

Suggested API:

- `motor_motion_replan(state_now, target, limits, &segment)`
- `motor_motion_eval(segment, t, &pos_ref, &vel_ref, &acc_ref)`
- `motor_motion_abort_to_hold(state_now, limits, &stop_segment)`

### Runtime Semantics

- deterministic priority: `abort > safety_hold > replan > normal advance`
- replan requests latched and consumed at loop boundary
- all transitions produce bounded outputs

### Validation

- Unit: continuity checks at replan boundaries
- Unit: limit enforcement at edges
- HIL: repeated target updates during motion with no spikes in `iq`/`vq`

## 5) Trigger Phase-Lock Mode

### Goal

Lock motion phase to an external timing source (camera/chopper/frame trigger):

- deterministic move per trigger
- bounded phase error
- graceful operation across low-speed stop-and-go and high-speed continuous motion

### Proposed Method

Add a digital phase-locked scheduler on top of sequence/profile engine:

- phase detector from trigger timestamp vs expected phase
- low-bandwidth phase/frequency correction loop
- optional feedforward from measured trigger period

This extends existing timer/external-trigger sequence support.

### Module Layout

- New: `modules/motor_core/include/motor/motion/trigger_pll.h`
- New: `modules/motor_core/src/motion/trigger_pll.c`
- Integrate in `app/src/shell_motion_sequence.c` and online position flow

Suggested API:

- `motor_trigger_pll_init(cfg, state)`
- `motor_trigger_pll_on_trigger(state, t_capture)`
- `motor_trigger_pll_step(cfg, state, t_now, &phase_correction)`

### Control Strategy

- low-speed regime: endpoint move + dwell with phase correction on next segment
- high-speed regime: continuous profile with small phase nudges

### Safety/Constraints

- glitch reject and minimum trigger interval
- fallback to internal scheduler after timeout/loss-of-trigger
- explicit status/fault counters exposed in shell

### Validation

- Unit: phase detector/loop update math
- HIL: trigger jitter injection and phase-error histogram

## 7) Commissioning Auto-Tune Pipeline with Confidence Gating

### Goal

One command performs identify + fit + tune + staged apply, only if quality is acceptable.

### Proposed Pipeline

1. Preconditions:
- correct state, armed/disarmed policy, thermal limits, supply checks

2. Data capture:
- Rs, RoverL, long Rs-est data
- mechanical response data for inertia/friction
- resonance sweep data (for anti-resonance)

3. Estimation/fits:
- electrical and mechanical parameter estimation
- confidence metrics per estimator (residual RMS, condition number, repeatability)

4. Controller synthesis:
- derive defaults for current loop, velocity/position (PI or MPR), DOB
- derive notch/ripple settings if enabled

5. Confidence gating:
- apply only parameters that pass thresholds
- keep previous values for failed estimates

6. Staging:
- preview -> apply -> verify -> commit (future persistence)

### Module Layout

- Extend `modules/motor_core/src/runtime/commission_tune.c`
- Add estimator helpers under `modules/motor_core/src/estimation`
- Add quality structs in `modules/motor_core/include/motor/estimation`

Suggested API:

- `motor_commission_run(cfg, io, &result)`
- `motor_commission_score(result, &confidence)`
- `motor_commission_apply(result, mask)`

### Shell Surface

Suggested command group:

- `motor commission auto start`
- `motor commission auto status`
- `motor commission auto apply`
- `motor commission auto report`

### Validation

- Unit: estimator scoring and threshold logic
- HIL: repeated runs for parameter variance and acceptance rate

## Integration Order (Recommended)

1. Commissioning confidence framework (item 7 foundation)
2. Trigger phase-lock mode (item 5)
3. Jerk-limited online replan (item 3)
4. Anti-resonance (item 2)
5. Ripple compensation map (item 1)

Reason:

- Item 7 provides robust quality gates and tuning infrastructure.
- Item 5/3 improve deterministic motion behavior directly.
- Items 2/1 depend on good identification and clean telemetry.

## Telemetry Additions

Add compact runtime telemetry (gated by Kconfig):

- ripple feedforward output and saturation counter
- anti-resonance activity and notch parameters
- trajectory replan count, abort count, continuity fault count
- trigger PLL phase error, frequency error, trigger reject count
- commissioning confidence scores and rejected-parameter mask

## Kconfig and Devicetree Split

Use Kconfig for feature toggles and buffer sizing:

- `CONFIG_MOTOR_RIPPLE_FF`
- `CONFIG_MOTOR_ANTI_RESONANCE`
- `CONFIG_MOTOR_SCURVE_REPLAN`
- `CONFIG_MOTOR_TRIGGER_PLL`
- `CONFIG_MOTOR_COMMISSION_AUTO`

Use devicetree for hardware/profile defaults:

- trigger input source/channel defaults
- encoder direction and installation-specific constants
- motor baseline profile defaults

## Acceptance Criteria

A feature is complete when:

- module has explicit init/reset/step API
- unit tests cover normal path + edge cases
- no ISR budget regression beyond agreed threshold
- HIL validation demonstrates measurable benefit
- shell commands provide status and control without ambiguity
