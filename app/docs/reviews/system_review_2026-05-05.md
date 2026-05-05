# Chopper Motor-Control System Review - 2026-05-05

## Purpose

This document records the current state of the chopper project after the recent
motor-core, encoder, commissioning, ISR, and shell refactors. It is intended to
be the durable map of what the system contains, how the pieces interact, and
where the design is strong or weak compared with two reference projects:

- TI C2000Ware MotorControl SDK at
  `/home/dgamroth/workspaces/motor/C2000Ware_MotorControl_SDK_5_04_00_00/libraries`.
- VESC firmware at `/home/dgamroth/workspaces/motor/bldc`.

## Executive Summary

The project now has a credible split between reusable real-time motor-control
algorithms in `modules/motor_core` and Zephyr/application integration in
`app/src`. The library side is organized in a TI-like way: filters, transforms,
observers, motion modules, control modules, estimators, compensation, protection,
and runtime policy. Unit-test coverage for library algorithms is broad and much
better than typical early embedded motor-control code.

The main weakness is not algorithm availability. The main weakness is that the
application layer still contains too much system behavior in large files,
especially commissioning and shell commands. The ISR path is much cleaner than it
was, but it still depends on a broad `struct motor_parameters` owner object and
some policy refresh code. The system is usable for generated/open-loop current
control and boot encoder mapping, but encoder-based velocity/position operation
is not yet tuned or proven stable.

## Current System Architecture

### Hardware and Zephyr Integration

Key files:

- `app/src/main.c`: starts the application and motor state-machine thread.
- `app/src/motor_hardware.c`: owns device binding and hardware readiness checks.
- `app/src/motor_states.c`: top-level SMF state machine, hardware setup, ISR
  callback installation, state table, and event dispatch.
- `app/src/motor_states_calibration.c`: boot calibration, electrical
  commissioning states, and generated alignment/calibration helpers.
- `app/src/motor_states_online.c`: online generated and encoder-control modes.
- `app/src/motor_isr_io.c`: ADC ISR glue, encoder request/collect bridge,
  PWM apply, fault snapshot publishing, and ISR timing telemetry.
- `app/src/motor_control_loop.c`: real-time control-loop dataflow from ADC and
  encoder sample to current control, PWM duty outputs, telemetry, and faults.

The control ISR is structured as:

1. Collect: drain encoder acquisition and package ADC inputs.
2. Process: run `motor_control_loop_step()`.
3. Apply: write PWM duty updates.
4. Telemetry: update ISR timing and bounded trace buffers.

This is the right high-level shape for a real-time motor controller. It mirrors
the "sample, run control, update PWM" style used by TI and VESC while still
being idiomatic Zephyr at the application boundary.

### State Machine and Commands

The application uses Zephyr SMF for lifecycle and operating modes. Shell commands
post API events into the state-machine thread instead of directly mutating
hardware in most paths.

Important operating modes:

- `ONLINE_VELOCITY_GENERATED`: generated-angle velocity drive using the current
  loop. This is the most proven motion path.
- `ONLINE_POSITION_GENERATED`: generated-angle position/profile drive.
- `ONLINE_CURRENT_ENCODER`: encoder-commutated direct Id/Iq control.
- `ONLINE_VELOCITY_ENCODER`: encoder-feedback velocity control.
- `ONLINE_POSITION_ENCODER`: encoder-feedback position/profile control.

Important commissioning commands:

- `motor commission boot <current_a> <mech_hz> <cycles>`: runtime-only boot gate
  until persistence exists. It performs current offsets, generated-sweep encoder
  mapping, applies encoder direction/offset, and runs a short +Iq validation.
- `motor commission validate current [iq_a] [hold_ms]`: current encoder smoke
  validation.
- `motor commission validate velocity [max_hz] [hold_ms]`: conservative
  velocity-encoder PI validation.
- `motor commission validate position [delta_deg] [hold_ms]`: bounded
  position-encoder validation.
- `motor commission auto ...`, `motor commission flux ...`, `motor commission
  mech ...`, and `motor commission detent ...`: longer identification and tuning
  flows.

### Encoder Acquisition and Feedback

Key files:

- `app/src/motor_encoder_acquisition.c`: application bridge for asynchronous
  encoder request/collect and diagnostic counters.
- `drivers/sensor/brcm_aeat9955_fast*` and `drivers/spi/rt_spi*` paths: reduced
  latency AEAT-9955 path used for real-time encoder acquisition.
- `modules/motor_core/src/observers/encoder_feedback_core.c`: control-quality
  state machine for fresh/warn/error/dropout behavior.
- `modules/motor_core/src/observers/encoder_feedback.c`: combines raw encoder
  sample, generated angle, observer, and feedback-quality output.
- `modules/motor_core/src/observers/angle_observer.c`: wrap, direction/offset,
  speed estimate, latency compensation, and dropout prediction.
- `modules/motor_core/src/observers/angle_path.c`: chooses encoder, generated,
  or propagated observer path.

Current intended behavior:

- Fresh valid encoder frames correct the observer.
- CRC/status/transport-bad frames are not fed into the observer correction.
- Bounded bad-sample windows propagate angle using `angle_observer_predict()`.
- Propagated samples are valid for FOC commutation but not fresh for outer-loop
  integration.
- After threshold, feedback becomes invalid/error and encoder modes should hold
  or fault deterministically.

This is a pragmatic solution for the AEAT-9955 path while still surfacing raw
frame diagnostics.

### motor_core Library Layout

`modules/motor_core` is now organized similarly to TI-style control libraries:

- `math`: angle wrapping, constants, PRBS, small matrix solve.
- `filters`: first-order/second-order filters and PI.
- `control`: current loop, transforms, FOC voltage/PWM, DQ decoupling, DOB, MPR,
  torque, velocity/position regulators.
- `motion`: angle generator, trajectories, quintic motion profile, motion
  planner, outer-loop decimation scheduler.
- `observers`: angle observer, encoder feedback, feedback quality.
- `calibration`: current offset, alignment, R/L identification, encoder-map
  detection, timing/window helpers.
- `estimation`: flux ID, mechanical ID, RLS, online Rs, thermal model.
- `compensation`: detent feedforward map.
- `runtime`: command arbitration, control policy, current-ref policy,
  outer-loop runtime, keepalive, commissioning runtime/tune, config snapshots.
- `protection`: interlocks.
- `telemetry`: capture structures.

This is a strong decomposition. The biggest remaining concern is that `runtime`
contains some modules that are conceptually policy/application glue rather than
pure algorithms. That is acceptable for now because the ISR still needs a stable
adapter layer between `app` and pure blocks.

## Real-Time ISR Suitability

### Strengths

- No dynamic allocation in the fast control path.
- Shell formatting/logging is not in `motor_control_loop_step()`.
- Encoder trace/fault/capture telemetry is bounded and opt-in.
- The ISR publish path uses an ISR-safe event enqueue for control-loop faults.
- Expensive validation is increasingly split into init/validate and fast-step
  variants (`*_step_fast`).
- Most library modules have explicit state/config structs that can be allocated
  statically.

### Weaknesses

- `motor_control_loop_step()` still takes `struct motor_parameters *`, a broad
  app-global owner containing calibration, shell-visible live data, telemetry,
  profiles, observers, runtime adapters, and state-machine data. This is not as
  clean as TI's small module handles or VESC's tightly controlled motor state
  objects.
- `struct motor_rt_control_ctx` avoids large stack allocation, but it is itself a
  fairly broad per-tick scratch object. It is better than stack churn, but it is
  not yet a minimal control kernel API.
- `motor_control_loop.c` remains large at about 1400 lines. It has stage helpers,
  but it still mixes policy derivation, encoder feedback, commissioning
  observation, current-reference policy, FOC, fault reporting, and live telemetry
  publication.
- `encoder1_callback()` does not use kernel queues, but it still evaluates
  feature flags/capture flags and updates acquisition enable state. For a direct
  ISR, the ideal is a minimal "request sample if gate enabled" path with policy
  state precomputed elsewhere.
- Gate-driver break callbacks still call `motor_api_post_error()`. If those
  callbacks are ever configured as zero-latency/direct ISR context, this must be
  moved to the same ISR-safe event/latch path as ADC faults.
- Telmetry at high decimation rates is bounded but can still perturb ISR timing;
  it should remain opt-in and explicitly disabled for performance measurements.

## Comparison: TI C2000Ware MotorControl SDK

### Where We Match TI Well

- Library taxonomy is now close: filters, transforms, PI, observers, angle_gen,
  traj, control, sensing, and utilities are separate concepts.
- Many module APIs use explicit config/state structs and no heap allocation.
- The generated angle and trajectory blocks map naturally to TI's `ANGLE_GEN`,
  `TRAJ`, and ramp modules.
- Current-loop and PWM synthesis are separated enough to unit test.
- Fast-path variants mirror TI's preference for prevalidated ISR block calls.

### Where TI Is Still Cleaner

- TI blocks are generally smaller and lower ceremony. A module handle points to a
  small object, setters/getters are inline, and the ISR calls direct `*_run()`
  functions with minimal policy logic.
- TI examples keep the ISR sequence explicit and hardware-near. Our ISR is more
  abstract because it also supports shell-configured modes, commissioning,
  telemetry, and runtime policy.
- TI's C2000 environment is built around deterministic peripheral triggering and
  direct hardware register access. Our Zephyr integration has more glue because
  it bridges drivers, SMF, shell, and asynchronous encoder acquisition.
- TI has mature position-sensing libraries for QEP/T-format/BiSS-C/PTO. Our
  encoder path is newer and still being hardened around AEAT-9955 protocol and
  low-latency SPI transport.

### Lessons To Keep Applying

- Keep hot ISR modules small, explicit, and prevalidated.
- Push configuration checks into init/setter functions.
- Prefer direct block calls in the ISR over dynamic policy branching where
  possible.
- Keep telemetry and commissioning outside the fast path unless specifically
  enabled.

## Comparison: VESC bldc

### Where We Match VESC Well

- We have a shell/terminal-driven commissioning workflow similar in spirit to
  VESC terminal commands for resistance, inductance, flux linkage, encoder
  detection, and sensor detection.
- The project has explicit fault snapshots and diagnostic counters.
- The encoder mapping workflow resembles VESC's encoder-detect concept:
  energize, move/observe, compute offset/direction, apply.
- Generated/open-loop FOC current drive is a proven fallback path, similar to
  VESC's open-loop FOC commands.

### Where VESC Is Stronger

- VESC has mature, field-tested motor detection flows that apply complete motor
  configuration results and expose clear terminal outputs.
- VESC has strong operator tooling expectations: detection commands, result
  reporting, fault history, and command-line workflows are first-class.
- VESC's control stack is more monolithic, but that means the runtime behavior is
  easier to reason about once configured.
- VESC has many hardware safety details accumulated from real products.

### Where Our Design Is Cleaner

- Our reusable algorithms are better isolated for unit testing than much of the
  VESC app/control monolith.
- Our motion/actuator policy model is more general. It can evolve toward FOC,
  brushed, and step/dir actuation using the same motion-control layer.
- Our Zephyr shell command tree is becoming orthogonal: `current`, `velocity`,
  `position`, `outer`, `commission`, `encoder`, `fault`, `safety`, and `state`.

## Testing Assessment

### Current Unit-Test Coverage

There are 29 unit-test suites under `tests/unit`, with more than 240 individual
`ZTEST()` cases by source inspection. Important covered areas include:

- `angle_observer`: observer update, prediction, wrap, offset, delay.
- `motor_encoder_feedback_core`: fresh/warn/error/dropout quality behavior.
- `motor_encoder_map_detect`: encoder mapping quality and rejection paths.
- `motion_profile` and `traj`: profile planning/evaluation and trajectory ramping.
- `motor_mpr`, `motor_dob`, `pi_controller`, `pi_filter`: control algorithms and
  edge cases.
- `motor_foc_voltage_pwm`: validation, decoupling, voltage limiting, PWM bounds,
  and fast-path equivalence.
- `motor_commission_estimators`, `motor_rl_ident`, `rs_online`,
  `rls_motor_est`, `thermal_model`: estimator behavior and reject paths.
- `motor_control_policy`, `runtime`, `motor_current_ref_policy_core`,
  `motor_keepalive_policy`, `motor_outer_loop_sched`: runtime policy blocks.

This is adequate coverage for pure library functions. The largest testing gap is
not pure algorithms; it is integration behavior across shell/API, SMF state
transitions, ISR event queues, and real hardware timing.

### Testing Gaps

- No automated HIL regression gate existed before this review.
- No hardware-map/Twister HIL setup is defined yet.
- No automated acceptance parser checks that commissioning results pass minimum
  thresholds; current HIL logs are operator-reviewed.
- No test currently asserts that every encoder-control mode is rejected before
  boot encoder mapping and accepted afterward at the shell/API level.
- No deterministic software-in-the-loop model validates velocity/position tuning
  against the estimated stepper parameters.
- No formal ISR timing budget test exists beyond runtime max-cycle telemetry.

### Recommended Test Policy

- Keep unit tests focused on `motor_core`; this is already working well.
- Add integration tests only where they protect state-machine/API contracts.
- Use telnet HIL scripts for bring-up evidence until a full Twister hardware map
  is worth the maintenance cost.
- Save HIL logs per test run and record key outcomes in review/bring-up docs.
- For every encoder-control bug fix, run at minimum:
  - `chopper.angle_observer.unit`
  - `chopper.motor_encoder_feedback_core.unit`
  - `chopper.motor_encoder_map_detect.unit`
  - `chopper.motor_control_policy.unit`
  - HIL `boot-commission`
  - HIL `encoder-validate` without position first.

## HIL Automation Added

A repeatable telnet harness now exists at:

```text
scripts/hil/hil_telnet.py
```

Examples:

```bash
python3 scripts/hil/hil_telnet.py status --host 10.0.0.171
python3 scripts/hil/hil_telnet.py boot-commission --yes-live-motion --host 10.0.0.171
python3 scripts/hil/hil_telnet.py encoder-validate --yes-live-motion --host 10.0.0.171
python3 scripts/hil/hil_telnet.py encoder-trace-open-loop --yes-live-motion --host 10.0.0.171
```

The script writes logs under `hil_logs/` by default and sends best-effort stop
commands at the end of live-motion scenarios.

## Current Known Hardware State From Latest HIL

Latest observed AEAT-9955 run after the stabilization changes:

- `motor commission boot 0.15 0.05 1` passed.
- Encoder mapping was valid with direction `-1`.
- Commutation offset was about `0.20 deg mechanical`.
- Mapping motion was about one mechanical revolution.
- Mapping warn/error counts were zero.
- `motor encoder control_status` reported ready after acquisition counters were
  reset.
- `current_encoder` smoke test did not fault but showed asymmetric +Iq/-Iq
  motion.
- `velocity_encoder` PI validation ran without encoder warnings/errors but was
  not stable; measured velocity overshot the small target.
- `position_encoder` was not tested after that velocity result.

## Strengths

- Good separation of algorithmic library code from Zephyr app integration.
- Strong native_sim unit-test base for pure algorithms.
- ISR path is staged and avoids dynamic allocation.
- Encoder diagnostic visibility is much better than typical first-pass motor
  firmware.
- Motion/actuator policy model is capable of supporting non-FOC actuators in the
  future.
- Commissioning commands are becoming explicit enough for repeatable workflows.
- Telnet shell gives a better HIL automation channel than UART under log load.

## Weaknesses and Risks

1. `shell_motion_commission.c` is too large. It contains command parsing,
   workflow orchestration, measurement loops, staged results, detent capture,
   auto tuning, and validation. This should be split by workflow once behavior
   stabilizes.
2. `shell_commands_state.c` and `shell_commands.c` are also large. The command
   tree is clearer now, but implementation files need further decomposition.
3. `motor_control_loop.c` still has too many responsibilities. It is much better
   than before but should eventually become a thin app adapter around a smaller
   reusable control-kernel step.
4. Encoder-control modes are not tuned. Current encoder works enough to avoid
   faults, but the torque response asymmetry and velocity overshoot show that
   sign/offset/tuning/detent issues remain.
5. The boot gate name can be misleading. It aligns/maps the encoder, but it does
   not replace full motor identification/tuning.
6. Runtime persistence is still missing, so every boot requires current offset
   and encoder mapping before encoder-control modes.
7. HIL scripts currently capture logs but do not yet enforce pass/fail criteria
   from parsed numeric thresholds.
8. Direct ISR policy needs one more tightening pass: precompute encoder sampling
   gates and keep `encoder1_callback()` as close as possible to a request-only
   routine.
9. Gate-driver break callbacks should be audited for ISR context. If direct or
   zero-latency, they must not call non-ISR-safe APIs.

## Priority Recommendations

1. Stabilize encoder-control PI before returning to MPR/DOB/detent FF.
2. Extend `motor commission boot` or add a `motor commission encoder robust`
   command that performs multi-cycle and bidirectional generated-sweep mapping.
3. Add explicit HIL pass/fail parsing to `scripts/hil/hil_telnet.py` once stable
   numeric thresholds are agreed.
4. Split `shell_motion_commission.c` into separate files by workflow:
   boot/encoder, motion threshold, flux, mechanical ID, detent, auto tune,
   validation.
5. Move one more layer from `motor_control_loop.c` into `motor_core`: a small
   control-kernel step that consumes explicit refs/measurements/config snapshots
   instead of `struct motor_parameters *`.
6. Keep applying the TI pattern: init/validate once, run fast blocks in ISR.
7. Use VESC as the benchmark for practical commissioning UX: commands should
   print parameters, quality metrics, fault reason, and suggested next command.
8. Add persistence only after the boot mapping and velocity PI behavior are
   repeatably stable.

## Immediate Next Validation Sequence

Recommended manual or scripted sequence after flashing:

```text
motor state status
motor safety timeout 0
motor commission boot 0.15 0.05 1
motor encoder control_status
motor commission validate current 0.03 160
motor commission validate velocity 0.03 1000
motor state status
motor encoder acquisition
motor fault snapshot status
```

Equivalent script:

```bash
python3 scripts/hil/hil_telnet.py encoder-validate \
  --yes-live-motion \
  --host 10.0.0.171 \
  --boot-current 0.15 \
  --boot-hz 0.05 \
  --cycles 1 \
  --current-iq 0.03 \
  --velocity-hz 0.03
```
