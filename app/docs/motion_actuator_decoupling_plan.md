# Motion Control / Motor Commutation Decoupling Plan

## Goal

Decouple the motion-control system from motor commutation so the same motion/profile/controller machinery can drive different motor kinds and actuator backends.

The motion layer should be motor-kind independent. It should be reusable for:

1. FOC hybrid stepper,
2. FOC BLDC/PMSM,
3. brushed DC or voice-coil style torque/voltage/current actuation,
4. traditional stepper step/direction output,
5. generated-angle open-loop operation,
6. encoder/observer closed-loop operation.

The immediate hardware goal is a reliable chopper/profile mode that does not require the encoder for commutation, while keeping current-regulated FOC and the existing protection behavior.

## Units And Frame Contracts

The motion layer uses mechanical SI units only:

1. position: radians at the load/shaft frame,
2. velocity: radians per second,
3. acceleration: radians per second squared,
4. torque: newton-meters when a torque-domain controller is active,
5. normalized effort: `[-1, 1]` only when a backend cannot expose physical units.

Backend adapters own all motor-specific conversions:

1. FOC stepper/BLDC owns mechanical-to-electrical angle conversion, pole-pair scaling, alignment offset, and electrical trim.
2. Traditional stepper owns mechanical-to-step conversion, microstep scaling, direction, and pulse timing.
3. Brushed backends own current/voltage/duty conversion and can ignore commutation angle.
4. Feedback adapters own encoder direction, wrapping, latency compensation, and sensor-specific quality flags.

`pole_pairs`, encoder direction, electrical offsets, phase order, microstep resolution, and PWM details must not leak into motion profile code.

## Design Rule

Motion control answers: where should the load/shaft go, how fast, and with what acceleration?

The servo layer answers: what actuator effort is needed to follow that motion?

The motor backend answers: how does that effort become hardware actuation for this motor kind?

These decisions should meet in the app-owned fast-loop orchestration layer. The profile generator should not know whether the motor is a stepper, BLDC, brushed motor, or step/direction drive. The motor backend should not know whether the target came from a velocity command, a quintic profile, or a trigger-driven sequence.

## Current State

`ONLINE_VELOCITY_OPEN` already approximates the desired model:

1. encoder read is disabled,
2. `MOTOR_FEATURE_ANGLE_GEN` is enabled,
3. `MOTOR_FEATURE_VELOCITY_TRAJ` is enabled,
4. the velocity trajectory drives `angle_gen_set_velocity(...)`,
5. the angle observer/path selects the generated angle as the commutation source,
6. the current-regulated FOC path still produces PWM.

The current coupling is mostly semantic and structural:

1. `feature_angle_gen` means both "use generated angle for commutation" and "open-loop behavior".
2. the reference stage blends motion generation, closed-loop control, current arbitration, and angle-generator updates.
3. profile sequencing currently reads like closed-loop position behavior, even though it should also be usable as a generated-angle source.
4. `Id/Iq` and commutation-specific concepts appear too early in the motion/control pipeline.
5. tests cannot easily exercise the pipeline without `motor_parameters`, SMF-derived mode flags, and app feature flags.

## Implementation Readiness Gate

Begin implementation with a behavior-preserving Phase 1 only. This first slice proves the policy model without changing the ISR behavior or shell workflow.

First slice:

1. add policy/data definitions,
2. add backend capability definitions,
3. add a pure policy-derive helper,
4. add unit tests for the existing modes,
5. do not alter `motor_control_loop.c` behavior,
6. do not add `profile_open` yet.

Phase 1 is complete only when:

1. the firmware still builds,
2. unit tests pass,
3. `velocity_open` maps to generated-angle/current-regulated FOC policy,
4. closed-loop modes map to encoder-required policies,
5. invalid backend/current/feedback combinations are rejected by tests.

After Phase 1, later phases can refactor the fast loop around the new refs with a known-good policy layer underneath.

## Implementation Status

Status as of 2026-04-30:

| Phase | Status | Notes |
| --- | --- | --- |
| Phase 1: Policy types | Complete | `motor_control_policy` and backend capability validation are implemented and unit-tested. |
| Phase 2: Fast-loop data-flow refs | Complete | The fast loop now passes explicit motion, feedback, actuator, angle, current, and commutation refs internally. |
| Phase 3: Motion/current-source split | Complete | Generated-angle velocity is driven from `motor_motion_ref`; current arbitration no longer owns angle-generator trajectory state. |
| Phase 4: `profile_open` | Complete | `ONLINE_PROFILE_OPEN` exists and drives generated mechanical angle from profile position without requiring encoder feedback. |
| Phase 5: Shell/status cleanup | Complete | `motor state policy` and `motor state status` expose motion source, feedback source, angle source, current source, backend kind, and encoder dependency. |
| Phase 6: Simulation/unit tests | Partially complete | Pure policy and motion tests cover profile-open encoder independence and generated-angle profile sequencing. Broader pipeline simulation remains future work. |
| Phase 7: HIL validation | Ready to start | The pre-Phase 7 ISR/API cleanup gate is complete; HIL remains the next validation step. |

Implemented commits:

1. `c43910b refactor(motion): introduce fast loop control refs`
2. `51c5fef refactor(motion): split trajectory from generated angle drive`
3. `dc1385b feat(motion): add profile open mode`
4. `a82d270 feat(motion): expose active control policy`
5. `4a29096 test(motion): expand decoupled policy coverage`
6. `3390000 perf(motor): move ISR scratch into persistent context`
7. `d7e2f64 perf(motor): flatten ISR FOC backend path`
8. `c04d0f7 perf(motor): split outer-loop ISR runtime stages`
9. `cc28cff perf(motor): remove ISR encoder context copy`
10. `910892b perf(motor): gate optional ISR features`
11. `cea69ce test(motor): document ISR measurement gates`

Validation evidence is recorded in `app/docs/plan/execution_log.md`.

## Pre-Phase 7 Cleanup Gate

Status: complete. Detailed evidence and repeatable commands are in
`app/docs/isr_measurement_workflow.md` and `app/docs/plan/execution_log.md`.

Before HIL validation, clean up the remaining real-time/API concerns found during review:

1. Replace any ISR path that calls `motor_api_post_error()` or `k_msgq_put_front()` with the ISR-safe event ring or a dedicated fault latch.
2. Keep shell/event operations out of direct ISR context; normal shell/API commands should continue to flow through state-machine events.
3. Decide whether `motor_control_loop_step()` remains an app orchestrator over `struct motor_parameters` or gets a smaller `motor_rt_control_ctx` API before adding more backends.
4. Move MPR/DOB initialization and full config validation out of hot `step()` calls where practical; the ISR should run validated modules.
5. Cap or precompute MPR horizon work for MCU use so an aggressive runtime setting cannot consume the 20 kHz ISR budget.

Phase 7 should validate behavior, not discover avoidable ISR/API boundary problems.

Detailed execution plan: `app/docs/isr_latency_ti_style_plan.md`.

## Target Pipeline

The fast loop should become an explicit pipeline:

```text
policy_derive()       -> motor_control_policy
motion_step()         -> motor_motion_ref
servo_step()          -> motor_actuator_ref
backend_step()        -> motor_backend_ref / motor_pwm_ref / step-dir command
telemetry_publish()   -> optional debug/live state
```

The app ISR still owns orchestration and hardware-facing side effects:

```text
collect ADC/encoder
derive policy from state + config snapshot
run motion source
derive actuator effort from motion/feedback
run selected motor backend
publish telemetry/report
```

`motor_core` should provide pure modules used by those steps. App code should decide which modules are active.

Feedback is a separate pipeline input, not the same thing as commutation angle:

```text
feedback_step()       -> motor_feedback_ref
```

Closed-loop servo modules consume feedback. FOC backends may also consume feedback-derived angle. Open-loop generated-angle backends can ignore encoder feedback for control while still publishing it for telemetry.

FOC-specific systems can refine the backend step internally:

```text
angle_select_step()   -> motor_angle_ref
current_ref_step()    -> motor_current_ref
foc_current_step()    -> motor_pwm_ref
```

Step/direction systems can map the same motion reference to pulse scheduling instead:

```text
motion_step()         -> motor_motion_ref
stepdir_step()        -> pulse interval / direction / enable
```

## Runtime Concepts

### Policy

The policy is derived from app state/mode flags and feature configuration. It should be a compact plain struct, suitable for tests.

```c
enum motor_motion_source {
	MOTOR_MOTION_SOURCE_HOLD,
	MOTOR_MOTION_SOURCE_VELOCITY_TRAJ,
	MOTOR_MOTION_SOURCE_PROFILE,
	MOTOR_MOTION_SOURCE_PROFILE_SEQUENCE,
};

enum motor_angle_source {
	MOTOR_ANGLE_SOURCE_GENERATED,
	MOTOR_ANGLE_SOURCE_ENCODER,
	MOTOR_ANGLE_SOURCE_PROPAGATED,
};

enum motor_feedback_source {
	MOTOR_FEEDBACK_NONE,
	MOTOR_FEEDBACK_ENCODER,
	MOTOR_FEEDBACK_SENSORLESS_OBSERVER,
	MOTOR_FEEDBACK_GENERATED_MODEL,
};

enum motor_current_source {
	MOTOR_CURRENT_SOURCE_ZERO,
	MOTOR_CURRENT_SOURCE_COMMANDED,
	MOTOR_CURRENT_SOURCE_VELOCITY_LOOP,
	MOTOR_CURRENT_SOURCE_POSITION_LOOP,
	MOTOR_CURRENT_SOURCE_CALIBRATION,
};

enum motor_actuator_kind {
	MOTOR_ACTUATOR_FOC_CURRENT,
	MOTOR_ACTUATOR_BRUSHED_CURRENT,
	MOTOR_ACTUATOR_BRUSHED_VOLTAGE,
	MOTOR_ACTUATOR_STEP_DIR,
};

struct motor_control_policy {
	enum motor_motion_source motion_source;
	enum motor_feedback_source feedback_source;
	enum motor_angle_source angle_source;
	enum motor_current_source current_source;
	enum motor_actuator_kind actuator_kind;
	bool encoder_read_enabled;
	bool encoder_required_for_control;
	bool current_loop_enabled;
	bool generated_angle_position_driven;
};
```

Initial policy mapping:

```text
velocity_open:
  motion_source = VELOCITY_TRAJ
  feedback_source = GENERATED_MODEL or NONE
  angle_source = GENERATED
  current_source = COMMANDED
  actuator_kind = FOC_CURRENT
  encoder_required_for_control = false

profile_open:
  motion_source = PROFILE or PROFILE_SEQUENCE
  feedback_source = GENERATED_MODEL or NONE
  angle_source = GENERATED
  current_source = COMMANDED
  actuator_kind = FOC_CURRENT
  encoder_required_for_control = false

torque:
  motion_source = HOLD
  feedback_source = ENCODER
  angle_source = ENCODER/PROPAGATED
  current_source = COMMANDED
  actuator_kind = FOC_CURRENT
  encoder_required_for_control = true

velocity_closed:
  motion_source = VELOCITY_TRAJ
  feedback_source = ENCODER
  angle_source = ENCODER/PROPAGATED
  current_source = VELOCITY_LOOP
  actuator_kind = FOC_CURRENT
  encoder_required_for_control = true

position:
  motion_source = PROFILE
  feedback_source = ENCODER
  angle_source = ENCODER/PROPAGATED
  current_source = POSITION_LOOP
  actuator_kind = FOC_CURRENT
  encoder_required_for_control = true

step_dir_profile:
  motion_source = PROFILE or PROFILE_SEQUENCE
  feedback_source = NONE, ENCODER, or GENERATED_MODEL
  angle_source = not applicable
  current_source = not applicable
  actuator_kind = STEP_DIR
  encoder_required_for_control = false unless configured as closed-loop stepper
```

### Motion Reference

Motion modules produce mechanical references. They do not produce PWM and do not inspect encoder health.

```c
struct motor_motion_ref {
	float32_t position_mech_rad;
	float32_t velocity_mech_rad_s;
	float32_t accel_mech_rad_s2;
	bool position_valid;
	bool velocity_valid;
	bool accel_valid;
	bool active;
};
```

Examples:

1. velocity trajectory produces valid velocity and integrated generated position,
2. quintic profile produces position, velocity, acceleration,
3. profile sequence triggers a new quintic segment and then the profile produces the reference.

### Angle Reference

Angle selection converts the chosen angle source into commutation angles.

```c
struct motor_angle_ref {
	float32_t mech_rad;
	float32_t elec_rad;
	float32_t elec_pred_rad;
	float32_t mech_speed_rad_s;
	float32_t elec_speed_rad_s;
	uint8_t source;
	uint8_t quality_flags;
	bool valid;
};
```

For generated-angle operation:

1. `mech_rad` comes from angle generator or motion profile position,
2. `elec_rad` is computed using pole-pair count and alignment/trim offset,
3. encoder quality does not gate commutation,
4. encoder can still be captured as telemetry.

For encoder operation:

1. `mech_rad` comes from `angle_observer`,
2. latency compensation remains in `angle_observer`,
3. propagated angle can be used for short dropouts according to observer policy.

### Feedback Reference

Feedback represents measured or estimated load/shaft state for servo control and telemetry. It is not automatically the commutation source.

```c
struct motor_feedback_ref {
	float32_t position_mech_rad;
	float32_t velocity_mech_rad_s;
	float32_t accel_mech_rad_s2;
	uint8_t source;
	uint8_t quality_flags;
	bool position_valid;
	bool velocity_valid;
	bool fresh;
};
```

Examples:

1. closed-loop position uses feedback position for position error,
2. closed-loop velocity uses feedback velocity for speed error,
3. generated-angle open-loop can publish generated feedback for telemetry and simulation,
4. FOC commutation can use feedback-derived angle only when the selected angle source requires it.

### Current Reference

Current-reference generation is one FOC-specific actuator-effort representation. It produces `Id/Iq` before the current loop.

```c
struct motor_current_ref {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	bool valid;
	bool reset_current_pi;
};
```

Initial behavior:

1. `profile_open` uses commanded current.
2. `velocity_open` uses commanded current.
3. closed-loop velocity and position use the existing outer-loop/MPR/PI path.
4. calibration/alignment continues to override current refs explicitly.

### Generic Actuator Reference

The servo layer should be able to produce a motor-kind-independent effort request where practical.

```c
struct motor_actuator_ref {
	float32_t position_mech_rad;
	float32_t velocity_mech_rad_s;
	float32_t torque_nm;
	float32_t force_or_effort;
	float32_t normalized_effort;
	bool position_valid;
	bool velocity_valid;
	bool torque_valid;
	bool effort_valid;
};
```

Backend adapters then convert this into motor-specific references:

1. FOC stepper/BLDC: `motor_current_ref` plus `motor_angle_ref`,
2. brushed current loop: signed current command,
3. brushed voltage loop: signed voltage or duty command,
4. step/direction: pulse interval, direction, and enable.

For the immediate chopper work, commanded `Id/Iq` remains the actuator effort.

### Backend Capabilities

Each backend should publish static capabilities so policy validation can reject invalid combinations.

```c
struct motor_actuator_caps {
	bool accepts_position;
	bool accepts_velocity;
	bool accepts_torque;
	bool accepts_current;
	bool accepts_voltage;
	bool accepts_step_dir;
	bool requires_commutation_angle;
	bool supports_open_loop;
	bool supports_closed_loop;
};
```

Examples:

1. FOC current backend accepts current/torque-derived current and requires commutation angle.
2. Brushed current backend accepts signed current and does not require commutation angle.
3. Brushed voltage backend accepts signed voltage or normalized effort and does not require commutation angle.
4. Step/direction backend accepts position/velocity timing and does not require commutation angle.

Policy validation should catch invalid pairings before the ISR uses them, such as FOC-only `Id/Iq` references with a step/direction backend.

### FOC Commutation Reference

The FOC backend consumes measured currents, current refs, bus voltage, and angle ref.

```c
struct motor_commutation_ref {
	float32_t id_ref_a;
	float32_t iq_ref_a;
	float32_t id_meas_a;
	float32_t iq_meas_a;
	float32_t angle_elec_rad;
	float32_t angle_elec_pred_rad;
	float32_t elec_speed_rad_s;
	float32_t vbus_v;
};
```

The first implementation keeps `MOTOR_ACTUATOR_FOC_CURRENT` only. Brushed and step/direction are architectural targets, not required deliverables for the first pass.

### Step/Direction Backend Reference

Traditional stepper control should not require the motion layer to know about microstep timing.

```c
struct motor_stepdir_ref {
	uint32_t step_interval_ticks;
	bool direction_positive;
	bool step_enable;
	bool valid;
};
```

This backend would consume `motor_motion_ref` and generate pulse timing. It may ignore `motor_angle_ref` entirely.

### Backend Adapter Boundary

Each backend should expose a typed adapter boundary. The plan should not force every backend through one vague struct.

Generic shape:

```c
int motor_backend_step(const struct motor_motion_ref *motion,
		       const struct motor_actuator_ref *effort,
		       const struct motor_feedback_ref *feedback,
		       void *backend_state,
		       void *backend_output);
```

Typed implementations should be preferred in real code:

1. `motor_foc_current_backend_step(...)`,
2. `motor_brushed_current_backend_step(...)`,
3. `motor_brushed_voltage_backend_step(...)`,
4. `motor_stepdir_backend_step(...)`.

The generic shape documents the layering. The typed functions keep runtime code efficient and unit-testable.

### Failure Policy

Failure handling must be backend-specific and explicit.

Open-loop generated-angle profile:

1. encoder invalid: continue control, record telemetry/error counters only,
2. motion ref invalid: hold generated angle and zero commanded current unless a calibration state overrides it,
3. current limit exceeded: apply existing overcurrent/protection path,
4. disarmed: zero `Id/Iq`, keep profile state deterministic.

Closed-loop FOC position/velocity:

1. encoder invalid beyond allowed propagation: zero torque-producing current or hold measured current according to existing interlock policy,
2. motion ref invalid: hold target and reset outer-loop integrators,
3. current limit exceeded: apply existing fault path,
4. disarmed: zero current and reset current/outer-loop integrators.

Brushed backend:

1. feedback invalid in closed-loop modes: coast, brake, hold, or fault according to configured policy,
2. voltage/current saturation: clamp and report saturation,
3. disarmed: command zero effort.

Step/direction backend:

1. motion ref invalid: stop pulse generation after a bounded stop if possible,
2. feedback invalid in closed-loop stepper mode: configurable hold/fault behavior,
3. command exceeds step-rate capability: clamp, reject, or fault according to policy,
4. disarmed: disable step output or hold enable according to configured drive policy.

## Ownership Boundaries

### `motor_core`

Reusable pure modules:

1. motion profile and profile sequence math,
2. angle generator,
3. angle observer and generated/encoder angle selection helpers,
4. outer-loop regulators,
5. current-reference arbitration/interlocks,
6. motor-kind-independent servo/controller primitives,
7. FOC current loop, decoupling, transforms, PWM synthesis,
8. future backend adapters such as brushed-current and step/direction helpers,
9. plain policy/data-flow helpers that do not include app state or shell semantics.

### `app`

System orchestration:

1. SMF states and mode requests,
2. shell commands,
3. feature flags and runtime config snapshot publication,
4. hardware ISR collect/apply,
5. telemetry capture and diagnostic rings,
6. mapping app states to `motor_control_policy`.

## Phase 1: Add Explicit Policy Types

Create policy definitions and derive policy from existing mode flags without changing runtime behavior.

Scope:

1. add policy enums and `struct motor_control_policy`,
2. name the backend selector as `actuator_kind` or similar, not `commutation_method`,
3. add `motor_feedback_source`,
4. add backend capability definitions,
5. add a pure policy-derive helper,
6. add status/string helpers if useful for tests and later shell output,
7. keep current behavior unchanged,
8. keep existing feature flags as compatibility inputs,
9. do not modify current-loop, FOC, or profile execution behavior.

Acceptance:

1. `velocity_open`, `torque`, `velocity_closed`, and `position` map to expected policies,
2. invalid policy/backend pairings are rejected by tests,
3. no shell-visible behavior changes,
4. unit tests cover policy derivation,
5. firmware build passes,
6. full unit suite passes.

## Phase 2: Introduce Data-Flow Structs In The Fast Loop

Refactor `app/src/motor_control_loop.c` so local stages pass explicit refs:

1. `motor_motion_ref`,
2. `motor_feedback_ref`,
3. `motor_actuator_ref`,
4. FOC-specific `motor_angle_ref`,
5. FOC-specific `motor_current_ref`,
6. FOC-specific `motor_commutation_ref`.

Scope:

1. preserve current `velocity_open` behavior,
2. preserve current closed-loop modes,
3. keep mechanical units in motion refs,
4. do not move app state into `motor_core`,
5. do not change shell commands yet.

Acceptance:

1. build passes,
2. runtime unit tests pass,
3. encoder capture compare still reports generated and encoder angles,
4. no new `motor_core` dependency on app headers,
5. motion tests do not require FOC-specific fields.

## Phase 3: Split Motion From Current Source

Make motion modules produce only mechanical references. Servo/backend policy decides whether those references drive:

1. generated angle,
2. velocity loop,
3. position loop,
4. commanded current,
5. a future step/direction pulse generator,
6. a future brushed current/voltage backend.

Scope:

1. separate `feature_velocity_traj` from `feature_angle_gen`,
2. make open-loop angle update consume `motor_motion_ref`,
3. keep current arbitration/interlocks after current-source selection.

Acceptance:

1. `velocity_open` still moves using commanded `Iq`,
2. closed-loop velocity still uses velocity trajectory as the velocity target,
3. current interlocks still zero current when disarmed.

## Phase 4: Add `profile_open`

Add a new online state/mode for generated-angle profile execution.

Behavior:

1. angle source is generated,
2. current source is commanded,
3. actuator kind is current-regulated FOC,
4. encoder read is optional telemetry,
5. profile sequence can trigger moves without encoder freshness.

Implementation notes:

1. profile position should set the generated mechanical angle directly,
2. profile velocity should remain available for telemetry and later feedforward,
3. the profile-open mode should not calculate closed-loop position error,
4. profile-open should use the same alignment/trim offset semantics as generated-angle velocity mode.

Acceptance:

1. new shell mode exists, likely `motor state mode profile_open`,
2. profile sequence can run in `profile_open` with encoder disabled,
3. current commands still require `motor arm`,
4. entering/exiting the mode resets profile/generator state deterministically.

## Phase 5: Shell And Telemetry Cleanup

Expose the new policy clearly enough for debugging.

Required shell/status additions:

1. current mode,
2. motion source,
3. angle source,
4. current source,
5. actuator/backend kind,
6. whether encoder is required for control.

Minimum profile-open workflow:

```text
motor state offline
motor safety timeout 0
motor arm
motor state mode profile_open
motor current id 0
motor current iq 0.15
motor profile seq clear
motor profile seq add <rad>
motor profile seq add <rad>
motor profile seq config <period_ms> <move_ms> <end_vel_hz> <loop>
motor profile seq start
```

Acceptance:

1. existing `velocity_open` workflow still works,
2. status output explains the active control policy,
3. encoder telemetry can be enabled in profile-open without becoming a control dependency.

## Phase 6: Simulation And Unit Tests

Add pure tests that do not need hardware, SMF, shell, or full `motor_parameters` where practical.

Test matrix:

```text
policy derivation:
  velocity_open, profile_open, torque, velocity_closed, position
  invalid backend/current/feedback combinations

motion stage:
  hold, velocity trajectory, quintic profile, sequence trigger
  all outputs in mechanical SI units

feedback stage:
  none, encoder, generated model, propagated observer

FOC angle stage:
  generated position-driven, generated velocity-driven, encoder, propagated

current stage:
  commanded, disarmed interlock, velocity loop, position loop, calibration override

backend stage:
  FOC current path accepts angle/current refs and clamps PWM
  future step/direction path maps motion refs to direction and timing
```

Simulation-style tests:

1. run a profile-open two-point sequence and verify generated angle follows the profile,
2. run the same profile with encoder disabled and verify no encoder fault gates commutation,
3. run closed-loop position policy and verify encoder invalidity gates control as expected,
4. verify disarm zeros current without corrupting profile state.
5. verify motion reference tests do not depend on FOC-specific fields.
6. feed the same motion reference into simulated FOC, brushed, and step/direction backend adapters.

## Phase 7: HIL Validation

Hardware checks:

1. `velocity_open` smoke test remains unchanged,
2. `profile_open` executes a small two-position move with encoder disabled,
3. `profile_open` executes trigger-driven sequence with encoder telemetry disabled,
4. optional encoder capture verifies generated/encoder phase relationship when the encoder is usable,
5. closed-loop modes are unchanged except for clearer status reporting.

## Non-Goals

1. Do not bypass the current loop.
2. Do not add raw voltage-vector commutation in this pass.
3. Do not require encoder feedback for open-loop profile execution.
4. Do not remove closed-loop position/velocity modes.
5. Do not make motion profiles aware of encoder health.
6. Do not rewrite the current loop or FOC math.
7. Do not implement brushed or step/direction backends in the first pass; keep the architecture ready for them.

## Migration Checkpoints

1. Add policy types and tests with no runtime behavior change.
2. Move fast-loop locals to explicit data-flow structs with no shell behavior change.
3. Preserve `velocity_open` as the first proof of generated-angle policy.
4. Add `profile_open` after the generated-angle path is explicit.
5. Only after HIL validation, consider renaming or reorganizing shell commands around motion/angle/current policy.
