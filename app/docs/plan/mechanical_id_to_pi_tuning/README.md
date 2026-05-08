# Mechanical ID To PI Tuning

## Goal
Make mechanical commissioning produce a repeatable inertia estimate that can safely drive model-based velocity and position PI tuning.

## Non-Goals
Do not tune MPR, DOB, or detent feedforward in this plan.
Do not persist mechanical settings automatically.
Do not add a new mechanical identification algorithm.

## Current Problem
The velocity PI loop cannot be evaluated if the controller is current-starved or if the mechanical model is too noisy. The existing staged mechanical ID can produce usable captures, but the aggregate gate only rejects extreme inertia spread and the tune path still has authority-floor heuristics that can dominate the model-based gains.

## Design
Use the existing staged mechanical estimator as the baseline. Treat inertia `J` as the primary output, treat viscous friction `B` as secondary, and calculate controller defaults from the measured model with guardrails. Velocity and position loops should run decimated from the 20 kHz current loop: velocity at 2 kHz and position at 200 Hz by default.

## Implementation Phases
Phase 1:
Tighten mechanical aggregate reporting and acceptance around `J` repeatability. Report coefficient of variation for `J`, `B`, and Coulomb friction.

Phase 2:
Change model PI derivation to use the motor current limit and the second-order model equations without an artificial low-speed current-authority floor.

Phase 3:
Set outer-loop default decimation to velocity=10 and position=100 ISR ticks. Ensure commissioning tune calculations use the velocity-loop sample time.

Phase 4:
Run focused unit tests and an optimized MT6835 build. Run HIL commissioning if the target shell is available.

## Acceptance Criteria
Mechanical aggregate rejects if accepted runs produce `J` CV above 0.50.
Commissioning output prints `J_cv`, `B_cv`, and `Tc_cv`.
Default tune output uses `iq_limit_a == max_current_a`.
Velocity PI gains match the model equations using measured `J`, `B`, and `Kt` with bounded nonnegative proportional gain.
Default outer-loop decimation is velocity=10 and position=100.

## HIL Evidence
Use the MT6835 profile with safe-ID overlays when hardware is available:

```bash
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -d /workspace/build/chopper/smartstepper_v2_mt6835_id'
```

After flash, run baseline commissioning, then advanced auto commissioning:

```text
motor commission run confirm apply
motor commission auto run confirm apply
motor commission auto status
```

Record output and final state in this plan's execution log.

## Risks
A stricter `J` repeatability gate may expose the existing mechanical excitation as insufficient. If that happens, keep the failure and create a separate excitation-improvement plan instead of lowering the bar.

## Done State
Code, docs, tests, build output, and HIL evidence are committed with a clear rollback point.
