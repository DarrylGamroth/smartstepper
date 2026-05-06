# Control System Improvement Plan Index

Date: 2026-05-05

## Goal

Coordinate the next control-system cleanup work through narrow plans that can be
implemented, validated, and reviewed independently.

## Non-Goals

- Do not use this index as an implementation plan.
- Do not combine persistence, shell cleanup, detent learning, DOB policy, MPR
  tuning, and encoder qualification into one implementation task.
- Do not require backward compatibility where simplification is the point of a
  plan.

## Current Problem

The project now has enough control features that broad plans make progress hard
to judge. Encoder sample quality, observer semantics, detent learning, MPR
tuning, DOB policy, shell status, and HIL evidence are related, but each has a
different failure mode and validation method.

## Plan Set

1. `encoder_sample_quality_plan.md`
   - Centralize encoder sample qualification and bad-sample policy.
2. `angle_observer_contract_plan.md`
   - Make the angle observer the single owner of angle wrapping, offsets,
     latency compensation, velocity, and trust semantics.
3. `hil_motion_regression_plan.md`
   - Establish repeatable HIL evidence before tuning more advanced features.
4. `mpr_bandwidth_interface_plan.md`
   - Replace operator-facing MPR presets with bandwidth commands:
     `motor velocity mpr bandwidth <hz>` and
     `motor position mpr bandwidth <hz>`.
5. `detent_learning_v2_plan.md`
   - Refine detent learning around controlled forward/reverse integration and
     validation before apply.
6. `dob_enable_policy_plan.md`
   - Define deterministic DOB readiness, reset, and fault/degrade behavior.
7. `motion_control_status_shell_plan.md`
   - Make the shell status tree clearly show mode, feedback, regulator,
     feedforward, and fault risk.

## Execution Order

1. Encoder sample quality.
2. Angle observer contract.
3. HIL motion regression baseline.
4. MPR bandwidth interface.
5. Detent learning v2.
6. DOB enable policy.
7. Shell/status cleanup.

## Cross-Plan Rules

- Use telnet HIL scripts when Ethernet is available.
- Keep generated/open-loop motion as a regression check.
- Keep PI encoder velocity as the known-good fallback.
- Keep raw engineering commands for MPR/DOB/debug, but prefer bandwidth/status
  commands for normal operation.
- Persistence remains out of scope until runtime behavior is proven.

## Evidence Tracking

For each completed plan, append:

- firmware commit,
- unit test command/result,
- firmware build command/result,
- HIL command/result,
- log or JSON report path,
- remaining caveats.

