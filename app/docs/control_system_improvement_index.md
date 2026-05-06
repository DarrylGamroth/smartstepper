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

## Completion Summary

Completed through: plan 7, `motion_control_status_shell_plan.md`.

Commits:

- `3eb4af5` - Implement encoder sample trust contract.
- `9ca4c6e` - Centralize angle observer contract.
- `d04a302` - Improve HIL motion regression gate.
- `0029ed8` - Add MPR bandwidth mapping helpers.
- `2baaaa3` - Refine detent learning validation policy.
- `2b7a103` - Define DOB readiness and reset policy.
- `44c5e14` - Add concise control status shell commands.

Validation summary:

- Unit tests passed for encoder feedback/control kernel, angle observer, MPR,
  detent map, DOB, and HIL telnet parser coverage.
- Firmware build passed with:

```text
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2'
```

- Target flash passed with:

```text
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
```

- Non-motion telnet status HIL passed after flashing:

```text
python3 scripts/hil/hil_telnet.py status --host 10.0.0.44 --json-report hil_logs/control_plan/status_shell_cleanup_after_flash.json
```

Remaining caveat:

- MT6835 commissioning and PI encoder-control baseline evidence is captured in
  `encoder_pi_tuning_hil_2026-05-05.md` and
  `commissioning_workflow_refactor_plan.md`. Those results supersede earlier
  generic "velocity encoder baseline unstable" notes for the MT6835 profile.
- Full live motion HIL for detent, MPR, DOB, and feature combinations is still
  not complete. That is an advanced-feature validation gap, not evidence that
  MT6835 commissioning or PI `velocity_encoder` is currently failing.
- AEAT-9955 encoder-control limitations are hardware/encoder-path specific and
  are documented separately in the AEAT-focused stabilization and transport
  plans.
