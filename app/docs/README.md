# Chopper Documentation Index

This directory is intentionally split between current operating documents and
historical plans. When making control changes, use the current documents below
as the source of truth and treat `archive/` as background evidence only.

## Current Entry Points

- `commissioning_workflow_refactor_plan.md`
  - Current MT6835 commissioning workflow and latest standard commissioning
    evidence.
- `encoder_pi_tuning_hil_2026-05-05.md`
  - Current MT6835 PI `current_encoder`, `velocity_encoder`, and initial
    `position_encoder` HIL evidence.
- `control_system_improvement_index.md`
  - Completed control-system improvement plan set and remaining advanced HIL
    caveats.
- `mpr_dob_detent_improvement_plan.md`
  - Current plan for detent feedforward, MPR, and DOB improvements.
- `plan/mechanical_identification_v2/README.md`
  - Current staged plan to replace the coupled 4-parameter mechanical fit with
    separate friction, inertia, detent, confidence-gating, and HIL workflow
    phases.
- `settings_persistence_plan.md`
  - Persistence readiness policy. Persistence remains opt-in/future until
    repeatable HIL gates prove behavior.
- `hil_motion_regression_plan.md`
  - HIL runner/gate behavior. Earlier failed logs are historical unless a newer
    current-baseline document references them.

## Architecture References

- `motion_actuator_decoupling_plan.md`
- `isr_latency_ti_style_plan.md`
- `rt_spi_transport_plan.md`
- `isr_measurement_workflow.md`
- `operating_mode_taxonomy.md`
- `motor_shell_command_tree.md`

## Completed Control-Improvement Task Plans

These are retained in place because `control_system_improvement_index.md`
references them directly as the completed plan set:

- `encoder_sample_quality_plan.md`
- `angle_observer_contract_plan.md`
- `mpr_bandwidth_interface_plan.md`
- `detent_learning_v2_plan.md`
- `dob_enable_policy_plan.md`
- `motion_control_status_shell_plan.md`

## Long-Form Plan Pack

The structured refactor plan pack lives in `plan/`.

Start with:

1. `plan/README.md`
2. `plan/00_scope.md`
3. `plan/01_architecture_target.md`
4. `plan/tasks/index.md`
5. `plan/validation.md`

## Archive Policy

`archive/` contains stale, superseded, or hardware-specific history. Do not use
archived docs to infer the current MT6835 control baseline unless a current doc
explicitly says to.

In particular:

- AEAT-9955 encoder-control instability does not imply MT6835 instability.
- Early boot/velocity tuning failures were superseded by later MT6835
  commissioning and PI tuning evidence.
- Completed migration/refactor execution logs are useful for context, not for
  current implementation direction.
