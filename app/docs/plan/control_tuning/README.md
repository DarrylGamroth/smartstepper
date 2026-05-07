# PI/MPR/DOB/Detent Control Tuning Plan Package

Date: 2026-05-07
Target hardware: `smartstepper_v2` with `configs/motor_mt6835_2a.overlay`
Persistence: out of scope until runtime behavior is repeatable.

## Objective

Return to controller work now that encoder acquisition, electrical ID, and standard commissioning are stable enough for closed-loop testing.

The work is staged deliberately:

1. Preserve a known-good commissioning and encoder-diagnostic baseline.
2. Establish velocity PI performance from measured `Kt/J/B/Tc`.
3. Compare velocity MPR using the user-facing `motor velocity mpr bandwidth <hz>` interface.
4. Enable DOB only after PI/MPR are stable and use it as residual disturbance correction.
5. Reintroduce detent feedforward only after the baseline regulator is clean.

## Execution Rules

- Do not tune nonzero `Id` in this plan. Use `Id = 0` unless a later MTPA/saliency plan explicitly changes it.
- Do not enable DOB or detent by default.
- Do not persist tuned values yet.
- Every HIL run must print encoder acquisition counters before verdict evaluation.
- Every live-motion HIL scenario must end through the standard stop/status postlude.
- Commit after each stable implementation stage.

## Files

- `00_scope.md`: boundaries and assumptions.
- `01_hil_baseline.md`: commissioning and diagnostics gate.
- `02_velocity_pi_baseline.md`: PI tuning and sweep validation.
- `03_velocity_mpr_bandwidth.md`: MPR bandwidth interface and comparison.
- `04_dob_gate.md`: DOB readiness and validation gate.
- `05_detent_ff_gate.md`: detent feedforward gate.
- `execution_log.md`: evidence trail.
