# Calibration and Control Hardening Plan

This plan addresses weak spots found in the commissioning and control paths after the motion/control refactors.

## Scope

- Make commissioning stage preconditions explicit and repeatable.
- Reject commissioning samples that are clean electrically but not representative of the requested motion.
- Apply measured Rs/R-over-L to current-loop PI gains when electrical identification succeeds.
- Make model source/confidence visible before MPR/DOB/detent are trusted.
- Make boot commissioning completion semantics deterministic.
- Harden HIL command timing for long commissioning commands.

## Non-Goals

- Persistence of commissioned values.
- Replacing PI/MPR/DOB algorithms.
- Changing encoder transport drivers.
- Retuning every motor profile empirically.

## Task Order

1. `T001_preconditions.md` - Shared commissioning precondition helper.
2. `T002_sample_qualification.md` - Motion-tracking qualification for commissioning capture.
3. `T003_current_pi_from_rl.md` - Apply current PI gains from measured Rs/R-over-L.
4. `T004_model_source_and_gating.md` - Model-source visibility and DOB gating.
5. `T005_boot_semantics_and_hil.md` - Boot command completion and HIL timeout hardening.
6. `T006_current_pi_validation_ld_lq.md` - Current PI validation and optional Ld/Lq estimation.

## Validation

- Unit tests for motor-core helpers touched by this work.
- Python HIL parser tests.
- Firmware build with the MT6835 overlay.
- Optional HIL boot + velocity validation after build/flash.

## Implementation Status

- T001: Implemented and build-validated.
- T002: Implemented and build-validated.
- T003: Implemented and build-validated.
- T004: Implemented and build-validated.
- T005: Implemented and build-validated.
- T006: Planned.
