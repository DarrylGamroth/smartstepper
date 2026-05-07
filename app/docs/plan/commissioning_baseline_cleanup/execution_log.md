# Commissioning Baseline Cleanup Execution Log

## 2026-05-07 - Plan Created

Inputs:

- User decisions from commissioning/control inventory review.
- `app/docs/control_commissioning_inventory_2026-05-07.md`.
- Local Zephyr settings backend availability: Settings supports ZMS/NVS/FCB/File;
  ZMS is selected as the preferred backend. NVS is fallback only. NVMEM exists
  separately for fixed cells over EEPROM/flash/OTP/BBRAM.

Plan status:

- T001: Planned.
- T002: Planned.
- T003: Planned.
- T004: Planned. Decision tightened on 2026-05-07: remove the scalar pulse/integrated
  inductance operator path now; only retain private helper code if dependency
  checking shows demod/saliency/tests still require it.
- T006: Expanded on 2026-05-07 with concrete DT ownership rules, current
  `user_parameters` vs `motor_parameters` placement review, cleanup targets for
  legacy/scalar properties, and guidance that Zephyr's generic Settings shell is
  debug-only rather than the typed motor settings interface.
- T005: Planned.
- T006: Planned.
- T007: Planned.
- T008: Planned.

Validation evidence:

- Plan-only change; no build required.

## 2026-05-07 - T007 Settings Persistence Implementation

- Implemented typed motor Settings keys under `motor/` using the Zephyr Settings/ZMS backend.
- Added guarded shell commands: `motor settings status`, `preview`, `save`, `load`, `clear`, and `autoload status`.
- Changed persistence direction from a packed blob to individual typed keys plus metadata (`schema_version`, `generation`, `valid_groups`).
- ADC current offsets are intentionally not persisted; they remain boot-calibrated runtime values.
- Autoload remains disabled; load is explicit and refused while armed, online, or calibrating.
- Validation evidence: firmware build passed with `west build --build-dir /workspace/build/chopper/smartstepper_v2`; focused persistence unit suite passed with `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_persistent_config.unit`.

## 2026-05-07 - T007 Controller Settings Intent Schema

- Replaced raw controller coefficient persistence with high-level controller intent.
- Controller settings now persist outer-loop mode, velocity bandwidth, position bandwidth, damping ratio, velocity Iq limit, DOB enable, and DOB gain scale.
- Velocity PI, position PI, velocity MPR, position MPR, and DOB coefficients are recomputed on load from the persisted intent and active motor model.
- Bumped motor settings schema to version 2 to avoid applying old raw-controller records.
- Validation evidence: firmware build passed with `west build --build-dir /workspace/build/chopper/smartstepper_v2`; focused persistence unit suite passed with `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_persistent_config.unit`.
