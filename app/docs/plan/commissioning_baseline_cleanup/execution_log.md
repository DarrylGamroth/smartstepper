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
