# Mechanical Identification V2 Execution Log

## 2026-05-07 - Plan Created

Context:

- Current full commissioning can complete, but recent MT6835 HIL showed low
  mechanical confidence: `J=0.00001599 +/- 0.00001062 kg*m^2`,
  `B=0.00003501 +/- 0.00004951 Nm/(rad/s)`, confidence `0.30`.
- The configured 39 mm NEMA17 fallback inertia is expected around
  `5.7e-6 kg*m^2` after fixing the `mg*cm^2` conversion.
- Negative viscous friction is no longer accepted, but the current estimator is
  still a coupled 4-parameter fit and remains poorly conditioned.

Plan status:

- T001: Planned.
- T002: Planned.
- T003: Planned.
- T004: Planned.
- T005: Planned.
- T006: Planned.

Validation evidence:

- Plan-only change; no build or HIL required yet.

## 2026-05-07 - V2 Implementation

Implemented tasks:

- T001: Added `motor_mech_friction_id_*` plateau estimator in `motor_core`.
  It fits `B`, `Tc`, and `T0` from low-acceleration bidirectional speed samples
  with directional coverage, residual, R2, and nonnegative physical checks.
- T002: Added `motor_mech_inertia_id_*` transient estimator. It subtracts the
  staged friction model and fits positive `J` from acceleration-rich samples
  with acceleration-direction coverage and plausibility checks relative to the
  configured fallback inertia.
- T003: Added pure `motor_detent_map_lookup()` and threaded optional active-map
  detent torque subtraction into mechanical identification. Results record
  whether active detent correction was used.
- T004: Mechanical commissioning now stages v2 subfit metadata and only marks
  `mech_valid` when friction, inertia, plausibility, and confidence pass. Low
  confidence or implausible inertia remains staged/diagnostic and will not apply
  over devicetree fallback values.
- T005: Commissioning samples now retain mechanical position and post-capture
  windowed acceleration. ISR capture cost remains a single stored position field;
  the derivative fit runs during finalize.
- T006: Added `motor commission mech status/active/clear/apply_staged` aliases,
  expanded status output with v2 fields, and added the `mechanical-id-v2` HIL
  telnet scenario plus parser tests.

Validation evidence:

- `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_detent_map.unit`
  - PASS: 8/8 test cases.
- `./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_commission_estimators.unit`
  - PASS: 24/24 test cases.
- `python3 -m py_compile scripts/hil/hil_telnet.py`
  - PASS.
- `python3 -m unittest scripts/hil/test_hil_telnet_parser.py`
  - PASS: 29 tests.
- MT6835 optimized firmware build:
  `podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -p auto -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2 -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/motor_mt6835_2a.overlay"'`
  - PASS. Size: FLASH 501280 B, RAM 207920 B.

HIL note:

- The `mechanical-id-v2` scenario is implemented but was not run in this pass;
  firmware build and host parser validation passed. Use:
  `python3 scripts/hil/hil_telnet.py mechanical-id-v2 --host 10.0.0.44 --yes-live-motion`.
