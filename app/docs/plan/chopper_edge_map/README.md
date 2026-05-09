# Chopper Edge Map

## Goal
Build a calibrated chopper wheel edge map from optical captures so position moves can target derived slot/tooth centerpoints instead of ideal 22.5 degree assumptions, and so PB4 can report blade state from encoder position.

## Non-Goals
Do not implement final hardware timer/capture ISR index playback in this phase.
Do not optimize position PI further until the index table exists.
Do not require mechanical ID to succeed.

## Current Problem
The position controller can move to angular targets, but the chopper wheel has real slot/tooth geometry. The application needs the actual centerpoints of the slots and teeth so each external tick can command the next calibrated index position. It also needs a deterministic blade-state GPIO that is independent of the optical emitter/capture path after calibration. The existing `motor chopper calib` command captures optical edges, but geometry is passed manually, only slot midpoints are produced, and the map is not persisted in a form that can regenerate both edge intervals and centerpoints.

## Design
Store chopper wheel geometry as devicetree defaults and runtime settings. For an 8-slot blade, default `chopper-slot-count=8`; the tooth count is derived as 8 unless a nonstandard wheel overrides it, producing 16 slot/tooth regions.

The canonical calibrated map is the physical edge list:

- `edge_count`
- sorted `edges_rad`
- `edge_region_after`, where each entry labels the interval after that edge as slot or tooth

Capture optical edges while running controlled velocity in either direction, average repeated edges over multiple revolutions, sort the edge angles, classify equal slot/tooth wheels by alternating parity with timer edge-status used only to choose the starting polarity, and derive every region centerpoint between adjacent edges. Apply the derived centerpoint table to the profile sequence as absolute position targets.

PB4 is the blade-state output. It is high for slot and low for tooth. Runtime code evaluates the calibrated edge intervals against the current encoder angle in the motor control loop, using a cached adjacent-first interval lookup before falling back to a full scan. PB4 is not driven by the photo-interrupter callback, so it remains valid after the photo-interrupter emitter is disabled.

## Implementation Phases
Phase 1:
Add `chopper-slot-count` and optional `chopper-tooth-count` devicetree properties, initialize runtime geometry from them, and add `motor chopper geometry [slots [teeth]]`.

Phase 2:
Refactor `motor chopper calib start` to use configured geometry, accept signed velocity, allow velocity encoder mode, and compute `slots + teeth` centerpoints from sorted edges.

Phase 3:
Add a `chopper` settings group for geometry and the canonical edge map. Centerpoints are derived after load and are not persisted.

Phase 4:
Build and run HIL capture on the MT6835 target after baseline electrical/encoder settings are loaded.

Phase 5:
Add PB4 blade-state output driven by the calibrated edge map and encoder angle in the control loop.

## Acceptance Criteria
`motor chopper geometry` reports `slots=8 teeth=8 centers=16` by default.
`motor chopper calib start <revs> <velocity_hz>` captures `16 * revs` edges for the 8-slot wheel.
`motor chopper calib status` prints 16 slot/tooth centerpoints approximately 22.5 degrees apart.
`motor chopper calib apply` loads 16 sequence points.
`motor settings save chopper` and `motor settings load chopper` preserve the geometry and edge map; centerpoints are derived at runtime.
PB4 is high in calibrated slot intervals and low in calibrated tooth intervals based on encoder position, regardless of whether the photo-interrupter emitter is enabled.
Legacy centerpoint settings are not loaded or migrated. Targets with old center-only chopper settings must clear settings and save a new edge map.

## HIL Evidence
Use the MT6835 HIL build. After baseline settings are loaded and generated-angle
motion works:

```text
motor state calibrate
motor safety timeout 0
motor arm
motor chopper geometry 8
motor current iq 0.07
motor chopper calib bidir 4 0.25 2.0
motor chopper calib status
motor current iq 0
motor velocity target 0
motor chopper calib apply
motor disarm
motor settings save chopper
```

## Risks
Missed optical edges will corrupt bin-to-edge association. The current implementation assumes no missed accepted edges during a capture pass and reports discarded edges but does not yet cross-correlate forward/reverse captures into one averaged map.

## Done State
Code, docs, build evidence, and HIL evidence are committed. Timer/capture-triggered index playback is tracked as follow-up work.

## Progress
2026-05-08:
Implemented devicetree-backed chopper geometry, timer capture edge-status reporting,
photo-interrupter emitter shell control, slot/tooth midpoint capture, and the
Settings/ZMS `chopper` group.

Build evidence:

```text
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build -b smartstepper_v2/stm32h743xx /workspace/chopper/app -d /workspace/build/chopper/smartstepper_v2_mt6835_067a -S serial-shell -S serial-console -- -DDTC_OVERLAY_FILE="boards/smartstepper_v2.overlay;configs/encoder_mt6835_rtspi.overlay;configs/commissioning_default.overlay;configs/motor_id_safe_067a.overlay" -DEXTRA_CONF_FILE="hil_shell.conf;logging.conf"'
Result: PASS
```

Devicetree evidence:

```text
/workspace/build/chopper/smartstepper_v2_mt6835_067a/zephyr/zephyr.dts contains capture-gpios and chopper-slot-count = <8>.
```

HIL evidence:

```text
Build:
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2_mt6835_067a'
Result: PASS

Flash:
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2_mt6835_067a --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
Result: PASS

Commands:
motor settings status
motor state calibrate
motor safety timeout 0
motor arm
motor current iq 0.07
motor chopper calib clear
motor chopper calib start 2 0.25
motor chopper calib status
motor current iq 0
motor velocity target 0
motor chopper calib apply
motor disarm
motor settings save chopper
motor settings status
motor chopper calib clear
motor chopper geometry
motor settings load chopper
motor chopper geometry

Result:
- Capture complete and valid.
- Edges: 32 / 32.
- Midpoints: 16.
- Labels: alternating 8 slot / 8 tooth.
- Spacing min/max/mean/error: 21.457 / 23.725 / 22.500 / 1.225 deg.
- `motor chopper calib apply` loaded 16 midpoint targets.
- `motor settings save chopper` saved the chopper settings group.
- `motor settings status` reports `chopper=YES` present and valid.
- After `motor chopper calib clear`, geometry reported `Map valid: NO`.
- After `motor settings load chopper`, geometry reported `Map valid: YES`.
```

Bidirectional HIL evidence:

```text
Fixes validated:
- Chopper capture now uses fresh raw encoder angle in controller coordinates.
- Generated-angle control remains the commutation source in `velocity_generated`.
- Raw encoder sample freshness is published separately from selected control
  feedback freshness, so generated modes can still use encoder diagnostics.
- Reverse captures invert rising/falling meaning when assigning the region after
  each sorted edge, keeping slot/tooth labels stable across direction.

Build:
podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2_mt6835_067a'
Result: PASS

Flash:
podman exec wonderful_goldberg bash -lc 'cd /workspace && west flash -d /workspace/build/chopper/smartstepper_v2_mt6835_067a --runner jlink --dev-id 10.0.0.70 --dev-id-type ip'
Result: PASS

Forward capture:
motor state calibrate
motor safety timeout 0
motor arm
motor current iq 0.07
motor chopper calib clear
motor chopper calib start 4 0.25
motor chopper calib status

Forward result:
- Edges: 64 / 64.
- Discarded: 142.
- Midpoints: 16.
- Labels: alternating 8 slot / 8 tooth.
- Spacing min/max/mean/error: 21.261 / 23.867 / 22.500 / 1.367 deg.

Reverse capture:
motor chopper calib clear
motor chopper calib start 4 -0.25
motor chopper calib status

Reverse result:
- Edges: 64 / 64.
- Discarded: 115.
- Midpoints: 16.
- Labels: alternating 8 slot / 8 tooth.
- Spacing min/max/mean/error: 21.197 / 23.790 / 22.500 / 1.303 deg.
- Midpoint table agrees with forward capture within roughly 0.5 deg.

Save:
motor current iq 0
motor velocity target 0
motor chopper calib apply
motor disarm
motor settings save chopper
motor settings status
motor chopper geometry

Save result:
- `motor settings status` reports `chopper=YES` present and valid.
- `motor chopper geometry` reports `Map valid: YES`.
- Chopper settings are persistent in Settings/ZMS. Enable boot restore with
  `motor settings autoload enable chopper` or by saving an autoload group set
  that includes `chopper`.
```

Bidirectional averaging command evidence:

```text
Command:
motor chopper calib bidir 4 0.25 2.0

Result:
- Forward edges/discarded: 64 / 150.
- Reverse edges/discarded: 64 / 248.
- F/R delta max/mean: 0.518 / 0.478 deg.
- Averaged spacing min/max/mean/error: 22.317 / 22.737 / 22.500 / 0.237 deg.
- The averaged map was staged by the command, applied with
  `motor chopper calib apply`, and saved with `motor settings save chopper`.
- `motor settings status` reports generation 7 with `chopper=YES` present and
  valid.
```

2026-05-08 edge-map canonicalization:

- Changed Settings/ZMS chopper persistence to store only the canonical edge map:
  `edge_count`, `edges_rad`, and `edge_region_after`.
- Removed legacy centerpoint key loading/migration. Existing center-only
  persisted maps are intentionally ignored; clear settings and regenerate the
  chopper map.
- Added shared edge-map derivation/lookup helper used by settings load,
  calibration, and runtime blade-state lookup.
- Added PB4 blade-state output: high=slot, low=tooth, driven from encoder angle
  against the calibrated edge intervals in the control loop.
- Added adjacent-first cached lookup so normal runtime checks inspect current,
  next, and previous edge intervals before a full scan.

Validation:

```text
rg -n "chopper_center|center_count|centers_rad|center_kind|MOTOR_SETTINGS_CHOPPER_FIELD_.*CENTER|KEY_CHOPPER_CENTER|motor/chopper/center" app/src app/include modules -S
Result: no matches

git diff --check
Result: PASS

podman exec wonderful_goldberg bash -lc 'cd /workspace && west build --build-dir /workspace/build/chopper/smartstepper_v2_mt6835_067a'
Result: PASS

Resolved DTS:
/chopper_blade_state {
    compatible = "rubus,app-gpio";
    gpios = < &gpiob 0x4 0x0 >;
    status = "okay";
};

./tests/run_unit_tests.sh wonderful_goldberg -s chopper.runtime.unit
Result: PASS, 9/9 cases

./tests/run_unit_tests.sh wonderful_goldberg -s chopper.motor_persistent_config.unit
Result: PASS, 4/4 cases
```

Note: the first parallel unit-test attempt raced on the shared
`/tmp/twister-unit` output directory. Rerunning the suites sequentially passed.

Autoload evidence:

```text
Commands:
motor settings autoload enable baseline chopper
motor settings autoload status
kernel reboot cold
motor settings autoload status
motor chopper geometry
motor settings status

Result:
- Boot log reported `Motor settings autoloaded groups: 0x00000043`.
- `0x43` is encoder + model_electrical + chopper.
- `motor settings autoload status` reports chopper enabled with baseline.
- `motor chopper geometry` reports `Map valid: YES` immediately after reboot.
- `motor settings status` reports `chopper=YES` present and valid.
```
