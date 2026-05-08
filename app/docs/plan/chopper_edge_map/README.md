# Chopper Edge Map

## Goal
Build a calibrated chopper wheel index table from optical edge captures so position moves target slot/tooth centerpoints instead of ideal 22.5 degree assumptions.

## Non-Goals
Do not implement final hardware timer/capture ISR index playback in this phase.
Do not optimize position PI further until the index table exists.
Do not require mechanical ID to succeed.

## Current Problem
The position controller can move to angular targets, but the chopper wheel has real slot/tooth geometry. The application needs the actual centerpoints of the slots and teeth so each external tick can command the next calibrated index position. The existing `motor chopper calib` command captures optical edges, but geometry is passed manually, only slot midpoints are produced, and the map is not persisted.

## Design
Store chopper wheel geometry as devicetree defaults and runtime settings. For an 8-slot blade, default `chopper-slot-count=8`; the tooth count is derived as 8 unless a nonstandard wheel overrides it, producing 16 slot/tooth centerpoints. Capture optical edges while running controlled velocity in either direction, average repeated edges over multiple revolutions, sort the edge angles, classify regions using timer rising/falling status, and calculate every region centerpoint between adjacent edges. Apply the resulting centerpoint table to the profile sequence as absolute position targets.

## Implementation Phases
Phase 1:
Add `chopper-slot-count` and optional `chopper-tooth-count` devicetree properties, initialize runtime geometry from them, and add `motor chopper geometry [slots [teeth]]`.

Phase 2:
Refactor `motor chopper calib start` to use configured geometry, accept signed velocity, allow velocity encoder mode, and compute `slots + teeth` centerpoints from sorted edges.

Phase 3:
Add a `chopper` settings group for geometry and the centerpoint table.

Phase 4:
Build and run HIL capture on the MT6835 target after baseline electrical/encoder settings are loaded.

## Acceptance Criteria
`motor chopper geometry` reports `slots=8 teeth=8 centers=16` by default.
`motor chopper calib start <revs> <velocity_hz>` captures `16 * revs` edges for the 8-slot wheel.
`motor chopper calib status` prints 16 slot/tooth centerpoints approximately 22.5 degrees apart.
`motor chopper calib apply` loads 16 sequence points.
`motor settings save chopper` and `motor settings load chopper` preserve the geometry and table.

## HIL Evidence
Use the MT6835 HIL build. After baseline settings are loaded and velocity PI is stable:

```text
motor state mode velocity_encoder
motor arm
motor velocity target 0
motor chopper geometry 8
motor chopper calib start 4 0.5
motor chopper calib status
motor chopper calib apply
motor settings save chopper
```

Repeat in reverse with `velocity_hz=-0.5` and compare the center table.

## Risks
Missed optical edges will corrupt bin-to-edge association. The first implementation assumes no missed edges during a capture pass and reports discarded edges but does not yet cross-correlate forward/reverse captures.

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
