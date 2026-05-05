# Boot Commissioning Plan

## Goal

Provide a repeatable boot-time commissioning workflow for hardware where encoder
mapping is not persisted yet. The workflow must run:

1. Current offset calibration.
2. Generated-sweep encoder mapping.
3. Apply the staged encoder mapping to the active runtime configuration.

This is intentionally runtime-only for now. The operator must run the command
after each boot until non-volatile persistence is added.

## Shell Interface

Add:

```text
motor commission boot [current_a] [mech_hz] [cycles]
```

Defaults:

- `current_a = 0.150`
- `mech_hz = 0.100`
- `cycles = 1.0`

The command prints progress for each stage and fails fast on state, calibration,
encoder acquisition, or mapping-quality errors.

## Implementation Plan

### Phase 1: Factor Encoder Mapping

- Extract argument validation for generated-sweep encoder mapping.
- Extract the current encoder mapping sweep implementation into an internal
  helper.
- Extract encoder mapping application into an internal helper.
- Keep existing `motor commission encoder run/apply/status/clear` behavior.

### Phase 2: Add Boot Workflow Command

- Request current offset calibration with `motor_api_request_calibrate()`.
- Wait for calibration to complete and for the state machine to reach
  `ONLINE_VELOCITY_GENERATED`.
- Arm control output using the same runtime state used by `motor arm`.
- Run generated-sweep encoder mapping.
- Apply the staged mapping.
- Leave current and velocity targets at zero at the end of the workflow.

### Phase 3: Register and Validate

- Register the new shell command under `motor commission`.
- Build firmware with the standard west build path.
- Run unit tests if touched library code requires it.
- Optionally flash and execute the command on hardware when safe.

## Progress

- [x] Plan created.
- [x] Encoder mapping factored.
- [x] Boot workflow command added.
- [x] Firmware build passed.
- [x] Hardware command smoke-tested.

## Validation Evidence

- `west build --build-dir /workspace/build/chopper/smartstepper_v2`: passed.
- `west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip`: passed.
- `motor commission boot 0.15 0.10 1`: passed on AEAT-9955 hardware.
  - Current offsets: `Ia=2.9943`, `Ib=2.9953`.
  - Encoder mapping: valid, direction `-1`, correlation `-0.9824`,
    mechanical offset `-1.584 deg`, 500 accepted samples, 0 rejected,
    0 warnings, 0 errors.
  - Post-check: `motor encoder control_status` reported `Ready: YES`.

## Operating Notes

Run after each boot:

```text
motor commission boot 0.15 0.10 1
```

Then encoder-based current, velocity, and position modes can use the applied
runtime mapping. Because this is not persisted, a reset or power cycle requires
running the command again.
