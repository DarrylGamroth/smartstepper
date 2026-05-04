# Encoder Mode Stabilization Plan

## Goal

Make `_encoder` control modes deterministic and safe to enter from a fresh boot. Encoder-control modes must not start unless the commutation mapping and encoder transport are known good. Generated/open-loop modes must not regress.

## Phase 1: Readiness Gate

- Add one application-level readiness check for encoder-control modes.
- Require:
  - `encoder1` device ready.
  - Generated-sweep encoder mapping has been applied.
  - Encoder acquisition is not busy during mode entry.
  - Test fault injection is disabled.
  - AEAT-9955 fast driver is in SPI4-8 CRC16 mode when that encoder is built in.
  - AEAT-9955 volatile registers match the expected SPI4-8/CRC16 runtime configuration when register reads are safe.
- Report a human-readable reject reason for shell/HIL use.

## Phase 2: Mode Entry Integration

- Reject shell requests for `current_encoder`, `velocity_encoder`, and `position_encoder` if readiness fails.
- Reject SMF mode-change events to encoder modes if readiness fails.
- When a stored requested online mode is invalid at online entry, fall back to `velocity_generated` and log the reason instead of entering an unsafe encoder mode.

## Phase 3: Diagnostics

- Add a shell status command showing the readiness gate and relevant encoder acquisition/protocol counters.
- Keep raw/capture trace functionality separate; high-rate telemetry remains opt-in.
- Generated-sweep mapping reports residuals explicitly. For hybrid steppers, the electrical offset RMS threshold is intentionally looser than a BLDC alignment threshold because detent torque/open-loop load angle modulate the rotor during the sweep.

## Phase 4: HIL Validation

From a power-cycled target:

1. Confirm AEAT protocol detect/status.
2. Confirm `motor gate reset` works and does not trigger MCPWM break storms.
3. Run boot current-offset calibration.
4. Run generated-sweep encoder mapping and apply it.
5. Test `current_encoder` with a low `Iq`.
6. Test `velocity_encoder` at low speed only after `current_encoder` is clean.
7. Test `position_encoder` hold/profile only after velocity is clean.

If the target enters an unknown or inconsistent state, treat it as a real fault to diagnose; do not hide it with retries.

## Implemented Fixes

- Added `motor encoder control_status` readiness diagnostics.
- Added encoder-mode readiness checks in shell mode requests, SMF mode-change handling, and stored online-mode resolution.
- Fixed AEAT-9955 SPI4-8 register reads to account for the device response latency on register accesses.
- Fixed generated-mode diagnostic capture so raw encoder samples are visible even when the encoder is not selected for control.
- Made generated-sweep encoder commissioning write current setpoints directly during synchronous capture.
- Added bounded-error tolerance to encoder mapping so a small number of rejected CRC/error samples does not invalidate an otherwise high-quality fit.
- Cleared acquisition counters after a successful mapping run because the mapping result preserves the rejected/error counts.
- Fixed encoder handoff by reseeding the angle observer on the first fresh encoder sample after a non-encoder source.
- Stopped stale-count accumulation when encoder sampling is disabled.
- Allowed encoder-to-encoder transitions while the realtime encoder acquisition is active; these transitions skip unsafe register rereads and treat historical counters as diagnostics.
- Reset position encoder hold entry to zero current/zero position target velocity instead of carrying the previous velocity-loop Iq seed.

## Validation Evidence

- Unit: `chopper.motor_encoder_feedback_core.unit` passed, including encoder handoff reseed coverage.
- Unit: `chopper.motor_encoder_map_detect.unit` passed, including bounded rejected-sample coverage.
- Build: `west build --build-dir /workspace/build/chopper/smartstepper_v2` passed.
- Flash: `west flash -d /workspace/build/chopper/smartstepper_v2 --runner jlink --dev-id 10.0.0.70 --dev-id-type ip` passed.
- HIL: fresh target boot detected AEAT-9955 in `spi4-8-crc16`, `CPOL=1 CPHA=1`, `Config0=0xF0`, `SPI4/UVW=0xC0`, `PSEL=0x00`.
- HIL: generated-sweep mapping passed with `dir=-1`, correlation about `-0.982`, offset about `-1.59 deg mechanical`, and no final-pass acquisition errors.
- HIL: `current_encoder -> velocity_encoder -> position_encoder` transitions completed without faults.
- HIL: `position_encoder` hold settled at approximately zero Iq and encoder acquisition counters showed no transport/frame/parity/CRC/status/glitch errors in the final pass.

## Remaining Observations

- Occasional isolated AEAT CRC errors still appear during long sweeps on some runs. The control path did not fault on isolated events, and the acquisition counters expose them for diagnosis.
- `motor_states` still sometimes logs `State timer expired without queued timeout event; proceeding` during offset calibration. This did not block the encoder-mode work, but it should be reviewed separately.
- `velocity_encoder` can demand current without producing smooth motion at low speed; this now appears to be tuning/commutation quality rather than an encoder transport or mode-entry fault.
