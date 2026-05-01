# Operating Mode Taxonomy

The firmware names operating modes by command domain and feedback/angle source.
Avoid `open`/`closed` in new APIs because it hides what signal is generated,
measured, or required for commutation.

## User-Facing Modes

| Shell mode | SMF state | Meaning | Encoder required for control |
| --- | --- | --- | --- |
| `current_encoder` | `ONLINE_CURRENT_ENCODER` | Encoder-commutated direct `Id/Iq` current command. This is not a torque regulator by itself; torque is produced through commanded current. | Yes |
| `velocity_generated` | `ONLINE_VELOCITY_GENERATED` | Generated-angle velocity mode with commanded `Id/Iq`. Useful when encoder feedback is unavailable or diagnostic-only. | No |
| `position_generated` | `ONLINE_POSITION_GENERATED` | Generated-angle position/profile mode with commanded `Id/Iq`. Used for profile moves and profile sequences without encoder feedback. | No |
| `velocity_encoder` | `ONLINE_VELOCITY_ENCODER` | Encoder-feedback velocity control. The velocity loop produces the current command. | Yes |
| `position_encoder` | `ONLINE_POSITION_ENCODER` | Encoder-feedback position/profile control. The position loop feeds the velocity/current control path. | Yes |

## Layering Rules

1. Motion generation is independent of actuator type.
2. Generated-angle modes may sample the encoder for telemetry, but the encoder is not a control dependency.
3. Encoder-feedback modes require a valid encoder/observer angle path for commutation and feedback.
4. Raw encoder trace is diagnostic sampling. It can force encoder SPI sampling without enabling encoder control.
5. The angle observer owns wrapping, offset, latency compensation, mechanical angle, electrical angle, and velocity estimate.
6. The shell should describe both the control source and telemetry source clearly; avoid using `enabled` without saying what is enabled.

## Encoder Terminology

Use these terms consistently:

- `encoder_control`: encoder is part of the active control/commutation path.
- `encoder_required`: current operating mode cannot run correctly without encoder feedback.
- `trace_enabled`: raw diagnostic recorder is active.
- `ctrl_en`: trace/capture sample was taken while encoder control was enabled.
- `fresh`: a new physical encoder sample was received.
- `clean`: sample is fresh and has no warning, error, or IO fault.

## Retired Names

Do not add new code or docs using these names:

- `offline` for the prepare/calibration state.
- `torque` for direct current command mode.
- `velocity_open` for generated-angle velocity mode.
- `profile_open` for generated-angle profile mode.
- `velocity_closed` for encoder-feedback velocity mode.
- Bare `position` for encoder-feedback position mode.

Use `motor state prepare` for the energized prepare/calibration path before
online operation.
