# Current Transition Matrix Baseline

## Lifecycle States

- `HW_INIT -> CTRL_INIT -> IDLE` on boot.
- `IDLE -> CALIBRATION` on calibration/commission request.
- `IDLE -> ONLINE_<requested>` on online request when calibration is complete.
- `IDLE -> PREPARE_ONLINE -> CALIBRATION -> ONLINE_<requested>` for prepare/calibration paths.
- `ONLINE_* -> IDLE` on idle request.
- `ONLINE_* -> ERROR` on error event.
- `ERROR -> IDLE` on clear error after basic reset.

## Online Modes

- `ONLINE_VELOCITY_GENERATED`: generated angle, commanded Id/Iq, velocity trajectory.
- `ONLINE_POSITION_GENERATED`: generated angle, commanded Id/Iq, position profile/sequence.
- `ONLINE_CURRENT_ENCODER`: encoder angle, commanded Id/Iq.
- `ONLINE_VELOCITY_ENCODER`: encoder angle/feedback, velocity loop current source.
- `ONLINE_POSITION_ENCODER`: encoder angle/feedback, position loop into velocity/current loop.

## Setup And Commissioning

- Boot calibration currently means current offset only.
- Full commissioning command runs electrical ID, generated-sweep encoder mapping, current validation, flux ID, mechanical ID, and tuning.
- Encoder mapping must be complete before encoder online modes.

## Observed Ambiguity

- `motor state mode <mode>` while in `IDLE` stages `requested_online_mode` only.
- `motor state online` is required to actually enter the staged mode.
- Command echo text can report a request was accepted before the SMF transition completes.
- `requested_online_mode` persists and can affect later workflows if not reset explicitly.

## Baseline HIL Observations

- Generated velocity mode entered and produced phase current/motion with `Iq=0.15 A`, `velocity=0.5 Hz`.
- Encoder current mode entered after commissioning and produced phase current/motion in a snapshot run, but behavior around detent/velocity workflows remains inconsistent.
- Detent capture did not begin visible movement in the interrupted HIL run because it did not reach the printed capture phase before abort.
