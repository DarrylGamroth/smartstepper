# T005 - Windowed Acceleration Estimation

## Problem

The current acceleration estimate is a first difference of measured velocity
plus a simple IIR filter. This is fragile because encoder quantization, observer
latency, and velocity-controller dynamics directly affect `alpha`, which drives
`J` estimation.

## Objective

Compute commissioning acceleration from buffered samples with a bounded,
repeatable offline/windowed estimator.

## Implementation

- Store enough position/velocity samples in the commissioning capture buffer for
  post-capture derivative estimation.
- Add a motor-core helper for acceleration estimation using one of:
  - central difference over a configurable window,
  - local linear fit to velocity,
  - local quadratic fit to position.
- Prefer deterministic fixed-window math suitable for MCU use.
- Keep ISR path simple: capture raw values; compute improved derivatives during
  finalize/fit, outside the ISR hot path.

## Acceptance Criteria

- Existing ISR capture cost does not meaningfully increase.
- Acceleration estimator reports invalid at capture boundaries or insufficient data.
- Inertia fit can choose raw-IIR or windowed acceleration for comparison.
- Shell/HIL output states which acceleration estimator was used.

## Tests

- Unit test constant velocity yields near-zero acceleration.
- Unit test known constant acceleration is recovered.
- Unit test noisy velocity improves with windowed fit compared with raw difference.
- Unit test boundary samples are rejected or marked invalid.

## HIL

- Capture the same mechanical profile with raw and windowed acceleration.
- Compare `J` repeatability and residual RMS.
