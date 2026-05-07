# T008 - Validation and HIL Evidence

## Required Tests

### Unit

- Electrical ID tests.
- Current PI recommendation tests.
- Encoder map detect tests.
- Persistent config pack/CRC tests once T007 is implemented.
- MPR/DOB/detent tests must still pass, but they are advanced/non-baseline.

### Build

- MT6835 overlay optimized build.
- AEAT overlay optimized build if touched.

### HIL Baseline

Run after flash/reset:

```text
motor state clear_error
motor disarm
motor state idle
motor safety timeout 0
motor commission baseline run apply
motor commission status
motor commission electrical status
motor commission encoder status
motor velocity pi status
motor state status
```

Acceptance:

- baseline ready YES,
- electrical ready YES,
- encoder mapped YES,
- current PI applied,
- flux measured or explicit fallback,
- no motor error,
- encoder transport/frame/crc/status/glitch counters within threshold.

### HIL Control Baseline

- `velocity_generated` smoke test.
- `velocity_encoder` PI sweep at low speeds.
- `position_generated` indexed move smoke test.
- `position_encoder` only after velocity PI baseline is stable.

## Evidence

Append evidence to `execution_log.md` with command, result, and key metrics.
