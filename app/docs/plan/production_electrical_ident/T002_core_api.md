# T002 - Motor-Core API

## Scope

Create a motor-core production electrical ID module, e.g.:

- `modules/motor_core/include/motor/estimation/electrical_id.h`
- `modules/motor_core/src/estimation/electrical_id.c`

## API Shape

- Config structs for Rs and inductance measurement.
- ISR-step accumulator functions for high-rate samples.
- Finalize functions that compute results and quality metrics.
- PI recommendation helper using measured `Rs`, `Ld`, and `Lq`.

## Constraints

- No dynamic allocation.
- No shell or Zephyr kernel dependencies in motor-core math.
- Bounded arrays or scalar accumulators only.
- Unit-testable with synthetic data.
