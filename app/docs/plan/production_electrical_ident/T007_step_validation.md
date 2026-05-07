# T007 - Current Step Validation

## Scope

Validate recommended current PI gains before promotion:

- Small positive/negative Id steps.
- Optional Iq steps after encoder mapping/current mode are safe.
- Measure rise time, overshoot, settling error, saturation, and fault status.

## Policy

- If validation passes, stage measured values for explicit apply.
- If validation fails, revert PI gains and keep production result rejected.
