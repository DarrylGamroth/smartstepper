# T002 - Shared Commissioning Overlay

## Goal

Split generic commissioning recipe defaults out of motor safe-ID overlays.

## Work

1. Add `app/configs/commissioning_default.overlay` for generic recipe values.
2. Move generic recipe defaults into that overlay:
   - sample counts,
   - settle/ramp timing,
   - demod pulse timing and half-cycles,
   - validation thresholds,
   - one-command commissioning attempt counts and confidence gates.
3. Keep motor/current-class-specific values in `motor_id_safe_*.overlay`:
   - `commission-electrical-rs-current-ma`, unless later derived from max current,
   - `commission-electrical-current-limit-ma`, unless later derived from max current,
   - max pulse voltage/current limits where motor/driver safety requires an
     override.
4. Update `AGENTS.md` build examples to compose overlays in this order:
   - board overlay,
   - encoder overlay,
   - shared commissioning overlay,
   - motor safe-ID overlay.
5. Keep full `motor_*.overlay` convenience profiles consistent or document them
   as convenience profiles that duplicate the composed stack.

## Constraints

- Motor-specific overrides must remain possible by placing the motor overlay
  after `commissioning_default.overlay`.
- Do not put encoder protocol details into the shared commissioning overlay.
- Do not put per-unit measured values into devicetree.

## Validation

- MT6835 composed build passes.
- AEAT-9955 composed build passes.
- Resolved `zephyr.dts` shows exactly one `/user_parameters` and one
  `/motor_parameters` node with expected overrides.
