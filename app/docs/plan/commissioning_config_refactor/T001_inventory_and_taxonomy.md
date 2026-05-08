# T001 - Inventory and Taxonomy

## Goal

Classify every devicetree parameter involved in commissioning so each value has
one clear owner.

## Work

1. Inventory properties in:
   - `dts/bindings/rubus,user-parameters.yaml`
   - `dts/bindings/rubus,motor-parameters.yaml`
   - `app/configs/motor_id_safe_*.overlay`
   - `app/configs/motor_*.overlay`
   - `app/configs/encoder_*_rtspi.overlay`
2. Classify each property as:
   - board hardware,
   - encoder transport/protocol,
   - motor identity,
   - hard safety limit,
   - conservative fallback model,
   - generic commissioning recipe,
   - experimental/diagnostic feature default,
   - per-unit commissioned value.
3. Mark stale or transitional properties:
   - `rs-est-*`: legacy DC Rs path, keep only while fallback state compiles.
   - `roverl-est-*`: keep as RoverL bootstrap settings.
   - scalar/pulse inductance settings: remove if only used by retired scalar L
     path.
   - RLS/thermal settings: move to experimental overlay if not baseline.
4. Record the classification table in this plan package before changing code.

## Constraints

- Do not rename bindings in this task.
- Do not remove runtime paths in this task.
- Do not move hardware topology to Settings.

## Validation

- Inventory document identifies an owner for every commissioning-related DT
  property.
- Stale/legacy properties have explicit disposition: keep, move, deprecate, or
  remove.
