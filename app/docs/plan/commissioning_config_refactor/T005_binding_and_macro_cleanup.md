# T005 - Binding and Macro Cleanup

## Goal

Make devicetree bindings and config macros match the new ownership model.

## Work

1. Keep `/motor_parameters` for motor identity, hard limits, and conservative
   fallback model:
   - `pole-pairs`,
   - `max-current-ma`,
   - `max-speed-hz`,
   - fallback `resistance-mohms`,
   - fallback `inductance-d-uh`,
   - fallback `inductance-q-uh`,
   - optional fallback `flux_linkage_uvphz` when a reliable profile value exists,
   - fallback `inertia-mgcm2`.
2. Keep `/user_parameters` for control/profile/procedure defaults for now, but
   document that the name is historical.
3. Add comments in bindings explaining:
   - fallback values are safe boot defaults, not active commissioned values,
   - active values live in RAM and later Settings/ZMS,
   - `commission-*` means procedure recipe, not physical identity.
4. Remove stale fallbacks from macros once legacy properties are retired.
5. Review whether derived defaults can reduce overlay clutter:
   - Rs current from max current,
   - electrical current limit from max current,
   - overcurrent threshold from max current,
   - provisional bandwidth from measured model.
6. Defer any node rename from `/user_parameters` to a later compatibility-free
   binding task.

## Constraints

- Do not rename `/user_parameters` in this task unless all C macros and overlays
  are updated together.
- Keep unit conversions centralized in `app/include/config.h` or a replacement
  config extraction layer.
- Do not add more compile-time macros for values that should become runtime
  active model values.

## Validation

- Bindings document fallback/procedure semantics clearly.
- Build passes for both composed motor profiles.
- `git grep` shows no production commissioning macro falling back to retired
  legacy properties.
