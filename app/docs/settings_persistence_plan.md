# Motor Settings Persistence Readiness Plan

Date: 2026-05-05

This document defines the persistence contract for commissioned motor data. It
is intentionally a readiness plan: no automatic boot load, EEPROM write, or
settings backend is enabled until encoder-control HIL stability is acceptable.

## Current Policy

- Runtime commissioning remains explicit and volatile.
- Boot does not autoload persisted motor parameters.
- No persistence command writes EEPROM/NVMEM yet.
- Devicetree motor parameters remain safe fallback defaults.
- A future persistence feature must be opt-in, shell-visible, versioned,
  CRC-protected, and rollback-safe.

## Storage Backend Direction

Preferred backend is Zephyr Settings over an NVMEM/EEPROM-backed storage area,
but the schema is independent of the backend. The first backend should use the
existing board EEPROM only after the application has explicit operator commands
for save/load/clear and HIL validation gates.

Expected backend options:

- Zephyr Settings with NVS/NVMEM backend if the board EEPROM path is stable.
- Direct NVMEM cell/partition access for a compact binary record if Settings
  overhead is too high.
- Two-slot record layout for rollback-safe writes if bypassing Settings.

## Schema

The binary schema lives in `modules/motor_core/include/motor/runtime/persistent_config.h`.
Version 1 record:

- Header:
  - magic: `MOTOR_PERSISTENT_CONFIG_MAGIC`.
  - schema version: `MOTOR_PERSISTENT_CONFIG_SCHEMA_V1`.
  - header/payload/record sizes.
  - generation counter.
  - validity flags.
  - payload CRC32.
- Payload groups:
  - current offsets: `Ia` and `Ib` ADC-current offsets.
  - encoder mapping: direction sign, mechanical commutation offset, electrical
    trim, mapping correlation, mapping residual.
  - motor model: `Rs`, `Ld`, `Lq`, flux linkage, `Kt`, inertia, viscous
    friction, Coulomb friction.
  - controller defaults: velocity PI, position PI, velocity MPR, position MPR,
    and velocity DOB defaults.
  - detent metadata: enable, bins, gain, current limit, and a future table CRC.

Detent table samples are intentionally not embedded in V1. A 256-bin table can
consume roughly 1 KiB by itself, so it should be stored as a separate chunk with
its own CRC/generation if persistence is later enabled.

## Validity Flags

The record can carry partial commissioned data. Apply code must only consume a
payload group if the corresponding flag is set and the record CRC/schema is
valid:

- `MOTOR_PERSISTENT_CONFIG_FLAG_CURRENT_OFFSETS_VALID`.
- `MOTOR_PERSISTENT_CONFIG_FLAG_ENCODER_MAPPING_VALID`.
- `MOTOR_PERSISTENT_CONFIG_FLAG_MOTOR_MODEL_VALID`.
- `MOTOR_PERSISTENT_CONFIG_FLAG_CONTROLLERS_VALID`.
- `MOTOR_PERSISTENT_CONFIG_FLAG_DETENT_META_VALID`.

## Load Policy

Future load command behavior:

1. Read the candidate record.
2. Validate magic, schema version, sizes, and payload CRC.
3. Print a preview of every group and validity flag.
4. Refuse automatic apply while the motor is armed or online.
5. Apply only explicitly requested groups.
6. Reinitialize affected fast-loop state after apply:
   - current PI integrators reset when current-loop gains/limits change.
   - angle observer reset when encoder mapping changes.
   - MPR/DOB states invalidated/reinitialized when model/controller settings
     change.
7. Keep boot autoload disabled until P10 regression gates pass repeatedly.

Future shell shape:

```text
motor settings status
motor settings preview
motor settings load [group-mask]
motor settings save [group-mask]
motor settings clear
motor settings autoload <on|off>   # disabled/hidden until HIL gate passes
```

## Save Policy

Future save command behavior:

1. Require IDLE/disarmed state unless explicitly saving non-control metadata.
2. Build a V1 payload from currently applied commissioning data.
3. Set only flags for groups that have valid runtime evidence.
4. Increment generation.
5. Compute CRC over the payload.
6. Write to the inactive slot or Settings key.
7. Read back and validate before reporting success.

## Rollback Policy

If using a two-slot binary backend:

- Store slot A and slot B records with independent generation and CRC.
- Load the highest-generation valid record.
- If the newest record is corrupt, fall back to the previous valid generation.
- Never erase the previous valid record until the new record validates after
  write/readback.

If using Settings:

- Store a single packed binary value for V1 first, not many independent float
  keys. This keeps schema/version/CRC behavior explicit.
- Use a second key for the detent table chunk if needed later.

## HIL Gate Before Enabling Autoload

Autoload should remain disabled until all required checks pass on the target:

- `status` HIL pass after flash/reset.
- boot commissioning pass.
- current_encoder validation pass.
- velocity_encoder validation pass with accepted gains and zero motor faults.
- position_encoder validation pass once the position loop is stable.
- no encoder acquisition hard errors above the accepted threshold.

Current state on 2026-05-05: velocity_encoder validation still fails the HIL
acceptance threshold, so persistence autoload must stay disabled.
